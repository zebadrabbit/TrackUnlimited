// Live NL2 telemetry recorder and comparison tool.
// Build:  clang++ -std=c++17 -Wall -Wextra -o record.exe record.cpp -lws2_32
// Run:    .\record.exe [--port 15151] [--host 127.0.0.1] [--seconds 90]
//                      [--track ../../NL2/trackdata.csv] [--out telemetry.csv]
//
// Keep the .exe on the output name. `-o record` emits an EXTENSIONLESS binary:
// Git Bash will happily run it, but PowerShell and Explorer do not recognise it
// as executable and pop the "how do you want to open this file?" dialog instead.
// The .gitignore covers both spellings, so there is no reason not to.
//
// This is the only file in the prototypes that touches a socket, and the only
// one that needs NoLimits 2 actually running. Everything it does with the bytes
// afterwards lives in NL2Telemetry.h, which is tested offline.
//
// What it is for, in order:
//   1. Identify which G axis NL2 reports gravity on, from the data rather than
//      from the docs — the telemetry spec never says what frame G is in.
//   2. Settle the lateral-G sign against our own model, at the same physical
//      point on the track, by correlating on world position.
//   3. Dump a speed trace to calibrate RollingResistance and DragK against.
//
// Both 2 and 3 are open items in Docs/PHASE0_FINDINGS.md "Still unknown".
//
// ponytail: blocking socket, one request in flight, poll as fast as replies
// come back. NL2 answers a Get Telemetry in well under a millisecond on
// loopback and the ride is the slow part. If this ever needs to not stall the
// caller, that is a thread, not a rewrite.

#include "NL2Telemetry.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
// windows.h defines max/min as macros, which breaks std::max at the call site.
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
using SocketHandle = SOCKET;
static const SocketHandle kInvalidSocket = INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
using SocketHandle = int;
static const SocketHandle kInvalidSocket = -1;
#endif

static const double Pi = 3.14159265358979323846;

// ------------------------------------------------------------------- sockets

struct FSocketLayer
{
    FSocketLayer()
    {
#ifdef _WIN32
        WSADATA Wsa;
        WSAStartup(MAKEWORD(2, 2), &Wsa);
#endif
    }
    ~FSocketLayer()
    {
#ifdef _WIN32
        WSACleanup();
#endif
    }
};

static void CloseSocket(SocketHandle S)
{
#ifdef _WIN32
    closesocket(S);
#else
    close(S);
#endif
}

static SocketHandle ConnectTo(const std::string& Host, int Port, std::string& Error)
{
    SocketHandle S = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (S == kInvalidSocket)
    {
        Error = "socket() failed";
        return kInvalidSocket;
    }

    sockaddr_in Addr;
    std::memset(&Addr, 0, sizeof(Addr));
    Addr.sin_family = AF_INET;
    Addr.sin_port = htons(static_cast<unsigned short>(Port));
    if (inet_pton(AF_INET, Host.c_str(), &Addr.sin_addr) != 1)
    {
        Error = "bad host address " + Host;
        CloseSocket(S);
        return kInvalidSocket;
    }

    if (connect(S, reinterpret_cast<sockaddr*>(&Addr), sizeof(Addr)) != 0)
    {
        Error = "cannot connect to " + Host + ":" + std::to_string(Port)
              + " — is NoLimits 2 running with telemetry enabled? "
                "(Main Menu -> Setup -> Others, or launch with --telemetry)";
        CloseSocket(S);
        return kInvalidSocket;
    }

    // Request/reply at ~100 Hz on tiny messages: Nagle would batch them into
    // 40 ms lumps and quantise the whole trace.
    int One = 1;
    setsockopt(S, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&One), sizeof(One));
    return S;
}

static bool SendAll(SocketHandle S, const std::vector<unsigned char>& Bytes)
{
    std::size_t Sent = 0;
    while (Sent < Bytes.size())
    {
        const int N = send(S, reinterpret_cast<const char*>(Bytes.data() + Sent),
                           static_cast<int>(Bytes.size() - Sent), 0);
        if (N <= 0)
        {
            return false;
        }
        Sent += static_cast<std::size_t>(N);
    }
    return true;
}

// Read until one whole message is available. Incomplete is normal — a 76-byte
// payload straddles TCP segments routinely — so only Malformed is fatal.
static bool ReadMessage(SocketHandle S, std::vector<unsigned char>& Buffer, FNL2Message& Out)
{
    for (;;)
    {
        std::size_t Used = 0;
        const ENL2Decode D = DecodeMessage(Buffer.data(), Buffer.size(), Out, Used);
        if (D == NL2Decode_Ok)
        {
            Buffer.erase(Buffer.begin(), Buffer.begin() + static_cast<std::ptrdiff_t>(Used));
            return true;
        }
        if (D == NL2Decode_Malformed)
        {
            return false;
        }

        char Chunk[2048];
        const int N = recv(S, Chunk, static_cast<int>(sizeof(Chunk)), 0);
        if (N <= 0)
        {
            return false;
        }
        Buffer.insert(Buffer.end(), Chunk, Chunk + N);
    }
}

// ---------------------------------------------------------------- correlation

// Arc length of each imported sample, so a telemetry position can be turned
// into an S on the reconstruction. NL2 world position is the only thing the two
// sides genuinely share — S origins differ (the importer rotates a circuit to
// start level) and the reconstruction is yaw-rotated and translated besides.
struct FTrackIndex
{
    std::vector<FNL2Sample> Samples; // rotated exactly as the importer rotates them
    std::vector<double> ArcLength;
    FTrack Track;

    bool Nearest(const FVec3& WorldPos, double& OutS, double& OutDistance) const
    {
        if (Samples.empty())
        {
            return false;
        }
        std::size_t Best = 0;
        double BestD = 1e300;
        for (std::size_t i = 0; i < Samples.size(); ++i)
        {
            const double D = Length(Samples[i].Position - WorldPos);
            if (D < BestD)
            {
                BestD = D;
                Best = i;
            }
        }
        OutS = ArcLength[Best];
        OutDistance = BestD;
        return true;
    }
};

static bool BuildIndex(const std::string& CsvPath, FTrackIndex& Out, std::string& Error)
{
    const FNL2Result Parsed = ReadNL2CsvFile(CsvPath);
    if (!Parsed.Ok())
    {
        Error = Parsed.Error;
        return false;
    }

    // Rotate here too, with the same call the importer makes, so ArcLength[i]
    // lines up with the FTrack built from the same list.
    Out.Samples = Parsed.Samples;
    RotateToLevelStart(Out.Samples);

    Error = TrackFromSamples(Out.Samples, Out.Track);
    if (!Error.empty())
    {
        return false;
    }

    Out.ArcLength.assign(Out.Samples.size(), 0.0);
    for (std::size_t i = 1; i < Out.Samples.size(); ++i)
    {
        Out.ArcLength[i] = Out.ArcLength[i - 1]
                         + Length(Out.Samples[i].Position - Out.Samples[i - 1].Position);
    }
    return true;
}

// Raw log plus, where the track is available, our own model's answer at the
// same place. State flags are included because "why is this all zeros" is
// answered by them and by nothing else.
static bool WriteLog(const std::string& Path, const std::vector<FNL2Telemetry>& Log,
                     const FTrackIndex& Index, bool bHaveTrack)
{
    std::ofstream Out(Path.c_str());
    if (!Out)
    {
        return false;
    }
    Out << "frame\tplay\tonride\tspeed\tposX\tposY\tposZ\tgx\tgy\tgz"
           "\tarcS\tmiss\ttrackPitchDeg\tourLateral\tourVertical\n";
    for (const FNL2Telemetry& T : Log)
    {
        double S = -1.0, Miss = -1.0, PitchDeg = 0.0, OurLat = 0.0, OurVert = 0.0;
        if (bHaveTrack && Index.Nearest(T.Position, S, Miss))
        {
            const FTrackFrame F = Index.Track.EvaluateAt(S);
            const FGForces G = FeltG(F, static_cast<double>(T.Speed));
            PitchDeg = std::asin(std::max(-1.0, std::min(1.0, F.Tangent.Z))) * 180.0 / Pi;
            OurLat = G.Lateral;
            OurVert = G.Vertical;
        }
        Out << T.Frame << '\t' << (T.InPlayMode() ? 1 : 0) << '\t' << (T.Onride() ? 1 : 0) << '\t'
            << T.Speed << '\t' << T.Position.X << '\t' << T.Position.Y << '\t' << T.Position.Z
            << '\t' << T.GX << '\t' << T.GY << '\t' << T.GZ << '\t' << S << '\t' << Miss << '\t'
            << PitchDeg << '\t' << OurLat << '\t' << OurVert << '\n';
    }
    return Out.good();
}

// ---------------------------------------------------------------------- main

int main(int argc, char** argv)
{
#ifdef _WIN32
    // This file is UTF-8 and the messages below contain em-dashes. A Windows
    // console defaults to a legacy OEM codepage and renders those as "ΓÇö", so
    // say what encoding we are actually emitting.
    SetConsoleOutputCP(CP_UTF8);
#endif
    // Line-buffered, so a run that stalls or is interrupted still shows how far
    // it got instead of losing everything in a block buffer.
    std::setvbuf(stdout, nullptr, _IOLBF, 4096);

    std::string Host = "127.0.0.1";
    int Port = 15151;
    double Seconds = 90.0;
    std::string TrackPath = "../../NL2/trackdata.csv";
    std::string OutPath = "telemetry.csv";

    for (int i = 1; i + 1 < argc; i += 2)
    {
        const std::string Key = argv[i];
        const std::string Value = argv[i + 1];
        if (Key == "--host") Host = Value;
        else if (Key == "--port") Port = std::atoi(Value.c_str());
        else if (Key == "--seconds") Seconds = std::atof(Value.c_str());
        else if (Key == "--track") TrackPath = Value;
        else if (Key == "--out") OutPath = Value;
        else { std::printf("unknown option %s\n", Key.c_str()); return 2; }
    }

    FTrackIndex Index;
    std::string Error;
    const bool bHaveTrack = BuildIndex(TrackPath, Index, Error);
    if (bHaveTrack)
    {
        std::printf("track: %s — %.3f m, %zu segments\n", TrackPath.c_str(),
                    Index.Track.TotalLength(), Index.Track.NumSegments());
    }
    else
    {
        std::printf("track: not loaded (%s)\n  — recording telemetry only, no comparison\n",
                    Error.c_str());
    }

    // Announced BEFORE the attempt, not after it succeeds. connect() blocks, and
    // a silent stall here looks identical to the program doing nothing at all.
    std::printf("connecting to %s:%d ...\n", Host.c_str(), Port);

    FSocketLayer Layer;
    const SocketHandle S = ConnectTo(Host, Port, Error);
    if (S == kInvalidSocket)
    {
        std::printf("\n%s\n", Error.c_str());
        return 1;
    }
    std::printf("connected to %s:%d\n", Host.c_str(), Port);

    // Version handshake first: it proves the framing is right before a long
    // recording, and fails loudly here rather than as a confusing empty log.
    std::vector<unsigned char> Buffer;
    FNL2Message Msg;
    if (!SendAll(S, EncodeRequest(NL2_GetVersion, 1)) || !ReadMessage(S, Buffer, Msg)
        || Msg.Type != NL2_Version || Msg.Data.size() < 4)
    {
        std::printf("version handshake failed (type %u)\n", Msg.Type);
        CloseSocket(S);
        return 1;
    }
    std::printf("NL2 version %u.%u.%u.%u\n", Msg.Data[0], Msg.Data[1], Msg.Data[2], Msg.Data[3]);
    std::printf("\nRide the coaster now. Recording for %.0f s — Ctrl-C to stop early.\n"
                "Sit still on level track for a moment first: that is what identifies\n"
                "which G axis carries gravity.\n\n",
                Seconds);

    std::vector<FNL2Telemetry> Log;
    int LastFrame = -1;
    std::uint32_t ReqId = 2;
    int PlayModeSamples = 0;

    // Bound by the WALL CLOCK, not by a poll count. Replies come back in well
    // under a millisecond on loopback, so a poll budget would burn through
    // "90 seconds" in about half of one and record almost nothing.
    const auto Deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(static_cast<long long>(Seconds * 1000.0));

    while (std::chrono::steady_clock::now() < Deadline)
    {
        if (!SendAll(S, EncodeRequest(NL2_GetTelemetry, ReqId++)))
        {
            std::printf("send failed — connection lost\n");
            break;
        }
        if (!ReadMessage(S, Buffer, Msg))
        {
            std::printf("stream desynchronised or closed\n");
            break;
        }
        if (Msg.Type != NL2_Telemetry)
        {
            continue;
        }

        FNL2Telemetry T;
        if (!ParseTelemetry(Msg.Data, T))
        {
            std::printf("unexpected telemetry payload size %zu (expected %zu)\n", Msg.Data.size(),
                        NL2TelemetryBytes);
            break;
        }
        // Dedupe on the rendered frame counter: polling outruns the renderer, and
        // repeated identical samples would weight a stationary train heavily in
        // the gravity-axis vote. Back off a millisecond when nothing new has been
        // drawn, so waiting on the renderer does not spin a core.
        if (T.Frame == LastFrame)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        LastFrame = T.Frame;
        Log.push_back(T);
        PlayModeSamples += T.InPlayMode() ? 1 : 0;

        if (Log.size() % 100 == 0)
        {
            std::printf("  %zu samples, speed %5.2f m/s%s   \r", Log.size(),
                        static_cast<double>(T.Speed), T.InPlayMode() ? "" : "  (NOT in play mode)");
            std::fflush(stdout);
        }
    }
    CloseSocket(S);
    std::printf("\n\nrecorded %zu samples, %d of them in play mode\n", Log.size(), PlayModeSamples);
    if (Log.empty())
    {
        return 1;
    }

    // Dump the raw log BEFORE any analysis, and before any early exit. A run
    // that produced nothing useful is exactly the run whose raw bytes you want
    // to look at, so bailing out without writing the file discards the evidence
    // at the one moment it matters.
    const bool bWroteLog = WriteLog(OutPath, Log, Index, bHaveTrack);
    if (bWroteLog)
    {
        std::printf("wrote %s (%zu rows)\n", OutPath.c_str(), Log.size());
    }

    // A recording of a paused game or a menu screen is all zeros, and every
    // conclusion below would be drawn from noise. Say so and stop, rather than
    // print a confident verdict about nothing.
    double FastestSeen = 0.0;
    int OnrideSamples = 0;
    for (const FNL2Telemetry& T : Log)
    {
        FastestSeen = std::max(FastestSeen, static_cast<double>(T.Speed));
        OnrideSamples += T.Onride() ? 1 : 0;
    }
    if (PlayModeSamples == 0 || FastestSeen < 1.0)
    {
        std::printf("\nNothing moved: top speed %.2f m/s, %d/%zu samples in play mode, "
                    "%d on-ride.\n",
                    FastestSeen, PlayModeSamples, Log.size(), OnrideSamples);
        std::printf("  The recorder was talking to NL2 (the handshake succeeded), but the\n"
                    "  train never moved. Load the park, start the ride, and keep NL2 in\n"
                    "  the foreground while this records — then run it again.\n");
        return 1;
    }

    // ---- 1. Which axis is gravity on?
    const FGravityAxisReport Grav = IdentifyGravityAxis(Log, 1.0);
    const char* AxisName[3] = {"x", "y", "z"};
    if (Grav.Axis < 0)
    {
        std::printf("\ngravity axis: UNDECIDED — no samples under 1 m/s. Record again and\n"
                    "  sit stationary in the station for a second or two.\n");
    }
    else
    {
        std::printf("\ngravity axis: G%s reads %+.4f at rest over %d samples "
                    "(next largest component %.4f)\n",
                    AxisName[Grav.Axis], Grav.Value, Grav.Samples, Grav.OtherMax);
        std::printf("  => NL2 reports %s\n",
                    Grav.Value > 0.0 ? "the force felt BY THE RIDER (same sense as our FeltG)"
                                     : "the acceleration OF THE CAR (opposite sense to our FeltG)");
    }

    // ---- 1b. Car-local or world? Decided on the lift hill, where the two
    // hypotheses finally disagree. Without this, a world-referenced G vector
    // would make "the lateral axis" a rotating quantity and every sign verdict
    // below meaningless.
    FGFrameReport GFrame;
    if (bHaveTrack && Grav.Axis >= 0)
    {
        std::vector<std::pair<double, double>> PitchAndReading;
        for (const FNL2Telemetry& T : Log)
        {
            double S = 0.0, Miss = 0.0;
            if (!Index.Nearest(T.Position, S, Miss) || Miss > 2.0)
            {
                continue;
            }
            // Steady climbing only: under acceleration the fore/aft term bleeds
            // into the comparison and the slope test stops being clean.
            if (T.Speed > 8.0)
            {
                continue;
            }
            const float G[3] = {T.GX, T.GY, T.GZ};
            const double Pitch = std::asin(std::max(-1.0, std::min(1.0,
                                    static_cast<double>(Index.Track.EvaluateAt(S).Tangent.Z))));
            PitchAndReading.push_back({Pitch, static_cast<double>(G[Grav.Axis])});
        }
        GFrame = ClassifyGFrame(PitchAndReading);

        if (GFrame.Frame == FGFrameReport::Undecided)
        {
            std::printf("\nG frame: UNDECIDED from %d steep samples "
                        "(mean cos(pitch) %.3f)\n"
                        "  Ride the lift hill slowly — that is the only place the car-local\n"
                        "  and world hypotheses predict different numbers.\n",
                        GFrame.Samples, GFrame.MeanCosPitch);
        }
        else
        {
            std::printf("\nG frame: %s — on slopes the gravity axis reads %.3f, "
                        "against cos(pitch) = %.3f and world = 1.000 (%d samples)\n",
                        GFrame.Frame == FGFrameReport::CarLocal ? "CAR-LOCAL" : "WORLD",
                        GFrame.MeanReading, GFrame.MeanCosPitch, GFrame.Samples);
            if (GFrame.Frame == FGFrameReport::World)
            {
                std::printf("  NOTE: world-referenced G means there is no fixed 'lateral'\n"
                            "  component — it rotates with the train's heading. The verdict\n"
                            "  below is NOT valid; rotate G by the car quaternion first.\n");
            }
        }
    }

    // ---- 2. Peak lateral comparison, correlated on world position.
    if (bHaveTrack && GFrame.Frame != FGFrameReport::World)
    {
        // Of the two non-gravity axes, lateral is the one that swings hardest
        // through the turns — fore/aft only moves under brakes and launches.
        int Lat = -1;
        double LatSwing = -1.0;
        for (int a = 0; a < 3; ++a)
        {
            if (a == Grav.Axis)
            {
                continue;
            }
            double Peak = 0.0;
            for (const FNL2Telemetry& T : Log)
            {
                const float G[3] = {T.GX, T.GY, T.GZ};
                Peak = std::max(Peak, static_cast<double>(std::fabs(G[a])));
            }
            if (Peak > LatSwing)
            {
                LatSwing = Peak;
                Lat = a;
            }
        }

        const FNL2Telemetry* Worst = nullptr;
        double WorstAbs = -1.0;
        for (const FNL2Telemetry& T : Log)
        {
            const float G[3] = {T.GX, T.GY, T.GZ};
            if (Lat >= 0 && std::fabs(G[Lat]) > WorstAbs)
            {
                WorstAbs = std::fabs(G[Lat]);
                Worst = &T;
            }
        }

        if (Worst != nullptr)
        {
            const float G[3] = {Worst->GX, Worst->GY, Worst->GZ};
            double S = 0.0, Miss = 0.0;
            Index.Nearest(Worst->Position, S, Miss);
            const FTrackFrame F = Index.Track.EvaluateAt(S);
            const FGForces Ours = FeltG(F, static_cast<double>(Worst->Speed));

            std::printf("\npeak lateral event\n");
            std::printf("  NL2:  G%s = %+.4f at %.2f m/s, matched to S = %.2f m "
                        "(position miss %.3f m)\n",
                        AxisName[Lat], static_cast<double>(G[Lat]),
                        static_cast<double>(Worst->Speed), S, Miss);
            std::printf("  ours: lateral %+.4f, vertical %+.4f, bank %.2f deg, radius %.2f m\n",
                        Ours.Lateral, Ours.Vertical, F.Roll * 180.0 / Pi,
                        1.0 / (std::fabs(F.YawCurvature) + std::fabs(F.PitchCurvature) + 1e-12));

            // A sign comparison is only meaningful when BOTH sides are clearly
            // off zero, and only when the two readings describe the same place.
            // Without these guards a stationary train reads "-0.0189 vs +0.0000"
            // and the tool cheerfully declares the conventions agree.
            const double Mine = Ours.Lateral;
            const double Theirs = static_cast<double>(G[Lat]);
            const double MinMeaningful = 0.15; // g
            if (std::fabs(Theirs) < MinMeaningful || std::fabs(Mine) < MinMeaningful)
            {
                std::printf("\n  ==> INCONCLUSIVE: peak lateral is only %.3f g (NL2) / %.3f g "
                            "(ours).\n      Ride a tight unbanked turn at speed — this track's is "
                            "at S = 73 m.\n",
                            std::fabs(Theirs), std::fabs(Mine));
            }
            else if (Miss > 2.0)
            {
                std::printf("\n  ==> INCONCLUSIVE: nearest track sample is %.2f m away, so the two\n"
                            "      readings are not describing the same place. Is this the same\n"
                            "      coaster the CSV was exported from?\n",
                            Miss);
            }
            else
            {
                const bool bAgree = (Mine > 0.0) == (Theirs > 0.0);
                std::printf("\n  ==> lateral sign %s. %s\n", bAgree ? "AGREES" : "IS INVERTED",
                            bAgree ? "No change needed; record the result in PHASE0_FINDINGS.md."
                                   : "Flip the lateral sign at the boundary, not inside FeltG.");
            }
        }
    }

    std::printf("\n%s holds the speed trace for calibrating RollingResistance / DragK.\n",
                OutPath.c_str());
    return 0;
}
