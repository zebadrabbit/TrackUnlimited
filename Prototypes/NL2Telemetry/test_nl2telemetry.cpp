// Unit tests for the Phase 0 NoLimits 2 telemetry protocol prototype.
// Build & run:  clang++ -std=c++17 -Wall -Wextra -o test_nl2telemetry.exe test_nl2telemetry.cpp && ./test_nl2telemetry.exe
//
// Entirely offline: the header has no sockets in it, so every framing and
// field-offset decision is checked against hand-built bytes with NL2 not
// running. The socket lives in record.cpp and is the only part that needs a
// live game.

#include "NL2Telemetry.h"

#include <cassert>
#include <cmath>
#include <cstdio>

static const double Pi = 3.14159265358979323846;

static bool Near(double A, double B, double Tol = 1e-6)
{
    return std::fabs(A - B) <= Tol;
}

// Big-endian float, built by hand so the test does not depend on the same
// helper it is testing.
static void AppendF32(std::vector<unsigned char>& Out, float V)
{
    std::uint32_t Bits = 0;
    std::memcpy(&Bits, &V, sizeof(Bits));
    Out.push_back(static_cast<unsigned char>(Bits >> 24));
    Out.push_back(static_cast<unsigned char>((Bits >> 16) & 0xFF));
    Out.push_back(static_cast<unsigned char>((Bits >> 8) & 0xFF));
    Out.push_back(static_cast<unsigned char>(Bits & 0xFF));
}

static void AppendI32(std::vector<unsigned char>& Out, std::int32_t V)
{
    const std::uint32_t U = static_cast<std::uint32_t>(V);
    Out.push_back(static_cast<unsigned char>(U >> 24));
    Out.push_back(static_cast<unsigned char>((U >> 16) & 0xFF));
    Out.push_back(static_cast<unsigned char>((U >> 8) & 0xFF));
    Out.push_back(static_cast<unsigned char>(U & 0xFF));
}

// A 76-byte telemetry payload with every field set to something distinguishable,
// so a swapped or shifted offset cannot pass.
static std::vector<unsigned char> MakeTelemetryPayload()
{
    std::vector<unsigned char> D;
    AppendI32(D, 0x3);  // state: in play mode + braking, not paused
    AppendI32(D, 1234); // frame
    AppendI32(D, 1);    // view mode: onride
    AppendI32(D, 2);    // coaster
    AppendI32(D, 7);    // style
    AppendI32(D, 3);    // train
    AppendI32(D, 4);    // car
    AppendI32(D, 5);    // seat
    AppendF32(D, 13.5f);                                  // speed
    AppendF32(D, 10.0f); AppendF32(D, 20.0f); AppendF32(D, 30.0f); // position x,y,z (NL2 axes)
    AppendF32(D, 0.1f); AppendF32(D, 0.2f); AppendF32(D, 0.3f); AppendF32(D, 0.4f); // quat
    AppendF32(D, -0.5f); AppendF32(D, 1.5f); AppendF32(D, 0.25f);  // G x,y,z
    assert(D.size() == NL2TelemetryBytes);
    return D;
}

static std::vector<unsigned char> Frame(std::uint16_t Type, std::uint32_t ReqId,
                                        const std::vector<unsigned char>& Data)
{
    std::vector<unsigned char> M;
    M.push_back('N');
    M.push_back(static_cast<unsigned char>(Type >> 8));
    M.push_back(static_cast<unsigned char>(Type & 0xFF));
    M.push_back(static_cast<unsigned char>(ReqId >> 24));
    M.push_back(static_cast<unsigned char>((ReqId >> 16) & 0xFF));
    M.push_back(static_cast<unsigned char>((ReqId >> 8) & 0xFF));
    M.push_back(static_cast<unsigned char>(ReqId & 0xFF));
    M.push_back(static_cast<unsigned char>(Data.size() >> 8));
    M.push_back(static_cast<unsigned char>(Data.size() & 0xFF));
    M.insert(M.end(), Data.begin(), Data.end());
    M.push_back('L');
    return M;
}

static void TestRequestEncoding()
{
    const std::vector<unsigned char> R = EncodeRequest(NL2_GetTelemetry, 0xDEADBEEF);
    assert(R.size() == NL2MinMessageBytes);
    assert(R[0] == 'N');
    assert(R[1] == 0 && R[2] == 5);                 // type 5, big-endian
    assert(R[3] == 0xDE && R[4] == 0xAD && R[5] == 0xBE && R[6] == 0xEF); // request id
    assert(R[7] == 0 && R[8] == 0);                 // no payload
    assert(R[9] == 'L');
    std::printf("  request: 10 bytes, 'N' type=5 id=DEADBEEF size=0 'L'\n");
}

static void TestDecodeRoundTrip()
{
    const std::vector<unsigned char> Wire = Frame(NL2_Telemetry, 42, MakeTelemetryPayload());
    FNL2Message M;
    std::size_t Used = 0;
    assert(DecodeMessage(Wire.data(), Wire.size(), M, Used) == NL2Decode_Ok);
    assert(Used == Wire.size());
    assert(M.Type == NL2_Telemetry);
    assert(M.RequestId == 42);
    assert(M.Data.size() == NL2TelemetryBytes);
    std::printf("  decode: type=%u id=%u payload=%zu bytes\n", M.Type, M.RequestId, M.Data.size());
}

static void TestFieldOffsets()
{
    FNL2Message M;
    std::size_t Used = 0;
    const std::vector<unsigned char> Wire = Frame(NL2_Telemetry, 1, MakeTelemetryPayload());
    assert(DecodeMessage(Wire.data(), Wire.size(), M, Used) == NL2Decode_Ok);

    FNL2Telemetry T;
    assert(ParseTelemetry(M.Data, T));
    assert(T.InPlayMode() && T.Braking() && !T.Paused() && T.Onride());
    assert(T.Frame == 1234 && T.Coaster == 2 && T.StyleId == 7);
    assert(T.Train == 3 && T.Car == 4 && T.Seat == 5);
    assert(Near(T.Speed, 13.5));

    // Position must arrive in PROTOTYPE axes: NL2 (10, 20, 30) with +Y up maps
    // to (10, -30, 20) with +Z up. If this ever reads (10, 20, 30), telemetry
    // and the CSV importer have drifted apart and nothing will correlate.
    assert(Near(T.Position.X, 10.0) && Near(T.Position.Y, -30.0) && Near(T.Position.Z, 20.0));

    assert(Near(T.QuatX, 0.1) && Near(T.QuatY, 0.2) && Near(T.QuatZ, 0.3) && Near(T.QuatW, 0.4));

    // G stays RAW and in file order — not remapped, because the frame is
    // undocumented and assuming it is the whole bug we are hunting.
    assert(Near(T.GX, -0.5) && Near(T.GY, 1.5) && Near(T.GZ, 0.25));
    std::printf("  fields: speed %.2f, pos mapped to +Z-up, G left raw (%.2f, %.2f, %.2f)\n",
                static_cast<double>(T.Speed), static_cast<double>(T.GX),
                static_cast<double>(T.GY), static_cast<double>(T.GZ));
}

static void TestPartialAndMalformed()
{
    const std::vector<unsigned char> Wire = Frame(NL2_Telemetry, 1, MakeTelemetryPayload());
    FNL2Message M;
    std::size_t Used = 0;

    // Every truncation must read as Incomplete, never Malformed. A 76-byte
    // payload routinely straddles TCP segments, and treating a short read as an
    // error would drop a message on most laps.
    for (std::size_t N = 0; N < Wire.size(); ++N)
    {
        assert(DecodeMessage(Wire.data(), N, M, Used) == NL2Decode_Incomplete);
    }
    assert(DecodeMessage(Wire.data(), Wire.size(), M, Used) == NL2Decode_Ok);

    std::vector<unsigned char> BadStart = Wire;
    BadStart[0] = 'X';
    assert(DecodeMessage(BadStart.data(), BadStart.size(), M, Used) == NL2Decode_Malformed);

    std::vector<unsigned char> BadEnd = Wire;
    BadEnd.back() = 'X';
    assert(DecodeMessage(BadEnd.data(), BadEnd.size(), M, Used) == NL2Decode_Malformed);

    // Wrong payload size must be refused rather than read past the end.
    std::vector<unsigned char> Short(NL2TelemetryBytes - 1, 0);
    FNL2Telemetry T;
    assert(!ParseTelemetry(Short, T));
    std::printf("  framing: %zu truncations incomplete, bad magic + short payload refused\n",
                Wire.size());
}

static void TestBackToBackMessages()
{
    // Polling fast means two replies can land in one read. The decoder must
    // peel them one at a time and report exactly how much to consume.
    std::vector<unsigned char> Two = Frame(NL2_Telemetry, 1, MakeTelemetryPayload());
    const std::vector<unsigned char> Second = Frame(NL2_OK, 2, {});
    Two.insert(Two.end(), Second.begin(), Second.end());

    FNL2Message M;
    std::size_t Used = 0;
    assert(DecodeMessage(Two.data(), Two.size(), M, Used) == NL2Decode_Ok);
    assert(M.Type == NL2_Telemetry && Used == Two.size() - Second.size());

    assert(DecodeMessage(Two.data() + Used, Two.size() - Used, M, Used) == NL2Decode_Ok);
    assert(M.Type == NL2_OK && M.Data.empty() && Used == Second.size());
    std::printf("  two messages in one read peeled cleanly\n");
}

static void TestGravityAxisIdentification()
{
    // Synthesise a log where Y carries gravity while the train is slow, and a
    // fast cornering section throws a big number onto X. Only the slow samples
    // should count, or the corner would win the vote.
    std::vector<FNL2Telemetry> Log;
    for (int i = 0; i < 50; ++i)
    {
        FNL2Telemetry T;
        T.Speed = 0.2f;
        T.GX = 0.01f; T.GY = 1.0f; T.GZ = -0.02f;
        Log.push_back(T);
    }
    for (int i = 0; i < 200; ++i)
    {
        FNL2Telemetry T;
        T.Speed = 20.0f;
        T.GX = 3.0f; T.GY = 1.4f; T.GZ = 0.1f;
        Log.push_back(T);
    }

    const FGravityAxisReport R = IdentifyGravityAxis(Log, 1.0);
    assert(R.Axis == 1);
    assert(R.Samples == 50);
    assert(Near(R.Value, 1.0, 1e-5));
    assert(R.OtherMax < 0.05);
    std::printf("  gravity axis: found axis %d = %+.3f from %d slow samples "
                "(next largest %.3f)\n",
                R.Axis, R.Value, R.Samples, R.OtherMax);

    // No slow samples at all must report "undecided", not guess from a corner.
    std::vector<FNL2Telemetry> FastOnly(Log.begin() + 50, Log.end());
    assert(IdentifyGravityAxis(FastOnly, 1.0).Axis == -1);
    std::printf("  gravity axis: undecided when nothing was slow enough\n");
}

static void TestGFrameClassification()
{
    const double Deg = Pi / 180.0;
    const double LiftPitch = 27.6 * Deg; // the real track's lift hill
    const double CosLift = std::cos(LiftPitch);

    // Car-local: the vertical component sags to cos(pitch) on the slope.
    std::vector<std::pair<double, double>> CarLocal;
    for (int i = 0; i < 60; ++i)
    {
        CarLocal.push_back({LiftPitch, CosLift});
    }
    const FGFrameReport A = ClassifyGFrame(CarLocal);
    assert(A.Frame == FGFrameReport::CarLocal);
    assert(A.Samples == 60);

    // World: it stays pinned at 1.0 no matter how steep the track is.
    std::vector<std::pair<double, double>> World;
    for (int i = 0; i < 60; ++i)
    {
        World.push_back({LiftPitch, 1.0});
    }
    assert(ClassifyGFrame(World).Frame == FGFrameReport::World);

    // Flat track cannot distinguish the two — both predict 1.0 — so the answer
    // must be Undecided rather than whichever rounding happened to win.
    std::vector<std::pair<double, double>> Flat;
    for (int i = 0; i < 60; ++i)
    {
        Flat.push_back({1.0 * Deg, 1.0});
    }
    assert(ClassifyGFrame(Flat).Frame == FGFrameReport::Undecided);

    // Too few steep samples is also Undecided, not a guess from three points.
    std::vector<std::pair<double, double>> Sparse(CarLocal.begin(), CarLocal.begin() + 3);
    assert(ClassifyGFrame(Sparse).Frame == FGFrameReport::Undecided);

    std::printf("  G frame: car-local vs world separated at %.1f deg "
                "(%.3f vs 1.000); flat and sparse both undecided\n",
                LiftPitch / Deg, CosLift);
}

int main()
{
    std::printf("NL2 telemetry protocol\n");
    TestRequestEncoding();
    TestDecodeRoundTrip();
    TestFieldOffsets();
    TestPartialAndMalformed();
    TestBackToBackMessages();
    TestGravityAxisIdentification();
    TestGFrameClassification();
    std::printf("all tests passed\n");
    return 0;
}
