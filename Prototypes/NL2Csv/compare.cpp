// Run OUR model over a track exported from NoLimits 2, and lay the result
// alongside NL2's own recorded ride.
//
// Build & run (Windows):
//   clang++ -std=c++17 -Wall -Wextra -O2 -o compare.exe compare.cpp
//   ./compare.exe ../../NL2/FlatPark_trackdata.csv ../NL2Telemetry/telemetry.csv
//
// The telemetry argument is optional. Without it this reports what the imported
// geometry is and what our physics predicts on it, which is worth having on its
// own — it is the only way to see whether a reference track is even suitable
// before spending a recording session on it.
//
// This is the point of NL2Csv and NL2Telemetry existing. The CSV gives us the
// SAME geometry NL2 ran, so the two simulators are not being compared on
// tracks that merely look alike; the telemetry gives NL2's own speed at each
// point. Everything either agrees or it does not, and the difference is a
// number rather than an impression.

#include "NL2Csv.h"
#include "../TrainPhysics/RideProfile.h"
#include "../TrainPhysics/TrainPhysics.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace
{

struct FRecorded
{
    double Speed = 0.0;
    double Height = 0.0;
    double ArcS = 0.0;
};

// Arc length is rebuilt from the recorded positions rather than trusted from
// the file's own column, so this does not depend on whatever track the
// recorder happened to be matching against at the time.
std::vector<FRecorded> ReadTelemetry(const std::string& Path)
{
    std::vector<FRecorded> Out;
    std::ifstream File(Path);
    if (!File)
    {
        return Out;
    }
    std::string Line;
    bool bHeader = true;
    double PrevX = 0.0, PrevY = 0.0, PrevZ = 0.0;
    bool bFirst = true;
    double S = 0.0;
    while (std::getline(File, Line))
    {
        if (bHeader) { bHeader = false; continue; }
        std::istringstream Row(Line);
        std::vector<std::string> Cell;
        std::string Field;
        while (std::getline(Row, Field, '\t')) { Cell.push_back(Field); }
        if (Cell.size() < 7 || std::atoi(Cell[2].c_str()) == 0) { continue; }

        const double X = std::atof(Cell[4].c_str());
        const double Y = std::atof(Cell[5].c_str());
        const double Z = std::atof(Cell[6].c_str());
        if (!bFirst)
        {
            const double Dx = X - PrevX, Dy = Y - PrevY, Dz = Z - PrevZ;
            S += std::sqrt(Dx * Dx + Dy * Dy + Dz * Dz);
        }
        bFirst = false;
        PrevX = X; PrevY = Y; PrevZ = Z;

        FRecorded R;
        R.Speed = std::atof(Cell[3].c_str());
        R.Height = Z;
        R.ArcS = S;
        Out.push_back(R);
    }
    return Out;
}

// Longest run over which the track climbs or falls by less than a millimetre
// per metre. This is the section that separates rolling resistance from drag:
// with the normal load pinned at 1 g, anything that varies with speed is drag
// and nothing else.
void ReportFlatRun(const FTrack& Track)
{
    const double Total = Track.TotalLength();
    const double Step = 1.0;
    double BestLen = 0.0, BestStart = 0.0, RunStart = 0.0, RunLen = 0.0;

    FTrackFrame Walk = Track.EvaluateAt(0.0);
    double S = 0.0;
    double PrevZ = Walk.Position.Z;
    while (S < Total)
    {
        const double Next = std::min(S + Step, Total);
        Walk = Track.AdvanceFrom(Walk, S, Next);
        const double Grade = std::fabs(Walk.Position.Z - PrevZ) / (Next - S);
        if (Grade < 0.001)
        {
            if (RunLen == 0.0) { RunStart = S; }
            RunLen += Next - S;
            if (RunLen > BestLen) { BestLen = RunLen; BestStart = RunStart; }
        }
        else
        {
            RunLen = 0.0;
        }
        PrevZ = Walk.Position.Z;
        S = Next;
    }
    std::printf("longest flat run: %.0f m from S=%.0f", BestLen, BestStart);
    if (BestLen < 80.0)
    {
        std::printf("   <-- SHORT. Separating drag from rolling resistance wants 100 m+\n");
    }
    else
    {
        std::printf("   (good: this is what decorrelates drag from rolling resistance)\n");
    }
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: compare <track.csv> [telemetry.csv]\n");
        return 1;
    }

    // Parsed and reconstructed as two steps on purpose, so a malformed file and
    // a file that is fine but describes a track we cannot rebuild report
    // differently.
    const FNL2Result Parsed = ReadNL2CsvFile(argv[1]);
    if (!Parsed.Error.empty())
    {
        std::printf("could not read %s: %s\n", argv[1], Parsed.Error.c_str());
        return 1;
    }
    FTrack Track;
    const std::string BuildError = TrackFromSamples(Parsed.Samples, Track);
    if (!BuildError.empty())
    {
        std::printf("could not reconstruct a track from %s:\n  %s\n", argv[1],
                    BuildError.c_str());
        return 1;
    }

    const double Total = Track.TotalLength();
    double Lowest = 1e9, Highest = -1e9;
    {
        FTrackFrame W = Track.EvaluateAt(0.0);
        for (double S = 0.0; S <= Total; S += 1.0)
        {
            W = Track.AdvanceFrom(W, S - 1.0 < 0.0 ? 0.0 : S - 1.0, S);
            Lowest = std::min(Lowest, W.Position.Z);
            Highest = std::max(Highest, W.Position.Z);
        }
    }

    std::printf("%s\n", argv[1]);
    std::printf("  %zu samples -> %zu segments, %.1f m, %.1f m of drop, C2=%s\n",
                Parsed.Samples.size(), Track.NumSegments(), Total, Highest - Lowest,
                Track.IsCurvatureContinuous(1e-6) ? "yes" : "no");
    std::printf("  ");
    ReportFlatRun(Track);

    // A drop of h gives at most sqrt(2gh) at the bottom, before any losses.
    std::printf("  ideal speed from that drop: %.1f km/h\n\n",
                std::sqrt(2.0 * GravityMs2 * (Highest - Lowest)) * 3.6);

    if (argc < 3)
    {
        std::printf("No telemetry given, so nothing to compare against.\n");
        return 0;
    }

    const std::vector<FRecorded> Rec = ReadTelemetry(argv[2]);
    if (Rec.size() < 50)
    {
        std::printf("%s: only %zu usable samples.\n", argv[2], Rec.size());
        return 1;
    }

    double TopRecorded = 0.0, RecLow = 1e9, RecHigh = -1e9;
    for (const FRecorded& R : Rec)
    {
        TopRecorded = std::max(TopRecorded, R.Speed);
        RecLow = std::min(RecLow, R.Height);
        RecHigh = std::max(RecHigh, R.Height);
    }
    std::printf("%s\n", argv[2]);
    std::printf("  %zu on-ride samples, %.1f m travelled, %.1f m of height, top %.1f km/h\n",
                Rec.size(), Rec.back().ArcS, RecHigh - RecLow, TopRecorded * 3.6);

    // A recording that never left the lift is the single most likely way for
    // this to go wrong, and it is not obvious from the file — it just looks
    // like a lot of rows. Chain speed is a few m/s and dead constant.
    if (TopRecorded < 5.0)
    {
        // Distance covered over speed, not height over speed — a lift climbs a
        // slope, so height alone understates how long it takes by the sine of
        // the grade, and the first version of this message did exactly that.
        const double Covered = Rec.back().ArcS;
        const double Elapsed = Covered / std::max(0.5, TopRecorded);
        std::printf("\n  This recording never got past the lift: nothing in it exceeds %.1f km/h,\n"
                    "  which is chain speed. It covered %.0f m in roughly %.0f s and the track is\n"
                    "  %.0f m long, so it needs about %.0fx as long to see the whole ride.\n",
                    TopRecorded * 3.6, Covered, Elapsed, Total,
                    std::max(2.0, Total / std::max(1.0, Covered)));
        return 1;
    }
    if (Rec.back().ArcS < Total * 0.5)
    {
        std::printf("\n  Covers only %.0f m of a %.0f m track — the ride was not finished.\n",
                    Rec.back().ArcS, Total);
    }

    // Our model over the same geometry, from the recorded entry speed at the
    // point the recording reaches full speed. No zones: this compares COASTING,
    // which is the only part where the two simulators are solving the same
    // problem — NL2's chain and brakes are its own and not ours to match.
    std::printf("\nOur model over the same geometry, coasting from the recorded speed:\n");
    std::printf("%10s %12s %12s %10s\n", "S (m)", "NL2 km/h", "ours km/h", "diff");

    std::size_t Start = 0;
    while (Start + 1 < Rec.size() && Rec[Start].Speed < TopRecorded * 0.9) { ++Start; }

    FTrain Train(Track);
    Train.Place(Rec[Start].ArcS, Rec[Start].Speed);
    double SumSq = 0.0;
    int Compared = 0;
    std::size_t Next = Start;
    for (std::size_t i = Start; i < Rec.size(); ++i)
    {
        while (Train.GetDistance() < Rec[i].ArcS && !Train.IsAtEnd())
        {
            Train.Step(1.0 / 480.0);
        }
        const double Diff = Train.GetSpeed() - Rec[i].Speed;
        SumSq += Diff * Diff;
        ++Compared;
        if (i == Next)
        {
            std::printf("%10.1f %12.1f %12.1f %+10.1f\n", Rec[i].ArcS, Rec[i].Speed * 3.6,
                        Train.GetSpeed() * 3.6, Diff * 3.6);
            Next += std::max<std::size_t>(1, (Rec.size() - Start) / 12);
        }
        if (Train.IsAtEnd()) { break; }
    }
    if (Compared > 0)
    {
        std::printf("\nrms speed difference over %d samples: %.3f m/s\n", Compared,
                    std::sqrt(SumSq / Compared));
        std::printf("A positive diff means WE are running fast, which is what too little\n"
                    "total resistance looks like. Defaults are Crr 0.006, DragK 0.00045.\n");
    }
    return 0;
}
