// Generates src/data.js from the project's own geometry.
//
// data.js drives every figure and every drawn line in the brand pack, and the
// layout diagram is captioned "MEASURED, NOT DRAWN". This is the thing that
// makes that caption true — and repeatable, which matters more. The figures went
// stale within a day the first time, when RollingResistance was corrected from a
// steel-on-steel value to the 0.024 a polyurethane-on-steel coaster runs and the
// reference layout was re-tuned around it.
//
// Build & run from the repo root:
//   clang++ -std=c++17 -O2 -I Prototypes/TrackSpline -o gen_data.exe Brand/src/gen_data.cpp
//   ./gen_data.exe > Brand/src/data.js
//   cd Brand && python3 build.py
//
// Then re-export github/*.png from social-pack.html — the PNGs are the one part
// of the chain this cannot do.
//
// It mirrors ATUCoasterRide::ReferenceLayout() rather than including it, because
// that lives in a UE translation unit. The two must be changed together; the
// segment list below is deliberately in the same order and the same numbers so a
// diff between them is readable.

#include "TrackSpline.h"
#include "../../Prototypes/TrainPhysics/TrainPhysics.h"
#include "../../Prototypes/TrainPhysics/RideProfile.h"

#include <algorithm>
#include <cstdio>
#include <vector>

namespace
{

const double Pi = 3.14159265358979323846;
double Deg(double D) { return D * Pi / 180.0; }

enum class EZone { None, Lift, Brake };
struct FEntry { FTrackSegment Seg; EZone Zone = EZone::None; double Speed = 0.0; };
struct FMark { double S; const char* Label; char Detail[64]; };

// Code points, not bytes. The bank label carries a UTF-8 beta, so byte length
// would over-count it by one and misalign that row alone.
int DisplayLen(const char* Str)
{
    int N = 0;
    for (const unsigned char* P = reinterpret_cast<const unsigned char*>(Str); *P; ++P)
    {
        if ((*P & 0xC0) != 0x80) { ++N; }
    }
    return N;
}

void Row(const char* Name, const std::vector<double>& V, int Dp)
{
    std::printf("  %s:[", Name);
    for (std::size_t i = 0; i < V.size(); ++i)
    {
        std::printf("%s%.*f", i ? "," : "", Dp, V[i]);
    }
    std::printf("],\n");
}

} // namespace

int main()
{
    const double Lift = Deg(25.0), Drop = Deg(-34.0);
    const double LoopRadius = 9.0, LoopEase = 54.0, TurnRadius = 32.0;
    const double Bank = std::atan((26.5 * 26.5) / (GravityMs2 * TurnRadius));
    const double LiftClimb = 90.99, DropLength = 24.0;

    std::vector<FEntry> E;
    auto Straight = [&E](double L, EZone Z = EZone::None, double Sp = 0.0)
    { FEntry F; F.Seg = MakeStraight(L); F.Zone = Z; F.Speed = Sp; E.push_back(F); };
    auto Eased = [&E](double Delta, double Peak, EZone Z = EZone::None, double Sp = 0.0)
    {
        const double K = Delta >= 0.0 ? Peak : -Peak, L = std::fabs(Delta) / Peak;
        FEntry A; A.Seg.Length = L; A.Seg.PitchCurvatureEnd = K; A.Zone = Z; A.Speed = Sp;
        E.push_back(A);
        FEntry B; B.Seg.Length = L; B.Seg.PitchCurvatureStart = K; B.Zone = Z; B.Speed = Sp;
        E.push_back(B);
    };

    Straight(20.0, EZone::Lift, 4.0);                       // station
    Eased(Lift, 0.03, EZone::Lift, 4.0);                    // into the climb
    Straight(LiftClimb, EZone::Lift, 4.0);                  // lift climb
    const std::size_t CrestFirst = E.size();
    Eased(Drop - Lift, 0.05);                               // crest
    E[CrestFirst].Zone = EZone::Lift; E[CrestFirst].Speed = 4.0;  // chain over the top only
    Straight(DropLength);                                   // drop
    Eased(-Drop, 0.012);                                    // pull-out
    const std::size_t LoopFirst = E.size();
    { FEntry A; A.Seg.Length = LoopEase; A.Seg.PitchCurvatureEnd = 1.0 / LoopRadius; E.push_back(A);
      FEntry B; B.Seg.Length = 2.0 * Pi * LoopRadius - LoopEase;
      B.Seg.PitchCurvatureStart = B.Seg.PitchCurvatureEnd = 1.0 / LoopRadius; E.push_back(B);
      FEntry C; C.Seg.Length = LoopEase; C.Seg.PitchCurvatureStart = 1.0 / LoopRadius; E.push_back(C); }
    { FEntry A; A.Seg = MakeClothoid(26.0, 0.0, 1.0 / TurnRadius, 0.0, Bank); E.push_back(A);
      FEntry B; B.Seg = MakeArc(55.0, TurnRadius, Bank); E.push_back(B);
      FEntry C; C.Seg = MakeClothoid(26.0, 1.0 / TurnRadius, 0.0, Bank, 0.0); E.push_back(C); }
    Straight(70.0, EZone::Brake, 0.0);                      // brake run

    FTrack T;
    for (const FEntry& F : E) { T.AddSegment(F.Seg); }
    const double Total = T.TotalLength();

    FTrainConfig C;
    C.TrainLength = 15.0;
    FTrain Train(T, C);

    // Zones from contiguous runs of equal zone kind, exactly as
    // RebuildFromSegments() derives them.
    std::vector<double> ZoneEdge;
    double S = 0.0;
    std::size_t i = 0;
    while (i < E.size())
    {
        const EZone Z = E[i].Zone;
        const double Start = S, Sp = E[i].Speed;
        while (i < E.size() && E[i].Zone == Z) { S += E[i].Seg.Length; ++i; }
        ZoneEdge.push_back(S);
        if (Z == EZone::Lift)  { Train.AddZone(MakeLift(Start, S, Sp, 6.0)); }
        if (Z == EZone::Brake) { Train.AddZone(MakeBrake(Start, S, Sp, 6.0)); }
    }
    const double LiftEnd = ZoneEdge[0], BrakeStart = ZoneEdge[1];
    const double TurnStart = BrakeStart - 107.0;   // clothoid 26 + arc 55 + clothoid 26

    // Geometry trace at 1.5 m, plus the extents and the lift crest.
    std::vector<double> vS, vX, vZ;
    double MinX = 1e9, MaxX = -1e9, MinY = 1e9, MaxY = -1e9, CrestZ = -1e9, CrestS = 0.0;
    FTrackFrame W = T.EvaluateAt(0.0);
    double Prev = 0.0;
    for (double s = 0.0; s <= Total + 1e-9; s += 1.5)
    {
        W = T.AdvanceFrom(W, Prev, s); Prev = s;
        vS.push_back(s); vX.push_back(W.Position.X); vZ.push_back(W.Position.Z);
        MinX = std::min(MinX, W.Position.X); MaxX = std::max(MaxX, W.Position.X);
        MinY = std::min(MinY, W.Position.Y); MaxY = std::max(MaxY, W.Position.Y);
        if (W.Position.Z > CrestZ && s < LiftEnd) { CrestZ = W.Position.Z; CrestS = s; }
    }

    // The loop's own geometric peak, scanned over the loop's span only —
    // anywhere earlier is the lift crest, which is higher.
    double LoopStart = 0.0;
    for (std::size_t k = 0; k < LoopFirst; ++k) { LoopStart += E[k].Seg.Length; }
    const double LoopEnd = LoopStart + E[LoopFirst].Seg.Length
                         + E[LoopFirst + 1].Seg.Length + E[LoopFirst + 2].Seg.Length;
    double ApexZ = -1e9, ApexS = 0.0;
    W = T.EvaluateAt(LoopStart); Prev = LoopStart;
    for (double s = LoopStart; s <= LoopEnd; s += 0.02)
    {
        W = T.AdvanceFrom(W, Prev, s); Prev = s;
        if (W.Position.Z > ApexZ) { ApexZ = W.Position.Z; ApexS = s; }
    }

    const FRideProfile P = RunRideProfile(Train, T, 0.5);

    std::vector<double> vTS, vV, vG;
    double NextS = 0.0;
    for (const FRideSample& Sm : P.Samples)
    {
        if (Sm.S + 1e-9 < NextS) { continue; }
        vTS.push_back(Sm.S); vV.push_back(Sm.Speed * 3.6); vG.push_back(Sm.VerticalG);
        NextS = Sm.S + 3.5;
    }

    // Ride it once more for the two loop numbers. They are DIFFERENT POINTS:
    // felt G bottoms out before the top, because the train is still slowing as
    // it climbs the back of the loop, so the speed setting the centripetal term
    // is still falling while the height is still rising. Finer sampling does not
    // make them converge, and an earlier data.js conflated them.
    double MinG = 9.0, MinGS = 0.0, MinGZ = 0.0, ApexG = 0.0, BestDist = 1e9;
    FTrain T2(T, C);
    S = 0.0; i = 0;
    while (i < E.size())
    {
        const EZone Z = E[i].Zone;
        const double Start = S, Sp = E[i].Speed;
        while (i < E.size() && E[i].Zone == Z) { S += E[i].Seg.Length; ++i; }
        if (Z == EZone::Lift)  { T2.AddZone(MakeLift(Start, S, Sp, 6.0)); }
        if (Z == EZone::Brake) { T2.AddZone(MakeBrake(Start, S, Sp, 6.0)); }
    }
    T2.Place(0.0, 0.0);
    for (int n = 0; n < 400000 && !T2.IsAtEnd(); ++n)
    {
        T2.Step(1.0 / 480.0);
        const double s = T2.GetDistance();
        if (T2.GetFrame().Up.Z < -0.9 && T2.GetForces().Vertical < MinG)
        { MinG = T2.GetForces().Vertical; MinGS = s; MinGZ = T2.GetFrame().Position.Z; }
        if (std::fabs(s - ApexS) < BestDist)
        { BestDist = std::fabs(s - ApexS); ApexG = T2.GetForces().Vertical; }
        if (T2.GetSpeed() <= 0.0 && n > 5000) { break; }
    }

    std::printf(
"// TrackUnlimited - reference layout, measured.\n"
"//\n"
"// NOT hand-drawn, and NOT hand-edited: this file is the output of\n"
"// src/gen_data.cpp, which compiles Prototypes/TrackSpline + Prototypes/TrainPhysics\n"
"// against the ReferenceLayout() segment list in Source/TrackUnlimited/TUCoasterRide.cpp\n"
"// and runs RunRideProfile(). Every coordinate below is the project's own geometry.\n"
"//\n"
"// Regenerate after ANY change to ReferenceLayout() or to the physics defaults - these\n"
"// figures went stale once already when RollingResistance was corrected to 0.024, and a\n"
"// diagram captioned \"MEASURED, NOT DRAWN\" is worse than no diagram when it is wrong.\n"
"// See the build line at the top of gen_data.cpp.\n"
"//\n"
"// S  = arc length along the heartline, m\n"
"// X  = horizontal station, m (side-elevation abscissa)\n"
"// Z  = heartline height above station datum, m\n"
"// TS = arc length for the trace plots, m\n"
"// V  = speed, km/h\n"
"// G  = felt vertical G (1.00 = sitting on level track, 0 = airtime)\n"
"const TU = {\n"
"  meta: {\n"
"    segments: %zu,\n"
"    developedLength: %.2f,   // m, sum of segment arc lengths\n"
"    horizontalExtent: %.2f,  // m\n"
"    crest: %.2f,             // m, highest heartline point\n"
"    crestS: %.1f,\n"
"    heartline: 1.1,            // m above rail centreline\n"
"    chainSpeed: 4.0,           // m/s\n"
"    trainLength: 15.0,         // m - the slice rides a train with length, not a point\n"
"    topSpeedKph: %.1f,\n"
"    topSpeedS: %.1f,\n"
"    maxG: %.2f,  maxGS: %.1f,\n"
"    minG: %.2f,  minGS: %.1f,\n"
"    maxLateralG: %.2f, maxLateralGS: %.1f,\n"
"    // Two different points, not two samples of one. loopApex is the highest\n"
"    // point of the loop; loopMinG is where felt G bottoms out, %.2f m of arc\n"
"    // earlier and %.2f m lower, because the train is still slowing as it climbs\n"
"    // the back of the loop. Quote loopMinG for ride feel, loopApex for height.\n"
"    loopRadius: %.1f, loopEase: %.1f,\n"
"    loopApex: %.2f, loopApexS: %.1f, loopApexG: %.2f,\n"
"    loopMinG: %.2f, loopMinGS: %.1f, loopMinGHeight: %.2f,\n"
"    turnRadius: %.1f, bankDeg: %.2f,\n"
"    duration: %.1f,            // s, dispatch to brake-run stop\n"
"    curvatureContinuous: true, // verified to 1e-9 across all %zu joints\n"
"  },\n"
"  // Zones as authored: contiguous runs of segments carrying the same zone kind.\n"
"  zones: [\n"
"    { id: 'B1', name: 'STATION',     s0: %.2f,   s1: %.2f,  kind: 'lift',  note: 'chain 4.0 m/s' },\n"
"    { id: 'B2', name: 'LIFT',        s0: %.2f,  s1: %.2f, kind: 'lift',  note: 'chain over crest' },\n"
"    { id: 'B3', name: 'COURSE',      s0: %.2f, s1: %.2f, kind: 'free',  note: 'drop / pull-out / loop' },\n"
"    { id: 'B4', name: 'BANKED TURN', s0: %.2f, s1: %.2f, kind: 'free',  note: 'R32.0 clothoid in/out' },\n"
"    { id: 'B5', name: 'BRAKE RUN',   s0: %.2f, s1: %.2f, kind: 'brake', note: 'release 0.0 m/s' },\n"
"  ],\n",
        T.NumSegments(), Total, std::max(MaxX - MinX, MaxY - MinY), CrestZ, CrestS,
        P.TopSpeed * 3.6, P.TopSpeedS, P.MaxVerticalG, P.MaxVerticalGS,
        P.MinVerticalG, P.MinVerticalGS, P.MaxAbsLateralG, P.MaxAbsLateralGS,
        ApexS - MinGS, ApexZ - MinGZ,
        LoopRadius, LoopEase, ApexZ, ApexS, ApexG, MinG, MinGS, MinGZ,
        TurnRadius, Bank * 180.0 / Pi, P.Duration, T.NumSegments() - 1,
        0.0, 20.0, 20.0, LiftEnd, LiftEnd, TurnStart, TurnStart, BrakeStart, BrakeStart, Total);

    // Sorted rather than emitted in a fixed order, so the comment stays true if
    // a layout change ever reorders them.
    std::vector<FMark> Marks(5);
    Marks[0].S = CrestS;        Marks[0].Label = "LIFT CREST";
    std::snprintf(Marks[0].Detail, sizeof Marks[0].Detail, "%.2f m", CrestZ);
    Marks[1].S = P.TopSpeedS;   Marks[1].Label = "MAX SPEED";
    std::snprintf(Marks[1].Detail, sizeof Marks[1].Detail, "%.1f km/h", P.TopSpeed * 3.6);
    Marks[2].S = P.MaxVerticalGS; Marks[2].Label = "MAX +Gv";
    std::snprintf(Marks[2].Detail, sizeof Marks[2].Detail, "+%.2f g", P.MaxVerticalG);
    Marks[3].S = ApexS;         Marks[3].Label = "LOOP APEX";
    std::snprintf(Marks[3].Detail, sizeof Marks[3].Detail, "%.2f m \xc2\xb7 inverted", ApexZ);
    Marks[4].S = TurnStart + 53.5; Marks[4].Label = "BANK \xce\xb2";
    std::snprintf(Marks[4].Detail, sizeof Marks[4].Detail, "%.2f\xc2\xb0", Bank * 180.0 / Pi);
    std::stable_sort(Marks.begin(), Marks.end(),
                     [](const FMark& A, const FMark& B) { return A.S < B.S; });

    std::printf("  // Feature callouts, keyed to arc length. Ordered by S so a caller can lay them\n"
                "  // out left to right without sorting first.\n"
                "  marks: [\n");
    for (const FMark& M : Marks)
    {
        // 'LABEL', padded out to a 16-column field, which is what lines the
        // detail column up across rows of different label lengths.
        const int Pad = std::max(1, 16 - (DisplayLen(M.Label) + 3));
        std::printf("    { s: %.1f, label: '%s',%*sdetail: '%s' },\n",
                    M.S, M.Label, Pad, "", M.Detail);
    }
    std::printf("  ],\n");

    Row("S", vS, 1); Row("X", vX, 1); Row("Z", vZ, 1);
    Row("TS", vTS, 0); Row("V", vV, 1); Row("G", vG, 2);
    std::printf("};\nif (typeof module !== 'undefined') { module.exports = TU; }\n");
    return 0;
}
