// Unit tests for the Phase 0 NoLimits 2 CSV import/export prototype.
// Build & run:  clang++ -std=c++17 -Wall -Wextra -o test_nl2csv.exe test_nl2csv.cpp && ./test_nl2csv.exe
//
// The whole suite is self-contained: it authors a track, writes it as NL2 CSV,
// reads it back, and asserts the reconstruction survives. No NoLimits 2 install
// is needed to verify the reader, which is the entire reason the writer exists.

#include "NL2Csv.h"
#include "../TrainPhysics/TrainPhysics.h"

#include <cassert>
#include <cstdio>

static const double Pi = 3.14159265358979323846;

static bool Near(double A, double B, double Tol = 1e-3)
{
    return std::fabs(A - B) <= Tol;
}

static double MaxAbs(double A, double B)
{
    return A > B ? A : B;
}

// ------------------------------------------------------------------ fixtures

static FTrackSegment PitchRamp(double L, double K0, double K1)
{
    FTrackSegment S;
    S.Length = L;
    S.PitchCurvatureStart = K0;
    S.PitchCurvatureEnd = K1;
    return S;
}

static FTrackSegment YawRamp(double L, double K0, double K1, double R0, double R1)
{
    FTrackSegment S;
    S.Length = L;
    S.YawCurvatureStart = K0;
    S.YawCurvatureEnd = K1;
    S.RollStart = R0;
    S.RollEnd = R1;
    return S;
}

// Station, eased 30-degree lift, crest, drop, pull-out, R=8 vertical loop, two
// banked turns, brake run — curvature-continuous throughout, so a faithful
// reconstruction must also pass IsCurvatureContinuous.
//
// It is also a rideable layout, not just a geometry fixture: +41.7 m at the
// crest, level again after the pull-out, a loop that peaks at +13.6 m, and a
// finish within 2 m of the station height. TestPhysicsAgreesOnBothTracks needs
// a train that actually completes a circuit, and an underpowered lift or a
// valley stall would only prove the tick cap works.
static FTrack MakeLayout()
{
    FTrack T;
    const double Bank = 45.0 * Pi / 180.0;

    // Entry ramp + arc + exit ramp turn a total of 0.75 rad, so the arc that
    // closes a full 2*pi loop is shorter than the naive 2*pi*R by exactly that.
    // Getting this wrong leaves the track climbing at 43 degrees afterwards.
    const double LoopArc = 8.0 * (2.0 * Pi - 0.75);

    T.AddSegment(MakeStraight(30.0));            // station
    T.AddSegment(PitchRamp(15.0, 0.0, 1.0 / 40.0));
    T.AddSegment(PitchRamp(6.0, 1.0 / 40.0, 1.0 / 40.0)); // pitch up to 30 degrees
    T.AddSegment(PitchRamp(15.0, 1.0 / 40.0, 0.0));
    T.AddSegment(MakeStraight(55.0));            // lift hill
    T.AddSegment(PitchRamp(12.0, 0.0, -1.0 / 25.0));
    T.AddSegment(PitchRamp(18.5, -1.0 / 25.0, -1.0 / 25.0)); // crest, over to -40
    T.AddSegment(PitchRamp(12.0, -1.0 / 25.0, 0.0));
    T.AddSegment(MakeStraight(38.0));            // drop
    T.AddSegment(PitchRamp(14.0, 0.0, 1.0 / 30.0));
    T.AddSegment(PitchRamp(7.0, 1.0 / 30.0, 1.0 / 30.0)); // pull-out, back to level
    T.AddSegment(PitchRamp(14.0, 1.0 / 30.0, 0.0));
    T.AddSegment(MakeStraight(20.0));
    T.AddSegment(PitchRamp(6.0, 0.0, 1.0 / 8.0));
    T.AddSegment(PitchRamp(LoopArc, 1.0 / 8.0, 1.0 / 8.0)); // R=8 vertical loop
    T.AddSegment(PitchRamp(6.0, 1.0 / 8.0, 0.0));
    T.AddSegment(MakeStraight(18.0));
    T.AddSegment(YawRamp(20.0, 0.0, 1.0 / 25.0, 0.0, Bank));      // banked left
    T.AddSegment(YawRamp(35.0, 1.0 / 25.0, 1.0 / 25.0, Bank, Bank));
    T.AddSegment(YawRamp(20.0, 1.0 / 25.0, 0.0, Bank, 0.0));
    T.AddSegment(MakeStraight(15.0));
    T.AddSegment(YawRamp(18.0, 0.0, -1.0 / 20.0, 0.0, -Bank));    // banked right
    T.AddSegment(YawRamp(28.0, -1.0 / 20.0, -1.0 / 20.0, -Bank, -Bank));
    T.AddSegment(YawRamp(18.0, -1.0 / 20.0, 0.0, -Bank, 0.0));
    T.AddSegment(MakeStraight(45.0));            // brake run
    return T;
}

// Author -> write -> read -> rebuild. Any failure along the way aborts loudly
// with the reader's own message rather than a bare assert.
static FTrack RoundTrip(const FTrack& Original, double Spacing)
{
    const std::string Csv = WriteNL2Csv(Original, Spacing);
    FTrack Rebuilt;
    const std::string Error = TrackFromNL2Csv(Csv, Rebuilt);
    if (!Error.empty())
    {
        // stderr, because abort() will not flush a buffered stdout and a silent
        // assert line number is a miserable way to debug a reader.
        std::fprintf(stderr, "  round-trip failed: %s\n", Error.c_str());
    }
    assert(Error.empty());
    return Rebuilt;
}

// Worst position error, and worst AND rms felt-G error between two tracks,
// sampled at a matched FRACTION of arc length so a small total-length
// difference does not masquerade as a geometry error.
//
// Rms is reported alongside max on purpose: the G error is sharply localised at
// curvature-slope kinks, so a max alone reads as a much worse fit than it is.
static void CompareTracks(const FTrack& A, const FTrack& B, int Samples, double SpeedMs,
                          double& OutMaxPos, double& OutMaxG, double& OutRmsG)
{
    OutMaxPos = 0.0;
    OutMaxG = 0.0;
    double SumSq = 0.0;
    for (int i = 0; i <= Samples; ++i)
    {
        const double U = static_cast<double>(i) / Samples;
        const FTrackFrame FA = A.EvaluateAt(U * A.TotalLength());
        const FTrackFrame FB = B.EvaluateAt(U * B.TotalLength());
        OutMaxPos = MaxAbs(OutMaxPos, Length(FA.Position - FB.Position));

        const FGForces GA = FeltG(FA, SpeedMs);
        const FGForces GB = FeltG(FB, SpeedMs);
        const double D = MaxAbs(std::fabs(GA.Lateral - GB.Lateral),
                                std::fabs(GA.Vertical - GB.Vertical));
        OutMaxG = MaxAbs(OutMaxG, D);
        SumSq += D * D;
    }
    OutRmsG = std::sqrt(SumSq / (Samples + 1));
}

// -------------------------------------------------------------- the axis map

static void TestAxisMapIsARotationNotAFlip()
{
    // Round-trips exactly, and — the part that actually matters — preserves
    // cross products. If this ever fails, the whole track is mirrored while
    // still looking self-consistent, which is the failure mode the findings
    // call out for the UE5 port.
    const FVec3 A{0.3, -0.7, 0.64807407};
    const FVec3 B{0.8, 0.6, 0.0};
    const FVec3 Back = ToNL2(FromNL2(A));
    assert(Near(Back.X, A.X, 1e-15) && Near(Back.Y, A.Y, 1e-15) && Near(Back.Z, A.Z, 1e-15));

    const FVec3 CrossThenMap = FromNL2(Cross(A, B));
    const FVec3 MapThenCross = Cross(FromNL2(A), FromNL2(B));
    assert(Length(CrossThenMap - MapThenCross) < 1e-15);

    // Vertical really is vertical in both directions.
    const FVec3 NL2Up = ToNL2({0.0, 0.0, 1.0});
    assert(Near(NL2Up.X, 0.0, 1e-15) && Near(NL2Up.Y, 1.0, 1e-15) && Near(NL2Up.Z, 0.0, 1e-15));
    std::printf("  axis map: rotation, cross-product preserving\n");
}

// ------------------------------------------------------------- round trips

static void TestStraightRoundTrips()
{
    FTrack T;
    T.AddSegment(MakeStraight(50.0));
    const FTrack R = RoundTrip(T, 0.25);

    assert(Near(R.TotalLength(), 50.0, 1e-9));
    double Pos = 0.0, G = 0.0, RmsG = 0.0;
    CompareTracks(T, R, 50, 20.0, Pos, G, RmsG);
    assert(Pos < 1e-9);
    assert(G < 1e-9);
    std::printf("  straight: length %.9f m, max pos err %.3e m\n", R.TotalLength(), Pos);
}

static void TestRolledStraightRoundTrips()
{
    // Constant bank on a straight: the path frame never turns, so this isolates
    // roll recovery from curvature recovery entirely.
    FTrack T;
    T.AddSegment(MakeStraight(40.0, 0.6));
    const FTrack R = RoundTrip(T, 0.25);

    for (int i = 0; i <= 20; ++i)
    {
        const double S = 40.0 * i / 20.0;
        assert(Near(R.EvaluateAt(S).Roll, 0.6, 1e-9));
    }
    std::printf("  rolled straight: roll recovered to 1e-9 rad\n");
}

static void TestBankedArcPinsTheSigns()
{
    // The sign test. A LEFT turn (+curvature) banked INTO the turn (+roll) must
    // come back as +curvature and +roll, not as their mirror. Getting either
    // backwards produces a track that is still perfectly self-consistent, so
    // nothing downstream would notice — this assert is the only thing between
    // that and a silently mirrored import.
    const double Bank = 0.5;
    FTrack T;
    T.AddSegment(MakeArc(60.0, 25.0, Bank));
    const FTrack R = RoundTrip(T, 0.2);

    const FTrackFrame F = R.EvaluateAt(30.0);
    assert(F.YawCurvature > 0.0);
    assert(Near(F.YawCurvature, 1.0 / 25.0, 1e-6));
    assert(F.Roll > 0.0);
    assert(Near(F.Roll, Bank, 1e-6));
    // Banking into a left turn tilts the rider's up toward the turn centre,
    // which is +Lateral (their left).
    assert(Dot(F.Up, F.PathLateral) > 0.0);

    double Pos = 0.0, G = 0.0, RmsG = 0.0;
    CompareTracks(T, R, 60, 18.0, Pos, G, RmsG);
    // Constant curvature and constant roll have no kink for the fit to round
    // off, so this is the tightest the reconstruction ever gets.
    assert(Pos < 1e-6);
    assert(G < 1e-6);
    std::printf("  banked left arc: k=+%.6f roll=+%.6f, max pos err %.3e m, max G err %.3e\n",
                F.YawCurvature, F.Roll, Pos, G);
}

static void TestVerticalLoopRoundTrips()
{
    // An inversion is where a frame error hides best: the findings record a
    // mutant that put the apex at z = -16 with every G reading numerically
    // identical. So assert the apex HEIGHT and its sign, not just the G.
    const double R = 8.0;
    FTrack T;
    T.AddSegment(PitchRamp(2.0 * Pi * R, 1.0 / R, 1.0 / R));
    const FTrack Re = RoundTrip(T, 0.2);

    const FTrackFrame Apex = Re.EvaluateAt(Pi * R);
    assert(Apex.Position.Z > 0.0);
    assert(Near(Apex.Position.Z, 2.0 * R, 1e-3));
    assert(Near(Apex.Up.Z, -1.0, 1e-3)); // inverted at the top

    // Closure: the loop must come back to where it started.
    const FTrackFrame End = Re.EvaluateAt(Re.TotalLength());
    assert(Length(End.Position) < 1e-3);

    double Pos = 0.0, G = 0.0, RmsG = 0.0;
    CompareTracks(T, Re, 80, 20.0, Pos, G, RmsG);
    assert(Pos < 1e-4);
    assert(G < 1e-4);
    std::printf("  R=8 loop: apex z %+.6f m, closure %.3e m, max pos err %.3e m, max G err %.3e\n",
                Apex.Position.Z, Length(End.Position), Pos, G);
}

static void TestBarrelRollUnwrapsThroughPi()
{
    // A full 2*pi roll passes through the atan2 branch cut twice. Without the
    // unwrap the recovered roll snaps sign mid-inversion and the rider frame
    // flips inside out.
    FTrack T;
    T.AddSegment(MakeClothoid(60.0, 0.0, 0.0, 0.0, 2.0 * Pi));
    const FTrack R = RoundTrip(T, 0.2);

    double Worst = 0.0;
    for (int i = 0; i <= 60; ++i)
    {
        const double U = static_cast<double>(i) / 60.0;
        const double Expected = 2.0 * Pi * U;
        Worst = MaxAbs(Worst, std::fabs(R.EvaluateAt(U * R.TotalLength()).Roll - Expected));
    }
    assert(Worst < 1e-6);
    std::printf("  2pi barrel roll: worst roll err %.3e rad (unwrapped cleanly)\n", Worst);
}

static void TestClosedCircuitRotatesToALevelStart()
{
    // NL2 opens a circuit export wherever its spline starts, not at the station
    // — the first real export tried here began 22 degrees into a drop, which
    // FTrack cannot represent without tilting the whole track against gravity.
    // A full vertical loop is the smallest closed circuit that has both pitched
    // and level samples, so it stands in for that file here.
    const double R = 8.0;
    FTrack T;
    T.AddSegment(PitchRamp(2.0 * Pi * R, 1.0 / R, 1.0 / R));

    FNL2Result Parsed = ParseNL2Csv(WriteNL2Csv(T, 0.2));
    assert(Parsed.Ok());

    // Re-open the file a quarter of the way up the loop: steeply pitched.
    const std::size_t N = Parsed.Samples.size();
    std::vector<FNL2Sample> Shifted;
    for (std::size_t i = 0; i < N - 1; ++i)
    {
        Shifted.push_back(Parsed.Samples[(i + N / 8) % (N - 1)]);
    }
    assert(std::fabs(Shifted[0].Front.Z) > 0.5); // genuinely steep, not a token tilt
    assert(!ValidateSamples(Shifted).empty());   // and rejected as handed in

    // Same data through the importer must succeed, because it is a closed
    // circuit with a level sample available.
    FTrack Rebuilt;
    const std::string Error = TrackFromSamples(Shifted, Rebuilt);
    if (!Error.empty())
    {
        std::fprintf(stderr, "  rotate-to-level failed: %s\n", Error.c_str());
    }
    assert(Error.empty());

    // Still the same loop: right length, starts level, and rises to 2R.
    assert(Near(Rebuilt.TotalLength(), T.TotalLength(), 1e-3));
    assert(Near(Rebuilt.EvaluateAt(0.0).Tangent.Z, 0.0, 1e-3));
    double ZMin = 1e9, ZMax = -1e9;
    for (int i = 0; i <= 200; ++i)
    {
        const double Z = Rebuilt.EvaluateAt(Rebuilt.TotalLength() * i / 200.0).Position.Z;
        ZMin = ZMin < Z ? ZMin : Z;
        ZMax = ZMax > Z ? ZMax : Z;
    }
    assert(Near(ZMax - ZMin, 2.0 * R, 1e-2));
    std::printf("  closed circuit: reopened at %.1f deg pitch, rotated to level, "
                "height %.4f m vs 2R %.1f m\n",
                std::asin(Shifted[0].Front.Z) * 180.0 / Pi, ZMax - ZMin, 2.0 * R);

    // An OPEN track with a pitched start has nowhere to rotate to, and must
    // still be refused rather than silently tilted.
    FTrack Ramp;
    Ramp.AddSegment(PitchRamp(30.0, 1.0 / 20.0, 1.0 / 20.0));
    FNL2Result Open = ParseNL2Csv(WriteNL2Csv(Ramp, 0.2));
    assert(Open.Ok());
    std::vector<FNL2Sample> OpenPitched(Open.Samples.begin() + 40, Open.Samples.end());
    FTrack Never;
    assert(!TrackFromSamples(OpenPitched, Never).empty());
    assert(Never.NumSegments() == 0);
    std::printf("  open track with pitched start still refused\n");
}

static void TestLayoutRoundTrips()
{
    const FTrack T = MakeLayout();
    assert(T.IsCurvatureContinuous());

    const FTrack R = RoundTrip(T, 0.25);
    // A faithful reconstruction of continuous data must itself be continuous.
    assert(R.IsCurvatureContinuous(1e-9));

    const double LengthErr = std::fabs(R.TotalLength() - T.TotalLength());
    assert(LengthErr < 1e-3);

    double Pos = 0.0, G = 0.0, RmsG = 0.0;
    CompareTracks(T, R, 400, 20.0, Pos, G, RmsG);
    assert(Pos < 5e-3);
    // The G error is NOT uniform. It lives at curvature-slope kinks — the
    // sharpest being the entry to the R=8 loop, where the ramp's slope drops to
    // zero across one joint — and a fit to samples cannot resolve a kink that
    // falls between two of them. So the max stays loose (measured 7.5e-03, and
    // it moves with how the sample points happen to straddle the kink) while
    // the rms is tight (measured 6.8e-04): at 0.25 m under 1% of sampled points
    // exceed 0.005 G.
    assert(G < 0.05);
    assert(RmsG < 5e-3);
    std::printf("  layout: %.3f m -> %.3f m in %zu segments, len err %.3e m, "
                "max pos err %.3e m, max G err %.3e, rms G err %.3e\n",
                T.TotalLength(), R.TotalLength(), R.NumSegments(), LengthErr, Pos, G, RmsG);
}

static void TestSpacingControlsAccuracy()
{
    // The reconstruction is a fit to sampled data, so denser samples must give
    // a closer fit — on BOTH position and G. This convergence is the assert
    // that separates "discretisation" from "bug": a sign or frame error would
    // not shrink with spacing.
    const FTrack T = MakeLayout();
    double CoarsePos = 0.0, CoarseG = 0.0, CoarseRms = 0.0;
    double FinePos = 0.0, FineG = 0.0, FineRms = 0.0;
    CompareTracks(T, RoundTrip(T, 1.0), 400, 20.0, CoarsePos, CoarseG, CoarseRms);
    CompareTracks(T, RoundTrip(T, 0.1), 400, 20.0, FinePos, FineG, FineRms);
    assert(FinePos < CoarsePos);
    assert(FineG < CoarseG);
    assert(FineRms < CoarseRms);
    std::printf("  spacing 1.0 m -> pos %.3e m / rms G %.3e;  0.1 m -> pos %.3e m / rms G %.3e\n",
                CoarsePos, CoarseRms, FinePos, FineRms);
}

// --------------------------------------------------------------- the payload

static void TestPhysicsAgreesOnBothTracks()
{
    // The point of the whole exercise: a train driven over the reconstruction
    // must behave like a train driven over the original. Speed is the strictest
    // check available, because it integrates every height the train visited.
    const FTrack T = MakeLayout();
    const FTrack R = RoundTrip(T, 0.25);

    FTrain A(T);
    FTrain B(R);
    // Grip has to out-pull gravity on the 30-degree lift (g*sin(30) = 4.9), or
    // the train stalls and the only thing this test proves is the tick cap.
    assert(A.AddZone(MakeLift(0.0, 145.0, 6.0, 12.0)));
    assert(B.AddZone(MakeLift(0.0, 145.0, 6.0, 12.0)));
    A.Place(0.0, 2.0);
    B.Place(0.0, 2.0);

    const double Dt = 1.0 / 60.0;
    double WorstSpeed = 0.0;
    int Ticks = 0;
    while (!A.IsAtEnd() && !B.IsAtEnd() && Ticks < 100000)
    {
        A.Step(Dt);
        B.Step(Dt);
        WorstSpeed = MaxAbs(WorstSpeed, std::fabs(A.GetSpeed() - B.GetSpeed()));
        ++Ticks;
    }
    assert(Ticks < 100000); // a stall here is a regression, not a hang
    assert(A.IsAtEnd() && B.IsAtEnd());
    assert(WorstSpeed < 0.05);
    std::printf("  physics: %d ticks, worst speed divergence %.3e m/s, final %.4f vs %.4f m/s\n",
                Ticks, WorstSpeed, A.GetSpeed(), B.GetSpeed());
}

// ------------------------------------------------------------ trust boundary

// A minimal valid two-row file in NL2 axes: forward +X, up +Y, left -Z.
static const char* ValidCsv()
{
    return "No.\tPosX\tPosY\tPosZ\tFrontX\tFrontY\tFrontZ\tLeftX\tLeftY\tLeftZ\tUpX\tUpY\tUpZ\n"
           "0\t0\t0\t0\t1\t0\t0\t0\t0\t-1\t0\t1\t0\n"
           "1\t1\t0\t0\t1\t0\t0\t0\t0\t-1\t0\t1\t0\n";
}

static void ExpectRejected(const char* Label, const std::string& Csv)
{
    FTrack Track;
    const std::string Error = TrackFromNL2Csv(Csv, Track);
    assert(!Error.empty());
    assert(Track.NumSegments() == 0); // failure must not half-build a track
    std::printf("  rejected %-18s -> %s\n", Label, Error.c_str());
}

static void TestMalformedInputIsRejected()
{
    // Sanity: the fixture the negatives are derived from is actually accepted.
    FTrack Good;
    assert(TrackFromNL2Csv(ValidCsv(), Good).empty());
    assert(Good.NumSegments() == 1);
    assert(Near(Good.TotalLength(), 1.0, 1e-12));

    ExpectRejected("empty", "");
    ExpectRejected("header only",
                   "No.\tPosX\tPosY\tPosZ\tFrontX\tFrontY\tFrontZ\tLeftX\tLeftY\tLeftZ\tUpX\tUpY"
                   "\tUpZ\n");
    ExpectRejected("one row", "0\t0\t0\t0\t1\t0\t0\t0\t0\t-1\t0\t1\t0\n");
    ExpectRejected("short row",
                   "0\t0\t0\t0\t1\t0\t0\t0\t0\t-1\t0\t1\t0\n"
                   "1\t1\t0\t0\t1\t0\t0\t0\t0\t-1\t0\n");
    ExpectRejected("extra column",
                   "0\t0\t0\t0\t1\t0\t0\t0\t0\t-1\t0\t1\t0\n"
                   "1\t1\t0\t0\t1\t0\t0\t0\t0\t-1\t0\t1\t0\t9\n");
    ExpectRejected("non-numeric",
                   "0\t0\t0\t0\t1\t0\t0\t0\t0\t-1\t0\t1\t0\n"
                   "1\t1\tfoo\t0\t1\t0\t0\t0\t0\t-1\t0\t1\t0\n");
    // strtod parses "nan" and "inf" happily; they must not reach the geometry.
    ExpectRejected("nan field",
                   "0\t0\t0\t0\t1\t0\t0\t0\t0\t-1\t0\t1\t0\n"
                   "1\t1\tnan\t0\t1\t0\t0\t0\t0\t-1\t0\t1\t0\n");
    ExpectRejected("non-unit basis",
                   "0\t0\t0\t0\t1\t0\t0\t0\t0\t-0.5\t0\t1\t0\n"
                   "1\t1\t0\t0\t1\t0\t0\t0\t0\t-0.5\t0\t1\t0\n");
    // Left flipped: still orthonormal, but Front x Left = -Up. This is the
    // mirrored-track case, and it must not be silently absorbed.
    ExpectRejected("wrong handedness",
                   "0\t0\t0\t0\t1\t0\t0\t0\t0\t1\t0\t1\t0\n"
                   "1\t1\t0\t0\t1\t0\t0\t0\t0\t1\t0\t1\t0\n");
    // Starts pitched 45 degrees up. Orthonormal and right-handed, but
    // reconstructing it would rotate the start flat and tilt the entire track
    // relative to gravity with no visible symptom.
    ExpectRejected("start not level",
                   "0\t0\t0\t0\t0.70710678\t0.70710678\t0\t0\t0\t-1\t-0.70710678\t0.70710678\t0\n"
                   "1\t0.70710678\t0.70710678\t0\t0.70710678\t0.70710678\t0\t0\t0\t-1"
                   "\t-0.70710678\t0.70710678\t0\n");
    ExpectRejected("coincident rows",
                   "0\t0\t0\t0\t1\t0\t0\t0\t0\t-1\t0\t1\t0\n"
                   "1\t0\t0\t0\t1\t0\t0\t0\t0\t-1\t0\t1\t0\n");
}

static void TestHeaderIsOptionalAndBlankLinesIgnored()
{
    // Real exports have a header; a hand-trimmed file might not. Both parse,
    // and neither is allowed to change the geometry.
    FTrack WithHeader, Without;
    assert(TrackFromNL2Csv(ValidCsv(), WithHeader).empty());
    assert(TrackFromNL2Csv("\n0\t0\t0\t0\t1\t0\t0\t0\t0\t-1\t0\t1\t0\n\n"
                           "1\t1\t0\t0\t1\t0\t0\t0\t0\t-1\t0\t1\t0\n\n",
                           Without)
               .empty());
    assert(WithHeader.NumSegments() == Without.NumSegments());
    assert(Near(WithHeader.TotalLength(), Without.TotalLength(), 1e-15));
    std::printf("  header optional, blank lines ignored\n");
}

static void TestWrittenFileIsReadableAsNL2()
{
    // Shape check on what NL2 will actually be handed: the documented header,
    // 13 tab-separated columns, and Y carrying height.
    FTrack T;
    T.AddSegment(PitchRamp(20.0, 0.0, 1.0 / 30.0));
    const std::string Csv = WriteNL2Csv(T, 5.0);

    assert(Csv.compare(0, 4, "No.\t") == 0);
    const std::size_t FirstNewline = Csv.find('\n');
    const std::size_t SecondNewline = Csv.find('\n', FirstNewline + 1);
    const std::string Row = Csv.substr(FirstNewline + 1, SecondNewline - FirstNewline - 1);
    int Tabs = 0;
    for (const char C : Row)
    {
        Tabs += (C == '\t');
    }
    assert(Tabs == 12);

    // Climbing track: NL2's up axis (Y) must be the one that increases.
    const FNL2Result Parsed = ParseNL2Csv(Csv);
    assert(Parsed.Ok());
    assert(Parsed.Samples.back().Position.Z > Parsed.Samples.front().Position.Z);
    std::printf("  written file: header + 13 tab-separated columns, %zu rows\n",
                Parsed.Samples.size());
}

int main()
{
    std::printf("NL2 CSV import/export\n");
    TestAxisMapIsARotationNotAFlip();
    TestStraightRoundTrips();
    TestRolledStraightRoundTrips();
    TestBankedArcPinsTheSigns();
    TestVerticalLoopRoundTrips();
    TestBarrelRollUnwrapsThroughPi();
    TestClosedCircuitRotatesToALevelStart();
    TestLayoutRoundTrips();
    TestSpacingControlsAccuracy();
    TestPhysicsAgreesOnBothTracks();
    TestMalformedInputIsRejected();
    TestHeaderIsOptionalAndBlankLinesIgnored();
    TestWrittenFileIsReadableAsNL2();
    std::printf("all tests passed\n");
    return 0;
}
