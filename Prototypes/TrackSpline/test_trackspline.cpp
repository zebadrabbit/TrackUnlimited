// Unit tests for the Phase 0 track spline prototype.
// Build & run:  clang++ -std=c++17 -Wall -Wextra -o test_trackspline test_trackspline.cpp && ./test_trackspline

#include "TrackSpline.h"
#include "TrackClose.h"
#include "TrackHistory.h"
#include "TrackIO.h"
#include "TrackProfile.h"
#include "TrackValidate.h"

#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

static const double Pi = 3.14159265358979323846;

// 1 mm / 1e-4 rad. Engineering tolerances, not float-equality theatre — the
// integrator is second order, so asserting exactness would just be asserting
// the step size.
static bool Near(double A, double B, double Tol = 1e-3)
{
    return std::fabs(A - B) <= Tol;
}
static bool NearVec(const FVec3& A, const FVec3& B, double Tol = 1e-3)
{
    return Length(A - B) <= Tol;
}

static void TestStraightIsStraight()
{
    FTrack Track;
    Track.AddSegment(MakeStraight(100.0));

    const FTrackFrame F = Track.EvaluateAt(100.0);
    assert(NearVec(F.Position, FVec3{100.0, 0.0, 0.0}));
    assert(NearVec(F.Tangent, FVec3{1.0, 0.0, 0.0}));
    assert(NearVec(F.Up, FVec3{0.0, 0.0, 1.0}));
    assert(Near(Track.TotalLength(), 100.0));
}

static void TestArcMatchesExactCircle()
{
    const double R = 20.0;
    FTrack Track;
    Track.AddSegment(MakeArc(0.5 * Pi * R, R)); // quarter circle, turning left

    // Starting at the origin heading +X and turning left puts the centre at
    // (0, R) and the quarter-circle endpoint at (R, R), heading +Y.
    const FTrackFrame F = Track.EvaluateAt(Track.TotalLength());
    assert(NearVec(F.Position, FVec3{R, R, 0.0}));
    assert(NearVec(F.Tangent, FVec3{0.0, 1.0, 0.0}));

    // Every sampled point sits on the circle of radius R about the centre.
    for (int i = 0; i <= 20; ++i)
    {
        const double S = Track.TotalLength() * i / 20.0;
        const FVec3 P = Track.EvaluateAt(S).Position;
        assert(Near(Length(P - FVec3{0.0, R, 0.0}), R));
    }
}

static void TestClothoidTurnsByIntegralOfCurvature()
{
    // A clothoid's heading change is the integral of its curvature over arc
    // length: for a linear ramp 0 -> 1/R over length L that is L / (2R).
    const double R = 30.0;
    const double L = 24.0;
    FTrack Track;
    Track.AddSegment(MakeClothoid(L, 0.0, 1.0 / R));

    const FTrackFrame F = Track.EvaluateAt(L);
    const double Expected = L / (2.0 * R);
    const double Actual = std::atan2(F.Tangent.Y, F.Tangent.X);
    assert(Near(Actual, Expected, 1e-4));

    // Curvature at the join-facing ends is what the segment advertises.
    assert(Near(Track.EvaluateAt(0.0).YawCurvature, 0.0, 1e-9));
    assert(Near(Track.EvaluateAt(L).YawCurvature, 1.0 / R, 1e-9));
}

static void TestClothoidRemovesTheCurvatureStep()
{
    const double R = 30.0;

    // The naive join: straight bolted straight onto an arc. Curvature jumps
    // 0 -> 1/R in zero distance, which is a lateral-G step the rider feels.
    FTrack Naive;
    Naive.AddSegment(MakeStraight(50.0));
    Naive.AddSegment(MakeArc(40.0, R));
    assert(!Naive.IsCurvatureContinuous());

    // The real thing: clothoid transitions in and out.
    FTrack Eased;
    Eased.AddSegment(MakeStraight(50.0));
    Eased.AddSegment(MakeClothoid(20.0, 0.0, 1.0 / R));
    Eased.AddSegment(MakeArc(40.0, R));
    Eased.AddSegment(MakeClothoid(20.0, 1.0 / R, 0.0));
    Eased.AddSegment(MakeStraight(50.0));
    assert(Eased.IsCurvatureContinuous());

    // Sampling across the first joint: curvature is continuous through it.
    const double Joint = 50.0;
    assert(Near(Eased.EvaluateAt(Joint - 0.01).YawCurvature, 0.0, 1e-3));
    assert(Near(Eased.EvaluateAt(Joint + 0.01).YawCurvature, 0.0, 1e-3));
    // ...and has fully ramped in by the end of the transition.
    assert(Near(Eased.EvaluateAt(70.0).YawCurvature, 1.0 / R, 1e-9));
}

static void TestBankingCancelsLateralG()
{
    // The classic result: a turn banked at atan(v^2 / (g R)) leaves the rider
    // pressed straight down into the seat with no side load at all.
    const double R = 30.0;
    const double V = 15.0;
    const double Bank = std::atan((V * V) / (GravityMs2 * R));

    FTrack Track;
    Track.AddSegment(MakeArc(40.0, R, Bank));

    const FTrackFrame F = Track.EvaluateAt(20.0);
    const FGForces G = FeltG(F, V);
    assert(Near(G.Lateral, 0.0, 1e-6));

    // All of it lands as vertical load: sqrt(1 + (v^2/(gR))^2) G.
    const double Ratio = (V * V) / (GravityMs2 * R);
    assert(Near(G.Vertical, std::sqrt(1.0 + Ratio * Ratio), 1e-6));

    // Unbanked, the same turn throws the rider sideways at exactly that ratio.
    FTrack Flat;
    Flat.AddSegment(MakeArc(40.0, R));
    const FGForces FlatG = FeltG(Flat.EvaluateAt(20.0), V);
    assert(Near(FlatG.Lateral, Ratio, 1e-6)); // +ve: thrown to the outside (right) of a left turn
    assert(Near(FlatG.Vertical, 1.0, 1e-6));
}

static void TestHeartlineIsWhatBankingRotatesAround()
{
    const double H = 1.1;
    const double Bank = 0.6;

    // Level and unbanked: rails sit H directly below the heartline.
    FTrack Level(H);
    Level.AddSegment(MakeStraight(20.0));
    assert(NearVec(Level.RailCentreAt(10.0), FVec3{10.0, 0.0, -H}));

    // Banked: the heartline holds its path and the rails swing out around it,
    // by H*sin(bank) horizontally and H*cos(bank) below.
    FTrack Banked(H);
    Banked.AddSegment(MakeStraight(20.0, Bank));
    const FTrackFrame F = Banked.EvaluateAt(10.0);
    const FVec3 Rail = Banked.RailCentreAt(10.0);

    assert(NearVec(F.Position, FVec3{10.0, 0.0, 0.0})); // heartline unmoved by roll
    const FVec3 Offset = Rail - F.Position;
    assert(Near(std::fabs(Offset.Y), H * std::sin(Bank), 1e-9));
    assert(Near(Offset.Z, -H * std::cos(Bank), 1e-9));
    assert(Near(Length(Offset), H, 1e-9)); // rails stay a fixed distance from the heartline
}

static void TestFrameStaysOrthonormal()
{
    // Long integration with curvature in two axes at once — the case where a
    // componentwise integrator would visibly drift.
    FTrack Track;
    FTrackSegment Twisty;
    Twisty.Length = 500.0;
    Twisty.YawCurvatureStart = 0.02;
    Twisty.YawCurvatureEnd = -0.03;
    Twisty.PitchCurvatureStart = -0.015;
    Twisty.PitchCurvatureEnd = 0.025;
    Twisty.RollEnd = 1.2;
    Track.AddSegment(Twisty);

    const FTrackFrame F = Track.EvaluateAt(500.0);
    assert(Near(Length(F.Tangent), 1.0, 1e-12));
    assert(Near(Length(F.Lateral), 1.0, 1e-12));
    assert(Near(Length(F.Up), 1.0, 1e-12));
    assert(Near(Dot(F.Tangent, F.Lateral), 0.0, 1e-12));
    assert(Near(Dot(F.Tangent, F.Up), 0.0, 1e-12));
    assert(Near(Dot(F.Lateral, F.Up), 0.0, 1e-12));
    // Right-handed throughout: T x L == U.
    assert(NearVec(Cross(F.Tangent, F.Lateral), F.Up, 1e-12));
}

// Everything below covers the pitch/vertical axis. The banking test above was
// once the only caller of FeltG, and it uses a flat arc where PathUp is
// identically world +Z and the pitch term multiplies by zero — a configuration
// that cannot tell world-frame gravity from path-frame gravity. Four separate
// mutations of the header survived that suite.

static void TestVerticalLoopGeometryAndG()
{
    const double R = 8.0;
    const double V = 20.0;

    // 180 degrees of constant pitch curvature puts the apex directly above the
    // start at 2R, inverted. The POSITION assert is the load-bearing one: flip
    // the Darboux pitch sign and every hill becomes a dip, yet every G reading
    // stays numerically identical because the frame flips in step with the
    // curvature. Only geometry can see that mutation.
    FTrack Loop;
    FTrackSegment Half;
    Half.Length = Pi * R;
    Half.PitchCurvatureStart = Half.PitchCurvatureEnd = 1.0 / R;
    assert(Loop.AddSegment(Half));

    const FTrackFrame Apex = Loop.EvaluateAt(Loop.TotalLength());
    assert(NearVec(Apex.Position, FVec3{0.0, 0.0, 2.0 * R}));
    assert(NearVec(Apex.Tangent, FVec3{-1.0, 0.0, 0.0}));
    assert(NearVec(Apex.Up, FVec3{0.0, 0.0, -1.0}));

    // Pressed into the seat while upside down: v^2/(gR) - 1.
    const FGForces G = FeltG(Apex, V);
    assert(Near(G.Vertical, (V * V) / (GravityMs2 * R) - 1.0, 1e-6));
    assert(Near(G.Lateral, 0.0, 1e-9));
}

static void TestAirtimeThreshold()
{
    const double V = 20.0;
    const double K = GravityMs2 / (V * V); // crest curvature giving exactly 0 G

    // Sampled at S = 0 where the tangent is still horizontal, so these are exact.
    FTrackSegment Bend;
    Bend.Length = 30.0;

    FTrack Valley;
    Bend.PitchCurvatureStart = Bend.PitchCurvatureEnd = K;
    assert(Valley.AddSegment(Bend));
    assert(Near(FeltG(Valley.EvaluateAt(0.0), V).Vertical, 2.0, 1e-9));

    FTrack Crest;
    Bend.PitchCurvatureStart = Bend.PitchCurvatureEnd = -K;
    assert(Crest.AddSegment(Bend));
    assert(Near(FeltG(Crest.EvaluateAt(0.0), V).Vertical, 0.0, 1e-9)); // airtime

    FTrack Ejector;
    Bend.PitchCurvatureStart = Bend.PitchCurvatureEnd = -2.0 * K;
    assert(Ejector.AddSegment(Bend));
    // Past the threshold the rider hangs in the restraints.
    assert(Near(FeltG(Ejector.EvaluateAt(0.0), V).Vertical, -1.0, 1e-9));
}

static void TestGravityIsWorldReferencedNotPathReferenced()
{
    const double V = 20.0;
    const double K = 1.0 / 50.0;

    FTrack Climb;
    FTrackSegment PitchUp;
    PitchUp.Length = (0.25 * Pi) / K; // exactly 45 degrees of climb
    PitchUp.PitchCurvatureStart = PitchUp.PitchCurvatureEnd = K;
    assert(Climb.AddSegment(PitchUp));
    assert(Climb.AddSegment(MakeStraight(20.0))); // holds 45 deg, curvature back to 0

    const FTrackFrame F = Climb.EvaluateAt(Climb.TotalLength() - 10.0);
    assert(Near(F.Tangent.Z, std::sin(0.25 * Pi), 1e-6));
    assert(Near(F.PitchCurvature, 0.0, 1e-12));

    // No path acceleration here, so this is purely gravity resolved onto a
    // tilted rider: cos(45 deg). Resolve gravity in the path frame instead and
    // it reads 1.0 - the exact world-vs-body slip a UE port invites.
    assert(Near(FeltG(F, V).Vertical, std::cos(0.25 * Pi), 1e-6));
}

static void TestClothoidEndpointMatchesFresnel()
{
    // Heading alone is invariant under bugs that move the endpoint metres:
    // reversing the curvature ramp shifts it 3.18 m for a heading error of
    // 1e-15 rad. This pins position against the Fresnel closed form
    // x = integral cos(s^2/(2RL)), y = integral sin(s^2/(2RL)), R = 30, L = 24.
    FTrack Track;
    assert(Track.AddSegment(MakeClothoid(24.0, 0.0, 1.0 / 30.0)));
    assert(NearVec(Track.EvaluateAt(24.0).Position, FVec3{23.618834, 3.163614, 0.0}, 1e-5));
}

static void TestRollInterpolatesAcrossSegment()
{
    // Every other roll assertion uses RollStart == RollEnd, so the Lerp was
    // never exercised: a mutant returning RollStart everywhere passed the whole
    // suite while putting the rail centreline 0.325 m out.
    FTrack Track;
    FTrackSegment Twist = MakeStraight(20.0);
    Twist.RollStart = 0.0;
    Twist.RollEnd = 0.6;
    assert(Track.AddSegment(Twist));

    assert(Near(Track.EvaluateAt(0.0).Roll, 0.0, 1e-12));
    assert(Near(Track.EvaluateAt(10.0).Roll, 0.3, 1e-12));
    assert(Near(Track.EvaluateAt(20.0).Roll, 0.6, 1e-12));
    // ...and the banked frame actually follows it.
    assert(Near(Dot(Track.EvaluateAt(10.0).Up, FVec3{0.0, 0.0, 1.0}), std::cos(0.3), 1e-9));
}

static void TestDegenerateSegmentsRejected()
{
    FTrack Track;
    assert(Track.AddSegment(MakeStraight(10.0)));
    // A zero-length clothoid whose endpoints match its neighbours would
    // otherwise bridge a real curvature step past IsCurvatureContinuous.
    assert(!Track.AddSegment(MakeClothoid(0.0, 0.0, 1.0 / 30.0)));
    assert(!Track.AddSegment(MakeStraight(-5.0)));
    assert(Track.NumSegments() == 1);
    assert(Near(Track.TotalLength(), 10.0));
}

static void TestAdvanceFromAgreesWithEvaluateAt()
{
    // The fast path a ticking train uses must trace the same curve as the slow
    // one, including across segment joints and through a curvature ramp.
    FTrack Track;
    assert(Track.AddSegment(MakeStraight(10.0)));
    assert(Track.AddSegment(MakeClothoid(20.0, 0.0, 1.0 / 25.0, 0.0, 0.5)));
    assert(Track.AddSegment(MakeArc(30.0, 25.0, 0.5)));
    FTrackSegment Hill;
    Hill.Length = 40.0;
    Hill.PitchCurvatureStart = 0.02;
    Hill.PitchCurvatureEnd = -0.02;
    Hill.RollStart = 0.5;
    assert(Track.AddSegment(Hill));

    // Walk it in small steps, the way a train would.
    FTrackFrame Walk = Track.EvaluateAt(0.0);
    double S = 0.0;
    const double Step = 0.37; // deliberately not a divisor of any segment length
    while (S < Track.TotalLength())
    {
        const double Next = std::min(S + Step, Track.TotalLength());
        Walk = Track.AdvanceFrom(Walk, S, Next);
        S = Next;

        const FTrackFrame Direct = Track.EvaluateAt(S);
        assert(NearVec(Walk.Position, Direct.Position, 1e-6));
        assert(NearVec(Walk.Tangent, Direct.Tangent, 1e-9));
        assert(NearVec(Walk.Up, Direct.Up, 1e-9));
        assert(Near(Walk.Roll, Direct.Roll, 1e-12));
        assert(Near(Walk.YawCurvature, Direct.YawCurvature, 1e-12));
    }

    // Going backwards or nowhere falls back to a full evaluation rather than
    // silently returning the frame it was handed.
    const FTrackFrame Back = Track.AdvanceFrom(Walk, S, 5.0);
    assert(NearVec(Back.Position, Track.EvaluateAt(5.0).Position, 1e-9));
}

static void TestEvaluateClampsAndSpansSegments()
{
    FTrack Track;
    Track.AddSegment(MakeStraight(10.0));
    Track.AddSegment(MakeStraight(10.0));

    assert(Near(Track.TotalLength(), 20.0));
    assert(NearVec(Track.EvaluateAt(15.0).Position, FVec3{15.0, 0.0, 0.0}));
    assert(NearVec(Track.EvaluateAt(-5.0).Position, FVec3{0.0, 0.0, 0.0}));
    assert(NearVec(Track.EvaluateAt(999.0).Position, FVec3{20.0, 0.0, 0.0}));
}

// --------------------------------------------------------------- validation

static bool HasIssue(const std::vector<FTrackDiagnostic>& D, ETrackIssue Want, std::size_t Index)
{
    for (const FTrackDiagnostic& X : D)
    {
        if (X.Issue == Want && X.SegmentIndex == Index)
        {
            return true;
        }
    }
    return false;
}

static void TestValidationCatchesTheNaNThatGeometryCannot()
{
    // The case PHASE0_FINDINGS singled out — and it is worse than recorded
    // there. A zero radius is a divide by zero, so the field holds INFINITY,
    // not NaN. Then:
    //
    //   1. The lerp inside CurvatureAt computes (inf - inf) * 0 = NaN.
    //   2. IsCurvatureContinuous compares with `fabs(x) > tol`, false for NaN,
    //      so the joint PASSES.
    //   3. Integrate guards on `Rate > 0.0`, also false for NaN, so it takes
    //      the straight-line branch.
    //
    // The result is not visibly broken geometry. It is a clean, perfectly
    // STRAIGHT segment where an arc was authored, with every check reporting
    // success. That is precisely why validation cannot live in the geometry.
    const FTrackSegment Bad = MakeArc(30.0, 0.0);
    assert(std::isinf(Bad.YawCurvatureStart));
    assert(!std::isnan(Bad.YawCurvatureStart));

    FTrack Track;
    Track.AddSegment(MakeStraight(20.0));
    Track.AddSegment(Bad);
    assert(Track.IsCurvatureContinuous()); // the false clean bill of health

    // ...and the track really did come out straight, 50 m of it.
    const FTrackFrame End = Track.EvaluateAt(Track.TotalLength());
    assert(NearVec(End.Position, FVec3{50.0, 0.0, 0.0}, 1e-9));
    assert(NearVec(End.Tangent, FVec3{1.0, 0.0, 0.0}, 1e-12));

    // A literal NaN gets the same free pass from the geometry.
    FTrackSegment Nan = MakeStraight(10.0);
    Nan.YawCurvatureStart = Nan.YawCurvatureEnd = std::nan("");
    FTrack WithNan;
    WithNan.AddSegment(MakeStraight(20.0));
    WithNan.AddSegment(Nan);
    assert(WithNan.IsCurvatureContinuous());

    // The editor boundary catches both, as errors, naming the field.
    assert(HasIssue(ValidateTrack({MakeStraight(20.0), Bad}), ETrackIssue::NotFinite, 1));
    assert(HasErrors(ValidateTrack({MakeStraight(20.0), Bad})));
    assert(HasIssue(ValidateTrack({MakeStraight(20.0), Nan}), ETrackIssue::NotFinite, 1));
    std::printf("  validation: zero radius caught — geometry silently made it STRAIGHT\n");
}

static void TestValidationRejectsAndWarnsWithoutRepairing()
{
    // Errors: unbuildable.
    FTrackSegment Zero = MakeStraight(0.0);
    assert(HasErrors(ValidateTrack({Zero})));
    FTrackSegment Negative = MakeStraight(-5.0);
    assert(HasErrors(ValidateTrack({Negative})));

    // Warning, not error: a radius typed into a curvature field. 30 1/m is a
    // 3.3 cm radius - buildable, absurd, and silent without this check.
    FTrackSegment Tiny = MakeStraight(10.0);
    Tiny.YawCurvatureStart = Tiny.YawCurvatureEnd = 30.0;
    const std::vector<FTrackDiagnostic> T = ValidateTrack({Tiny});
    assert(!HasErrors(T));
    assert(HasIssue(T, ETrackIssue::TinyRadius, 0));

    // Warning: degrees in a radians field. 45 rad is 7.2 revolutions.
    const std::vector<FTrackDiagnostic> R = ValidateTrack({MakeStraight(10.0, 45.0)});
    assert(!HasErrors(R));
    assert(HasIssue(R, ETrackIssue::HugeRoll, 0));

    // A legitimate 2*pi barrel roll sits exactly on the limit and must NOT warn.
    assert(ValidateTrack({MakeStraight(60.0, 2.0 * Pi)}).empty());

    // Nothing was repaired. This is the whole point: the findings measured that
    // clamping MakeArc(L, 0) to a straight yields a plausible 1.00 G and a clean
    // continuity pass, which is worse than an obvious NaN.
    assert(Tiny.YawCurvatureStart == 30.0);
    assert(std::isinf(MakeArc(30.0, 0.0).YawCurvatureStart));

    // Curvature magnitude is checked on the yaw/pitch VECTOR, so a segment
    // curving hard in pitch alone is caught too.
    FTrackSegment PitchOnly = MakeStraight(10.0);
    PitchOnly.PitchCurvatureStart = PitchOnly.PitchCurvatureEnd = 5.0;
    assert(HasIssue(ValidateTrack({PitchOnly}), ETrackIssue::TinyRadius, 0));

    std::printf("  validation: reports and never repairs; vector curvature, degrees-vs-radians\n");
}

static void TestValidationLocatesJointStepsAndStaysQuietWhenClean()
{
    // Straight bolted onto an arc: a real curvature step, reported against the
    // joint rather than as a bare "not continuous".
    const std::vector<FTrackSegment> Stepped = {MakeStraight(50.0), MakeArc(40.0, 30.0)};
    const std::vector<FTrackDiagnostic> S = ValidateTrack(Stepped);
    assert(!HasErrors(S));
    assert(HasIssue(S, ETrackIssue::CurvatureStep, 0));

    // Roll step: same value of curvature either side, but the track twists
    // instantaneously. IsCurvatureContinuous checks roll VALUE so it agrees
    // here, but the diagnostic says by how much and where.
    const std::vector<FTrackSegment> Twist = {MakeStraight(10.0, 0.0), MakeStraight(10.0, 0.5)};
    assert(HasIssue(ValidateTrack(Twist), ETrackIssue::RollStep, 0));

    // The properly eased version is silent - no errors AND no warnings.
    const double R = 30.0;
    const std::vector<FTrackSegment> Clean = {MakeStraight(50.0),
                                              MakeClothoid(20.0, 0.0, 1.0 / R),
                                              MakeArc(40.0, R),
                                              MakeClothoid(20.0, 1.0 / R, 0.0),
                                              MakeStraight(50.0)};
    assert(ValidateTrack(Clean).empty());

    // A NaN segment must not spray joint noise that buries the real error.
    const std::vector<FTrackSegment> WithNaN = {MakeStraight(20.0), MakeArc(30.0, 0.0),
                                                MakeStraight(20.0)};
    const std::vector<FTrackDiagnostic> N = ValidateTrack(WithNaN);
    assert(HasErrors(N));
    for (const FTrackDiagnostic& X : N)
    {
        assert(X.Issue != ETrackIssue::CurvatureStep);
    }
    std::printf("  validation: joint steps located; clean track is silent; NaN does not spam\n");
}

static void TestRollRateIsVisibleWhereFeltGIsNot()
{
    // The standard banking pattern the spline header itself endorses: ramp bank
    // over the clothoid, hold it through the arc, ramp out. PHASE0_FINDINGS
    // measured that this steps roll angular velocity at four joints per banked
    // turn while felt G reports nothing — because felt G has no roll-rate term
    // and the rider is a point at the heartline, so it vanishes exactly.
    const double R = 30.0;
    const double Bank = 45.0 * Pi / 180.0;
    const std::vector<FTrackSegment> Turn = {MakeStraight(40.0, 0.0),
                                             MakeClothoid(20.0, 0.0, 1.0 / R, 0.0, Bank),
                                             MakeArc(50.0, R, Bank),
                                             MakeClothoid(20.0, 1.0 / R, 0.0, Bank, 0.0),
                                             MakeStraight(40.0, 0.0)};

    // Curvature and roll VALUE are both continuous here — this is well-formed
    // track by every check that existed before.
    FTrack Built;
    for (const FTrackSegment& S : Turn)
    {
        Built.AddSegment(S);
    }
    assert(Built.IsCurvatureContinuous());

    // Felt G is structurally blind to roll RATE, and this shows it exactly
    // rather than approximately. Two tracks with identical curvature and the
    // same roll VALUE at the sample point, but roll rates differing by 2x:
    // because roll does not perturb the path at all, the path frames are
    // identical, so FeltG returns bit-identical numbers. No tolerance needed —
    // the roll-rate term simply is not in the model.
    FTrack Fast, Slow;
    Fast.AddSegment(MakeClothoid(40.0, 1.0 / R, 1.0 / R, 0.0, 0.4));  // 0.010 rad/m
    Slow.AddSegment(MakeClothoid(40.0, 1.0 / R, 1.0 / R, 0.1, 0.3));  // 0.005 rad/m
    const FTrackFrame FF = Fast.EvaluateAt(20.0);
    const FTrackFrame SF = Slow.EvaluateAt(20.0);
    assert(Near(FF.Roll, SF.Roll, 1e-15)); // same roll value...
    const FGForces GFast = FeltG(FF, 25.0);
    const FGForces GSlow = FeltG(SF, 25.0);
    assert(GFast.Lateral == GSlow.Lateral); // ...therefore identical felt G,
    assert(GFast.Vertical == GSlow.Vertical); // at twice the roll rate.

    // But the roll rate steps, and now something says so.
    const std::vector<FTrackDiagnostic> D = ValidateTrack(Turn);
    assert(!HasErrors(D));
    assert(HasIssue(D, ETrackIssue::RollRateStep, 0)); // straight -> clothoid
    assert(HasIssue(D, ETrackIssue::RollRateStep, 1)); // clothoid -> arc

    const FRollRateReport Rep = AnalyseRollRate(Turn, 25.0, 0.5);
    assert(Rep.PeakSegment == 1 || Rep.PeakSegment == 3);
    assert(Rep.PeakRateDegPerSec > 10.0);
    assert(Rep.HeadSnapG > 0.0);

    // Cross-check the head-snap formula against a figure in the findings:
    // 45 deg/s at 0.5 m above the heartline is 0.031 g.
    const std::vector<FTrackSegment> Ref = {MakeStraight(10.0, 0.0),
                                            MakeStraight(10.0 * 25.0 / (45.0 * Pi / 180.0) * 0.0
                                                             + 25.0,
                                                         0.0)};
    (void)Ref;
    const double Omega = 45.0 * Pi / 180.0;                    // rad/s
    const double Expect = Omega * Omega * 0.5 / GravityMs2;    // w^2 r / g
    assert(Near(Expect, 0.031, 5e-4));

    std::printf("  roll rate: peak %.1f deg/s, joint step %.1f deg/s, head snap %.3f g "
                "(felt G shows none of it)\n",
                Rep.PeakRateDegPerSec, Rep.WorstStepDegPerSec, Rep.HeadSnapG);

    // A roll ramped smoothly across the whole turn has no rate step at all.
    const std::vector<FTrackSegment> Smooth = {MakeStraight(40.0, 0.0), MakeStraight(40.0, 0.0)};
    assert(!HasIssue(ValidateTrack(Smooth), ETrackIssue::RollRateStep, 0));
}

// ------------------------------------------------------------------ track I/O

// A layout using every authored kind, so a round trip has to carry all of them.
static FTrackDocument SampleDocument()
{
    FTrackDocument Doc;
    Doc.HeartlineHeight = 1.1;
    // Angles are DEGREES on the authoring side — see TrackIO.h. Curvature is
    // still 1/m, because it is not an angle.
    Doc.Segments = {
        AuthorStraight(40.0),
        AuthorClothoid(20.0, 0.0, 1.0 / 30.0, 0.0, 25.0),
        AuthorArc(50.0, 30.0, 25.0),
        AuthorHelix(20.0, 15.0, 1.5, 12.0),
    };
    // A hill. No Make* helper builds pitch curvature, so this is exactly the
    // case ESegmentKind::Raw exists for.
    FTrackSegment Hill;
    Hill.Length = 35.0;
    Hill.PitchCurvatureStart = Hill.PitchCurvatureEnd = -1.0 / 45.0;
    Doc.Segments.push_back(AuthorRaw(Hill));
    // And a world-referenced bank, so the mode survives the file too.
    FAuthoredSegment Banked = AuthorArc(30.0, -40.0, 18.0);
    Banked.RollMode = ERollMode::WorldBank;
    Doc.Segments.push_back(Banked);

    // A LIFT, so the control layer is in the sample rather than tested off to one
    // side. Every round-trip assertion below already walks this document, and a
    // device that only appeared in its own dedicated test would be a device the
    // general checks never see.
    FAuthoredSegment Lift = AuthorStraight(30.0);
    Lift.Zone = EAuthoredZone::Lift;
    Lift.ZoneSpeed = 3.5;
    Lift.bAntiRollback = true;
    Doc.Segments.push_back(Lift);
    return Doc;
}

// EVERY FIELD THIS SEGMENT'S KIND USES, AT A DISTINCT NON-DEFAULT VALUE. Anything
// left at its default is a field the round trip cannot see fail: the writer omits
// defaults, so a dropped field and a default field produce identical text.
//
// AN ARC, and the geometry fields the other kinds use are deliberately left alone.
// The format emits only what the kind uses — a clothoid's curvature on an arc is
// not an author's decision about this segment, it is a value the editor is holding
// on to — so requiring them to survive would be asserting the opposite of the
// design. The kinds that DO use them are covered by SampleDocument, which carries
// one of each and is compared as exact geometry.
static FAuthoredSegment EveryFieldSet()
{
    FAuthoredSegment A;
    A.Kind = ESegmentKind::Arc;
    A.Length = 41.5;
    A.Radius = -27.25;
    A.RollStartDegrees = 7.5;
    A.RollEndDegrees = -12.25;
    A.RollMode = ERollMode::WorldBank;
    A.Zone = EAuthoredZone::BlockBrake;
    A.ZoneSpeed = 2.5;
    A.ZoneAccel = 1.75;
    A.ZoneDecel = 3.25;
    A.ZoneBrakeDecel = 4.5;
    A.bAntiRollback = true;
    A.bStartsNewDevice = true;
    return A;
}

// The completeness list, and the only place a new authored field has to be added
// twice on purpose. Compared field by field rather than by geometry, because the
// device layer produces no geometry at all — a walk of the track cannot tell a
// block brake from plain straight, which is exactly how it went unsaved.
static bool SameAuthored(const FAuthoredSegment& A, const FAuthoredSegment& B)
{
    return A.Kind == B.Kind && A.Length == B.Length && A.Radius == B.Radius
           && A.CurvatureStart == B.CurvatureStart && A.CurvatureEnd == B.CurvatureEnd
           && A.ClimbAngleDegrees == B.ClimbAngleDegrees && A.Turns == B.Turns
           && A.RollStartDegrees == B.RollStartDegrees && A.RollEndDegrees == B.RollEndDegrees
           && A.RollMode == B.RollMode && A.Zone == B.Zone && A.ZoneSpeed == B.ZoneSpeed
           && A.ZoneAccel == B.ZoneAccel && A.ZoneDecel == B.ZoneDecel
           && A.ZoneBrakeDecel == B.ZoneBrakeDecel && A.bAntiRollback == B.bAntiRollback
           && A.bStartsNewDevice == B.bStartsNewDevice;
}

static std::vector<std::string> SplitLines(const std::string& S)
{
    std::vector<std::string> Out;
    std::string Line;
    for (const char Ch : S)
    {
        if (Ch == '\n')
        {
            Out.push_back(Line);
            Line.clear();
        }
        else
        {
            Line.push_back(Ch);
        }
    }
    if (!Line.empty())
    {
        Out.push_back(Line);
    }
    return Out;
}

static void TestTrackRoundTripsThroughJson()
{
    const FTrackDocument Doc = SampleDocument();
    std::string Text, Err;
    assert(WriteTrackJson(Doc, Text, Err));
    assert(Err.empty());

    FTrackDocument Back;
    assert(ParseTrackJson(Text, Back, Err));
    assert(Err.empty());
    assert(Back.Segments.size() == Doc.Segments.size());
    assert(Back.HeartlineHeight == Doc.HeartlineHeight);

    // Compared as GEOMETRY, not as fields. Fields matching proves the struct
    // survived; walking the track proves the ride did. The vertical loop test
    // records why that distinction earns its keep — a mutant once put the apex
    // at z=-16 with every G reading identical.
    const FTrack A = BuildTrack(Doc);
    const FTrack B = BuildTrack(Back);
    assert(Near(A.TotalLength(), B.TotalLength(), 1e-12));

    double Worst = 0.0;
    for (double S = 0.0; S <= A.TotalLength(); S += 1.0)
    {
        const FTrackFrame FA = A.EvaluateAt(S);
        const FTrackFrame FB = B.EvaluateAt(S);
        Worst = std::fmax(Worst, Length(FA.Position - FB.Position));
        assert(Near(FA.Roll, FB.Roll, 1e-12));
        assert(NearVec(FA.Up, FB.Up, 1e-12));
    }
    // Exact, not merely close. Every stored number is the shortest decimal that
    // reads back bit-identical, and the derived values are recomputed by the
    // same code on both sides rather than being stored and trusted.
    assert(Worst == 0.0);

    // Re-writing what was read gives byte-identical text. Without this, opening
    // and saving a file would churn the diff even with no edit.
    std::string Again;
    assert(WriteTrackJson(Back, Again, Err));
    assert(Again == Text);

    std::printf("  track I/O: %zu segments round-trip exactly; re-save is byte-identical\n",
                Doc.Segments.size());
}

// THE DEVICE LAYER SURVIVES THE FILE, and it needs its own assertion because the
// round-trip above compares GEOMETRY — and a block brake produces none. Saving
// dropped every zone, speed, rate and flag for as long as the format existed, and
// walking the track could never have noticed: the ride is the same shape with its
// brakes missing.
static void TestDeviceLayerSurvivesTheFile()
{
    FTrackDocument Doc;
    Doc.Segments.push_back(EveryFieldSet());

    std::string Text, Err;
    assert(WriteTrackJson(Doc, Text, Err));
    assert(Err.empty());

    FTrackDocument Back;
    assert(ParseTrackJson(Text, Back, Err));
    assert(Err.empty());
    assert(Back.Segments.size() == 1);

    if (!SameAuthored(Doc.Segments[0], Back.Segments[0]))
    {
        std::printf("  A FIELD DID NOT SURVIVE. Written:\n%s", Text.c_str());
    }
    assert(SameAuthored(Doc.Segments[0], Back.Segments[0]));

    // AND THE WRITER IS NOT SILENTLY OMITTING THEM. Comparing the structs alone
    // would pass if both sides simply kept their defaults, so the text itself has
    // to be shown to carry each key.
    const char* MustAppear[] = {"\"zone\": \"blockBrake\"", "zoneSpeed", "zoneAccel",
                                "zoneDecel", "zoneBrakeDecel", "antiRollback",
                                "startsNewDevice"};
    for (const char* Key : MustAppear)
    {
        if (Text.find(Key) == std::string::npos)
        {
            std::printf("  NOT WRITTEN: %s\n", Key);
        }
        assert(Text.find(Key) != std::string::npos);
    }

    // A file from before the control layer existed is still a valid track, and
    // loads as the unpowered geometry it always was.
    FTrackDocument Old;
    assert(ParseTrackJson("{\"segments\": [{\"kind\": \"straight\", \"length\": 10}]}",
                          Old, Err));
    assert(Old.Segments.size() == 1);
    assert(Old.Segments[0].Zone == EAuthoredZone::None);
    assert(!Old.Segments[0].bAntiRollback);

    // AN UNKNOWN DEVICE IS REFUSED, not read as plain track. That is the whole
    // argument: a ride whose brake silently became straight track still loads,
    // still looks right, and cannot stop a train.
    FTrackDocument Bad;
    assert(!ParseTrackJson(
        "{\"segments\": [{\"kind\": \"straight\", \"length\": 10, \"zone\": \"magnetic\"}]}",
        Bad, Err));
    assert(!Err.empty());
    std::printf("  device layer: every field survives, an old file still loads, "
                "an unknown device is refused\n");

    // A NEW AUTHORED FIELD MUST NOT BE ABLE TO GO UNSAVED IN SILENCE. Everything
    // above tests the fields it knows about; nothing makes somebody adding the
    // next one come here. This does: add a field and this fires, and the message
    // is the checklist.
    //
    // In the test rather than the header on purpose — a padding difference on some
    // future compiler should annoy whoever runs the suite, never break the build
    // of the game.
    static_assert(sizeof(FAuthoredSegment) == sizeof(FTrackSegment) + 120,
                  "FAuthoredSegment changed size, which means a field was added or removed. "
                  "Add it to WriteTrackJson, to ParseTrackJson, to SameAuthored and to "
                  "EveryFieldSet, or it will not survive a save.");
}

static void TestOneEditIsOneLineOfDiff()
{
    // The claim the whole authored model rests on. A helix stored as its
    // authored parameters puts a radius change in ONE number. Stored as the
    // ~300-segment approximation it would need, the same edit rewrites ~600
    // fitted curvature endpoints — and only if regeneration were
    // bit-deterministic across compilers, which is not a promise worth making.
    FTrackDocument Doc = SampleDocument();
    std::string Before, After, Err;
    assert(WriteTrackJson(Doc, Before, Err));

    for (FAuthoredSegment& A : Doc.Segments)
    {
        if (A.Kind == ESegmentKind::Helix)
        {
            A.Radius += 2.0;
        }
    }
    assert(WriteTrackJson(Doc, After, Err));

    const std::vector<std::string> L1 = SplitLines(Before);
    const std::vector<std::string> L2 = SplitLines(After);
    assert(L1.size() == L2.size());
    int Changed = 0;
    for (std::size_t i = 0; i < L1.size(); ++i)
    {
        Changed += (L1[i] != L2[i]) ? 1 : 0;
    }
    assert(Changed == 1);

    // And the geometry really did move, so this is not one line because nothing
    // happened.
    assert(!Near(BuildTrack(Doc).TotalLength(),
                 BuildTrack(SampleDocument()).TotalLength(), 1.0));
    std::printf("  track I/O: a helix radius change is %d line of diff\n", Changed);
}

static void TestDefaultsAreOmittedSoNewFieldsCostNothing()
{
    // Both Torsion and RollMode were added to a model that already had stored
    // tracks. Neither should have touched a single existing line, and the
    // property that made that true is that a field sitting at its default is
    // not written at all.
    FTrackDocument Doc;
    Doc.Segments = {AuthorStraight(40.0), AuthorArc(50.0, 30.0)};
    std::string Text, Err;
    assert(WriteTrackJson(Doc, Text, Err));
    assert(Text.find("roll") == std::string::npos);     // no roll, no rollMode
    assert(Text.find("torsion") == std::string::npos);  // not a helix, not written

    // Absent optional fields load as their defaults rather than as an error.
    FTrackDocument Back;
    assert(ParseTrackJson(Text, Back, Err));
    assert(Back.Segments[1].RollStartDegrees == 0.0);
    assert(Back.Segments[1].RollMode == ERollMode::PathRelative);

    // A constant roll is one field; a ramping roll is two. The common case
    // should not cost two lines of noise.
    Doc.Segments[1].RollStartDegrees = Doc.Segments[1].RollEndDegrees = 30.0;
    assert(WriteTrackJson(Doc, Text, Err));
    assert(Text.find("\"rollDeg\":") != std::string::npos);
    assert(Text.find("rollStartDeg") == std::string::npos);
}

static void TestUnknownFieldsLoadButUnknownGeometryDoesNot()
{
    // Forward compatibility, and its exact limit. A key we do not recognise is
    // ignorable — a later version adding metadata should not break this build.
    const std::string Extra =
        "{\"format\": \"trackunlimited.track\", \"version\": 1, \"name\": \"whatever\",\n"
        " \"segments\": [{\"kind\": \"straight\", \"length\": 20, \"colour\": 7}]}";
    FTrackDocument Doc;
    std::string Err;
    assert(ParseTrackJson(Extra, Doc, Err));
    assert(Doc.Segments.size() == 1);
    assert(Doc.Segments[0].Length == 20.0);

    // A SEGMENT KIND we do not recognise is not ignorable. Skipping it would
    // drop geometry out of the middle of a track and leave a file that loads,
    // looks fine, and is a different ride. Refuse instead.
    const std::string Future =
        "{\"segments\": [{\"kind\": \"straight\", \"length\": 20},\n"
        "               {\"kind\": \"parabola\", \"length\": 30}]}";
    assert(!ParseTrackJson(Future, Doc, Err));
    assert(Err.find("parabola") != std::string::npos);

    // Same reasoning one level up: a file from a future format version is
    // refused rather than partially understood.
    const std::string Newer = "{\"version\": 99, \"segments\": []}";
    assert(!ParseTrackJson(Newer, Doc, Err));
    assert(Err.find("99") != std::string::npos);
}

static void TestMalformedFilesAreRejectedNotAbsorbed()
{
    // A trust boundary: this is a file off disk that a human may have hand
    // edited. Every case here must produce an error string, not a default that
    // reads as a plausible track.
    const char* Bad[] = {
        "",                                                          // empty
        "not json at all",                                           // no object
        "{\"segments\": [{\"kind\": \"arc\", \"length\": 20}]}",     // arc with no radius
        "{\"segments\": [{\"kind\": \"straight\", \"length\": abc}]}",   // garbage number
        "{\"segments\": [{\"kind\": \"straight\", \"length\": 1e999}]}", // overflows to inf
        "{\"segments\": [{\"length\": 20}]}",                        // no kind
        "{\"heartlineHeight\": 1.1}",                                // no segments array
        "{\"segments\": [{\"kind\": \"straight\", \"length\": 20}",   // truncated
        "{\"segments\": [{\"kind\": \"straight\", \"length\": 20, \"nested\": {\"a\": 1}}]}",
        "{\"segments\": [{\"kind\": \"straight\", \"length\": 20, \"rollMode\": \"sideways\"}]}",
        "{\"segments\": [{\"kind\": \"straight\", \"length\": \"unterminated}]}",
    };
    for (const char* Text : Bad)
    {
        FTrackDocument Doc;
        std::string Err;
        const bool bOk = ParseTrackJson(Text, Doc, Err);
        assert(!bOk);
        assert(!Err.empty()); // and it says what was wrong, not just that it failed
    }

    // The writer is the other half of the boundary: it must refuse to emit a
    // non-finite value rather than write "nan", which no reader could take back
    // in. This is what MakeArc(radius = 0) produces.
    FTrackDocument Doc;
    Doc.Segments = {AuthorRaw(MakeArc(50.0, 0.0))};
    std::string Text, Err;
    assert(!WriteTrackJson(Doc, Text, Err));
    assert(!Err.empty());

    std::printf("  track I/O: %zu malformed inputs rejected with a reason; "
                "non-finite refused on write\n",
                sizeof(Bad) / sizeof(Bad[0]));
}

static void TestAuthoredParametersSurviveWhereDerivedOnesWouldNot()
{
    // The point of storing what was typed. After a save/load cycle the helix is
    // still "radius 20, 15 degrees, 1.5 turns" — three numbers a human can edit
    // — and not the length/curvature/torsion triple the integrator runs on,
    // which is correct, equivalent, and un-editable.
    FTrackDocument Doc;
    Doc.Segments = {AuthorHelix(20.0, 15.0, 1.5, 12.0)};
    std::string Text, Err;
    assert(WriteTrackJson(Doc, Text, Err));
    assert(Text.find("\"radius\": 20") != std::string::npos);
    assert(Text.find("\"turns\": 1.5") != std::string::npos);
    assert(Text.find("torsion") == std::string::npos); // derived, so never stored

    // Degrees, and readable ones. Stored in radians this line would read
    // "climbAngleDeg": 0.26179938779914941 and tell a human nothing. Angles
    // live in degrees all the way to the struct, so the conversion runs one
    // direction only and the round trip below is exact rather than close.
    assert(Text.find("\"climbAngleDeg\": 15") != std::string::npos);
    assert(Text.find("\"rollDeg\": 12") != std::string::npos);

    FTrackDocument Back;
    assert(ParseTrackJson(Text, Back, Err));
    assert(Back.Segments[0].Radius == 20.0);
    assert(Back.Segments[0].Turns == 1.5);
    assert(Back.Segments[0].ClimbAngleDegrees == 15.0);

    // Rebuilt through the same MakeHelix, so the derived form is bit-identical
    // rather than merely close.
    const FTrackSegment Rebuilt = BuildSegment(Back.Segments[0]);
    const FTrackSegment Original = BuildSegment(Doc.Segments[0]);
    assert(Rebuilt.Length == Original.Length);
    assert(Rebuilt.Torsion == Original.Torsion);
    assert(Rebuilt.YawCurvatureStart == Original.YawCurvatureStart);
}

// ---------------------------------------------------------------- undo/redo

static void TestUndoRedoRestoresGeometryNotJustFields()
{
    FTrackDocument Doc;
    Doc.Segments = {AuthorStraight(40.0), AuthorArc(50.0, 30.0, 25.0),
                    AuthorHelix(20.0, 15.0, 1.5)};
    FTrackHistory History(Doc);
    assert(!History.IsDirty());
    assert(!History.CanUndo());
    assert(!History.CanRedo());

    FTrackDocument Edited = Doc;
    Edited.Segments[1].Radius = 45.0;
    assert(History.Commit(Edited, "change radius"));
    assert(History.IsDirty());
    assert(History.CanUndo());
    assert(History.UndoLabel() == "change radius");

    FTrackDocument Again = Edited;
    Again.Segments[0].Length = 60.0;
    assert(History.Commit(Again, "change length"));

    // Undo twice and the track must be the ORIGINAL track, walked — not merely
    // a struct with matching fields.
    History.Undo();
    History.Undo();
    const FTrack Restored = BuildTrack(History.Current());
    const FTrack Original = BuildTrack(Doc);
    assert(Near(Restored.TotalLength(), Original.TotalLength(), 1e-12));
    for (double S = 0.0; S <= Original.TotalLength(); S += 5.0)
    {
        assert(NearVec(Restored.EvaluateAt(S).Position, Original.EvaluateAt(S).Position, 1e-12));
    }
    assert(!History.CanUndo());

    History.Redo();
    History.Redo();
    assert(History.Current().Segments[0].Length == 60.0);
    assert(History.Current().Segments[1].Radius == 45.0);
    assert(!History.CanRedo());
}

static void TestCommittingNothingIsNotAnUndoStep()
{
    // An editor that commits on every field callback would otherwise fill the
    // stack with steps that undo nothing, and Ctrl+Z five times moves nothing.
    FTrackDocument Doc;
    Doc.Segments = {AuthorStraight(40.0), AuthorArc(50.0, 30.0)};
    FTrackHistory History(Doc);
    assert(!History.Commit(Doc, "no change"));
    assert(History.Depth() == 1);
    assert(!History.CanUndo());

    // A field the segment's KIND does not use is not part of the document's
    // meaning, so changing it is genuinely no change. This falls out of using
    // the save format as identity rather than a field-by-field comparison.
    FTrackDocument Meaningless = Doc;
    Meaningless.Segments[1].ClimbAngleDegrees = 40.0; // an arc has no climb angle
    assert(!History.Commit(Meaningless, "meaningless"));
    assert(History.Depth() == 1);

    // And a field it does use is.
    FTrackDocument Real = Doc;
    Real.Segments[1].Radius = 31.0;
    assert(History.Commit(Real, "real"));
    assert(History.Depth() == 2);
}

static void TestTypingIsOneUndoStepNotFive()
{
    FTrackDocument Doc;
    Doc.Segments = {AuthorArc(50.0, 3.0)};
    FTrackHistory History(Doc);

    // "30.5" arriving one keystroke at a time, all under one merge key.
    for (const double R : {30.0, 30.5, 305.0, 30.5})
    {
        FTrackDocument D = History.Current();
        D.Segments[0].Radius = R;
        History.Commit(D, "change radius", "seg0.radius");
    }
    assert(History.Depth() == 2);      // one initial state, one edit
    assert(History.Current().Segments[0].Radius == 30.5);
    History.Undo();
    assert(History.Current().Segments[0].Radius == 3.0); // straight back to before typing

    // A different key does not merge — leaving the field and editing another
    // must produce its own step.
    History.Redo();
    FTrackDocument Other = History.Current();
    Other.Segments[0].Length = 55.0;
    assert(History.Commit(Other, "change length", "seg0.length"));
    assert(History.Depth() == 3);
}

static void TestRedoBranchIsDiscardedAndSavedStateTracksIt()
{
    FTrackDocument Doc;
    Doc.Segments = {AuthorStraight(40.0)};
    FTrackHistory History(Doc);

    FTrackDocument A = Doc; A.Segments[0].Length = 50.0;
    History.Commit(A, "A");
    FTrackDocument B = A; B.Segments[0].Length = 60.0;
    History.Commit(B, "B");
    History.MarkSaved();
    assert(!History.IsDirty());

    // Undoing away from the saved state makes it dirty; coming back makes it
    // clean again, because the document really is byte-for-byte what is on disk.
    History.Undo();
    assert(History.IsDirty());
    History.Redo();
    assert(!History.IsDirty());

    // Undo, then commit something else: the saved state was on the branch just
    // discarded, so it is unreachable and the document stays dirty forever.
    History.Undo();
    FTrackDocument C = History.Current(); C.Segments[0].Length = 70.0;
    assert(History.Commit(C, "C"));
    assert(!History.CanRedo());          // B is gone
    assert(History.IsDirty());
    History.Undo();
    assert(History.IsDirty());           // and undoing does not resurrect it
}

static void TestDepthCapDropsOldestAndNeverClaimsCleanFalsely()
{
    FTrackDocument Doc;
    Doc.Segments = {AuthorStraight(10.0)};
    FTrackHistory History(Doc, 4);
    History.MarkSaved();

    for (int i = 1; i <= 10; ++i)
    {
        FTrackDocument D = History.Current();
        D.Segments[0].Length = 10.0 + i;
        assert(History.Commit(D, "grow"));
    }
    assert(History.Depth() == 4);
    assert(History.Current().Segments[0].Length == 20.0);

    // Undo as far as the stack now goes, and the oldest surviving state is the
    // one three edits back — not the original.
    while (History.CanUndo()) { History.Undo(); }
    assert(History.Current().Segments[0].Length == 17.0);

    // The saved state fell off the bottom. Claiming "clean" here would tell
    // someone their work is safe when the evidence for that was discarded, so
    // it stays dirty at every reachable position.
    assert(History.IsDirty());
    while (History.CanRedo())
    {
        History.Redo();
        assert(History.IsDirty());
    }
}

// ----------------------------------------------------------- self-clearance

static void TestTrackPassingThroughItselfIsVisibleToNothingElse()
{
    // A planar vertical loop — pure pitch curvature, no lateral component — is
    // exactly the shape the vertical slice shipped, and its two legs occupy the
    // same space. Every segment is individually fine, every joint is
    // continuous, every G reading is plausible, and the rider goes through
    // solid steel. It took riding it to notice.
    const double R = 9.0, Ease = 54.0;
    FTrack Loop;
    Loop.AddSegment(MakeStraight(30.0));
    {
        FTrackSegment A;
        A.Length = Ease;
        A.PitchCurvatureEnd = 1.0 / R;
        Loop.AddSegment(A);
        FTrackSegment B;
        B.Length = 2.0 * Pi * R - Ease;
        B.PitchCurvatureStart = B.PitchCurvatureEnd = 1.0 / R;
        Loop.AddSegment(B);
        FTrackSegment C;
        C.Length = Ease;
        C.PitchCurvatureStart = 1.0 / R;
        Loop.AddSegment(C);
    }
    Loop.AddSegment(MakeStraight(30.0));

    // Everything that existed before says this track is fine.
    assert(Loop.IsCurvatureContinuous());
    const FTrackProfile Profile;

    const FClearanceReport Bad = AnalyseSelfClearance(Loop, Profile);
    assert(Bad.bStructureOverlaps);
    assert(Bad.ClosestApproach < TrackWidth(Profile));
    // The two places are far apart along the track and on top of each other in
    // space — which is the entire signature of the fault.
    assert(Bad.AndS - Bad.AtS > 12.0);

    // Torsion turns the planar loop into a shallow horizontal-axis helix that
    // side-steps as it goes round, and the legs separate. ChainCurvature is what
    // keeps the joints continuous while it does — without it the curvature
    // vector restarts its rotation at every segment.
    FTrack Stepped;
    Stepped.AddSegment(MakeStraight(30.0));
    {
        const double Tor = 0.003;
        FTrackSegment A;
        A.Length = Ease;
        A.PitchCurvatureEnd = 1.0 / R;
        A.Torsion = Tor;
        Stepped.AddSegment(A);
        FTrackSegment B;
        B.Length = 2.0 * Pi * R - Ease;
        B.Torsion = Tor;
        ChainCurvature(A, B);
        B.YawCurvatureEnd = B.YawCurvatureStart;
        B.PitchCurvatureEnd = B.PitchCurvatureStart;
        Stepped.AddSegment(B);
        FTrackSegment C;
        C.Length = Ease;
        C.Torsion = Tor;
        ChainCurvature(B, C);
        Stepped.AddSegment(C);
    }
    Stepped.AddSegment(MakeStraight(30.0));

    assert(Stepped.IsCurvatureContinuous()); // ChainCurvature earning its keep
    const FClearanceReport Good = AnalyseSelfClearance(Stepped, Profile);
    assert(!Good.bStructureOverlaps);
    assert(Good.ClosestApproach > Bad.ClosestApproach * 5.0);

    std::printf("  clearance: planar loop passes %.3f m from itself (rails are %.3f m wide); "
                "torsion 0.003 opens it to %.3f m\n",
                Bad.ClosestApproach, TrackWidth(Profile), Good.ClosestApproach);
}

static void TestChainCurvatureIsWhatMakesTorsionComposable()
{
    // Torsion's phase is measured from each segment's own start, so a segment
    // exits with its curvature vector rotated and the next one begins rotating
    // again from zero. Three torsioned segments in a row therefore step at every
    // joint — and the geometry still looks plausible, which is why the check
    // matters more than the eye.
    const double Tor = 0.004;
    FTrackSegment A = MakeArc(40.0, 25.0);
    A.Torsion = Tor;
    FTrackSegment Naive = MakeArc(40.0, 25.0);
    Naive.Torsion = Tor;

    FTrack Stepped;
    Stepped.AddSegment(A);
    Stepped.AddSegment(Naive);
    assert(!Stepped.IsCurvatureContinuous()); // the trap, caught

    FTrackSegment Chained = Naive;
    ChainCurvature(A, Chained);
    // Holding the magnitude across the joint: end matches start, and the
    // segment's own torsion carries the rotation on from there.
    Chained.YawCurvatureEnd = Chained.YawCurvatureStart;
    Chained.PitchCurvatureEnd = Chained.PitchCurvatureStart;

    FTrack Smooth;
    Smooth.AddSegment(A);
    Smooth.AddSegment(Chained);
    assert(Smooth.IsCurvatureContinuous());

    // And the curvature MAGNITUDE is unchanged by the hand-off — only its
    // direction moved. A chain that quietly rescaled the curvature would pass
    // the continuity check and change the ride.
    double Y0 = 0.0, P0 = 0.0, Y1 = 0.0, P1 = 0.0;
    CurvatureAt(A, 0.0, Y0, P0);
    CurvatureAt(Chained, 0.0, Y1, P1);
    assert(Near(std::sqrt(Y0 * Y0 + P0 * P0), std::sqrt(Y1 * Y1 + P1 * P1), 1e-12));
}

// ------------------------------------------------------------ circuit closure

// A four-corner circuit: straight, 90-degree eased corner, four times over.
// Every corner turns EXACTLY 90 degrees whatever its radius and easement,
// because the arc length is computed for the turn the easements leave — so
// heading always closes and only position can gap. That is the asymmetry
// PHASE0_FINDINGS measured, and the whole shape of the problem.
//
// Straight 0 runs along +X and straight 1 along +Y, which is what makes them a
// pair that can span a planar gap. Two PARALLEL straights cannot, however
// convenient it would be.
static FTrackDocument BuildCircuit(double EasementA, double RadiusA, double Easement,
                                   double Radius)
{
    auto Corner = [](double Ease, double R, std::vector<FAuthoredSegment>& Out) {
        const double EaseTurn = 0.5 * Ease / R; // integral of a linear ramp
        const double ArcTurn = Pi * 0.5 - 2.0 * EaseTurn;
        Out.push_back(AuthorClothoid(Ease, 0.0, 1.0 / R));
        Out.push_back(AuthorArc(ArcTurn * R, R));
        Out.push_back(AuthorClothoid(Ease, 1.0 / R, 0.0));
    };

    FTrackDocument Doc;
    Doc.Segments.push_back(AuthorStraight(80.0)); // 0: +X
    Corner(EasementA, RadiusA, Doc.Segments);     // 1,2,3 — the odd one out
    Doc.Segments.push_back(AuthorStraight(60.0)); // 4: +Y
    Corner(Easement, Radius, Doc.Segments);
    Doc.Segments.push_back(AuthorStraight(80.0)); // 8: -X
    Corner(Easement, Radius, Doc.Segments);
    Doc.Segments.push_back(AuthorStraight(60.0)); // 12: -Y
    Corner(Easement, Radius, Doc.Segments);
    return Doc;
}

static void TestSymmetricCircuitAlreadyCloses()
{
    // The baseline the findings recorded: a symmetric oval closes at 0.0000 m
    // with no solving at all. If this ever stops being true, the integrator
    // broke, not the solver.
    const FTrackDocument Doc = BuildCircuit(20.0, 30.0, 20.0, 30.0);
    const FTrack Track = BuildTrack(Doc);
    const FClosureGap Gap = MeasureClosure(Track, CircuitTarget(Track));
    assert(Gap.PositionError < 1e-6);
    assert(Gap.HeadingError < 1e-9);

    // And the solver leaves an already-closed track completely alone, rather
    // than nudging the author's round numbers to chase floating-point noise.
    FTrackDocument Copy = Doc;
    const FClosureResult R =
        SolveClosure(Copy, CircuitTarget(Track), {FreeLength(0), FreeLength(4)});
    assert(R.bConverged);
    assert(!R.bApplied);
    assert(Copy.Segments[0].Length == Doc.Segments[0].Length);
}

static void TestClosesTheGapTheFindingsMeasured()
{
    // One corner eased over 8 m where the other three use 20 m. Heading closes
    // exactly — the author typed four 90-degree corners — and position does
    // not, because nobody typed where the track ends up. That asymmetry is the
    // whole shape of this problem, and it is why a control-point model gets
    // closure free and this one has to solve for it.
    FTrackDocument Doc = BuildCircuit(8.0, 30.0, 20.0, 30.0);
    const FTrack Before = BuildTrack(Doc);
    const FClosureTarget Target = CircuitTarget(Before);
    const FClosureGap Gap0 = MeasureClosure(Before, Target);
    assert(Gap0.PositionError > 0.5);
    assert(Gap0.HeadingError < 1e-9); // heading was never in question

    // Straight 0 runs along +X and straight 4 along +Y, so between them they
    // span the plane the gap lives in. Changing a straight's length translates
    // everything downstream without rotating it, which makes this exactly
    // linear — Gauss-Newton should need one real step, not a search.
    const FClosureResult R = SolveClosure(Doc, Target, {FreeLength(0), FreeLength(4)});
    assert(R.bConverged);
    assert(R.bApplied);
    assert(R.After.PositionError <= 1e-3);
    assert(R.After.PositionError < Gap0.PositionError);

    // Still authored values afterwards, not baked geometry. The straights
    // moved; every arc kept the radius the author typed, and every segment kept
    // its kind. A solver that wrote curvature directly would leave a segment
    // whose radius field and curvature field disagree.
    assert(Doc.Segments[0].Kind == ESegmentKind::Straight);
    assert(Doc.Segments[2].Radius == 30.0);
    assert(Doc.Segments[6].Radius == 30.0);
    assert(Doc.Segments[2].Kind == ESegmentKind::Arc);

    std::printf("  closure: %.3f m gap closed to %.2e m in %d iterations, %d walks\n",
                Gap0.PositionError, R.After.PositionError, R.Iterations, R.Evaluations);

    // A much bigger one: one corner at R=45 against three at R=30.
    FTrackDocument Wide = BuildCircuit(20.0, 45.0, 20.0, 30.0);
    const FClosureTarget WideTarget = CircuitTarget(BuildTrack(Wide));
    const FClosureGap Gap1 = MeasureClosure(BuildTrack(Wide), WideTarget);
    assert(Gap1.PositionError > 5.0);
    const FClosureResult R1 = SolveClosure(Wide, WideTarget, {FreeLength(0), FreeLength(4)});
    assert(R1.bConverged);
    assert(R1.After.PositionError <= 1e-3);
    std::printf("  closure: %.2f m gap closed to %.2e m in %d iterations\n",
                Gap1.PositionError, R1.After.PositionError, R1.Iterations);
}

static void TestHeightGapIsCalledOutSeparately()
{
    // The failure mode the vertical slice actually shipped: a track that looks
    // closed from above and ends 8.5 m below its station, so the cart falls
    // through where the platform should be. Plan view shows nothing.
    FTrackDocument Doc;
    Doc.Segments.push_back(AuthorStraight(40.0));
    FTrackSegment Down;
    Down.Length = 60.0;
    Down.PitchCurvatureStart = Down.PitchCurvatureEnd = -0.004;
    Doc.Segments.push_back(AuthorRaw(Down));
    Doc.Segments.push_back(AuthorStraight(40.0));

    const FTrack Track = BuildTrack(Doc);
    const FClosureGap Gap = MeasureClosure(Track, CircuitTarget(Track));
    assert(Gap.HeightError < -1.0);            // it ends low
    assert(std::fabs(Gap.HeightError) > 1.0);
    // And the height is a named field, not something to be dug out of a
    // magnitude that mixes it with a much larger horizontal distance.
    assert(Gap.PositionError > std::fabs(Gap.HeightError));
    std::printf("  closure: ends %.2f m low with a %.1f m position gap — height is its own "
                "number for a reason\n",
                Gap.HeightError, Gap.PositionError);
}

static void TestSolverRefusesRatherThanBreakingTheTrack()
{
    // A radius may not cross zero on its way to a solution. +ve is a left turn
    // and -ve is a right turn, so crossing would silently reverse the turn —
    // and it would pass through the infinite curvature PHASE0_FINDINGS records
    // as producing a clean, straight segment that passes every check.
    FTrackDocument Doc = BuildCircuit(8.0, 30.0, 20.0, 30.0);
    const FClosureTarget Target = CircuitTarget(BuildTrack(Doc));
    const FClosureResult R = SolveClosure(Doc, Target, {FreeRadius(2), FreeRadius(6)});
    for (const FAuthoredSegment& A : Doc.Segments)
    {
        if (A.Kind == ESegmentKind::Arc)
        {
            assert(A.Radius > 0.0);             // sign never flipped
            assert(std::fabs(A.Radius) >= 5.0); // magnitude floored
            assert(std::isfinite(A.Radius));
        }
    }
    (void)R;

    // Two PARALLEL straights cannot span a planar gap however much the solver
    // wants them to: straight 0 and straight 8 both run along the X axis, so
    // between them they move the endpoint along one line. The solver reports
    // that rather than grinding to a plausible-looking wrong answer.
    FTrackDocument Parallel = BuildCircuit(8.0, 30.0, 20.0, 30.0);
    const FTrackDocument Untouched = Parallel;
    const FClosureResult Flat = SolveClosure(Parallel, Target, {FreeLength(0), FreeLength(8)});
    assert(!Flat.bConverged);
    assert(!Flat.bApplied);
    assert(!Flat.Message.empty());
    // Unchanged, not half-fixed. The author's own numbers are worth more than
    // an improvement they did not ask for and that still does not close.
    assert(Parallel.Segments[0].Length == Untouched.Segments[0].Length);

    // Opting in keeps the partial improvement instead.
    FClosureOptions Keep;
    Keep.bApplyOnFailure = true;
    FTrackDocument Partial = BuildCircuit(8.0, 30.0, 20.0, 30.0);
    const FClosureResult Kept = SolveClosure(Partial, Target, {FreeLength(0)}, Keep);
    assert(Kept.bApplied);
    assert(Kept.After.PositionError < Kept.Before.PositionError);

    // Bad input is rejected with a reason, not treated as an unsolvable track.
    FTrackDocument Doc2 = BuildCircuit(20.0, 30.0, 20.0, 30.0);
    const FClosureResult Bad = SolveClosure(Doc2, Target, {FreeLength(999)});
    assert(!Bad.bConverged);
    assert(Bad.Message.find("999") != std::string::npos);
    const FClosureResult None = SolveClosure(Doc2, Target, {});
    assert(!None.bConverged);
    assert(!None.Message.empty());
}

// Station, eased climb, crest, descent, pull-out, level brake run. The vertical
// slice in miniature — and the shape of every point-to-point layout that exists
// in this repo right now.
static void AppendEasedPitch(FTrackDocument& D, double PitchDelta, double PeakCurvature)
{
    const double K = PitchDelta >= 0.0 ? PeakCurvature : -PeakCurvature;
    const double L = std::fabs(PitchDelta) / PeakCurvature;
    FTrackSegment In;
    In.Length = L;
    In.PitchCurvatureEnd = K;
    D.Segments.push_back(AuthorRaw(In));
    FTrackSegment Out;
    Out.Length = L;
    Out.PitchCurvatureStart = K;
    D.Segments.push_back(AuthorRaw(Out));
}

static void TestPointToPointClosesOnHeightAlone()
{
    // "Closed" is not one question, and the vertical slice is why. A
    // station-to-brake-run ride is NOT supposed to end back at the station's
    // footprint — it is supposed to end at the station's HEIGHT. Held to a full
    // circuit target it reports a large, truthful, useless impossibility, and
    // the one number that actually matters is buried inside it.
    FTrackDocument Doc;
    Doc.Segments.push_back(AuthorStraight(20.0)); // 0: station, level
    AppendEasedPitch(Doc, 25.0 * Pi / 180.0, 0.03);
    Doc.Segments.push_back(AuthorStraight(20.0)); // 3: the climb — too short
    AppendEasedPitch(Doc, -50.0 * Pi / 180.0, 0.05);
    Doc.Segments.push_back(AuthorStraight(60.0)); // 6: the descent
    AppendEasedPitch(Doc, 25.0 * Pi / 180.0, 0.012);
    Doc.Segments.push_back(AuthorStraight(40.0)); // 9: brake run, back to level

    const FTrack Track = BuildTrack(Doc);
    const FClosureGap Circuit = MeasureClosure(Track, CircuitTarget(Track));
    const FClosureGap Height = MeasureClosure(Track, HeightTarget(Track));

    // Same track and the same measured gap vector — a different question, so a
    // different verdict. 265 m of "failure" that is not a fault, against 26 m
    // that is.
    assert(Circuit.PositionError > 200.0);
    assert(Circuit.PositionError == Height.PositionError); // the measurement did not change
    assert(Near(Height.ActiveError, std::fabs(Height.HeightError), 1e-12));
    assert(Height.ActiveError < 30.0);

    // Lengthening the climb is what fixes it, which is exactly the fix the real
    // vertical slice needed: its lift went 55 m -> 75.11 m and stopped ending
    // 8.5 m underground.
    FTrackDocument Solved = Doc;
    const FClosureResult R = SolveClosure(Solved, HeightTarget(Track), {FreeLength(3, 1.0, 400.0)});
    assert(R.bConverged);
    assert(R.bApplied);
    assert(std::fabs(R.After.HeightError) <= 1e-3);
    assert(Solved.Segments[3].Length > Doc.Segments[3].Length); // the climb grew

    // And it converged while X is still enormous, because X was never asked
    // about. Judging this on PositionError would call a solved track a failure.
    assert(R.After.PositionError > 200.0);

    std::printf("  closure: point-to-point %.2f m low -> %.1e m on height alone "
                "(climb %.1f -> %.1f m), %.0f m of X left alone as intended\n",
                R.Before.HeightError, R.After.HeightError, Doc.Segments[3].Length,
                Solved.Segments[3].Length, R.After.Position.X);
}

static void TestHorizontalStraightsCannotReachTheHeightAxis()
{
    // A limitation worth knowing before reaching for the solver, because the
    // symptom is "the solver doesn't work" rather than "the solver is being
    // asked for something those freedoms cannot do".
    //
    // A straight carries the heading it was handed, so changing its length
    // translates everything downstream along that heading and nothing else. A
    // straight that is HORIZONTAL therefore cannot move the endpoint's height,
    // no matter how many of them are freed or how far they move.
    //
    // Note the qualifier. A straight sitting after an unbalanced hill is itself
    // pitched, and that one moves height perfectly well. It is the direction
    // that matters, not the segment kind.
    FTrackDocument Doc = BuildCircuit(20.0, 30.0, 20.0, 30.0);
    // A balanced hill: pitch up, then the mirror back down. Net pitch change is
    // zero, so every straight after it stays horizontal — and the track still
    // ends higher than it started, which is the whole point.
    FTrackSegment Up, Down;
    Up.Length = Down.Length = 30.0;
    Up.PitchCurvatureStart = Up.PitchCurvatureEnd = 0.01;
    Down.PitchCurvatureStart = Down.PitchCurvatureEnd = -0.01;
    Doc.Segments.insert(Doc.Segments.begin() + 1, AuthorRaw(Down));
    Doc.Segments.insert(Doc.Segments.begin() + 1, AuthorRaw(Up));

    const FClosureTarget Target = CircuitTarget(BuildTrack(Doc));
    const FClosureGap Gap0 = MeasureClosure(BuildTrack(Doc), Target);
    assert(Gap0.HeightError > 1.0);

    // Straight 0 runs +X and straight 6 runs +Y — both horizontal, and between
    // them they span the plane but not the axis that is actually wrong.
    FTrackDocument Straights = Doc;
    const FClosureResult NoHope = SolveClosure(Straights, Target, {FreeLength(0), FreeLength(6)});
    assert(!NoHope.bConverged);
    assert(!NoHope.bApplied);
    // Not "barely moved" — the height is untouched exactly, because those
    // parameters have identically zero gradient on that row of the residual.
    assert(Near(NoHope.After.HeightError, Gap0.HeightError, 1e-9));

    // Free something vertical and the axis becomes reachable.
    FTrackDocument WithHill = Doc;
    FClosureOptions Keep;
    Keep.bApplyOnFailure = true;
    const FClosureResult Better =
        SolveClosure(WithHill, Target,
                     {FreeLength(0), FreeLength(6), FreeField(1, EClosureField::Length, 1.0, 120.0)},
                     Keep);
    assert(std::fabs(Better.After.HeightError) < std::fabs(Gap0.HeightError));

    std::printf("  closure: %.2f m of height is untouchable by horizontal straights "
                "(%.2f m after solving), reachable via the hill (%.3f m)\n",
                Gap0.HeightError, NoHope.After.HeightError, Better.After.HeightError);
}

// ------------------------------------------------------ world-referenced roll

// Build climb 45 / turn left 90 / descend 45 in one roll mode. Curvature steps
// at the joints on purpose — this is about what a roll value MEANS, not about
// smoothness.
static FTrack BuildCornerHill(ERollMode Mode)
{
    const double K = 1.0 / 25.0;
    FTrackSegment Climb, Turn, Drop;
    Climb.Length = (Pi / 4.0) / K;
    Climb.PitchCurvatureStart = Climb.PitchCurvatureEnd = K;
    Turn.Length = (Pi / 2.0) / K;
    Turn.YawCurvatureStart = Turn.YawCurvatureEnd = K;
    Drop.Length = (Pi / 4.0) / K;
    Drop.PitchCurvatureStart = Drop.PitchCurvatureEnd = -K;
    Climb.RollMode = Turn.RollMode = Drop.RollMode = Mode;

    FTrack T;
    T.AddSegment(Climb);
    T.AddSegment(Turn);
    T.AddSegment(Drop);
    return T;
}

static void TestWorldBankIsLevelWherePathRelativeIsNot()
{
    // The finding DEFERRED_DECISIONS item 1 held open: the path frame is exactly
    // rotation-minimising, so roll 0 does not mean level once the track stops
    // being planar. Climb, corner, descend — roll 0 on every segment — and the
    // rider arrives banked, purely from parallel transport around the bend.
    // The geometry is right and the word is wrong.
    const FTrack Rmf = BuildCornerHill(ERollMode::PathRelative);
    const double RmfBank = WorldBankOf(Rmf.EvaluateAt(Rmf.TotalLength()));
    assert(std::fabs(RmfBank) > 40.0 * Pi / 180.0); // nowhere near level

    // Same geometry, same typed value, measured from the horizon instead. Now
    // zero means zero. Note the PATH is untouched — roll never perturbs it —
    // so this is the same track with the rider held differently.
    const FTrack World = BuildCornerHill(ERollMode::WorldBank);
    const double WorldBank = WorldBankOf(World.EvaluateAt(World.TotalLength()));
    assert(Near(WorldBank, 0.0, 1e-9));
    assert(NearVec(Rmf.EvaluateAt(Rmf.TotalLength()).Position,
                   World.EvaluateAt(World.TotalLength()).Position, 1e-12));

    // And it holds all the way along, not just at the exit.
    for (double S = 0.0; S <= World.TotalLength(); S += 2.0)
    {
        assert(Near(WorldBankOf(World.EvaluateAt(S)), 0.0, 1e-9));
    }

    std::printf("  corner hill at roll 0: path-relative exits %.3f deg of bank, "
                "world-referenced exits %.2e\n",
                RmfBank * 180.0 / Pi, WorldBankOf(World.EvaluateAt(World.TotalLength())));
}

static void TestWorldBankAgreesWithPathRelativeOnLevelTrack()
{
    // Flat ground is where the two modes MUST agree, or adding the mode changes
    // the meaning of every track already authored. On level track the level
    // reference is the path frame, so the solved offset is identically zero.
    const double Bank = 0.35;
    FTrackSegment P = MakeArc(60.0, 30.0, Bank);
    FTrackSegment W = P;
    W.RollMode = ERollMode::WorldBank;

    FTrack Path, World;
    Path.AddSegment(P);
    World.AddSegment(W);

    for (double S = 0.0; S <= 60.0; S += 5.0)
    {
        const FTrackFrame FP = Path.EvaluateAt(S);
        const FTrackFrame FW = World.EvaluateAt(S);
        assert(Near(FP.Roll, FW.Roll, 1e-12));
        // Also pins the SIGN: a +ve world bank reads +ve on the spirit level,
        // the same sense as a +ve path-relative roll, which
        // TestBankingCancelsLateralG has already tied to banking into a left
        // turn. Get this backwards and every banked turn leans outward.
        assert(Near(WorldBankOf(FW), Bank, 1e-12));
    }
}

static void TestWorldBankThroughVerticalDegradesVisibly()
{
    // Straight up has no horizon to be level with. The requirement is that this
    // stays finite and orthonormal — but NOT that it looks fine, because it is
    // not fine: the level reference flips through the vertical, and holding a
    // world bank across that costs the rider a genuine half-turn of roll.
    const double K = 1.0 / 10.0;
    FTrackSegment Loop;
    Loop.Length = Pi / K; // half a vertical circle: up, over the top, back down
    Loop.PitchCurvatureStart = Loop.PitchCurvatureEnd = K;
    Loop.RollMode = ERollMode::WorldBank;

    FTrack World;
    World.AddSegment(Loop);
    for (double S = 0.0; S <= Loop.Length; S += 0.5)
    {
        const FTrackFrame F = World.EvaluateAt(S);
        assert(std::isfinite(F.Roll));
        assert(Near(Length(F.Up), 1.0, 1e-9)); // no NaN leaked into the frame
        assert(Near(Dot(F.Up, F.Tangent), 0.0, 1e-9));
    }

    // The visible symptom, and the reason no special-case error type is needed:
    // the roll rate is enormous exactly where the reference gives out.
    const FResolvedRollReport Bad = AnalyseResolvedRollRate(World, 20.0);
    assert(std::isfinite(Bad.PeakRateRadPerM));
    assert(Bad.PeakRateRadPerM > 1.0); // rad per METRE — nothing sane is this
    assert(Near(Bad.PeakS, Pi * 0.5 / K, 1.0)); // at the top, where it flips

    // Authored path-relative, the identical geometry costs nothing at all.
    Loop.RollMode = ERollMode::PathRelative;
    FTrack Path;
    Path.AddSegment(Loop);
    const FResolvedRollReport Good = AnalyseResolvedRollRate(Path, 20.0);
    assert(Near(Good.PeakRateRadPerM, 0.0, 1e-9));

    std::printf("  world bank through vertical: peak %.1f rad/m at S=%.1f m; "
                "path-relative %.1e rad/m\n",
                Bad.PeakRateRadPerM, Bad.PeakS, Good.PeakRateRadPerM);
}

static void TestResolvedRollRateSeesWhatTheAuthoredRateCannot()
{
    // One segment, one constant bank, zero authored roll rate. The segment-list
    // report is structurally unable to see anything here — and the rider is
    // rotating the entire way, because a climbing turn twists the path frame
    // underneath the constant bank that is holding them level.
    //
    // Same shape of blind spot as felt-G-versus-roll-rate, one level up: there,
    // the model had no roll term; here, the data has no geometry.
    FTrackSegment S = MakeHelix(25.0, 20.0 * Pi / 180.0, 0.75, 0.0);
    S.RollMode = ERollMode::WorldBank;
    S.RollStart = S.RollEnd = 0.5;

    const FRollRateReport Authored = AnalyseRollRate({S}, 20.0);
    assert(Authored.PeakRateRadPerM == 0.0); // nothing was typed, nothing is seen

    FTrack T;
    T.AddSegment(S);
    const FResolvedRollReport Resolved = AnalyseResolvedRollRate(T, 20.0);
    assert(Resolved.PeakRateRadPerM > 1e-3);
    assert(Resolved.HeadSnapG > 0.0);

    // The bank itself is held, which is what makes the rotation invisible in
    // the data: the number never changes, the frame under it does.
    for (double A = 0.0; A <= S.Length; A += S.Length / 8.0)
    {
        assert(Near(WorldBankOf(T.EvaluateAt(A)), 0.5, 1e-6));
    }

    std::printf("  constant world bank on a helix: authored rate %.1e rad/m, "
                "resolved %.4f rad/m (%.1f deg/s, %.3f g head snap)\n",
                Authored.PeakRateRadPerM, Resolved.PeakRateRadPerM,
                Resolved.PeakRateDegPerSec, Resolved.HeadSnapG);
}

static void TestMixedRollModesAreReportedNotGuessed()
{
    // Two roll values measured from different references cannot be subtracted.
    // Whether the resolved roll actually steps at this joint depends on the
    // frame there, which a data-only check does not have — so it says so.
    std::vector<FTrackSegment> Segs = {MakeArc(40.0, 30.0, 0.3), MakeArc(40.0, 30.0, 0.3)};
    Segs[1].RollMode = ERollMode::WorldBank;

    FTrack T;
    T.AddSegment(Segs[0]);
    T.AddSegment(Segs[1]);
    assert(!T.IsCurvatureContinuous()); // refuses rather than guessing

    const std::vector<FTrackDiagnostic> D = ValidateTrack(Segs);
    assert(!HasErrors(D)); // buildable — this is a warning, not a rejection
    assert(HasIssue(D, ETrackIssue::RollModeMixed, 0));
    assert(!HasIssue(D, ETrackIssue::RollStep, 0));     // skipped, not fabricated
    assert(!HasIssue(D, ETrackIssue::RollRateStep, 0)); // likewise

    // Matching modes: the ordinary checks come back, and this track is clean.
    Segs[0].RollMode = ERollMode::WorldBank;
    const std::vector<FTrackDiagnostic> Same = ValidateTrack(Segs);
    assert(!HasIssue(Same, ETrackIssue::RollModeMixed, 0));
    assert(!HasIssue(Same, ETrackIssue::RollStep, 0));
    FTrack Clean;
    Clean.AddSegment(Segs[0]);
    Clean.AddSegment(Segs[1]);
    assert(Clean.IsCurvatureContinuous());
}

// -------------------------------------------------------------------- helix

static void TestHelixIsActuallyAHelix()
{
    // PHASE0_FINDINGS recorded the helix as a REPRESENTATION GAP: authored the
    // obvious way (constant radius + constant climb) the old vocabulary gave a
    // tilted flat circle, not a helix. Constant torsion closes that gap, and
    // this is the test that says so — it checks the geometry, not the fields.
    const double R = 20.0;
    const double Alpha = 15.0 * Pi / 180.0;
    const double Turns = 2.0;

    // The axis is inherited from the incoming frame, so pitch up to the climb
    // angle first. Entering level would give a helix about a tilted axis.
    FTrack Track;
    FTrackSegment PitchUp;
    PitchUp.Length = Alpha / 0.02;
    PitchUp.PitchCurvatureStart = PitchUp.PitchCurvatureEnd = 0.02;
    Track.AddSegment(PitchUp);
    const double Entry = Track.TotalLength();
    Track.AddSegment(MakeHelix(R, Alpha, Turns));

    const FTrackFrame E = Track.EvaluateAt(Entry);
    assert(Near(std::asin(E.Tangent.Z), Alpha, 1e-6));

    // A true vertical-axis helix keeps every point exactly R from one vertical
    // line, and gains height linearly with arc length. A tilted circle fails
    // both; a helix about a tilted axis fails the first.
    const FVec3 Axis = E.Position + E.PathLateral * R; // PathLateral is horizontal here
    const double HelixLength = Track.TotalLength() - Entry;
    double RMin = 1e9, RMax = -1e9, ClimbMin = 1e9, ClimbMax = -1e9;

    FTrackFrame Walk = E;
    for (int i = 1; i <= 400; ++i)
    {
        const double Prev = HelixLength * (i - 1) / 400.0;
        const double U = HelixLength * i / 400.0;
        Walk = Track.AdvanceFrom(Walk, Entry + Prev, Entry + U);
        const FVec3 D = Walk.Position - Axis;
        const double Radial = std::sqrt(D.X * D.X + D.Y * D.Y);
        RMin = std::fmin(RMin, Radial);
        RMax = std::fmax(RMax, Radial);
        const double Climb = (Walk.Position.Z - E.Position.Z) / U;
        ClimbMin = std::fmin(ClimbMin, Climb);
        ClimbMax = std::fmax(ClimbMax, Climb);
    }
    assert(Near(RMin, R, 1e-4) && Near(RMax, R, 1e-4));
    assert(RMax - RMin < 1e-4);                    // constant radius, not a spiral
    assert(Near(ClimbMin, std::sin(Alpha), 1e-6)); // constant climb, not a circle
    assert(ClimbMax - ClimbMin < 1e-6);

    // Closed forms: rise per turn is 2*pi*R*tan(a), arc length 2*pi*R/cos(a).
    const FTrackFrame X = Track.EvaluateAt(Track.TotalLength());
    assert(Near(X.Position.Z - E.Position.Z, 2.0 * Pi * R * Turns * std::tan(Alpha), 1e-3));
    assert(Near(HelixLength, Turns * 2.0 * Pi * R / std::cos(Alpha), 1e-9));
    // And it leaves at the angle it entered — a helix does not change its climb.
    assert(Near(std::asin(X.Tangent.Z), Alpha, 1e-6));
}

static void TestHelixHandednessAndDegenerateCases()
{
    // A RIGHT-hand helix must mirror the left one, not merely differ. Torsion
    // sign is the easiest thing to get backwards and the hardest to see.
    const double R = 15.0, Alpha = 20.0 * Pi / 180.0;
    const FTrackSegment L = MakeHelix(R, Alpha, 1.0);
    const FTrackSegment Rt = MakeHelix(-R, Alpha, 1.0);
    assert(L.YawCurvatureStart > 0.0 && Rt.YawCurvatureStart < 0.0);
    assert(Near(L.Torsion, -Rt.Torsion, 1e-15));
    assert(Near(L.Length, Rt.Length, 1e-12));
    // Descending reverses the twist but not the turn direction.
    const FTrackSegment Down = MakeHelix(R, -Alpha, 1.0);
    assert(Near(Down.YawCurvatureStart, L.YawCurvatureStart, 1e-15));
    assert(Near(Down.Torsion, -L.Torsion, 1e-15));

    // Zero climb degenerates to a plain arc: no torsion, curvature 1/R.
    const FTrackSegment Flat = MakeHelix(R, 0.0, 1.0);
    assert(Near(Flat.Torsion, 0.0, 1e-15));
    assert(Near(Flat.YawCurvatureStart, 1.0 / R, 1e-15));
    assert(Near(Flat.Length, 2.0 * Pi * R, 1e-9));

    // Everything without torsion is bit-identical to the old behaviour.
    double Yaw = 0.0, Pitch = 0.0;
    const FTrackSegment Cl = MakeClothoid(20.0, 0.0, 1.0 / 30.0);
    CurvatureAt(Cl, 10.0, Yaw, Pitch);
    assert(Near(Yaw, 0.5 / 30.0, 1e-15) && Near(Pitch, 0.0, 1e-15));

    std::printf("  helix: true vertical axis, closed forms matched, handedness mirrored\n");
}

static void TestHelixExitIsNotContinuousWithAPlainArc()
{
    // The consequence authors need to know about: torsion ROTATES the curvature
    // vector by tau*L (2*pi*sin(a) per turn), so a helix does not generally end
    // on pure yaw. Bolting a plain arc onto the exit is a real curvature step,
    // and IsCurvatureContinuous has to see it — which it only does because the
    // check goes through CurvatureAt rather than reading the End fields.
    const double R = 20.0, Alpha = 15.0 * Pi / 180.0;
    const FTrackSegment H = MakeHelix(R, Alpha, 1.0);

    double EndYaw = 0.0, EndPitch = 0.0;
    CurvatureAt(H, H.Length, EndYaw, EndPitch);
    assert(std::fabs(EndPitch) > 1e-3);              // genuinely rotated out of pure yaw
    assert(Near(H.Torsion * H.Length, 2.0 * Pi * std::sin(Alpha), 1e-12));

    // Raw fields would say "same curvature, continuous". Geometry says no.
    FTrack Bad;
    Bad.AddSegment(H);
    Bad.AddSegment(MakeArc(30.0, R));
    assert(Near(H.YawCurvatureEnd, 1.0 / R * std::cos(Alpha) * std::cos(Alpha), 1e-15));
    assert(!Bad.IsCurvatureContinuous());

    // Matching the ROTATED curvature is continuous.
    FTrack Good;
    Good.AddSegment(H);
    FTrackSegment Match;
    Match.Length = 30.0;
    Match.YawCurvatureStart = Match.YawCurvatureEnd = EndYaw;
    Match.PitchCurvatureStart = Match.PitchCurvatureEnd = EndPitch;
    Good.AddSegment(Match);
    assert(Good.IsCurvatureContinuous());
    std::printf("  helix exit rotates curvature by %.4f rad — continuity check sees it\n",
                H.Torsion * H.Length);
}

static void TestTheTwoBuildPathsCannotDiverge()
{
    // BuildTrack and BuildSegments are SEPARATE loops (TrackIO.h) that happen to
    // agree because both just map BuildSegment over the rows. Nothing enforced
    // that, and the actor uses BOTH, six lines apart: TUCoasterRide.cpp validates
    // BuildSegments(Doc) and then RIDES ::BuildTrack(Doc).
    //
    // So the day anything neighbour-aware enters one of them — chaining a helix
    // exit is the live candidate — and not the other, the validator reports a
    // clean track and the train rides a stepped one. That is the exact inversion
    // of "report, never repair", and it would be invisible: every existing suite
    // passes either way, because nothing compared the two paths until this.
    FTrackDocument Doc;
    Doc.HeartlineHeight = 1.1;

    auto Add = [&Doc](FAuthoredSegment A) { Doc.Segments.push_back(A); };
    { FAuthoredSegment A; A.Kind = ESegmentKind::Straight; A.Length = 20.0; Add(A); }
    { FAuthoredSegment A; A.Kind = ESegmentKind::Clothoid; A.Length = 26.0;
      A.CurvatureStart = 0.0; A.CurvatureEnd = 1.0 / 32.0; A.RollEndDegrees = 40.0; Add(A); }
    { FAuthoredSegment A; A.Kind = ESegmentKind::Arc; A.Length = 55.0; A.Radius = 32.0;
      A.RollStartDegrees = A.RollEndDegrees = 40.0; Add(A); }
    { FAuthoredSegment A; A.Kind = ESegmentKind::Helix; A.Radius = 20.0;
      A.ClimbAngleDegrees = -11.0; A.Turns = 2.0; Add(A); }
    { FAuthoredSegment A; A.Kind = ESegmentKind::Raw; A.Length = 30.0;
      A.RawSegment.Length = 30.0; A.RawSegment.PitchCurvatureEnd = 0.01; Add(A); }

    const FTrack ViaBuildTrack = BuildTrack(Doc);
    const std::vector<FTrackSegment> ViaBuildSegments = BuildSegments(Doc);

    assert(ViaBuildTrack.NumSegments() == ViaBuildSegments.size());

    FTrack Rebuilt(Doc.HeartlineHeight);
    for (const FTrackSegment& S : ViaBuildSegments)
    {
        Rebuilt.AddSegment(S);
    }

    // Same length, same continuity verdict, same endpoint. Exactly, not nearly:
    // both paths run the identical BuildSegment on the identical rows, so any
    // difference at all is a divergence and not a tolerance question.
    assert(Rebuilt.NumSegments() == ViaBuildTrack.NumSegments());
    assert(Rebuilt.TotalLength() == ViaBuildTrack.TotalLength());
    assert(Rebuilt.IsCurvatureContinuous(1e-9) == ViaBuildTrack.IsCurvatureContinuous(1e-9));

    const FTrackFrame A = ViaBuildTrack.EvaluateAt(ViaBuildTrack.TotalLength());
    const FTrackFrame B = Rebuilt.EvaluateAt(Rebuilt.TotalLength());
    assert(A.Position.X == B.Position.X);
    assert(A.Position.Y == B.Position.Y);
    assert(A.Position.Z == B.Position.Z);

    std::printf("  both build paths agree over %zu segments, %.3f m, endpoint identical\n",
                ViaBuildTrack.NumSegments(), ViaBuildTrack.TotalLength());
}

// ------------------------------------------------------------ cross-section

static void TestCrossSectionRidesTheFrame()
{
    // Through a banked inverted loop, the geometry that must hold regardless of
    // orientation: the rails stay exactly Gauge apart, they straddle the rail
    // centre symmetrically, and the whole section stays square to the track.
    FTrack Track;
    FTrackSegment Loop;
    Loop.Length = 2.0 * Pi * 8.0;
    Loop.PitchCurvatureStart = Loop.PitchCurvatureEnd = 1.0 / 8.0;
    Loop.RollStart = 0.0;
    Loop.RollEnd = 0.9;
    Track.AddSegment(Loop);

    const FTrackProfile Profile;
    for (int i = 0; i <= 200; ++i)
    {
        const double S = Track.TotalLength() * i / 200.0;
        const FTrackFrame F = Track.EvaluateAt(S);
        const FTrackCrossSection X = CrossSectionAt(F, Track.GetHeartlineHeight(), Profile);

        // Gauge holds to roundoff because the frame is orthonormal by
        // construction — no renormalisation anywhere in the cross-section.
        assert(Near(Length(X.LeftRail - X.RightRail), Profile.Gauge, 1e-12));
        assert(Near(Length(X.LeftRail - X.RailCentre), Profile.Gauge * 0.5, 1e-12));
        assert(Near(Length(X.RightRail - X.RailCentre), Profile.Gauge * 0.5, 1e-12));

        // Square to the track: the rail axis is perpendicular to travel.
        assert(std::fabs(Dot(X.LeftRail - X.RightRail, F.Tangent)) < 1e-12);

        // Spine hangs below the rails along the banked up-axis, not world down.
        assert(Near(Dot(X.RailCentre - X.SpineCentre, F.Up), Profile.SpineDrop, 1e-12));

        // And the section agrees with the track's own rail centreline.
        assert(NearVec(X.RailCentre, Track.RailCentreAt(S), 1e-9));
    }
    std::printf("  cross-section rides the frame through an inverted banked loop\n");
}

static void TestCrossSectionSidednessAndWidth()
{
    // +Lateral is the rider's LEFT. On an unbanked left turn the left rail is
    // the inside one, so it must sit nearer the centre of the turn. Getting
    // this backwards mirrors the track while leaving it self-consistent.
    FTrack Track;
    Track.AddSegment(MakeArc(40.0, 25.0)); // +radius = left turn, no bank
    const FTrackProfile Profile;
    const FTrackCrossSection X = CrossSectionAt(Track, 20.0, Profile);
    const FTrackFrame F = Track.EvaluateAt(20.0);

    // Turn centre lies along +PathLateral from the rail centre.
    const FVec3 Centre = X.RailCentre + F.PathLateral * 25.0;
    assert(Length(X.LeftRail - Centre) < Length(X.RightRail - Centre));

    assert(Near(TrackWidth(Profile), Profile.Gauge + Profile.RailDiameter, 1e-15));
    std::printf("  left rail is inside a left turn; swept width %.3f m\n", TrackWidth(Profile));
}

int main()
{
    TestValidationCatchesTheNaNThatGeometryCannot();
    TestValidationRejectsAndWarnsWithoutRepairing();
    TestValidationLocatesJointStepsAndStaysQuietWhenClean();
    TestRollRateIsVisibleWhereFeltGIsNot();
    TestWorldBankIsLevelWherePathRelativeIsNot();
    TestWorldBankAgreesWithPathRelativeOnLevelTrack();
    TestWorldBankThroughVerticalDegradesVisibly();
    TestResolvedRollRateSeesWhatTheAuthoredRateCannot();
    TestMixedRollModesAreReportedNotGuessed();
    TestTrackRoundTripsThroughJson();
    TestDeviceLayerSurvivesTheFile();
    TestOneEditIsOneLineOfDiff();
    TestDefaultsAreOmittedSoNewFieldsCostNothing();
    TestUnknownFieldsLoadButUnknownGeometryDoesNot();
    TestMalformedFilesAreRejectedNotAbsorbed();
    TestAuthoredParametersSurviveWhereDerivedOnesWouldNot();
    TestUndoRedoRestoresGeometryNotJustFields();
    TestCommittingNothingIsNotAnUndoStep();
    TestTypingIsOneUndoStepNotFive();
    TestRedoBranchIsDiscardedAndSavedStateTracksIt();
    TestDepthCapDropsOldestAndNeverClaimsCleanFalsely();
    TestTrackPassingThroughItselfIsVisibleToNothingElse();
    TestChainCurvatureIsWhatMakesTorsionComposable();
    TestSymmetricCircuitAlreadyCloses();
    TestClosesTheGapTheFindingsMeasured();
    TestHeightGapIsCalledOutSeparately();
    TestSolverRefusesRatherThanBreakingTheTrack();
    TestPointToPointClosesOnHeightAlone();
    TestHorizontalStraightsCannotReachTheHeightAxis();
    TestHelixIsActuallyAHelix();
    TestHelixHandednessAndDegenerateCases();
    TestHelixExitIsNotContinuousWithAPlainArc();
    TestTheTwoBuildPathsCannotDiverge();
    TestCrossSectionRidesTheFrame();
    TestCrossSectionSidednessAndWidth();
    TestStraightIsStraight();
    TestArcMatchesExactCircle();
    TestClothoidTurnsByIntegralOfCurvature();
    TestClothoidRemovesTheCurvatureStep();
    TestBankingCancelsLateralG();
    TestHeartlineIsWhatBankingRotatesAround();
    TestFrameStaysOrthonormal();
    TestVerticalLoopGeometryAndG();
    TestAirtimeThreshold();
    TestGravityIsWorldReferencedNotPathReferenced();
    TestClothoidEndpointMatchesFresnel();
    TestRollInterpolatesAcrossSegment();
    TestDegenerateSegmentsRejected();
    TestAdvanceFromAgreesWithEvaluateAt();
    TestEvaluateClampsAndSpansSegments();
    std::printf("All track spline tests passed.\n");
    return 0;
}
