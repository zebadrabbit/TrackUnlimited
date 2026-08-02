// Unit tests for the Phase 0 track spline prototype.
// Build & run:  clang++ -std=c++17 -Wall -Wextra -o test_trackspline test_trackspline.cpp && ./test_trackspline

#include "TrackSpline.h"
#include "TrackProfile.h"
#include "TrackValidate.h"

#include <cassert>
#include <cstdio>

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
    TestHelixIsActuallyAHelix();
    TestHelixHandednessAndDegenerateCases();
    TestHelixExitIsNotContinuousWithAPlainArc();
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
