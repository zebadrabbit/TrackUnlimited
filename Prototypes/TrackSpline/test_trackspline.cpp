// Unit tests for the Phase 0 track spline prototype.
// Build & run:  clang++ -std=c++17 -Wall -Wextra -o test_trackspline test_trackspline.cpp && ./test_trackspline

#include "TrackSpline.h"

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

int main()
{
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
