// Unit tests for the Phase 0 train physics prototype.
// Build & run:  clang++ -std=c++17 -Wall -Wextra -o test_trainphysics test_trainphysics.cpp && ./test_trainphysics

#include "TrainPhysics.h"
#include "RideProfile.h"

#include <cassert>
#include <cstdio>

static const double Pi = 3.14159265358979323846;

static bool Near(double A, double B, double Tol = 1e-3)
{
    return std::fabs(A - B) <= Tol;
}

static FTrainConfig Frictionless()
{
    FTrainConfig C;
    C.RollingResistance = 0.0;
    C.DragK = 0.0;
    return C;
}

static FTrack MakeLevelStraight(double Length)
{
    FTrack Track;
    Track.AddSegment(MakeStraight(Length));
    return Track;
}

// A full vertical loop: 2*pi*R of constant pitch curvature returns to its own
// start, so height in equals height out.
static FTrack MakeFullLoop(double R)
{
    FTrack Track;
    FTrackSegment Loop;
    Loop.Length = 2.0 * Pi * R;
    Loop.PitchCurvatureStart = Loop.PitchCurvatureEnd = 1.0 / R;
    Track.AddSegment(Loop);
    return Track;
}

// Every run loop is bounded, so a regression that stalls a train fails on a
// line number instead of hanging with no output. The budgets are generous but
// finite — EvaluateAt is O(track length), so an unbounded loop is minutes of
// silence, not an obvious hang.
static double RunToEnd(FTrain& Train, double Dt, int MaxSteps = 200000)
{
    int Steps = 0;
    while (!Train.IsAtEnd() && Steps < MaxSteps)
    {
        Train.Step(Dt);
        ++Steps;
    }
    assert(Steps < MaxSteps);
    return Steps * Dt;
}

static void RunUntilDistance(FTrain& Train, double Dt, double TargetS, int MaxSteps = 200000)
{
    int Steps = 0;
    while (Train.GetDistance() < TargetS && Steps < MaxSteps)
    {
        Train.Step(Dt);
        ++Steps;
    }
    assert(Steps < MaxSteps);
}

static void TestGravityIsExactEnergyExchange()
{
    const double R = 8.0;
    const double V0 = 25.0;
    const FTrack Loop = MakeFullLoop(R);

    // Apex: all the height is paid for out of kinetic energy.
    FTrain Train(Loop, Frictionless());
    Train.Place(0.0, V0);
    const double ApexS = Pi * R;
    RunUntilDistance(Train, 1.0 / 600.0, ApexS);
    const double ExpectedApex = std::sqrt(V0 * V0 - 2.0 * GravityMs2 * 2.0 * R);
    assert(Near(Train.GetSpeed(), ExpectedApex, 1e-2));

    // ...and all of it comes back on the way down. This is the assertion a
    // force-integrating model fails: it would arrive with a slightly different
    // speed than it left, every lap.
    RunToEnd(Train, 1.0 / 600.0);
    assert(Near(Train.GetSpeed(), V0, 1e-6));
}

static void TestEnergyResultIsStepSizeIndependent()
{
    const FTrack Loop = MakeFullLoop(8.0);

    // A coarse tick and a fine one must agree, because gravity's contribution
    // is read off the track rather than integrated. 1/30 s is a deliberately
    // bad timestep; the point is that it does not matter here.
    double Speeds[2];
    const double Dts[2] = {1.0 / 30.0, 1.0 / 300.0};
    for (int i = 0; i < 2; ++i)
    {
        FTrain Train(Loop, Frictionless());
        Train.Place(0.0, 25.0);
        RunToEnd(Train, Dts[i]);
        Speeds[i] = Train.GetSpeed();
    }
    assert(Near(Speeds[0], Speeds[1], 1e-6));
}

static void TestRollingFrictionStoppingDistance()
{
    // On level track the normal load is exactly 1 g, so deceleration is a
    // constant Crr*g and the stop is a closed form: v0^2 / (2*Crr*g).
    // Crr is exaggerated well past a real steel coaster's to keep the track
    // short — EvaluateAt is O(track length), so a 500 m test track is slow.
    FTrainConfig Config;
    Config.RollingResistance = 0.05;
    Config.DragK = 0.0;

    const double V0 = 5.0;
    const double Expected = (V0 * V0) / (2.0 * Config.RollingResistance * GravityMs2);

    const FTrack Track = MakeLevelStraight(Expected + 15.0);
    FTrain Train(Track, Config);
    Train.Place(0.0, V0);

    for (int i = 0; i < 20000 && Train.GetSpeed() > 0.0; ++i)
    {
        Train.Step(1.0 / 240.0);
    }
    assert(Train.GetSpeed() == 0.0);
    assert(Near(Train.GetDistance(), Expected, 0.05));
}

static void TestDragAloneDecaysExponentiallyWithDistance()
{
    // Drag only, level track: v dv/dx = -k v^2, so v(x) = v0 * exp(-k*x).
    FTrainConfig Config;
    Config.RollingResistance = 0.0;
    Config.DragK = 0.0018;

    const double V0 = 30.0;
    const double Distance = 100.0;

    const FTrack Track = MakeLevelStraight(Distance);
    FTrain Train(Track, Config);
    Train.Place(0.0, V0);
    RunToEnd(Train, 1.0 / 500.0);

    assert(Near(Train.GetSpeed(), V0 * std::exp(-Config.DragK * Distance), 1e-2));
}

static void TestFrictionScalesWithNormalLoad()
{
    // A banked turn costs more speed than a straight of the same length at the
    // same entry speed, because rolling resistance follows the normal load and
    // a banked turn loads the wheels at sqrt(1 + (v^2/gR)^2) g. A model that
    // assumed a flat 1 g would report these two as identical.
    //
    // Both tracks are dead level, so gravity contributes exactly nothing and
    // the entire difference is load.
    FTrainConfig Config;
    Config.RollingResistance = 0.02;
    Config.DragK = 0.0;

    const double V0 = 20.0;
    const double Length = 60.0;
    const double R = 30.0;

    const FTrack Level = MakeLevelStraight(Length);
    FTrain Straight(Level, Config);
    Straight.Place(0.0, V0);
    RunToEnd(Straight, 1.0 / 500.0);

    FTrack Curve;
    Curve.AddSegment(MakeArc(Length, R, std::atan((V0 * V0) / (GravityMs2 * R))));
    FTrain Banked(Curve, Config);
    Banked.Place(0.0, V0);
    RunToEnd(Banked, 1.0 / 500.0);

    assert(Near(Curve.EvaluateAt(Curve.TotalLength()).Position.Z, 0.0, 1e-9));
    assert(Banked.GetSpeed() < Straight.GetSpeed());

    const double StraightLoss = V0 - Straight.GetSpeed();
    const double BankedLoss = V0 - Banked.GetSpeed();

    // Two-sided, because a one-sided bound cannot tell the right load model
    // from a bigger wrong one: dropping the sqrt, or summing |lat|+|vert|
    // instead of taking the hypotenuse, both push this ratio well past 1.65.
    // The ratio shifts by only 0.0006 between dt = 1/30 and 1/2000, so the
    // tolerance is about the model, not the timestep.
    assert(Near(BankedLoss / StraightLoss, 1.6515, 0.01));

    // The same turn UNBANKED must cost exactly the same. Banking rotates the
    // rider around the heartline; it does not change how hard the wheels are
    // pressed into the rail. This is what distinguishes the hypotenuse from
    // using the vertical component alone, which a banked-only test cannot see
    // because banking nulls the lateral term.
    FTrack Flat;
    Flat.AddSegment(MakeArc(Length, R));
    FTrain Unbanked(Flat, Config);
    Unbanked.Place(0.0, V0);
    RunToEnd(Unbanked, 1.0 / 500.0);
    assert(Near(Unbanked.GetSpeed(), Banked.GetSpeed(), 1e-12));
}

static void TestZoneHoldsTargetAgainstLosses()
{
    // A powered section has to spend part of its authority cancelling friction
    // and drag, not just closing the speed gap. Ignore that and the train
    // oscillates around the target instead of sitting on it.
    FTrainConfig Config;
    Config.RollingResistance = 0.02;
    Config.DragK = 0.001;

    const FTrack Track = MakeLevelStraight(80.0);
    FTrain Train(Track, Config);
    assert(Train.AddZone(MakeLift(0.0, 80.0, 5.0, 3.0)));
    Train.Place(0.0, 1.0);

    RunUntilDistance(Train, 1.0 / 60.0, 60.0);
    assert(Near(Train.GetSpeed(), 5.0, 1e-9));
    // Holding a steady speed on the level means no fore-aft load at all.
    assert(Near(Train.GetTangentialG(), 0.0, 1e-9));
}

static void TestOverlappingZonesTakeTheMostRestrictive()
{
    // Overlap is an authoring error, but a ride control system must fail toward
    // the slower answer rather than toward whichever zone was added last.
    const FTrack Track = MakeLevelStraight(100.0);
    FTrain Train(Track, Frictionless());

    assert(Train.AddZone(MakeBrake(0.0, 100.0, 5.0, 4.0)));   // slow it down
    assert(Train.AddZone(MakeLaunch(0.0, 100.0, 30.0, 8.0))); // added later, wants it fast

    Train.Place(0.0, 20.0);
    RunUntilDistance(Train, 1.0 / 60.0, 80.0);
    assert(Near(Train.GetSpeed(), 5.0, 1e-9));
}

static void TestLiftHoldsChainSpeedBothWays()
{
    const FTrack Track = MakeLevelStraight(100.0);

    // Slow train gets pulled up to chain speed.
    FTrain Slow(Track, Frictionless());
    Slow.AddZone(MakeLift(0.0, 100.0, 5.0));
    Slow.Place(0.0, 1.0);
    for (int i = 0; i < 300; ++i) { Slow.Step(1.0 / 60.0); }
    assert(Near(Slow.GetSpeed(), 5.0, 1e-9));

    // Fast train gets held back to it — a chain does both.
    FTrain Fast(Track, Frictionless());
    Fast.AddZone(MakeLift(0.0, 100.0, 5.0));
    Fast.Place(0.0, 12.0);
    for (int i = 0; i < 300; ++i) { Fast.Step(1.0 / 60.0); }
    assert(Near(Fast.GetSpeed(), 5.0, 1e-9));
}

static void TestBrakeOnlySlows()
{
    const FTrack Track = MakeLevelStraight(150.0);

    FTrain Train(Track, Frictionless());
    // 8 m/s^2 of bite brings 25 m/s down to 6 in about 37 m, comfortably
    // inside the 70 m brake run.
    Train.AddZone(MakeBrake(40.0, 110.0, 6.0, 8.0));
    Train.Place(0.0, 25.0);

    // Before the brake run, nothing touches it.
    RunUntilDistance(Train, 1.0 / 120.0, 35.0);
    assert(Near(Train.GetSpeed(), 25.0, 1e-9));

    // Through it, down to release speed and held there.
    RunUntilDistance(Train, 1.0 / 120.0, 105.0);
    assert(Near(Train.GetSpeed(), 6.0, 1e-9));

    // A brake never pushes: past the zone it coasts, it does not speed back up.
    const double Released = Train.GetSpeed();
    RunToEnd(Train, 1.0 / 120.0);
    assert(Train.GetSpeed() <= Released + 1e-9);
}

static void TestLaunchProducesForeAftG()
{
    const FTrack Track = MakeLevelStraight(120.0);

    FTrain Train(Track, Frictionless());
    Train.AddZone(MakeLaunch(0.0, 100.0, 40.0, 12.0));
    Train.Place(0.0, 0.5);

    Train.Step(1.0 / 60.0);
    // 12 m/s^2 of push is about 1.22 G into the seat back.
    assert(Near(Train.GetTangentialG(), 12.0 / GravityMs2, 1e-6));

    RunUntilDistance(Train, 1.0 / 240.0, 90.0);
    assert(Near(Train.GetSpeed(), 40.0, 1e-9));
    // Once it is up to speed the push stops, and so does the fore-aft load.
    assert(Near(Train.GetTangentialG(), 0.0, 1e-9));
}

static void TestFreeRollingFeelsNoForeAftForce()
{
    // The discriminating case for the fore-aft convention. A train coasting
    // down a slope is being accelerated BY gravity, not pushed against it, so
    // the rider feels nothing fore-aft — same reason free fall feels weightless.
    // Report raw dv/dt instead and this reads 0.7 G on a 45-degree drop.
    FTrack Track;
    FTrackSegment PitchDown;
    PitchDown.Length = (0.25 * Pi) / 0.02; // 45 degrees down
    PitchDown.PitchCurvatureStart = PitchDown.PitchCurvatureEnd = -0.02;
    Track.AddSegment(PitchDown);
    Track.AddSegment(MakeStraight(40.0));

    FTrain Train(Track, Frictionless());
    Train.Place(0.0, 8.0);

    RunUntilDistance(Train, 1.0 / 500.0, PitchDown.Length + 20.0);

    // Genuinely on a steep descent...
    assert(Near(Train.GetFrame().Tangent.Z, -std::sin(0.25 * Pi), 1e-6));
    // ...gaining speed fast...
    assert(Train.GetSpeed() > 20.0);
    // ...and feeling no fore-aft load at all.
    assert(Near(Train.GetTangentialG(), 0.0, 1e-3));
}

// A track that climbs at a constant Grade radians after a short easement, so a
// zone can be tested against gravity rather than only on the level.
static FTrack MakeConstantGrade(double Grade, double StraightLength, double& OutStraightStart)
{
    FTrack Track;
    FTrackSegment Pitch;
    Pitch.Length = std::fabs(Grade) / 0.02;
    Pitch.PitchCurvatureStart = Pitch.PitchCurvatureEnd = (Grade >= 0.0 ? 0.02 : -0.02);
    Track.AddSegment(Pitch);
    Track.AddSegment(MakeStraight(StraightLength));
    OutStraightStart = Pitch.Length;
    return Track;
}

static void TestDepartsFromRestOnAGradient()
{
    // A cart placed at the top of a drop at rest is the vertical slice's most
    // obvious first experiment. With position integrated from entry speed alone
    // it never moves: no travel means no height change means no speed, forever.
    const double Grade = -0.5235987755982988; // 30 degrees down
    double Start = 0.0;
    const double Run = 50.0;
    const FTrack Track = MakeConstantGrade(Grade, Run, Start);

    FTrain Train(Track, Frictionless());
    Train.Place(Start + 1.0, 0.0);
    const double Z0 = Train.GetFrame().Position.Z;

    RunToEnd(Train, 1.0 / 240.0);

    assert(Train.GetDistance() > Start + 45.0);
    // All of the height it dropped came back as speed: v = sqrt(2*g*drop).
    const double Drop = Z0 - Train.GetFrame().Position.Z;
    assert(Drop > 20.0);
    assert(Near(Train.GetSpeed(), std::sqrt(2.0 * GravityMs2 * Drop), 1e-6));
}

static void TestUnderpoweredLiftCannotCreateEnergy()
{
    // A chain rated below g*sin(grade) physically cannot hold the train on that
    // grade, let alone pull it up. Applying the zone as a per-time speed clamp
    // after a per-distance energy step made it climb anyway — inventing energy
    // every tick and arriving at the crest.
    const double Grade = 0.5235987755982988; // 30 degrees up; needs 4.903 m/s^2
    double Start = 0.0;
    const FTrack Track = MakeConstantGrade(Grade, 50.0, Start);

    FTrain Train(Track, Frictionless());
    assert(Train.AddZone(MakeLift(Start, Start + 50.0, 5.0, 3.0))); // only 3.0
    Train.Place(Start + 1.0, 0.0);
    const double Z0 = Train.GetFrame().Position.Z;

    for (int i = 0; i < 1200; ++i) { Train.Step(1.0 / 60.0); }

    assert(Train.GetSpeed() == 0.0);
    assert(Near(Train.GetFrame().Position.Z, Z0, 1e-9)); // gained no height at all
}

static void TestAdequateLiftHoldsChainSpeedUphill()
{
    // The same hill with a chain that can out-pull gravity: it reaches chain
    // speed and holds it exactly, spending 4.903 m/s^2 of its 12 on gravity.
    const double Grade = 0.5235987755982988;
    double Start = 0.0;
    const FTrack Track = MakeConstantGrade(Grade, 50.0, Start);

    FTrain Train(Track, Frictionless());
    assert(Train.AddZone(MakeLift(Start, Start + 50.0, 5.0, 12.0)));
    Train.Place(Start + 1.0, 0.0);
    const double Z0 = Train.GetFrame().Position.Z;

    for (int i = 0; i < 800; ++i) { Train.Step(1.0 / 60.0); }

    assert(Near(Train.GetSpeed(), 5.0, 1e-9));
    assert(Train.GetFrame().Position.Z > Z0 + 15.0);
    // Holding a constant speed up a 30-degree grade is 0.5 G of fore-aft load.
    assert(Near(Train.GetTangentialG(), std::sin(Grade), 1e-6));
}

static void TestAddZoneRejectsMalformedZones()
{
    const FTrack Track = MakeLevelStraight(100.0);
    FTrain Train(Track, Frictionless());

    assert(!Train.AddZone({50.0, 20.0, 5.0, 1.0, 1.0}));  // inverted span
    assert(!Train.AddZone({10.0, 10.0, 5.0, 1.0, 1.0}));  // zero length
    assert(!Train.AddZone({0.0, 50.0, 5.0, 1.0, -4.0}));  // negative decel: a runaway brake
    assert(!Train.AddZone({0.0, 50.0, -5.0, 1.0, 1.0}));  // negative target
    assert(Train.AddZone({0.0, 50.0, 5.0, 1.0, 1.0}));

    // None of the rejected zones took effect.
    Train.Place(60.0, 20.0);
    for (int i = 0; i < 60; ++i) { Train.Step(1.0 / 60.0); }
    assert(Near(Train.GetSpeed(), 20.0, 1e-9));
}

static void TestStallsInsteadOfProducingNaN()
{
    // A train sent at a loop too slowly cannot crest it. It must stop cleanly
    // rather than take the square root of a negative number.
    const FTrack Loop = MakeFullLoop(8.0);
    FTrain Train(Loop, Frictionless());
    Train.Place(0.0, 5.0);

    int Steps = 0;
    while (Train.GetSpeed() > 0.0 && Steps < 20000) { Train.Step(1.0 / 240.0); ++Steps; }

    assert(Train.GetSpeed() == 0.0);
    assert(!std::isnan(Train.GetDistance()));
    assert(Train.GetDistance() < Pi * 8.0); // never reached the apex

    // And it stays stopped rather than drifting or reversing.
    const double Stalled = Train.GetDistance();
    for (int i = 0; i < 200; ++i) { Train.Step(1.0 / 240.0); }
    assert(Train.GetSpeed() == 0.0);
    assert(Train.GetDistance() == Stalled);
}

static void TestStepRejectsBadDeltas()
{
    const FTrack Track = MakeLevelStraight(100.0);
    FTrain Train(Track, Frictionless());
    Train.Place(10.0, 20.0);

    Train.Step(0.0);
    Train.Step(-1.0);
    Train.Step(std::nan(""));
    assert(Train.GetDistance() == 10.0);
    assert(Train.GetSpeed() == 20.0);
    // A rejected step must not poison the G readouts either — a paused frame
    // dividing by dt would hand NaN to every HUD and comfort accumulator.
    assert(!std::isnan(Train.GetTangentialG()));
    assert(!std::isnan(Train.GetForces().Vertical));

    // Place clamps rather than trusting its caller.
    Train.Place(-50.0, 10.0);
    assert(Train.GetDistance() == 0.0);
    Train.Place(9999.0, 10.0);
    assert(Train.GetDistance() == Track.TotalLength());
    // ...and the cached frame follows it, rather than going stale.
    assert(Near(Train.GetFrame().Position.X, Track.TotalLength(), 1e-9));
}

// ------------------------------------------------------------------ rollback

// Level run-in, then a climb far too long to crest. Whatever a train is sent
// at this, it runs out of height somewhere on the slope.
static FTrack UncrestableClimb()
{
    FTrack T;
    T.AddSegment(MakeStraight(40.0));
    FTrackSegment In;
    In.Length = 26.18;
    In.PitchCurvatureEnd = 0.02;
    T.AddSegment(In);
    FTrackSegment Out;
    Out.Length = 26.18;
    Out.PitchCurvatureStart = 0.02;
    T.AddSegment(Out);
    T.AddSegment(MakeStraight(200.0)); // 100 m of rise at 30 degrees
    return T;
}

static void TestRollbackReturnsExactlyTheEnergyItSpent()
{
    // The assertion that says the sign handling is right rather than merely
    // plausible. Frictionless, a train sent up a hill it cannot crest must come
    // back past its starting point at exactly the speed it left — the hill
    // stored the energy and gave all of it back.
    //
    // A resistance term multiplied by SIGNED travel instead of distance covered
    // would fail here in the most flattering possible way: the train would come
    // back FASTER than it left.
    const FTrack Climb = UncrestableClimb();
    FTrainConfig C = Frictionless();
    C.bAllowRollback = true;

    FTrain Train(Climb, C);
    Train.Place(10.0, 15.0);

    bool bWentUp = false;
    double ReturnSpeed = 0.0;
    for (int i = 0; i < 60000; ++i)
    {
        Train.Step(1.0 / 480.0);
        if (Train.GetDistance() > 60.0) { bWentUp = true; }
        if (bWentUp && Train.IsRollingBack() && Train.GetDistance() <= 10.0)
        {
            ReturnSpeed = Train.GetSpeed();
            break;
        }
    }
    assert(bWentUp);
    assert(ReturnSpeed > 0.0);
    assert(Near(ReturnSpeed, 15.0, 1e-2));

    std::printf("  rollback: left at 15.000 m/s, came back at %.3f m/s\n", ReturnSpeed);
}

static void TestRollbackIsOffByDefaultAndChangesNothingWhenItIs()
{
    // Same track, same train, rollback off: it stops dead where it ran out, and
    // stays there. This is the Phase 0 behaviour every earlier number was
    // measured against, so it has to survive the feature being added.
    const FTrack Climb = UncrestableClimb();
    FTrainConfig Off = Frictionless();
    assert(Off.bAllowRollback == false);

    FTrain Train(Climb, Off);
    Train.Place(10.0, 15.0);
    for (int i = 0; i < 40000 && Train.GetSpeed() > 0.0; ++i)
    {
        Train.Step(1.0 / 480.0);
    }
    const double Stalled = Train.GetDistance();
    assert(Train.GetSpeed() == 0.0);
    assert(!Train.IsRollingBack());
    for (int i = 0; i < 500; ++i) { Train.Step(1.0 / 480.0); }
    assert(Train.GetDistance() == Stalled); // did not creep, either way
    assert(Train.GetSpeed() == 0.0);
}

static void TestRollingBackIntoAValleySettlesRatherThanOscillatingForever()
{
    // With resistance, the oscillation has to decay: friction opposes travel in
    // BOTH directions, so each pass costs energy. If it were applied along
    // signed travel it would pump the train instead of damping it, and this
    // would run forever.
    // A valley with ground on BOTH sides. The first version of this ran the
    // train back off the start of the track, where it clamped at S = 0 and
    // could never turn round again — one reversal, not an oscillation. Running
    // off the end of the track is not the same event as running out of energy,
    // and a fixture that confuses them is testing the wrong thing.
    FTrack Valley;
    {
        FTrackSegment Down; // 0 -> -34 degrees
        Down.Length = 60.0;
        Down.PitchCurvatureStart = Down.PitchCurvatureEnd = -0.01;
        Valley.AddSegment(Down);
        FTrackSegment Through; // -34 through level to +34: the bottom is at S=120
        Through.Length = 120.0;
        Through.PitchCurvatureStart = Through.PitchCurvatureEnd = 0.01;
        Valley.AddSegment(Through);
        FTrackSegment Level; // +34 back to level, so the far side has a top
        Level.Length = 60.0;
        Level.PitchCurvatureStart = Level.PitchCurvatureEnd = -0.01;
        Valley.AddSegment(Level);
    }

    FTrainConfig C;
    C.RollingResistance = 0.02; // exaggerated so it settles inside the budget
    C.DragK = 0.001;
    C.bAllowRollback = true;

    FTrain Train(Valley, C);
    Train.Place(20.0, 2.0);

    int Reversals = 0;
    double LastDir = 1.0;
    double PeakSpeed = 0.0;
    for (int i = 0; i < 200000; ++i)
    {
        Train.Step(1.0 / 240.0);
        const double V = Train.GetVelocity();
        PeakSpeed = std::fmax(PeakSpeed, std::fabs(V));
        const double Dir = V > 0.01 ? 1.0 : (V < -0.01 ? -1.0 : LastDir);
        if (Dir != LastDir) { ++Reversals; LastDir = Dir; }
        if (i > 40000 && std::fabs(V) < 0.05) { break; }
    }
    assert(Reversals >= 2);                 // it really did swing back and forth
    assert(std::fabs(Train.GetVelocity()) < 0.5); // and lost the energy doing it
    assert(Train.GetDistance() > 0.0 && Train.GetDistance() < Valley.TotalLength());

    std::printf("  rollback: %d reversals in the valley, settled at %.1f m doing %.3f m/s\n",
                Reversals, Train.GetDistance(), std::fabs(Train.GetVelocity()));
}

// -------------------------------------------------------------- train length

// A symmetric airtime hill: nose up, crest, nose back to level. Pitch curvature
// integrates to zero over the three segments, and the shape is symmetric about
// the crest, so the track leaves at the same height and heading it arrived at.
static FTrack AirtimeHill(double K, double L)
{
    FTrack T;
    T.AddSegment(MakeStraight(60.0));
    FTrackSegment Up;
    Up.Length = L;
    Up.PitchCurvatureStart = Up.PitchCurvatureEnd = K;
    T.AddSegment(Up);
    FTrackSegment Crest;
    Crest.Length = 2.0 * L;
    Crest.PitchCurvatureStart = Crest.PitchCurvatureEnd = -K;
    T.AddSegment(Crest);
    FTrackSegment Down;
    Down.Length = L;
    Down.PitchCurvatureStart = Down.PitchCurvatureEnd = K;
    T.AddSegment(Down);
    T.AddSegment(MakeStraight(60.0));
    return T;
}

static void TestTrainLengthIsBitIdenticalAtZero()
{
    // The default is a point mass, and every number recorded before Phase 2 was
    // measured with one. Adding length must not move any of them, so this is
    // asserted exactly rather than within a tolerance.
    const FTrack Hill = AirtimeHill(0.02, 25.0);
    FTrainConfig Point;
    Point.TrainLength = 0.0;

    FTrain A(Hill, Point);
    FTrain B(Hill, FTrainConfig()); // whatever the default happens to be
    A.Place(0.0, 25.0);
    B.Place(0.0, 25.0);
    for (int i = 0; i < 4000 && !A.IsAtEnd(); ++i)
    {
        A.Step(1.0 / 240.0);
        B.Step(1.0 / 240.0);
        assert(A.GetSpeed() == B.GetSpeed());
        assert(A.GetDistance() == B.GetDistance());
    }
    // And with no length, every car is the same car.
    assert(A.GetForcesAt(+7.5).Vertical == A.GetForces().Vertical);
    assert(A.GetForcesAt(-7.5).Vertical == A.GetForces().Vertical);
    assert(A.GetFrontS() == A.GetDistance());
}

static void TestLengthConservesEnergyOverASymmetricHill()
{
    // The mean-height accounting has to be conservative, or length quietly
    // becomes an energy source. Frictionless, and the hill returns to the
    // height and heading it started at, so the exit speed must equal the entry
    // speed however long the train is.
    // Measured between two points where the WHOLE train is on flat track: the
    // lead-in runs 0-60 m and the run-out 160-220 m, so a 24 m train centred at
    // 30 m and again at 190 m is entirely on the level both times and the two
    // mean heights are directly comparable.
    //
    // Not measured to the end of the track, deliberately. Samples clamp at the
    // track ends, so a train hanging off a point-to-point layout has its
    // overhanging mass piled at the endpoint and its mean height is wrong for
    // as long as that lasts. That is a real limitation of the model rather than
    // something this test should paper over.
    const FTrack Hill = AirtimeHill(0.015, 25.0);
    for (const double Length : {0.0, 8.0, 15.0, 24.0})
    {
        FTrainConfig C = Frictionless();
        C.TrainLength = Length;
        FTrain Train(Hill, C);
        Train.Place(30.0, 22.0);
        for (int i = 0; i < 40000 && Train.GetDistance() < 190.0; ++i)
        {
            Train.Step(1.0 / 480.0);
        }
        assert(Train.GetDistance() >= 190.0);
        assert(Near(Train.GetSpeed(), 22.0, 1e-3));
    }
}

static void TestLongTrainCrestsFasterThanAPoint()
{
    // The mechanism, isolated. A train straddling a crest has its centre of
    // mass BELOW the crest, so it has not paid the full height and arrives
    // travelling faster than a point mass would. This is the whole of what
    // length does to the physics; everything else follows from it.
    const FTrack Hill = AirtimeHill(0.02, 25.0);
    const double CrestS = 60.0 + 25.0 + 25.0; // middle of the crest segment

    double SpeedAtCrest[2] = {0.0, 0.0};
    double MinVertical[2] = {9.0, 9.0};
    const double Lengths[2] = {0.0, 15.0};

    for (int i = 0; i < 2; ++i)
    {
        FTrainConfig C = Frictionless();
        C.TrainLength = Lengths[i];
        FTrain Train(Hill, C);
        Train.Place(20.0, 25.0);
        bool bRecorded = false;
        for (int Step = 0; Step < 20000 && !Train.IsAtEnd(); ++Step)
        {
            Train.Step(1.0 / 480.0);
            if (!bRecorded && Train.GetDistance() >= CrestS)
            {
                SpeedAtCrest[i] = Train.GetSpeed();
                bRecorded = true;
            }
            MinVertical[i] = std::fmin(MinVertical[i], Train.GetForces().Vertical);
        }
    }

    assert(SpeedAtCrest[1] > SpeedAtCrest[0]);   // the train is faster over the top
    assert(MinVertical[1] < MinVertical[0]);     // and therefore lighter in the seat
    std::printf("  train length: point mass crests at %.3f m/s (%.3f G), 15 m train at "
                "%.3f m/s (%.3f G)\n",
                SpeedAtCrest[0], MinVertical[0], SpeedAtCrest[1], MinVertical[1]);
}

// The shape of a real airtime hill: a gentle rise to the crest and a sharp fall
// away from it. Deliberately NOT symmetric — see the test below for why that is
// the whole point.
static FTrack AsymmetricCrest()
{
    FTrack T;
    T.AddSegment(MakeStraight(60.0));
    FTrackSegment Up; // gentle, to +18 degrees
    Up.Length = 31.4;
    Up.PitchCurvatureStart = Up.PitchCurvatureEnd = 0.01;
    T.AddSegment(Up);
    FTrackSegment Crest; // sharp, carrying on down to -40 degrees
    Crest.Length = 20.3;
    Crest.PitchCurvatureStart = Crest.PitchCurvatureEnd = -0.05;
    T.AddSegment(Crest);
    FTrackSegment Recover;
    Recover.Length = 35.05;
    Recover.PitchCurvatureStart = Recover.PitchCurvatureEnd = 0.02;
    T.AddSegment(Recover);
    T.AddSegment(MakeStraight(60.0));
    return T;
}

static void TestBackCarIsThrownHarderThanTheFront()
{
    // The claim PHASE0_FINDINGS makes about why point-mass sims feel wrong, now
    // measurable — and with the condition on it that the findings did not state.
    //
    // Every car shares one speed at any instant, because the train is rigid and
    // on rails. So the difference between cars is entirely about WHEN each one
    // reaches the crest and how fast the train happens to be moving then.
    //
    // On a SYMMETRIC hill there is no difference at all. The front car crests
    // when the centre of mass is half a train short of the crest; the back car
    // crests when it is half a train past. Symmetric and frictionless, those two
    // positions are the same height, so the speeds are identical and so is the
    // airtime. The first version of this test used a symmetric hill and
    // correctly measured no effect.
    //
    // It takes an ASYMMETRIC crest — gentle rise, sharp fall, which is what real
    // airtime hills are. Then the centre of mass is LOWER when the back car
    // crests than it was when the front car did, so the train is moving faster,
    // and the back car takes the same curvature harder.
    const FTrack Hill = AsymmetricCrest();
    FTrainConfig C = Frictionless();
    C.TrainLength = 15.0;

    FTrain Train(Hill, C);
    Train.Place(20.0, 25.0);

    double FrontMin = 9.0, BackMin = 9.0, CentreMin = 9.0;
    for (int Step = 0; Step < 20000 && !Train.IsAtEnd(); ++Step)
    {
        Train.Step(1.0 / 480.0);
        FrontMin = std::fmin(FrontMin, Train.GetForcesAt(+7.5).Vertical);
        BackMin = std::fmin(BackMin, Train.GetForcesAt(-7.5).Vertical);
        CentreMin = std::fmin(CentreMin, Train.GetForces().Vertical);
    }

    assert(BackMin < FrontMin);   // the back gets thrown harder
    assert(BackMin < CentreMin);  // and harder than the reading at the heartline
    std::printf("  train length: over an asymmetric crest the front car sees %+.3f G, the "
                "centre %+.3f G, the back car %+.3f G\n",
                FrontMin, CentreMin, BackMin);

    // And on a symmetric hill the two ends agree, which is the control that
    // stops the assertion above passing for the wrong reason.
    const FTrack Symmetric = AirtimeHill(0.02, 25.0);
    FTrain Even(Symmetric, C);
    Even.Place(20.0, 25.0);
    double EvenFront = 9.0, EvenBack = 9.0;
    for (int Step = 0; Step < 20000 && !Even.IsAtEnd(); ++Step)
    {
        Even.Step(1.0 / 480.0);
        EvenFront = std::fmin(EvenFront, Even.GetForcesAt(+7.5).Vertical);
        EvenBack = std::fmin(EvenBack, Even.GetForcesAt(-7.5).Vertical);
    }
    assert(Near(EvenFront, EvenBack, 0.01));
}

// ------------------------------------------------------------- ride profile

// Station, eased climb, crest, drop, level run-out. The shape of a ride,
// stripped to what a profile needs to have something to say about.
static FTrack ProfileTestTrack(double LiftClimb)
{
    auto EasedPitch = [](FTrack& T, double PitchDelta, double Peak) {
        const double K = PitchDelta >= 0.0 ? Peak : -Peak;
        const double L = std::fabs(PitchDelta) / Peak;
        FTrackSegment In;
        In.Length = L;
        In.PitchCurvatureEnd = K;
        T.AddSegment(In);
        FTrackSegment Out;
        Out.Length = L;
        Out.PitchCurvatureStart = K;
        T.AddSegment(Out);
    };

    FTrack T;
    T.AddSegment(MakeStraight(20.0));
    EasedPitch(T, 25.0 * Pi / 180.0, 0.03);
    T.AddSegment(MakeStraight(LiftClimb));
    EasedPitch(T, -50.0 * Pi / 180.0, 0.05);
    T.AddSegment(MakeStraight(30.0));
    EasedPitch(T, 25.0 * Pi / 180.0, 0.012);
    T.AddSegment(MakeArc(60.0, 40.0, 25.0 * Pi / 180.0));
    T.AddSegment(MakeStraight(60.0));
    return T;
}

static void TestRideProfileMeasuresTheWholeRideAtEditTime()
{
    const FTrack Track = ProfileTestTrack(60.0);
    FTrain Train(Track);
    Train.AddZone(MakeLift(0.0, 140.0, 4.0, 6.0)); // must run PAST the crest, not to the top of the climb

    const FRideProfile P = RunRideProfile(Train, Track, 1.0);
    assert(P.bCompleted);
    assert(P.Samples.size() > 100);
    assert(P.Duration > 10.0);

    // Sampled by ARC LENGTH, not by time, so the data does not thin out exactly
    // where the ride is fastest and the G is worth looking at.
    for (std::size_t i = 1; i < P.Samples.size(); ++i)
    {
        assert(P.Samples[i].S >= P.Samples[i - 1].S);
    }

    // Extremes carry WHERE, because "4.25 g" is a number and "4.25 g at
    // S=310 m" is something an author can go and look at.
    assert(P.TopSpeed > 20.0);
    assert(P.TopSpeedS > 100.0);           // at the bottom of the drop, not on the lift
    assert(P.MaxVerticalG > 1.5);          // the pull-out
    assert(P.MaxVerticalGS > 100.0);
    assert(P.HighestHeight > 20.0);
    assert(P.LowestHeight <= 0.0);

    // The lift holds chain speed, so the profile knows the train crawls up and
    // only then goes fast — a speed trace that peaked on the lift would mean
    // the zone was not being applied at all.
    double SpeedOnLift = 0.0;
    for (const FRideSample& S : P.Samples)
    {
        if (S.S > 40.0 && S.S < 80.0) { SpeedOnLift = S.Speed; }
    }
    assert(Near(SpeedOnLift, 4.0, 0.5));

    std::printf("  ride profile: %.0f m in %.1f s, top %.1f km/h at S=%.0f, "
                "vertical %+.2f..%+.2f, lowest %.1f m\n",
                Track.TotalLength(), P.Duration, P.TopSpeed * 3.6, P.TopSpeedS, P.MinVerticalG,
                P.MaxVerticalG, P.LowestHeight);
}

static void TestRideProfileReportsAStallRatherThanHanging()
{
    // The single most useful thing an editor can say, and the only way to know
    // is to run it: this train does not get round. A 6 m lift cannot carry it
    // over what a 60 m lift could.
    const FTrack Track = ProfileTestTrack(6.0);
    FTrain Train(Track);
    Train.AddZone(MakeLift(0.0, 40.0, 4.0, 6.0));

    const FRideProfile P = RunRideProfile(Train, Track, 1.0);
    assert(!P.bCompleted);
    assert(P.StalledAtS > 0.0);
    assert(P.StalledAtS < Track.TotalLength());
    // And it says where, so the author can see which hill did it.
    assert(P.StalledHeight > -100.0 && P.StalledHeight < 100.0);
    assert(!P.Samples.empty()); // the run up to the stall is still worth having

    std::printf("  ride profile: stalled at S=%.1f m, %.1f m up — reported, not hung\n",
                P.StalledAtS, P.StalledHeight);
}

static void TestRideProfileReportsARollbackDistinctlyFromAStall()
{
    // Same failing ride, two configurations. Both say the train does not get
    // round; only one says it is loose on the track heading backwards, which is
    // a different fault with a different fix.
    const FTrack Climb = UncrestableClimb();

    // Launched, because RunRideProfile starts from rest and this track opens
    // with 40 m of level: a standing train there never moves, and "stalled at
    // 0 m" would be true and useless.
    FTrainConfig Stops;
    FTrain A(Climb, Stops);
    A.AddZone(MakeLaunch(0.0, 40.0, 15.0, 6.0));
    const FRideProfile Stalled = RunRideProfile(A, Climb, 1.0);
    assert(!Stalled.bCompleted);
    assert(!Stalled.bRolledBack);
    assert(Stalled.StalledAtS > 50.0); // out on the climb, not in the launch

    FTrainConfig Rolls;
    Rolls.bAllowRollback = true;
    FTrain B(Climb, Rolls);
    B.AddZone(MakeLaunch(0.0, 40.0, 15.0, 6.0));
    const FRideProfile Back = RunRideProfile(B, Climb, 1.0);
    assert(!Back.bCompleted);
    assert(Back.bRolledBack);
    assert(Back.RolledBackAtS > 50.0);
    // Caught as it turns, not after it has run all the way home.
    assert(Near(Back.RolledBackAtS, Stalled.StalledAtS, 3.0));

    std::printf("  ride profile: stalls at %.1f m; with rollback on, reported rolling back "
                "at %.1f m\n",
                Stalled.StalledAtS, Back.RolledBackAtS);
}

static void TestRideProfileCarriesRollRateWhichGCannot()
{
    // A banked turn entered from level track twists the rider while the G trace
    // says nothing about it, because felt G has no roll-rate term at all. The
    // profile carries it as its own channel for exactly that reason.
    // Rolled in over 20 m and out over 40 m, so the peak is unambiguously the
    // entry transition rather than a coin flip between two identical ones.
    //
    // The roll-out clothoid is not decoration: without it the arc ends at 45
    // degrees of bank against a straight at 0, which is a 45 degree
    // INSTANTANEOUS twist. The first version of this fixture had exactly that,
    // and the profile correctly reported the step as the peak — a real
    // authoring error caught by the thing being tested.
    FTrack Track;
    Track.AddSegment(MakeStraight(40.0));
    Track.AddSegment(MakeClothoid(20.0, 0.0, 1.0 / 30.0, 0.0, 45.0 * Pi / 180.0));
    Track.AddSegment(MakeArc(50.0, 30.0, 45.0 * Pi / 180.0));
    Track.AddSegment(MakeClothoid(40.0, 1.0 / 30.0, 0.0, 45.0 * Pi / 180.0, 0.0));
    Track.AddSegment(MakeStraight(40.0));

    FTrain Train(Track);
    Train.AddZone(MakeLaunch(0.0, 20.0, 25.0, 6.0));

    const FRideProfile P = RunRideProfile(Train, Track, 0.5);
    assert(P.bCompleted);
    assert(P.MaxAbsRollRate > 10.0);
    // On the entry transition — not on a straight, and not in the arc where the
    // bank is being held constant and the rider is no longer rotating.
    assert(P.MaxAbsRollRateS > 38.0 && P.MaxAbsRollRateS < 62.0);

    std::printf("  ride profile: peak roll rate %.1f deg/s at S=%.1f m, where the G trace "
                "reports nothing\n",
                P.MaxAbsRollRate, P.MaxAbsRollRateS);
}

int main()
{
    TestGravityIsExactEnergyExchange();
    TestEnergyResultIsStepSizeIndependent();
    TestRollingFrictionStoppingDistance();
    TestDragAloneDecaysExponentiallyWithDistance();
    TestFrictionScalesWithNormalLoad();
    TestZoneHoldsTargetAgainstLosses();
    TestOverlappingZonesTakeTheMostRestrictive();
    TestLiftHoldsChainSpeedBothWays();
    TestBrakeOnlySlows();
    TestLaunchProducesForeAftG();
    TestFreeRollingFeelsNoForeAftForce();
    TestDepartsFromRestOnAGradient();
    TestUnderpoweredLiftCannotCreateEnergy();
    TestAdequateLiftHoldsChainSpeedUphill();
    TestAddZoneRejectsMalformedZones();
    TestStallsInsteadOfProducingNaN();
    TestStepRejectsBadDeltas();
    TestRollbackReturnsExactlyTheEnergyItSpent();
    TestRollbackIsOffByDefaultAndChangesNothingWhenItIs();
    TestRollingBackIntoAValleySettlesRatherThanOscillatingForever();
    TestTrainLengthIsBitIdenticalAtZero();
    TestLengthConservesEnergyOverASymmetricHill();
    TestLongTrainCrestsFasterThanAPoint();
    TestBackCarIsThrownHarderThanTheFront();
    TestRideProfileMeasuresTheWholeRideAtEditTime();
    TestRideProfileReportsAStallRatherThanHanging();
    TestRideProfileReportsARollbackDistinctlyFromAStall();
    TestRideProfileCarriesRollRateWhichGCannot();
    std::printf("All train physics tests passed.\n");
    return 0;
}
