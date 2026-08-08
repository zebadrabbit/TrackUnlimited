// Build & run:  clang++ -std=c++17 -Wall -Wextra -O2 -o test_tracksensors test_tracksensors.cpp && ./test_tracksensors
//
// Drives FTrackSensors with bare numbers. No FTrack, no FTrain, no engine — the
// same discipline as test_ridesignals, and for the same reason: a sensor is a
// point and a boolean, and anything it needs beyond that is a design mistake.

#include "TrackSensors.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

namespace
{

// Sensors at four block boundaries on a 600 m circuit.
std::vector<double> Boundaries() { return {0.0, 150.0, 300.0, 450.0}; }

// One train, one scan.
void Scan(FTrackSensors& S, double Rear, double Front, bool bCircuit = false)
{
    S.BeginScan();
    S.Cover(Rear, Front, bCircuit, 600.0);
    S.EndScan();
}

void TestASensorIsAPointAndABoolean()
{
    FTrackSensors S(Boundaries());
    assert(S.Num() == 4);
    assert(!S.IsBlocked(1));

    // Nowhere near it.
    Scan(S, 100.0, 115.0);
    assert(!S.IsBlocked(1));
    assert(S.Read(1).Rising == 0);

    // Covering it: the train's span contains the sensor's point.
    Scan(S, 145.0, 160.0);
    assert(S.IsBlocked(1));
    assert(S.Read(1).Rising == 1);
    assert(S.Read(1).Falling == 0);

    // Still covering — an edge is a CHANGE, so it must not count twice. A logic
    // layer that acted on level rather than edge would enter the same block once
    // per frame for as long as the train sat there.
    Scan(S, 146.0, 161.0);
    assert(S.IsBlocked(1));
    assert(S.Read(1).Rising == 1);

    // Past it.
    Scan(S, 155.0, 170.0);
    assert(!S.IsBlocked(1));
    assert(S.Read(1).Rising == 1);
    assert(S.Read(1).Falling == 1);
}

void TestWholeTrainPassedIsNotTheSameAsNoseArrived()
{
    // THE QUESTION A BLOCK SYSTEM ACTUALLY ASKS, and the one a real controller
    // answers by counting flags: has an ENTIRE train gone past, or is one merely
    // sitting on top of the sensor? Getting these two confused releases a block
    // with a train still straddling its boundary.
    FTrackSensors S(Boundaries());

    Scan(S, 100.0, 115.0);
    assert(!S.HasPassedCompletely(1));   // not yet arrived

    Scan(S, 145.0, 160.0);
    assert(S.IsBlocked(1));
    assert(!S.HasPassedCompletely(1));   // ARRIVED, but straddling it

    Scan(S, 155.0, 170.0);
    assert(S.HasPassedCompletely(1));    // and now genuinely through
}

void TestOneSensorCannotBeClearedByTheWrongTrain()
{
    // A sensor is ONE device. Two trains in a scan must not each decide whether
    // it is blocked — if the second overwrote the first, a train parked on a
    // switch would read clear the moment anything else was updated, which is the
    // same class of bug the multi-train work found in the occupancy layer.
    FTrackSensors S(Boundaries());

    S.BeginScan();
    S.Cover(145.0, 160.0);   // train A is sitting on sensor 1
    S.Cover(400.0, 415.0);   // train B is nowhere near it
    S.EndScan();

    assert(S.IsBlocked(1));
    assert(S.Read(1).Rising == 1);

    // And the order cannot matter either.
    FTrackSensors T(Boundaries());
    T.BeginScan();
    T.Cover(400.0, 415.0);
    T.Cover(145.0, 160.0);
    T.EndScan();
    assert(T.IsBlocked(1));
}

void TestTheSeamIsJustAnotherStretchOfTrack()
{
    // A train straddling the circuit's seam has a rear GREATER than its front. On
    // a circuit that is a real state, not a caller error, and the sensor at 0.0
    // is covered by it — the same trap the occupancy layer hit, met again one
    // layer down.
    FTrackSensors S(Boundaries());

    Scan(S, 590.0, 5.0, true);
    assert(S.IsBlocked(0));          // the seam sensor sees it
    assert(!S.IsBlocked(1));         // and nothing else does
    assert(!S.IsBlocked(3));

    // Off a circuit the same pair is a caller passing its arguments backwards,
    // and gets sorted — which covers everything from 5 to 590 and so trips 1, 2
    // and 3 as well. Different meaning, deliberately.
    FTrackSensors T(Boundaries());
    Scan(T, 590.0, 5.0, false);
    assert(T.IsBlocked(1));
    assert(T.IsBlocked(2));
}

void TestATrainCanCrossASensorInsideOneScan()
{
    // RECORDED, NOT FIXED, and the same shape as the known block-skip limit in
    // FRideSignals. A sensor is sampled once per scan, so a train that arrives
    // and leaves between two scans is never seen at all — no rise, no fall, no
    // trace. At 60 Hz and 30 m/s a train moves 0.5 m a frame and the train itself
    // is 15 m long, so this needs a scan slower than the train is long.
    //
    // Real hardware has the same failure and solves it in hardware: the switch
    // latches, so a brief trip is held until the controller reads it. If this
    // ever matters, that latch is the fix, not a faster scan.
    FTrackSensors S(Boundaries());

    Scan(S, 100.0, 115.0);       // before sensor 1
    Scan(S, 200.0, 215.0);       // after it, having never been seen on it
    assert(S.Read(1).Rising == 0);
    assert(!S.HasPassedCompletely(1));
}

void TestTheCounterAgreesWithPerfectKnowledge()
{
    // THE TEST THAT MAKES THE SENSOR LAYER WORTH HAVING. Run a train round a ring
    // and compare, every scan, what the counter DERIVES from trips against what a
    // layer with perfect knowledge of the span would say. If these ever disagree,
    // the sensors are not sufficient and the control system is still cheating.
    const std::vector<double> B = Boundaries();
    const double Total = 600.0;
    const double Len = 15.0;

    FTrackSensors S(B);
    FBlockCounter Counter(S);

    // SEEDED, with the train clear of every sensor — the operator's sweep before
    // the ride opens. Start it straddling one and the seed has to say "in both",
    // which is a fair state but a poor place to begin a test about counting.
    //
    // The 0.3 keeps the train's ends off the sensors EXACTLY, and that is about
    // the comparison rather than the counter. A train whose tail sits precisely on
    // a boundary is in both blocks, which the counter gets right; the ground truth
    // below is a plain interval test that does not know 0.0 and 600.0 are the same
    // point on a ring, so it disagrees at that one measure-zero position. Landing
    // on a boundary is covered by the straddle tests above.
    const double Start = 75.3;
    Scan(S, Start - Len * 0.5, Start + Len * 0.5, true);
    Counter.Scan();
    Counter.Seed(0);

    for (double Centre = Start; Centre < Total * 3.0 + Start; Centre += 0.5)
    {
        double Rear = Centre - Len * 0.5;
        double Front = Centre + Len * 0.5;
        while (Rear >= Total) { Rear -= Total; }
        while (Front >= Total) { Front -= Total; }
        while (Rear < 0.0) { Rear += Total; }
        while (Front < 0.0) { Front += Total; }
        Scan(S, Rear, Front, true);
        Counter.Scan();
        assert(!Counter.IsInconsistent());

        // Ground truth: does the train's span touch the block at all? The counter
        // reads occupied from the moment the nose reaches the block's start until
        // the tail passes its end, which is exactly a CLOSED interval overlap —
        // and the closed end is what makes a straddling train belong to both.
        for (std::size_t b = 0; b < B.size(); ++b)
        {
            const double Lo = B[b];
            const double Hi = (b + 1 < B.size()) ? B[b + 1] : Total;
            const bool bWraps = Front < Rear;
            const bool bTruth = bWraps
                ? (Hi >= Rear || Lo <= Front)               // [Rear,Total] u [0,Front]
                : (Front >= Lo && Rear <= Hi);
            assert(Counter.IsOccupied(b) == bTruth);
            assert(!Counter.IsOverOccupied(b));
        }
    }
}

// Ground truth for the ring, so a fault test can say what the counter SHOULD
// have believed while it believes something else.
bool SpanTouches(const std::vector<double>& B, std::size_t b, double Total,
                 double Rear, double Front)
{
    const double Lo = B[b];
    const double Hi = (b + 1 < B.size()) ? B[b + 1] : Total;
    return (Front < Rear) ? (Hi >= Rear || Lo <= Front) : (Front >= Lo && Rear <= Hi);
}

void TestADeadSENSORIsWhatTheSecondMethodExISTSToCatch()
{
    // THE FAULT THAT MATTERS MOST, because a sensor is the PLC's only view of
    // where the trains are. Until this existed, the cross-check between the
    // counter and the interlocking had only ever been proven by MUTATING the
    // counter's own falling-edge rule — which shows the assertion bites, and says
    // nothing about whether a real sensor failure is caught. Different claims,
    // and only one of them is about safety.
    const std::vector<double> B = Boundaries();
    const double Total = 600.0;
    const double Len = 15.0;

    FTrackSensors S(B);
    FBlockCounter Counter(S);

    const double Start = 75.3;
    Scan(S, Start - Len * 0.5, Start + Len * 0.5, true);
    Counter.Scan();
    Counter.Seed(0);

    // Sensor 2 dies. Nothing announces it — that is the point of a dead switch,
    // and it is why a second means of detection is bought rather than a better
    // single one.
    S.Fail(2, FTrackSensors::ESensorFault::Dead);

    bool bEverDisagreed = false;
    for (double Centre = Start; Centre < Total * 2.0 + Start; Centre += 0.5)
    {
        double Rear = Centre - Len * 0.5;
        double Front = Centre + Len * 0.5;
        while (Rear >= Total) { Rear -= Total; }
        while (Front >= Total) { Front -= Total; }
        Scan(S, Rear, Front, true);
        Counter.Scan();

        for (std::size_t b = 0; b < B.size(); ++b)
        {
            if (Counter.IsOccupied(b) != SpanTouches(B, b, Total, Rear, Front))
            {
                bEverDisagreed = true;
            }
        }
    }

    // THE DETECTION. The counter and a span-based interlocking now disagree, and
    // in the actor that difference trips the E-stop. Neither can say which of them
    // is wrong — which is precisely the property a second detection method buys.
    assert(bEverDisagreed);

    // And it is the DEAD one that did it: block 2's boundary never trips, so the
    // train enters and is never counted in.
    assert(S.Read(2).Rising == 0);
    assert(S.Read(2).Falling == 0);
    assert(S.Read(1).Rising > 0);          // its neighbours are fine
}

void TestAStuckOnSensorAndAChatteringOneAreDifferentFailures()
{
    // A dead switch under-reports and a stuck one over-reports, and they are not
    // mirror images: a block that never goes occupied lets a train in on top of
    // another, where one that never goes clear STOPS THE RIDE. The second fails
    // safe and the first does not, which is the entire argument for wiring
    // detection so that the safe direction is the de-energised one.
    const std::vector<double> B = Boundaries();

    {
        FTrackSensors S(B);
        S.Fail(1, FTrackSensors::ESensorFault::StuckOn);
        Scan(S, 300.0, 315.0, true);       // nowhere near sensor 1
        assert(S.IsBlocked(1));            // says otherwise
        assert(S.Read(1).Rising == 1);     // and produced an edge that never happened
    }

    {
        // A loose connection: toggling regardless of reality, and DETERMINISTIC,
        // because a scenario that reproduces differently every run cannot prove
        // anything.
        FTrackSensors S(B);
        S.ChatterScans = 2;
        S.Fail(0, FTrackSensors::ESensorFault::Chatter);
        for (int i = 0; i < 20; ++i) { Scan(S, 300.0, 315.0, true); }
        assert(S.Read(0).Rising > 2);      // edges from nothing at all
        assert(S.Read(0).Falling > 2);

        // Same script, same result. The property a fault-injection scenario needs.
        FTrackSensors T(B);
        T.ChatterScans = 2;
        T.Fail(0, FTrackSensors::ESensorFault::Chatter);
        for (int i = 0; i < 20; ++i) { Scan(T, 300.0, 315.0, true); }
        assert(T.Read(0).Rising == S.Read(0).Rising);
        assert(T.Read(0).Falling == S.Read(0).Falling);
    }
}

void TestAHealthySensorIsUNAFFECTEDByTheFaultMachinery()
{
    // The fault path must cost nothing when nothing is faulted, or every figure
    // measured before this existed quietly moved.
    const std::vector<double> B = Boundaries();
    FTrackSensors S(B);
    S.Fail(1, FTrackSensors::ESensorFault::Dead);
    S.ClearFaults();
    Scan(S, 145.0, 160.0, true);
    assert(S.IsBlocked(1));
    assert(S.FaultOn(1) == FTrackSensors::ESensorFault::None);
}

// ------------------------------------------------------ speed traps

// A trap pair 2 m apart at 100 m, plus the four boundary sensors. Two switches
// and a surveyed gap is the whole of it.
std::vector<double> WithTrap() { return {0.0, 100.0, 102.0, 150.0, 300.0, 450.0}; }

// Run a train past at a constant speed, scanning at 240 Hz, and give the trap
// every scan. TrainLen matters: a sensor rises when the NOSE reaches it.
void RunPast(FTrackSensors& S, FSpeedTraps& Traps, double SpeedMs,
             double FromS, double ToS, double TrainLen = 15.0)
{
    const double Dt = 1.0 / 240.0;
    for (double Nose = FromS; Nose <= ToS; Nose += SpeedMs * Dt)
    {
        S.BeginScan();
        S.Cover(Nose - TrainLen, Nose, false, 600.0);
        S.EndScan();
        Traps.Scan(Dt);
    }
}

void TestATrapMeasuresSpeedFromEDGESAndASurveyedGap()
{
    // No position and no train identity — two switches and the time between
    // their rising edges. Exactly what a real installation measures, and exactly
    // what this layer is entitled to know.
    FTrackSensors S(WithTrap());
    FSpeedTraps Traps(S);
    assert(Traps.Survey(1, 2));                 // 100 m and 102 m
    assert(Traps.Num() == 1);
    assert(std::fabs(Traps.SpecOf(0).SeparationM - 2.0) < 1e-12);

    // SURVEYED, NOT DERIVED. The gap was read once, here, at commissioning.
    // Nothing per-scan looks at a position again.
    assert(!Traps.Read(0).bValid);

    RunPast(S, Traps, 20.0, 90.0, 130.0);
    const FSpeedTrapReading& R = Traps.Read(0);
    assert(R.bValid);
    assert(std::fabs(R.SpeedMs - 20.0) < 0.5);  // within the scan quantisation
    assert(!R.bArmed);                          // the pass completed
    assert(R.Reversed == 0);

    std::printf("  a 2 m trap reads %.2f m/s for a 20 m/s train, over %d scans\n",
                R.SpeedMs, R.OverScans);
}

void TestTheQUANTISATIONIsRealAndIsREPORTED()
{
    // THE THING THAT MAKES A TRAP A DESIGN DECISION RATHER THAN A COMPONENT.
    //
    // A scan is a tick, so a trap resolves time to one of them. At 240 Hz a train
    // doing 30 m/s covers 0.125 m per scan, so a 1 m gap is measured in 8 ticks —
    // 12.5% error before anything else goes wrong. Widen the gap and the error
    // falls; a 10 m gap on the same train is 80 ticks and 1.25%.
    //
    // Reported rather than enforced, because how much error a caller can live
    // with depends on what it is about to do with the number.
    const double Fast = 30.0;
    double NarrowErr = 0.0, WideErr = 0.0;
    int NarrowScans = 0, WideScans = 0;

    {
        FTrackSensors S({0.0, 100.0, 101.0});
        FSpeedTraps T(S);
        assert(T.Survey(1, 2));
        RunPast(S, T, Fast, 90.0, 130.0);
        assert(T.Read(0).bValid);
        NarrowErr = std::fabs(T.Read(0).SpeedMs - Fast) / Fast;
        NarrowScans = T.Read(0).OverScans;
        assert(std::fabs(SpeedResolutionFraction(T.Read(0)) - 1.0 / NarrowScans) < 1e-12);
    }
    {
        FTrackSensors S({0.0, 100.0, 110.0});
        FSpeedTraps T(S);
        assert(T.Survey(1, 2));
        RunPast(S, T, Fast, 90.0, 140.0);
        assert(T.Read(0).bValid);
        WideErr = std::fabs(T.Read(0).SpeedMs - Fast) / Fast;
        WideScans = T.Read(0).OverScans;
    }

    // WHAT IS ASSERTED IS THE BOUND, NOT THE REALISED ERROR, and the first
    // version of this test asserted the wrong one and failed. Quantisation error
    // is not monotonic in gap width on any single pass: it depends on where the
    // two edges happen to land relative to a tick, so a narrow trap can simply be
    // luckier than a wide one.
    //
    // The BOUND is the thing a designer can work against, and it is the number a
    // trap should be commissioned from. Both readings sit inside their own, and
    // the wide trap's is an order of magnitude tighter.
    assert(WideScans > NarrowScans * 5);
    assert(NarrowErr <= 1.0 / static_cast<double>(NarrowScans));
    assert(WideErr <= 1.0 / static_cast<double>(WideScans));
    assert(1.0 / static_cast<double>(WideScans) < 0.02);
    std::printf("  1 m trap: %d scans, %.2f%% error inside a %.1f%% bound;"
                " 10 m: %d scans, %.2f%% inside %.2f%%\n",
                NarrowScans, NarrowErr * 100.0, 100.0 / NarrowScans,
                WideScans, WideErr * 100.0, 100.0 / WideScans);
}

void TestABackwardsPassIsCOUNTEDAndIsNotASpeed()
{
    // A train tripping the SECOND switch without the first is going backwards, or
    // the first switch missed it. Either is worth knowing and neither is a speed,
    // so it is counted and no measurement is invented.
    FTrackSensors S(WithTrap());
    FSpeedTraps Traps(S);
    assert(Traps.Survey(1, 2));

    // Rolling back: the nose crosses 102 first, then 100.
    const double Dt = 1.0 / 240.0;
    for (double Nose = 110.0; Nose >= 90.0; Nose -= 5.0 * Dt)
    {
        S.BeginScan();
        S.Cover(Nose - 15.0, Nose, false, 600.0);
        S.EndScan();
        Traps.Scan(Dt);
    }
    assert(!Traps.Read(0).bValid);          // nothing was measured
    assert(Traps.Read(0).Reversed > 0);     // and the reason is recorded
    std::printf("  a backwards pass is counted (%d), never turned into a speed\n",
                Traps.Read(0).Reversed);
}

void TestATrainThatSTOPSBetweenTheSwitchesDisarmsTheTrap()
{
    // Without a timeout the trap stays armed for ever, and the NEXT train through
    // is measured against a clock started minutes ago — a speed that is wrong by
    // however long the ride was held. Worse than no reading, because it looks
    // like one.
    FTrackSensors S(WithTrap());
    FSpeedTraps Traps(S);
    assert(Traps.Add({1, 2, 2.0, /*ArmedScansLimit*/ 240}));   // one second

    // Nose over the first switch, then stopped short of the second.
    const double Dt = 1.0 / 240.0;
    for (int i = 0; i < 600; ++i)
    {
        S.BeginScan();
        S.Cover(86.0, 101.0, false, 600.0);
        S.EndScan();
        Traps.Scan(Dt);
    }
    assert(!Traps.Read(0).bArmed);
    assert(Traps.Read(0).TimedOut == 1);
    assert(!Traps.Read(0).bValid);
    std::printf("  a train stopped between the switches disarms rather than lying later\n");
}

void TestWhatATrapIsFOR()
{
    // The measured gap this exists to close: outer brake dead, the arriving train
    // not stopped, rolling into an empty block at 30.5 m/s with the interlocking
    // silent because the block ahead genuinely was free.
    //
    // The trap turns that into a question the ride can answer. Stopping distance
    // is v^2/2a — the same arithmetic the build-time check already runs against
    // every holding device — and the only new thing is a MEASURED speed rather
    // than a predicted one.
    //
    // The circuit's outer brake is 37.5 m long and bites at about 6 m/s^2.
    const double BlockLength = 37.5;
    const double Bite = 6.0;

    assert(StoppingDistanceM(15.5, Bite) < BlockLength);   // a normal arrival fits
    assert(StoppingDistanceM(30.5, Bite) > BlockLength);   // the failure does not
    std::printf("  15.5 m/s needs %.1f m and fits in 37.5; 30.5 m/s needs %.1f m and does not\n",
                StoppingDistanceM(15.5, Bite), StoppingDistanceM(30.5, Bite));

    // And the decision is NOT taken here. A sensor says what it sees; what a ride
    // does about a train arriving too fast is the PLC's job, the same rule the
    // drives' fault detection runs on. Nothing in FSpeedTraps can stop anything.
}

} // namespace

// ===================== WHERE THE COUNTER STOPS BEING TRUE =====================
//
// `SIGNALLING.md` and `CLAUDE.md` both say the counter is proven equal to
// perfect knowledge over three laps, and it is — FORWARDS. This measures the
// other direction rather than assuming it, because "we think it is forward-only"
// and "here is the metre at which it goes wrong" are different kinds of claim
// and only one of them can be checked later.
//
// It is a LATENT limit, not a live defect: rollback is off by default and the
// block layer has never been given a reversing train. It becomes live the day
// FTrainConfig::bAllowRollback is turned on for a layout with sensors.
void TestTheCounterIsFORWARDONLYAndThisIsWhereItBreaks()
{
    FTrackSensors S(Boundaries());
    FBlockCounter C(S);

    // Train of 20 m, seeded honestly in block 0 by the operator's sweep.
    C.Seed(0);
    const double Half = 10.0;

    auto At = [&](double Centre) { Scan(S, Centre - Half, Centre + Half); C.Scan(); };

    // ---- Forwards over the boundary at 150 m: block 0 empties, block 1 fills.
    At(100.0);
    assert(C.TrainsIn(0) == 1 && C.TrainsIn(1) == 0);
    At(155.0);                       // nose past 150, tail still short of it
    assert(C.TrainsIn(0) == 1 && C.TrainsIn(1) == 1);   // straddling: holds both
    At(200.0);                       // fully into block 1
    assert(C.TrainsIn(0) == 0 && C.TrainsIn(1) == 1);
    std::printf("  forwards over a boundary: 0 -> straddle -> 1, correct\n");

    // ---- And now backwards over the same boundary. The train really is
    // returning to block 0, and the counter says something else.
    At(155.0);
    At(100.0);

    const int InZero = C.TrainsIn(0);
    const int InOne = C.TrainsIn(1);
    std::printf("  backwards over the same boundary: block 0 = %d, block 1 = %d"
                " (truth is 1 and 0)\n", InZero, InOne);

    // A rising edge means "metal arrived over me" and nothing more. The counter
    // reads it as "a nose entered the block ahead", which is true only while
    // trains go one way — reversing, the same edge is a train LEAVING that
    // block. So the crossing is counted the wrong way round twice over: block 1
    // is credited with a train that left it and block 0 debited for one that
    // arrived.
    assert(InZero != 1 || InOne != 0);   // it does NOT agree with the truth
    assert(InOne == 2 && InZero == -1);  // measured, so a change to it is visible

    // AND THE GOOD NEWS, WHICH IS THE REASON THIS TEST IS WORTH MORE THAN A
    // COMMENT: it fails toward being DETECTED rather than toward a plausible
    // wrong answer.
    //
    // Block 0 at -1 is below zero, which the counter already calls a LIE — "told
    // a train left somewhere it was never told one arrived" — and block 1 at 2
    // trips over-occupancy, which is the collision condition. Both are E-stop
    // conditions the ride already acts on. A reversing train therefore stops the
    // ride loudly instead of running with an interlocking that quietly believes
    // the wrong thing, which is the correct direction for this to be wrong in.
    assert(C.IsInconsistent());
    assert(C.IsOverOccupied(1));

    // THE FIX IS HARDWARE, and it is the reason this test measures the defect
    // rather than working around it: a real axle counter uses TWO heads a few
    // centimetres apart and reads the order of their edges. That is the same
    // idiom FSpeedTraps already uses — two switches and a surveyed gap — and it
    // belongs in the sensor layer, not as a rule bolted onto the counter.
    // See Docs/DIRECTION_AND_ROUTES.md.

}

int main()
{
    TestASensorIsAPointAndABoolean();
    TestWholeTrainPassedIsNotTheSameAsNoseArrived();
    TestOneSensorCannotBeClearedByTheWrongTrain();
    TestTheSeamIsJustAnotherStretchOfTrack();
    TestATrainCanCrossASensorInsideOneScan();
    TestTheCounterAgreesWithPerfectKnowledge();
    TestADeadSENSORIsWhatTheSecondMethodExISTSToCatch();
    TestAStuckOnSensorAndAChatteringOneAreDifferentFailures();
    TestAHealthySensorIsUNAFFECTEDByTheFaultMachinery();

    TestATrapMeasuresSpeedFromEDGESAndASurveyedGap();
    TestTheQUANTISATIONIsRealAndIsREPORTED();
    TestABackwardsPassIsCOUNTEDAndIsNotASpeed();
    TestATrainThatSTOPSBetweenTheSwitchesDisarmsTheTrap();
    TestWhatATrapIsFOR();
    TestTheCounterIsFORWARDONLYAndThisIsWhereItBreaks();

    std::printf("test_tracksensors: all assertions passed\n");
    return 0;
}
