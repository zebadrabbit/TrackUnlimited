// Build & run:  clang++ -std=c++17 -Wall -Wextra -O2 -o test_tracksensors test_tracksensors.cpp && ./test_tracksensors
//
// Drives FTrackSensors with bare numbers. No FTrack, no FTrain, no engine — the
// same discipline as test_ridesignals, and for the same reason: a sensor is a
// point and a boolean, and anything it needs beyond that is a design mistake.

#include "TrackSensors.h"

#include <cassert>
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

} // namespace

int main()
{
    TestASensorIsAPointAndABoolean();
    TestWholeTrainPassedIsNotTheSameAsNoseArrived();
    TestOneSensorCannotBeClearedByTheWrongTrain();
    TestTheSeamIsJustAnotherStretchOfTrack();
    TestATrainCanCrossASensorInsideOneScan();
    TestTheCounterAgreesWithPerfectKnowledge();

    std::printf("test_tracksensors: all assertions passed\n");
    return 0;
}
