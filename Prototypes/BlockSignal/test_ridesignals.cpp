// Build & run:  clang++ -std=c++17 -Wall -Wextra -o test_ridesignals.exe test_ridesignals.cpp && ./test_ridesignals.exe
//
// Drives FRideSignals with bare numbers — no FTrack, no FTrain, no engine. That
// the whole layer is testable this way is the point of it taking doubles.

#include "RideSignals.h"

#include <cassert>
#include <cstdio>
#include <vector>

namespace
{

// The reference layout's own blocks: lift, course, brake over 591.72 m.
std::vector<double> RefBoundaries() { return {0.0, 160.674, 521.719}; }

void TestBlockAt()
{
    const FRideSignals S(RefBoundaries(), 0.0, 1);
    assert(S.NumBlocks() == 3);

    // Inclusive at the boundary that OPENS a block.
    assert(S.BlockAt(0.0) == 0);
    assert(S.BlockAt(160.673) == 0);
    assert(S.BlockAt(160.674) == 1);
    assert(S.BlockAt(521.718) == 1);
    assert(S.BlockAt(521.719) == 2);
    assert(S.BlockAt(591.72) == 2);

    // Cannot underflow, and does not run off the end. A negative S should not be
    // reachable, but returning block 0 is the only safe answer if it is.
    assert(S.BlockAt(-1.0) == 0);
    assert(S.BlockAt(1e9) == 2);
}

void TestRepairAndReporting()
{
    // IsWellFormed reports; the constructor repairs. Both exist so a caller can
    // choose, which is the split the rest of the project uses.
    assert(FRideSignals::IsWellFormed({0.0, 10.0, 20.0}));
    assert(!FRideSignals::IsWellFormed({}));
    assert(!FRideSignals::IsWellFormed({5.0, 10.0}));         // does not start at 0
    assert(!FRideSignals::IsWellFormed({0.0, 10.0, 10.0}));   // not unique
    assert(!FRideSignals::IsWellFormed({0.0, 20.0, 10.0}));   // not ascending

    // An empty list would otherwise index out of range on the first Update.
    const FRideSignals Empty({}, 0.0, 1);
    assert(Empty.NumBlocks() == 1);
    assert(Empty.Boundaries()[0] == 0.0);

    const FRideSignals Messy({20.0, 10.0, 10.0}, 0.0, 1);
    const std::vector<double> Want{0.0, 10.0, 20.0};
    assert(Messy.Boundaries() == Want);
}

void TestLookaheadIsClamped()
{
    // A lookahead at or past the block count would deny every dispatch for the
    // life of the ride, and the only symptom would be a ride that never starts.
    const FRideSignals Big(RefBoundaries(), 0.0, 99);
    assert(Big.Lookahead() == 2);          // 3 blocks -> at most 2

    const FRideSignals Zero(RefBoundaries(), 0.0, 0);
    assert(Zero.Lookahead() == 1);
}

void TestOccupancyFollowsTheTrain()
{
    FRideSignals S(RefBoundaries(), 0.0, 1);

    // Wholly inside block 0.
    assert(S.Update(0.0, 15.0));
    assert(S.GetState(0) == EBlockState::Occupied);
    assert(S.GetState(1) == EBlockState::Clear);

    // Straddling 0|1 holds BOTH. This is the case a point-mass model cannot
    // represent and the reason the layer takes a range at all.
    assert(S.Update(155.0, 170.0));
    assert(S.GetState(0) == EBlockState::Occupied);
    assert(S.GetState(1) == EBlockState::Occupied);

    // Fully into block 1: block 0 releases.
    assert(S.Update(200.0, 215.0));
    assert(S.GetState(0) == EBlockState::Clear);
    assert(S.GetState(1) == EBlockState::Occupied);
    assert(S.Violations() == 0);
}

void TestBufferWithholdsClear()
{
    FRideSignals S(RefBoundaries(), 5.0, 1);

    assert(S.Update(0.0, 15.0));
    assert(S.Update(200.0, 215.0));

    // The whole point of the buffer: the block does NOT go clear the instant the
    // tail crosses out of it.
    assert(S.GetState(0) == EBlockState::Buffer);
    assert(S.GetBufferRemaining(0) == 5.0);

    S.Tick(2.0);
    assert(S.GetState(0) == EBlockState::Buffer);
    S.Tick(3.1);
    assert(S.GetState(0) == EBlockState::Clear);

    // A zero-length or negative tick must not age the overlap. Guarded once, in
    // FBlockController; asserted here so a wrapper can never quietly re-add it.
    FRideSignals T(RefBoundaries(), 5.0, 1);
    assert(T.Update(0.0, 15.0));
    assert(T.Update(200.0, 215.0));
    T.Tick(0.0);
    T.Tick(-100.0);
    assert(T.GetBufferRemaining(0) == 5.0);
}

void TestRollbackIsSymmetric()
{
    // A train running out of energy and coming back down needs no direction
    // logic at all: the range diff is symmetric. If this ever needs a special
    // case, the range diff has been replaced with something worse.
    FRideSignals S(RefBoundaries(), 0.0, 1);

    assert(S.Update(155.0, 170.0));   // straddling forwards
    assert(S.GetState(0) == EBlockState::Occupied);
    assert(S.GetState(1) == EBlockState::Occupied);

    assert(S.Update(140.0, 155.0));   // rolled back, wholly into block 0
    assert(S.GetState(0) == EBlockState::Occupied);
    assert(S.GetState(1) == EBlockState::Clear);

    assert(S.Update(155.0, 170.0));   // and forwards again, re-entering 1
    assert(S.GetState(1) == EBlockState::Occupied);
    assert(S.Violations() == 0);
}

void TestLapEndTeleport()
{
    // Place(0, 0) at the end of a lap: the old range exits and arms its overlaps,
    // the new range enters. No special case, and no violation.
    FRideSignals S(RefBoundaries(), 5.0, 1);

    assert(S.Update(560.0, 575.0));   // in the brake run
    assert(S.GetState(2) == EBlockState::Occupied);

    assert(S.Update(0.0, 15.0));      // teleported to the station
    assert(S.GetState(2) == EBlockState::Buffer);
    assert(S.GetState(0) == EBlockState::Occupied);
    assert(S.Violations() == 0);
}

void TestViolationIsReported()
{
    FRideSignals S(RefBoundaries(), 30.0, 1);

    // Train A runs block 0 then leaves it holding a long overlap.
    assert(S.Update(0.0, 15.0));
    assert(S.Update(200.0, 215.0));
    assert(S.GetState(0) == EBlockState::Buffer);

    // Coming back into a block still holding its overlap is a violation. It is
    // reported, and physical truth still wins — the block reads OCCUPIED,
    // because a train really is inside it.
    assert(!S.Update(0.0, 15.0));
    assert(S.Violations() == 1);
    assert(S.GetState(0) == EBlockState::Occupied);
}

void TestDispatchPermissive()
{
    FRideSignals S(RefBoundaries(), 0.0, 1);

    // Train sitting in the station (block 0). Releasing it means entering block
    // 1, which is clear.
    assert(S.Update(0.0, 15.0));
    assert(S.CanDispatchInto(1));

    // THE CASE THAT MOTIVATES KEYING THE PERMISSIVE TO THE DESTINATION: a train
    // stopped short, straddling 0|1. Asking to be released INTO block 1 must not
    // be denied by block 1 being occupied BY THE ASKING TRAIN — nothing would
    // ever clear it, so the ride would deadlock permanently.
    assert(S.Update(155.0, 170.0));
    assert(S.GetState(1) == EBlockState::Occupied);
    assert(S.CanDispatchInto(1));

    // A block occupied by something else is a genuine denial. Simulated by
    // leaving an overlap running in the destination.
    FRideSignals T(RefBoundaries(), 30.0, 1);
    assert(T.Update(200.0, 215.0));   // occupy block 1
    assert(T.Update(0.0, 15.0));      // move away; block 1 holds its overlap
    assert(T.GetState(1) == EBlockState::Buffer);
    assert(!T.CanDispatchInto(1));
    T.Tick(31.0);
    assert(T.CanDispatchInto(1));
}

void TestDispatchFailsClosed()
{
    // Every degenerate query denies. A safety function that returns true when it
    // does not understand the question is the wrong way round, and this is the
    // direction FBlockController already fails in.
    FRideSignals S(RefBoundaries(), 0.0, 1);
    assert(!S.CanDispatchInto(3));      // out of range
    assert(!S.CanDispatchInto(99));

    // Lookahead 2 over three blocks: releasing into 1 needs 1 AND 2 clear. This
    // is the braking-distance rule as the prototype actually expresses it — a
    // count of blocks, not a computed stopping distance.
    FRideSignals L(RefBoundaries(), 30.0, 2);

    // Run a lap so block 2 is left holding an overlap, then come back to the
    // station. One train, so a not-clear block ahead has to be manufactured this
    // way rather than with a second train.
    assert(L.Update(560.0, 575.0));     // block 2
    assert(L.Update(0.0, 15.0));        // back in the station; 2 holds its overlap
    assert(L.GetState(2) == EBlockState::Buffer);

    // Block 1 is clear but block 2 is not, and the train is in block 0 so neither
    // is skipped. A lookahead of 1 would wrongly permit this; 2 catches it.
    assert(!L.CanDispatchInto(1));
    assert(L.Lookahead() == 2);
    L.Tick(31.0);
    assert(L.CanDispatchInto(1));
}

void TestKnownLimitBlockSkippedInOneStep()
{
    // Recorded, not fixed. A block crossed ENTIRELY between two Updates is never
    // entered and never arms its overlap. This asserts the limitation exists so
    // it is a known quantity rather than a surprise — if a future change makes
    // this pass through the middle block, this test SHOULD fail and be deleted.
    const std::vector<double> Tight{0.0, 10.0, 10.4, 200.0};
    FRideSignals S(Tight, 5.0, 1);

    assert(S.Update(0.0, 5.0));         // block 0
    assert(S.GetState(1) == EBlockState::Clear);

    assert(S.Update(20.0, 25.0));       // jumped clean over block 1 (0.4 m long)
    assert(S.GetState(2) == EBlockState::Occupied);
    assert(S.GetState(1) == EBlockState::Clear);   // never occupied, never buffered
    assert(S.Violations() == 0);
}

} // namespace

int main()
{
    TestBlockAt();
    TestRepairAndReporting();
    TestLookaheadIsClamped();
    TestOccupancyFollowsTheTrain();
    TestBufferWithholdsClear();
    TestRollbackIsSymmetric();
    TestLapEndTeleport();
    TestViolationIsReported();
    TestDispatchPermissive();
    TestDispatchFailsClosed();
    TestKnownLimitBlockSkippedInOneStep();

    std::printf("test_ridesignals: all assertions passed\n");
    return 0;
}
