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

// Six 100 m blocks. Long enough that a 15 m train sits wholly inside one, so a
// straddle is something a test asks for rather than something it trips over.
std::vector<double> Wide() { return {0.0, 100.0, 200.0, 300.0, 400.0, 500.0}; }

void TestTwoTrainsCannotShareABlock()
{
    // The case the old single-train version SUPPRESSED. "Already in this block"
    // matched on the last train to call Update, so OnTrainEnter — the only
    // function that can report a violation — never ran, and Violations() read 0
    // with two trains confirmed co-resident.
    FRideSignals S(Wide(), 0.0, 1, 2);
    assert(S.NumTrains() == 2);

    assert(S.Update(0, 210.0, 225.0));    // A wholly inside block 2
    assert(S.GetState(2) == EBlockState::Occupied);

    assert(!S.Update(1, 205.0, 220.0));   // B rolls in on top of it
    assert(S.Violations() == 1);

    // Physical truth still wins, exactly as with one train: the block reads
    // OCCUPIED because trains really are inside it.
    assert(S.GetState(2) == EBlockState::Occupied);
    assert(S.OccupiedBy(0, 2));
    assert(S.OccupiedBy(1, 2));
}

void TestATrainDoesNotReleaseAnothersBlock()
{
    // Co-residency needs a violation to set up — that is what co-residency IS.
    // What is under test is the block state AFTERWARDS: the old exit loop walked
    // "the" old range, so one train moving on released blocks it had never been
    // in, and a block read CLEAR with a train parked in it.
    FRideSignals S(Wide(), 0.0, 1, 2);

    assert(S.Update(0, 290.0, 305.0));    // A straddles 2|3
    assert(!S.Update(1, 310.0, 325.0));   // B into 3, which A holds
    assert(S.Violations() == 1);

    assert(S.Update(0, 250.0, 265.0));    // A backs off wholly into block 2
    assert(S.GetState(3) == EBlockState::Occupied);   // B is still standing in it
    assert(S.GetState(2) == EBlockState::Occupied);

    // And it does release once the last train genuinely leaves.
    assert(S.Update(1, 410.0, 425.0));
    assert(S.GetState(3) == EBlockState::Clear);
    assert(S.Violations() == 1);          // no new violation from either move
}

void TestPermissiveIsKeyedToTheAskingTrain()
{
    FRideSignals S(Wide(), 0.0, 1, 2);

    assert(S.Update(1, 5.0, 20.0));       // B in the station, block 0
    assert(S.Update(0, 105.0, 120.0));    // A ahead in block 1 — and A updated LAST

    // THE FAIL-OPEN THIS REPLACED. With one shared range, "skip blocks the asking
    // train holds" read as "skip blocks the LAST train to update holds", so asked
    // on B's behalf it skipped A's block as B's own and GRANTED B entry into a
    // block A was standing in.
    assert(!S.CanDispatchInto(1, 1));
    assert(!S.CanRelease(1, 15.0));

    // And the exclusion still does its real job: A must not be denied its own
    // dispatch by the block A is standing in, or nothing ever clears it.
    assert(S.CanDispatchInto(0, 1));
    assert(S.CanRelease(0, 112.0));       // into block 2, which is clear

    // Once A moves on, B goes.
    assert(S.Update(0, 210.0, 225.0));
    assert(S.CanRelease(1, 15.0));
    assert(S.Violations() == 0);
}

void TestCanReleaseWrapsTheCircuit()
{
    // The one place a circuit is assumed rather than a point-to-point layout:
    // a train in the last block is released into the first.
    FRideSignals S(Wide(), 0.0, 1, 2);
    assert(S.Update(0, 510.0, 525.0));    // A in the last block, 5
    assert(S.CanRelease(0, 515.0));       // -> block 0, clear

    assert(S.Update(1, 5.0, 20.0));       // B now sitting in block 0
    assert(!S.CanRelease(0, 515.0));
    assert(S.Violations() == 0);
}

void TestSingleTrainFormsDenyRatherThanAlias()
{
    // A two-train caller that forgets its index would otherwise drive both trains
    // through slot 0 and see no interlocking at all — the exact failure this class
    // was rewritten to end. Deny instead, and change nothing while denying.
    FRideSignals S(Wide(), 0.0, 1, 2);
    assert(!S.Update(0.0, 15.0));
    assert(!S.CanDispatchInto(1));
    assert(S.GetState(0) == EBlockState::Clear);
    assert(!S.Occupies(0));
    assert(S.Violations() == 0);          // a denial is not a signalling violation

    // On a one-train instance they are the three-argument calls, unchanged. Every
    // other test in this file is that assertion.
    FRideSignals One(Wide(), 0.0, 1);
    assert(One.NumTrains() == 1);
    assert(One.Update(0.0, 15.0));
    assert(One.CanDispatchInto(1));
}

void TestUnknownTrainDenies()
{
    // A train this instance was not sized for has occupancy nowhere, so admitting
    // it would let it run the circuit invisibly. Fail closed, like everything else.
    FRideSignals S(Wide(), 0.0, 1, 2);
    assert(!S.Update(2, 0.0, 15.0));
    assert(!S.CanDispatchInto(2, 1));
    assert(!S.CanRelease(2, 15.0));
    assert(!S.OccupiedBy(2, 0));
    assert(S.GetState(0) == EBlockState::Clear);

    // Zero trains would deny every update and every permissive for the life of
    // the ride, with nothing anywhere saying why.
    const FRideSignals Zero(Wide(), 0.0, 1, 0);
    assert(Zero.NumTrains() == 1);
}

void TestOccupiesIsPerBlockAndPerTrain()
{
    FRideSignals S(Wide(), 0.0, 1, 2);
    assert(S.Update(0, 105.0, 120.0));
    assert(S.Update(1, 310.0, 325.0));

    // A lamp asks about the block; a dispatcher asks about the train.
    assert(S.Occupies(1) && S.Occupies(3));
    assert(!S.Occupies(2));
    assert(S.OccupiedBy(0, 1) && !S.OccupiedBy(1, 1));
    assert(S.OccupiedBy(1, 3) && !S.OccupiedBy(0, 3));
}

void TestTwoTrainsRoundACircuitCleanly()
{
    // The whole point: two trains, a lap each, one block apart, and nothing to
    // report. Trains only move when their own permissive grants, which is the
    // policy a dispatcher implements — asserted here on bare numbers so it is
    // pinned without FTrain.
    FRideSignals S(Wide(), 0.0, 2, 2);
    const double Len = 15.0, Total = 600.0, Step = 30.0;
    double At[2] = {0.0, 540.0};          // B one block behind A, on its heels
    int Holds = 0, Moves = 0;

    for (int Frame = 0; Frame < 400; ++Frame)
    {
        for (std::size_t t = 0; t < 2; ++t)
        {
            double Next = At[t] + Step;
            if (Next >= Total)
            {
                Next -= Total;            // the lap seam, tail and all
            }
            // A quarter of a block a step, so nothing is ever skipped whole, and
            // a train asks only when its nose is about to change block.
            const std::size_t Ahead = S.BlockAt(Next + Len);
            if (Ahead != S.BlockAt(At[t] + Len) && !S.CanDispatchInto(t, Ahead))
            {
                ++Holds;
                continue;
            }
            At[t] = Next;
            assert(S.Update(t, At[t], At[t] + Len));
            ++Moves;
        }
        S.Tick(1.0);                      // once per FRAME, not once per train
    }

    // Not vacuous: B really was held at a signal, repeatedly, and both trains
    // really did keep running. A version that granted everything would still
    // report zero violations here, so the hold count is the assertion that bites.
    assert(S.Violations() == 0);
    assert(Holds > 0);
    assert(Moves > 600);
}

void TestASeamStraddleHoldsTwoBlocksNotTheWholeRing()
{
    // On a circuit a train really can have its tail in the last block and its
    // nose in the first, and then rear > front. Sorting that pair — which is the
    // right thing on a strip — turns "I hold blocks 5 and 0" into "I hold 0
    // through 5", so ONE train claims the entire circuit and nothing else can
    // move anywhere, for ever, with no violation reported.
    FRideSignals S(Wide(), 0.0, 1, 2, true);

    assert(S.Update(0, 590.0, 5.0));
    assert(S.OccupiedBy(0, 5));
    assert(S.OccupiedBy(0, 0));
    for (std::size_t b = 1; b <= 4; ++b)
    {
        assert(!S.OccupiedBy(0, b));
    }
    assert(S.GetState(5) == EBlockState::Occupied);
    assert(S.GetState(0) == EBlockState::Occupied);
    assert(S.GetState(3) == EBlockState::Clear);

    // Tail round as well: the last block releases, exactly like any other exit.
    assert(S.Update(0, 10.0, 25.0));
    assert(S.GetState(5) == EBlockState::Clear);
    assert(S.OccupiedBy(0, 0));
    assert(S.Violations() == 0);
}

void TestASeamStraddleStillCollidesAndStillDenies()
{
    // The seam is not a hiding place. A train sitting in block 0 must still be
    // seen by one arriving across the seam, and must still deny it.
    FRideSignals S(Wide(), 0.0, 1, 2, true);

    assert(S.Update(1, 20.0, 35.0));          // B parked in block 0
    assert(!S.CanDispatchInto(0, 0));         // A may not be let into it
    assert(!S.Update(0, 590.0, 5.0));         // and arriving anyway is a violation
    assert(S.Violations() == 1);
    assert(S.OccupiedBy(0, 0) && S.OccupiedBy(1, 0));
}

void TestWithoutCircuitAReversedPairIsStillSorted()
{
    // Unchanged, and it has to be: a point-to-point layout has no seam, so a
    // reversed pair there is a caller passing its arguments the wrong way round.
    FRideSignals S(Wide(), 0.0, 1);
    assert(S.Update(120.0, 105.0));
    assert(S.OccupiedBy(0, 1));
    assert(!S.OccupiedBy(0, 0));
    assert(!S.OccupiedBy(0, 5));
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

    TestTwoTrainsCannotShareABlock();
    TestATrainDoesNotReleaseAnothersBlock();
    TestPermissiveIsKeyedToTheAskingTrain();
    TestCanReleaseWrapsTheCircuit();
    TestSingleTrainFormsDenyRatherThanAlias();
    TestUnknownTrainDenies();
    TestOccupiesIsPerBlockAndPerTrain();
    TestTwoTrainsRoundACircuitCleanly();
    TestASeamStraddleHoldsTwoBlocksNotTheWholeRing();
    TestASeamStraddleStillCollidesAndStillDenies();
    TestWithoutCircuitAReversedPairIsStillSorted();

    std::printf("test_ridesignals: all assertions passed\n");
    return 0;
}
