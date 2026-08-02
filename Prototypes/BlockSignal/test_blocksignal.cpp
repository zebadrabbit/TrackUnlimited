// Unit tests for the Phase 0 block-signaling prototype.
// Build & run:  clang++ -std=c++17 -Wall -Wextra -o test_blocksignal test_blocksignal.cpp && ./test_blocksignal

#include "BlockSignal.h"

#include <cassert>
#include <cstdio>

static FBlockController MakeCircuit(std::size_t NumBlocks, double BufferSeconds)
{
    return FBlockController(std::vector<FBlockConfig>(NumBlocks, FBlockConfig{BufferSeconds}));
}

static void TestFullCycle()
{
    FBlockController C = MakeCircuit(3, 2.0);
    assert(C.GetState(0) == EBlockState::Clear);

    assert(C.OnTrainEnter(0));
    assert(C.GetState(0) == EBlockState::Occupied);

    assert(C.OnTrainExit(0));
    assert(C.GetState(0) == EBlockState::Buffer);
    assert(C.GetBufferRemaining(0) == 2.0);

    C.Tick(1.0);
    assert(C.GetState(0) == EBlockState::Buffer);
    assert(C.GetBufferRemaining(0) == 1.0);

    C.Tick(1.0);
    assert(C.GetState(0) == EBlockState::Clear);
    assert(C.GetBufferRemaining(0) == 0.0);
}

static void TestZeroBufferClearsImmediately()
{
    FBlockController C = MakeCircuit(2, 0.0);
    C.OnTrainEnter(1);
    assert(C.OnTrainExit(1));
    assert(C.GetState(1) == EBlockState::Clear);
}

static void TestEnterViolation()
{
    FBlockController C = MakeCircuit(2, 5.0);
    C.OnTrainEnter(0);
    // Second train entering an occupied block: violation reported, but the
    // block still reads OCCUPIED (physical truth).
    assert(!C.OnTrainEnter(0));
    assert(C.GetState(0) == EBlockState::Occupied);

    // Entering during buffer is also a violation.
    C.OnTrainExit(0);
    assert(C.GetState(0) == EBlockState::Buffer);
    assert(!C.OnTrainEnter(0));
    assert(C.GetState(0) == EBlockState::Occupied);
}

static void TestExitWithoutEntryRejected()
{
    FBlockController C = MakeCircuit(2, 1.0);
    assert(!C.OnTrainExit(0));
    assert(C.GetState(0) == EBlockState::Clear);
}

static void TestDispatchPermissive()
{
    // Station at block 0, circuit of 4.
    FBlockController C = MakeCircuit(4, 1.0);
    C.OnTrainEnter(0); // train sitting in the station

    assert(C.CanDispatch(0));        // block 1 clear
    assert(C.CanDispatch(0, 3));     // blocks 1,2,3 all clear

    // Another train occupies block 2: 1-block lookahead still permissive,
    // 2-block lookahead is not.
    C.OnTrainEnter(2);
    assert(C.CanDispatch(0, 1));
    assert(!C.CanDispatch(0, 2));

    // Buffer counts as not-clear for permissives.
    C.OnTrainExit(2);
    assert(C.GetState(2) == EBlockState::Buffer);
    assert(!C.CanDispatch(0, 2));

    // Once the overlap elapses, permissive opens up.
    C.Tick(1.0);
    assert(C.CanDispatch(0, 2));
}

static void TestDispatchLookaheadWraps()
{
    FBlockController C = MakeCircuit(3, 1.0);
    C.OnTrainEnter(0);
    // Lookahead from block 2 wraps to block 0, which is occupied.
    assert(!C.CanDispatch(2, 1));
    // Lookahead >= circuit size would include the asking train's own block: deny.
    assert(!C.CanDispatch(1, 3));
    assert(!C.CanDispatch(1, 99));
}

static void TestGuardsFailSafe()
{
    FBlockController C = MakeCircuit(4, 2.0);
    // Zero lookahead checks nothing; dispatching always means entering the next
    // block, so this must deny rather than wave the train through.
    assert(!C.CanDispatch(0, 0));

    C.OnTrainEnter(0);
    C.OnTrainExit(0);
    assert(C.GetState(0) == EBlockState::Buffer);
    // Neither a zero nor a negative dt may touch the countdown. (The same
    // comparison rejects NaN, which would otherwise latch the block forever.)
    C.Tick(0.0);
    C.Tick(-1.0);
    assert(C.GetBufferRemaining(0) == 2.0);
    assert(C.GetState(0) == EBlockState::Buffer);
}

int main()
{
    TestFullCycle();
    TestZeroBufferClearsImmediately();
    TestEnterViolation();
    TestExitWithoutEntryRejected();
    TestDispatchPermissive();
    TestDispatchLookaheadWraps();
    TestGuardsFailSafe();
    std::printf("All block-signal tests passed.\n");
    return 0;
}
