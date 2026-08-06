// Asserts for SignalWatch.h.
//
//   clang++ -std=c++17 -Wall -Wextra -o test_signalwatch test_signalwatch.cpp && ./test_signalwatch

#include "SignalWatch.h"

#include <cassert>
#include <cstdio>

namespace
{

void TestSteadyStateSaysNothing()
{
    // The whole reason this is edge-triggered. A SCADA log that wrote every point
    // every scan would be thirty lines a frame and unreadable within a second.
    FSignalWatch W;
    W.Changed(0, 7);                         // seed
    for (int i = 0; i < 100000; ++i)
    {
        assert(!W.Changed(0, 7));
    }
    std::printf("  100000 frames of an unchanging signal: nothing logged\n");
}

void TestTheFirstObservationIsASeedNotAChange()
{
    // WITHOUT THIS, FRAME ONE REPORTS EVERYTHING. Every block, platform, drive and
    // lamp differs from a default nobody set, so t=0 buries the first real event
    // under thirty transitions that did not happen. Same requirement FBlockCounter
    // has, and for the same reason: a detector with no baseline cannot tell "it
    // moved" from "I have just started looking".
    FSignalWatch W;
    assert(!W.Changed(0, 5));                // seeded at 5, not "0 -> 5"
    assert(!W.Changed(1, 0));                // and seeding at the DEFAULT still counts
    assert(W.IsSeeded(0) && W.IsSeeded(1));

    // Now it bites.
    assert(W.Changed(0, 6));
    assert(W.Changed(1, 1));
    std::printf("  first observation seeds; the second one reports\n");
}

void TestSeedingAtTheDefaultValueStillSeeds()
{
    // The subtle half of the above. A channel whose real first state happens to
    // equal the zero the vector was filled with must still be MARKED seeded —
    // otherwise it is indistinguishable from an unseeded one for ever, and the
    // first time it changes it seeds instead of reporting. That loses the first
    // real transition on every signal that starts at zero, which is most of them:
    // a block starts clear, a drive starts stopped, a lamp starts dark.
    FSignalWatch W;
    assert(!W.Changed(3, 0));
    assert(W.IsSeeded(3));
    assert(W.Changed(3, 1));                 // reported, not swallowed
    std::printf("  a channel that starts at zero still seeds, and its first "
                "change is not swallowed\n");
}

void TestChannelsAreIndependent()
{
    FSignalWatch W;
    for (std::size_t c = 0; c < 8; ++c) { W.Changed(c, 0); }

    assert(W.Changed(4, 1));
    for (std::size_t c = 0; c < 8; ++c)
    {
        if (c != 4) { assert(!W.Changed(c, 0)); }
    }
    assert(W.Peek(4) == 1);
    assert(W.Peek(5) == 0);
    std::printf("  one channel moving does not disturb its neighbours\n");
}

void TestItRemembersWhatItMovedFrom()
{
    // "READY -> DEPARTING" is a sentence; "DEPARTING" is half of one, and the
    // half that is missing is the one that says what just happened.
    FSignalWatch W;
    W.ChangedFrom(0, 2);                     // seed at 2
    assert(W.ChangedFrom(0, 5));
    assert(W.Previous() == 2);
    assert(W.ChangedFrom(0, 9));
    assert(W.Previous() == 5);
    std::printf("  reports what it moved FROM: 2 -> 5 -> 9\n");
}

void TestGrowsOnDemand()
{
    // A layout with more blocks simply observes more channels; a caller should
    // not have to count them up front and a rebuild changes the number.
    FSignalWatch W(2);
    assert(W.Num() == 2);
    assert(!W.Changed(50, 1));               // seeds, does not fall off the end
    assert(W.Num() >= 51);
    assert(W.Changed(50, 2));
    std::printf("  grows on demand to channel 50\n");
}

void TestForgetMakesTheNextObservationASeedAgain()
{
    // A REBUILD IS NOT A TRANSITION. After a layout changes, channel 4 is a
    // different block from the channel 4 that was being watched, and carrying the
    // old value across reports a change that never happened — on every channel
    // whose meaning moved, which is most of them.
    FSignalWatch W;
    W.Changed(0, 1);
    W.Changed(1, 2);
    assert(W.Changed(0, 9));                 // normal transition

    W.Forget();
    assert(!W.Changed(0, 3));                // seeds again rather than "9 -> 3"
    assert(!W.Changed(1, 7));
    assert(W.Changed(0, 4));                 // and works normally afterwards
    std::printf("  Forget() reseeds, so a rebuild does not manufacture "
                "transitions\n");
}

void TestAWorkedFrameLoop()
{
    // What the actor actually does: walk the same lists the panel walks, observe
    // each one, and log only what moved. Here a block goes occupied, then buffer,
    // then clear, over a hundred frames of nothing.
    FSignalWatch W;
    int Logged = 0;
    const int States[] = {0, 0, 0, 1, 1, 1, 1, 2, 2, 0, 0, 0};

    for (int Frame = 0; Frame < 12; ++Frame)
    {
        if (W.ChangedFrom(0, States[Frame])) { ++Logged; }
    }
    // Seeded clear, then clear->occupied, occupied->buffer, buffer->clear.
    assert(Logged == 3);
    std::printf("  twelve frames, three states, three log lines\n");
}

} // namespace

int main()
{
    std::printf("SignalWatch: what changed, and when\n\n");

    TestSteadyStateSaysNothing();
    TestTheFirstObservationIsASeedNotAChange();
    TestSeedingAtTheDefaultValueStillSeeds();
    TestChannelsAreIndependent();
    TestItRemembersWhatItMovedFrom();
    TestGrowsOnDemand();
    TestForgetMakesTheNextObservationASeedAgain();
    TestAWorkedFrameLoop();

    std::printf("\ntest_signalwatch: all assertions passed.\n");
    return 0;
}
