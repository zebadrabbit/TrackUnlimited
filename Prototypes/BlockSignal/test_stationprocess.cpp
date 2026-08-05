// Build & run:  clang++ -std=c++17 -Wall -Wextra -O2 -o test_stationprocess test_stationprocess.cpp && ./test_stationprocess
//
// Drives FStationProcess with bare booleans. No FTrain, no track, no engine — the
// same discipline as the sensor and drive suites, and for the same reason: every
// gate at a station is a contact, and anything the logic needs beyond a set of
// contacts is a design mistake.

#include "StationProcess.h"

#include <cassert>
#include <cstdio>
#include <cstring>

namespace
{

const double Dt = 1.0 / 240.0;

// A train that has arrived and stopped at its mark.
FStationInputs Stopped()
{
    FStationInputs In;
    In.bTrainPresent = true;
    In.bTrainInPosition = true;
    return In;
}

void TestTheSequenceIsTheWholeThing()
{
    // ARRIVE, UNLOAD, LOAD, SECURE, READY. The order matters and each step waits
    // for its own contact, so nothing here can be satisfied by time passing.
    FStationProcess S(EStationRole::Combined);
    assert(S.GetPhase() == EStationPhase::Empty);
    assert(!S.IsReadyToDispatch());

    FStationInputs In;
    In.bTrainPresent = true;
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Arriving);
    assert(std::strcmp(S.WhatIsHolding(), "train not in position") == 0);

    In = Stopped();
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Unloading);
    assert(std::strcmp(S.WhatIsHolding(), "unloading") == 0);

    // Loading cannot be reported complete before unloading is — the contact is
    // there, but the sequence has not reached it.
    In.bLoadComplete = true;
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Unloading);

    In.bUnloadComplete = true;
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Loading);
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Securing);
    assert(std::strcmp(S.WhatIsHolding(), "restraints not locked") == 0);
    assert(!S.IsReadyToDispatch());

    In.bRestraintsLocked = true;
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Securing);
    assert(std::strcmp(S.WhatIsHolding(), "platform not clear") == 0);

    In.bPlatformClear = true;
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Ready);
    assert(S.IsReadyToDispatch());
    assert(std::strcmp(S.WhatIsHolding(), "") == 0);
}

void TestReadyIsNotLatched()
{
    // THE SAFETY PROPERTY. A permissive on a real PLC is a condition evaluated
    // every scan, not a state you arrive at and keep. A restraint popping open in
    // Ready has to take it away the same frame — and a station that latched would
    // show "ready" on the panel while the machine refused to dispatch, which is
    // precisely the disagreement a panel exists to prevent.
    FStationProcess S(EStationRole::Combined);
    FStationInputs In = Stopped();
    In.bUnloadComplete = true;
    In.bLoadComplete = true;
    In.bRestraintsLocked = true;
    In.bPlatformClear = true;

    S.Update(In);   // Unloading
    S.Update(In);   // Loading
    S.Update(In);   // Securing
    S.Update(In);   // Ready
    assert(S.IsReadyToDispatch());

    In.bRestraintsLocked = false;
    S.Update(In);
    assert(!S.IsReadyToDispatch());
    assert(S.GetPhase() == EStationPhase::Securing);
    assert(std::strcmp(S.WhatIsHolding(), "restraints not locked") == 0);

    // And it comes back, without redoing the load.
    In.bRestraintsLocked = true;
    S.Update(In);
    assert(S.IsReadyToDispatch());

    // The all-clear is the same: an operator withdrawing it stops the ride.
    In.bPlatformClear = false;
    S.Update(In);
    assert(!S.IsReadyToDispatch());
}

void TestTheDispatchIsLatchedEvenThoughReadinessIsNot()
{
    // THE PAIR, and it reads like a contradiction until you watch it deadlock. The
    // conditions are re-derived every scan and any of them can take the permission
    // away — right up until the train is released. After that it is MOVING, and
    // there is no such thing as un-dispatching a moving train.
    //
    // Leaving this out killed the real circuit. A released train rolls off its stop
    // mark, so it is no longer "in position", so the sequence is no longer
    // complete, so the permissive drops, so the dispatcher re-commands the brake —
    // and the train stops with its tail still over its own mark and sits there for
    // the rest of the session with nothing reporting anything wrong. The launch
    // drive saw no train at all in seven minutes.
    FStationProcess S(EStationRole::Combined);
    FStationInputs In = Stopped();
    In.bUnloadComplete = In.bLoadComplete = In.bRestraintsLocked = In.bPlatformClear = true;
    for (int i = 0; i < 4; ++i) { S.Update(In); }
    assert(S.IsReadyToDispatch());

    // Rolling: off the mark, still on the platform, and STILL PERMITTED.
    In.bTrainInPosition = false;
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Departing);
    assert(S.IsReadyToDispatch());
    assert(std::strcmp(S.WhatIsHolding(), "") == 0);

    // And a train that stops again mid-departure — which is exactly what the
    // re-braking produced — keeps its permission rather than being trapped.
    In.bTrainInPosition = true;
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Departing);
    assert(S.IsReadyToDispatch());

    // Only leaving the platform ends it.
    In.bTrainPresent = false;
    S.Update(In);
    assert(!S.IsReadyToDispatch());
}

void TestTheNextTrainDoesAllOfItAgain()
{
    // A station that remembered the last train's checks would dispatch the next
    // one on them, which is a train leaving with the restraints of the train
    // before it. Everything resets when the platform empties.
    FStationProcess S(EStationRole::Combined);
    FStationInputs In = Stopped();
    In.bUnloadComplete = In.bLoadComplete = In.bRestraintsLocked = In.bPlatformClear = true;
    for (int i = 0; i < 4; ++i) { S.Update(In); }
    assert(S.IsReadyToDispatch());

    // Off the mark, still on the platform: DEPARTING, not arriving. Same two
    // inputs as a train rolling in, opposite meaning.
    In.bTrainInPosition = false;
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Departing);

    // Gone.
    In.bTrainPresent = false;
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Empty);

    // The next one, with every contact still asserted from the last train — the
    // worst case, and it must not shortcut. It walks the sequence again.
    In.bTrainPresent = true;
    In.bTrainInPosition = true;
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Unloading);
    assert(!S.IsReadyToDispatch());
}

void TestAnUnloadPlatformDoesNotWaitForRestraints()
{
    // The Space Mountain shape: riders get OFF here and ON somewhere else, so the
    // train leaves this platform EMPTY with its restraints open, ready to board at
    // the next one. A rule that demanded locked restraints here would deadlock the
    // ride — nobody is in the train to close them.
    FStationProcess S(EStationRole::Unload);
    assert(!S.NeedsLoad());
    assert(!S.NeedsRestraints());

    FStationInputs In = Stopped();
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Unloading);

    In.bUnloadComplete = true;
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Securing);

    // Restraints deliberately left OPEN. Only the all-clear is outstanding.
    assert(std::strcmp(S.WhatIsHolding(), "platform not clear") == 0);
    In.bPlatformClear = true;
    S.Update(In);
    assert(S.IsReadyToDispatch());
}

void TestALoadPlatformSkipsUnloadEntirely()
{
    // The other half. Nobody is aboard to get off, so waiting for an unload
    // confirmation is waiting for something that will never be asserted.
    FStationProcess S(EStationRole::Load);
    assert(!S.NeedsUnload());
    assert(S.NeedsRestraints());

    FStationInputs In = Stopped();
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Loading);
    assert(std::strcmp(S.WhatIsHolding(), "loading") == 0);

    In.bLoadComplete = true;
    In.bRestraintsLocked = true;
    In.bPlatformClear = true;
    S.Update(In);
    assert(S.GetPhase() == EStationPhase::Securing);
    S.Update(In);
    assert(S.IsReadyToDispatch());
}

void TestTheCrewIsTheOnlyThingRunningOnTime()
{
    // The stand-in for riders, and it is a separate object precisely so it can be
    // deleted. It writes the contacts; it never decides anything.
    FStationProcess S(EStationRole::Combined);
    FAutoStationCrew Crew;
    Crew.UnloadSeconds = 2.0;
    Crew.LoadSeconds = 3.0;
    Crew.SecureSeconds = 1.0;

    FStationInputs In;
    In.bTrainPresent = true;
    In.bTrainInPosition = true;

    double T = 0.0;
    while (!S.IsReadyToDispatch() && T < 60.0)
    {
        S.Update(In);
        Crew.Serve(S, In, Dt);
        T += Dt;
    }
    assert(S.IsReadyToDispatch());

    // 2 unload + 3 load + the securing step, plus a scan apiece for the phase to
    // advance. The point is that the total is the SUM of the steps rather than any
    // one of them.
    //
    // Securing is now 2 s rather than the crew's 1, because the RESTRAINT BARS take
    // 2 s to travel and that is hardware rather than a crew allowance. The
    // all-clear needs the bars locked AND the walk-round done, so the longer of the
    // two governs — which is the right shape: an operator cannot sign off bars that
    // are still moving, however quickly they walk.
    assert(T > 6.9 && T < 7.2);

    // The train leaves and the next one arrives to a clean platform: the crew's
    // contacts are withdrawn, so nothing is pre-confirmed.
    In.bTrainInPosition = false;
    In.bTrainPresent = false;
    S.Update(In);
    Crew.Serve(S, In, Dt);
    assert(!In.bRestraintsLocked);
    assert(!In.bLoadComplete);
    assert(!In.bUnloadComplete);
    assert(!In.bPlatformClear);
}

void TestNothingHappensUntilTheTrainStops()
{
    // A crew that started work on a moving train would have riders boarding one
    // that has not arrived. The dwell clock does not run in Arriving.
    FStationProcess S(EStationRole::Combined);
    FAutoStationCrew Crew;

    FStationInputs In;
    In.bTrainPresent = true;   // in the zone, still rolling
    for (int i = 0; i < 240 * 30; ++i)
    {
        S.Update(In);
        Crew.Serve(S, In, Dt);
    }
    assert(S.GetPhase() == EStationPhase::Arriving);
    assert(!In.bUnloadComplete);
    assert(!S.IsReadyToDispatch());
}

void TestManualModeChangesWhoDecidesTheTimingAndNothingElse()
{
    // THE ONE RULE THAT MATTERS ABOUT MANUAL MODE. It changes who decides WHEN,
    // never whether the safety logic can be bypassed — a design that let an
    // operator override a permissive would be unsafe and also a poor simulation of
    // a real system, where that is precisely what interlocks exist to prevent.
    FStationProcess S(EStationRole::Combined, EDispatchMode::Manual);
    FStationInputs In = Stopped();

    // Button held from the moment the train arrives, every step incomplete. It
    // must buy nothing at all.
    In.bDispatchRequest = true;
    for (int i = 0; i < 8; ++i) { S.Update(In); }
    assert(!S.IsReadyToDispatch());
    assert(std::strcmp(S.WhatIsHolding(), "unloading") == 0);

    // Everything done EXCEPT the press, which is now the only thing outstanding.
    In.bDispatchRequest = false;
    In.bUnloadComplete = In.bLoadComplete = In.bRestraintsLocked = In.bPlatformClear = true;
    for (int i = 0; i < 4; ++i) { S.Update(In); }
    assert(!S.IsReadyToDispatch());
    assert(std::strcmp(S.WhatIsHolding(), "waiting for dispatch") == 0);

    In.bDispatchRequest = true;
    S.Update(In);
    assert(S.IsReadyToDispatch());

    // And in automatic the same inputs go without anyone touching anything.
    FStationProcess A(EStationRole::Combined, EDispatchMode::Automatic);
    FStationInputs Auto = Stopped();
    Auto.bUnloadComplete = Auto.bLoadComplete = true;
    Auto.bRestraintsLocked = Auto.bPlatformClear = true;
    for (int i = 0; i < 4; ++i) { A.Update(Auto); }
    assert(A.IsReadyToDispatch());
}

void TestATiedDownDispatchButtonDispatchesNothing()
{
    // A wedged or taped control is the abuse real ride control designs against —
    // with two buttons far enough apart that one person cannot hold both. The
    // release rule is the cheap half of the same idea and catches the same thing:
    // the button must be let go and pressed again FOR EACH TRAIN.
    FStationProcess S(EStationRole::Combined, EDispatchMode::Manual);
    FStationInputs In = Stopped();
    In.bUnloadComplete = In.bLoadComplete = true;
    In.bRestraintsLocked = In.bPlatformClear = true;

    // Train one: released, then pressed. Goes.
    In.bDispatchRequest = false;
    for (int i = 0; i < 4; ++i) { S.Update(In); }
    In.bDispatchRequest = true;
    S.Update(In);
    assert(S.IsReadyToDispatch());

    // It leaves with the button STILL HELD, and the next train arrives to a button
    // that was never released.
    In.bTrainInPosition = false;
    S.Update(In);
    In.bTrainPresent = false;
    S.Update(In);

    In.bTrainPresent = true;
    In.bTrainInPosition = true;
    for (int i = 0; i < 6; ++i) { S.Update(In); }
    assert(!S.IsReadyToDispatch());
    assert(std::strcmp(S.WhatIsHolding(), "dispatch button not released") == 0);

    // Let go, press again: now it goes. Which is a human being present.
    In.bDispatchRequest = false;
    S.Update(In);
    In.bDispatchRequest = true;
    S.Update(In);
    assert(S.IsReadyToDispatch());
}

void TestARestraintIsCOMMANDEDAndCONFIRMED()
{
    // THE POINT OF SEPARATING THE SWITCH FROM THE SENSOR. On every real console
    // GATES and RESTRAINTS are selector switches and LOCK HARNESS is a button: the
    // operator COMMANDS them and sensors then confirm. A thing that has been told
    // to close is not a thing that HAS closed.
    FCommandedBank B;
    B.TravelSeconds = 2.0;
    B.Groups = 4;

    assert(!B.IsClosedAndLocked());
    B.Command(true);
    assert(B.IsCommandedClosed());
    assert(!B.IsClosedAndLocked());     // told, not done
    assert(B.IsInTransit());

    for (int i = 0; i < 240 * 3; ++i) { B.Tick(Dt); }
    assert(B.IsClosedAndLocked());
    assert(!B.IsInTransit());
    assert(B.GroupsConfirmed() == 4);

    // AND A STUCK GROUP NEVER GETS THERE. Commanded closed, three of four bars
    // down, and the bank correctly refuses to call itself locked - which is the
    // failure a walk-round exists to find and the one a single bool could not say.
    FCommandedBank Stuck;
    Stuck.TravelSeconds = 1.0;
    Stuck.Groups = 4;
    Stuck.Command(true);
    for (int i = 0; i < 240 * 10; ++i) { Stuck.Tick(Dt, 2); }
    assert(Stuck.IsCommandedClosed());
    assert(Stuck.GroupsConfirmed() == 3);
    assert(!Stuck.IsClosedAndLocked());
    assert(Stuck.IsInTransit());
}

void TestAStuckRestraintHoldsTheDispatchForEver()
{
    // WHY THIS WAS WORTH CHANGING. Under the old crew the lock was asserted on a
    // clock, so a bar that never closed was declared shut a second and a half later
    // and the train went. Now the contact comes from the bank's own sensors, and a
    // bar that will not travel holds the dispatch indefinitely.
    FStationProcess S(EStationRole::Combined);
    FAutoStationCrew Crew;
    Crew.UnloadSeconds = 1.0;
    Crew.LoadSeconds = 1.0;
    Crew.SecureSeconds = 1.0;
    Crew.Restraints.TravelSeconds = 1.0;
    Crew.StuckGroup = 1;                 // one bar will not come down

    FStationInputs In;
    In.bTrainPresent = true;
    In.bTrainInPosition = true;

    for (int i = 0; i < 240 * 60; ++i)   // a full minute of trying
    {
        S.Update(In);
        Crew.Serve(S, In, Dt);
    }
    assert(!S.IsReadyToDispatch());
    assert(S.GetPhase() == EStationPhase::Securing);
    assert(std::strcmp(S.WhatIsHolding(), "restraints not locked") == 0);
    assert(Crew.Restraints.IsCommandedClosed());     // the switch WAS thrown
    assert(!Crew.Restraints.IsClosedAndLocked());    // the bars did not arrive

    // Clear the fault and it completes, without redoing the load.
    Crew.StuckGroup = -1;
    for (int i = 0; i < 240 * 5; ++i)
    {
        S.Update(In);
        Crew.Serve(S, In, Dt);
    }
    assert(S.IsReadyToDispatch());
}

void TestAStuckGATEHoldsItToo()
{
    // GATES ARE THE SAME DEVICE IN A DIFFERENT PLACE, and the all-clear waits on
    // both. A gate that will not shut is somebody able to walk onto the track while
    // a train is dispatched, which is the thing gates exist to prevent — so it
    // holds the dispatch exactly as a stuck bar does.
    //
    // Worth its own test rather than trusting the shared type: the two banks are
    // commanded from the same line, and a copy-paste that ticked one and tested the
    // other would pass every restraint test in this file.
    FStationProcess S(EStationRole::Combined);
    FAutoStationCrew Crew;
    Crew.UnloadSeconds = 1.0;
    Crew.LoadSeconds = 1.0;
    Crew.SecureSeconds = 1.0;
    Crew.Restraints.TravelSeconds = 1.0;
    Crew.Gates.TravelSeconds = 1.0;
    Crew.StuckGate = 0;                  // one gate section jammed open

    FStationInputs In;
    In.bTrainPresent = true;
    In.bTrainInPosition = true;

    for (int i = 0; i < 240 * 30; ++i)
    {
        S.Update(In);
        Crew.Serve(S, In, Dt);
    }
    assert(!S.IsReadyToDispatch());

    // The BARS are fine — it is only the gate outstanding, which is why the two are
    // separate devices rather than one "platform secure" bool.
    assert(Crew.Restraints.IsClosedAndLocked());
    assert(!Crew.Gates.IsClosedAndLocked());
    assert(In.bRestraintsLocked);
    assert(!In.bPlatformClear);
    assert(std::strcmp(S.WhatIsHolding(), "platform not clear") == 0);

    Crew.StuckGate = -1;
    for (int i = 0; i < 240 * 5; ++i)
    {
        S.Update(In);
        Crew.Serve(S, In, Dt);
    }
    assert(S.IsReadyToDispatch());
}

} // namespace

int main()
{
    TestTheSequenceIsTheWholeThing();
    TestReadyIsNotLatched();
    TestTheDispatchIsLatchedEvenThoughReadinessIsNot();
    TestTheNextTrainDoesAllOfItAgain();
    TestAnUnloadPlatformDoesNotWaitForRestraints();
    TestALoadPlatformSkipsUnloadEntirely();
    TestTheCrewIsTheOnlyThingRunningOnTime();
    TestNothingHappensUntilTheTrainStops();
    TestManualModeChangesWhoDecidesTheTimingAndNothingElse();
    TestATiedDownDispatchButtonDispatchesNothing();
    TestARestraintIsCOMMANDEDAndCONFIRMED();
    TestAStuckRestraintHoldsTheDispatchForEver();
    TestAStuckGATEHoldsItToo();

    std::printf("test_stationprocess: all assertions passed\n");
    return 0;
}
