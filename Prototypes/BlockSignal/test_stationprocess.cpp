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

    // 2 + 3 + 1 of dwell, plus a scan apiece for the phase to advance. The point
    // is that the total is the SUM of the steps rather than any one of them.
    assert(T > 5.9 && T < 6.2);

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

    std::printf("test_stationprocess: all assertions passed\n");
    return 0;
}
