// Asserts for PlcUnit.h — the controller as a machine.
//
//   clang++ -std=c++17 -Wall -Wextra -o test_plcunit test_plcunit.cpp && ./test_plcunit

#include "PlcUnit.h"
#include "TrackDrives.h"

#include <cassert>
#include <cstdio>
#include <cstring>

namespace
{

// A powered controller with a matching program and a walked course — the state
// a real one is in when the ride opens.
FPlcUnit Ready(std::uint64_t Identity = 0xABCDEF)
{
    FPlcUnit P;
    P.PowerOn();
    P.SetLayoutIdentity(Identity);
    P.LoadProgram(Identity);
    P.DeclareCourseClear();
    return P;
}

void TestAFreshControllerWillNotRun()
{
    // POWER-UP IS NOT A RESTORE. Block occupancy is derived from edges the
    // controller watched happen, and one that was off watched nothing — so on
    // power-up it knows where no train is. Real practice is a manual course-clear
    // walkdown and an operator reset, and a machine that skipped it would be
    // inventing occupancy it cannot possibly know.
    FPlcUnit P;
    assert(!P.IsPowered());
    assert(!P.OutputsEnabled());

    P.PowerOn();
    assert(P.IsPowered());
    assert(P.GetMode() == EPlcMode::Stopped);
    assert(!P.IsCourseClear());
    assert(!P.RequestMode(EPlcMode::Run));
    assert(std::strcmp(P.WhyNotRun(), "no program loaded") == 0);

    P.SetLayoutIdentity(7);
    P.LoadProgram(7);
    assert(!P.RequestMode(EPlcMode::Run));
    assert(std::strcmp(P.WhyNotRun(), "course not declared clear") == 0);

    assert(P.DeclareCourseClear());
    assert(P.RequestMode(EPlcMode::Run));
    assert(P.OutputsEnabled());
    std::printf("  a fresh controller refuses RUN until the course is walked\n");
}

void TestAPowerCycleForgetsTheCourse()
{
    // The Tier 1 requirement that was previously unimplementable, because
    // nothing could be switched off.
    FPlcUnit P = Ready();
    assert(P.RequestMode(EPlcMode::Run));
    assert(P.OutputsEnabled());

    P.PowerOff();
    P.PowerOn();
    assert(!P.IsCourseClear());
    assert(!P.OutputsEnabled());
    assert(!P.RequestMode(EPlcMode::Run));
    std::printf("  a power cycle forgets the walkdown, as it must\n");
}

void TestAProgramForTheWrongLayoutREFUSESToRun()
{
    // THE MOST IMMEDIATELY USEFUL THING HERE. A program built against a
    // different track is the failure behind "I changed the code and the editor
    // is still doing the old thing" — a class that has bitten this project and
    // had no detector whatsoever.
    //
    // Both identities are DERIVED from the block and zone walk rather than
    // typed, so they cannot drift apart through somebody forgetting to update
    // one of them.
    FPlcUnit P;
    P.PowerOn();
    P.SetLayoutIdentity(0x1111);
    P.LoadProgram(0x2222);          // built for a different track
    P.DeclareCourseClear();

    assert(!P.ProgramMatchesLayout());
    assert(!P.RequestMode(EPlcMode::Run));
    assert(std::strcmp(P.WhyNotRun(), "program does not match this layout") == 0);
    assert(!P.OutputsEnabled());

    // Rebuild it for the track it is actually on.
    P.LoadProgram(0x1111);
    assert(P.RequestMode(EPlcMode::Run));
    assert(P.OutputsEnabled());
    std::printf("  a program built for another layout refuses to run\n");
}

void TestPROGRAMModeStopsTheRide()
{
    // The honest home for "maintenance is working on it", which had nowhere to
    // be before. Not a fault, not an E-stop — a key turned deliberately.
    FPlcUnit P = Ready();
    P.RequestMode(EPlcMode::Run);
    assert(P.OutputsEnabled());

    assert(P.RequestMode(EPlcMode::Program));
    assert(!P.OutputsEnabled());
    assert(P.GetMode() == EPlcMode::Program);

    // And back, without another walkdown — the course was never left, and
    // demanding one for a key turn would make PROGRAM cost more than it should.
    assert(P.RequestMode(EPlcMode::Run));
    assert(P.OutputsEnabled());
    std::printf("  PROGRAM mode holds the ride, and returns without a re-walk\n");
}

void TestLeavingRunIsNEVERRefused()
{
    // A mode change out of RUN that could be refused would be a stop with a
    // precondition, and stops do not have those. Asserted from the worst state
    // available: faulted, mismatched, and with no course clear.
    FPlcUnit P = Ready();
    P.RequestMode(EPlcMode::Run);
    P.Scan(1.0, true);                     // watchdog trip
    P.LoadProgram(0xDEAD);                 // and now the program is wrong too

    assert(P.RequestMode(EPlcMode::Stopped));
    assert(P.GetMode() == EPlcMode::Stopped);
    std::printf("  leaving RUN is never refused, whatever else is wrong\n");
}

void TestTheWatchdogIsAFaultAndItLatches()
{
    // A missed scan deadline is a FAULT on real hardware, not a late scan. The
    // detector already existed one layer up — the accumulator drops a backlog it
    // cannot work off — and reporting that as a note rather than a trip was the
    // controller having a symptom with nowhere to put it.
    FPlcUnit P = Ready();
    P.RequestMode(EPlcMode::Run);

    for (int i = 0; i < 100; ++i) { P.Scan(1.0 / 240.0, false); }
    assert(!P.IsFaulted());
    assert(P.OutputsEnabled());
    assert(P.ScanCount() == 100);

    P.Scan(0.4, true);
    assert(P.IsFaulted());
    assert(!P.OutputsEnabled());

    // LATCHED. Scans coming in on time afterwards do not clear it — a watchdog
    // that healed itself is one nobody ever sees, and being seen is the point.
    for (int i = 0; i < 1000; ++i) { P.Scan(1.0 / 240.0, false); }
    assert(P.IsFaulted());

    // And it takes a person, who has to stop the machine to do it. Clearing a
    // fault on something still executing is clearing it without having looked.
    assert(!P.ClearFault());
    P.RequestMode(EPlcMode::Stopped);
    assert(P.ClearFault());
    assert(!P.IsFaulted());
    std::printf("  the watchdog trips, latches, and needs a person and a stop\n");
}

void TestAFaultedControllerCannotPreventASTOP()
{
    // CONSTRAINT 7, MADE STRUCTURAL RATHER THAN PROMISED.
    //
    // This is the standard PLC, not the safety chain. It may withhold permission
    // to RUN; it has no authority over stopping, and there must be no path here
    // by which it could acquire one. The E-stop lives in FTrackDrives and the
    // brakes are fail-safe, so neither routes through this class.
    //
    // Asserted against the real drive layer rather than by reading the code: a
    // controller in its worst possible state, and the stop still works.
    FPlcUnit P;
    P.PowerOn();
    P.Scan(1.0, true);                  // faulted
    assert(P.IsFaulted());
    assert(!P.OutputsEnabled());

    FTrackDrives D(2);
    D.Preset(0, 20.0);
    D.Preset(1, 6.0);
    assert(D.Output(0) == 20.0);

    // The stop is not asked to consult the controller, and does not.
    assert(D.TripEmergencyStop("with the PLC down"));
    assert(D.IsEmergencyStopped());
    assert(D.Output(0) == 0.0);
    assert(D.Output(1) == 0.0);
    std::printf("  a dead controller cannot prevent an E-stop — constraint 7 holds\n");
}

void TestANotCommandingControllerLeavesTheRideSAFE()
{
    // THE ONE THE RIDE FOUND, an hour after this file was written.
    //
    // A caller that reacts to !OutputsEnabled() by simply SKIPPING its dispatch
    // logic has not stopped commanding — it has left the LAST command standing.
    // On a ride that has just opened that is each zone's preset, so every brake
    // sits at its release speed and every train leaves. Measured: three
    // signalling violations 1.47 s after a watchdog trip, from a controller that
    // had faulted and was doing nothing.
    //
    // The header of PlcUnit.h claimed a device with no command falls to its safe
    // state "exactly as it would with the cabinet unplugged". Nothing made that
    // true. This asserts the shape a caller has to implement for it to be.
    FPlcUnit P = Ready();
    P.RequestMode(EPlcMode::Run);

    FTrackDrives D(3);
    D.Preset(0, 20.0);          // a brake at its release speed
    D.Preset(1, 6.0);
    D.Preset(2, 4.0);
    assert(P.OutputsEnabled());
    assert(D.Output(0) == 20.0);

    // The watchdog trips. In a real cabinet the PLC's output card de-energises,
    // the drives lose their enable, and the brakes bite because they are
    // spring-applied.
    P.Scan(0.4, true);
    assert(P.IsFaulted());
    assert(!P.OutputsEnabled());

    // WHAT A CALLER MUST DO. Not "skip the dispatcher" — command zero.
    if (!P.OutputsEnabled())
    {
        for (std::size_t z = 0; z < D.Num(); ++z) { D.Command(z, 0.0); }
    }
    D.Tick(1.0 / 240.0);

    for (std::size_t z = 0; z < D.Num(); ++z)
    {
        assert(D.Output(z) == 0.0);
    }
    std::printf("  a controller that is not commanding leaves every drive at zero\n");
}

void TestOutputsEnabledIsPermissionNotCompulsion()
{
    // It PERMITS; it does not compel. False here means the controller is
    // commanding nothing, which stops a ride only because a device with no
    // command falls to its safe state — exactly as it would with the controller
    // unplugged. Nothing in this class reaches out and stops anything.
    FPlcUnit P = Ready();
    assert(!P.OutputsEnabled());        // Stopped: no commands
    P.RequestMode(EPlcMode::Run);
    assert(P.OutputsEnabled());
    P.PowerOff();
    assert(!P.OutputsEnabled());        // unplugged reads the same as stopped
    std::printf("  outputs enabled is permission, not compulsion\n");
}

} // namespace

int main()
{
    std::printf("PlcUnit: the controller as a machine in the cabinet\n\n");

    TestAFreshControllerWillNotRun();
    TestAPowerCycleForgetsTheCourse();
    TestAProgramForTheWrongLayoutREFUSESToRun();
    TestPROGRAMModeStopsTheRide();
    TestLeavingRunIsNEVERRefused();
    TestTheWatchdogIsAFaultAndItLatches();
    TestAFaultedControllerCannotPreventASTOP();
    TestANotCommandingControllerLeavesTheRideSAFE();
    TestOutputsEnabledIsPermissionNotCompulsion();

    std::printf("\ntest_plcunit: all assertions passed.\n");
    return 0;
}
