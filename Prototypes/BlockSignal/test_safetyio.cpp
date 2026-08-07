// Asserts for SafetyIo.h — Tier 1's wiring.
//
//   clang++ -std=c++17 -Wall -Wextra -O2 -o test_safetyio test_safetyio.cpp && ./test_safetyio
//
// Every assertion here is a FAILURE MODE OF THE WIRING rather than of the logic.
// The logic in this file is trivial; what is being checked is that the physical
// faults a real installation is built against land on the safe side.

#include "SafetyIo.h"
#include "TrackDrives.h"

#include <cassert>
#include <cstdio>
#include <cstring>

namespace
{

const double Dt = 1.0 / 240.0;

void Run(FSafetyRelay& R, int Scans, bool bResetHeld = false)
{
    for (int i = 0; i < Scans; ++i)
    {
        R.ScanReset(bResetHeld);
        R.Scan(Dt);
    }
}

// A relay that has been reset and is running, which is where a ride opens.
FSafetyRelay Running()
{
    FSafetyRelay R;
    R.Wiring().Release();
    Run(R, 240);
    Run(R, 10, true);      // press
    Run(R, 10, false);     // and release: the edge
    return R;
}

void TestABROKENWireIsAPRESSEDButton()
{
    // DE-ENERGISE TO TRIP, and it is the idea the whole discipline rests on.
    //
    // A safety input is a circuit that carries current while all is well, so a
    // stop is demanded by the ABSENCE of current. A broken wire, a pulled
    // connector, a dead power supply and a pressed button are the same signal —
    // and nothing here has to know which, which is the point.
    FSafetyRelay R = Running();
    assert(R.IsEnabled());
    assert(R.PowerFlows());

    // Somebody presses the button.
    R.Wiring().Press();
    Run(R, 1);
    assert(!R.IsEnabled());
    assert(!R.PowerFlows());

    // And now instead: nobody touches anything, and the cable is cut. The
    // normally-closed channel goes dead and the normally-open channel goes dead
    // with it — no current anywhere.
    FSafetyRelay S = Running();
    assert(S.IsEnabled());
    S.Wiring().Wire(false, false);      // both loops dead
    Run(S, 1);
    assert(!S.IsEnabled());
    assert(!S.PowerFlows());
    std::printf("  a broken wire reads exactly like a pressed button\n");
}

void TestASHORTToTheSupplyIsCaughtByOPPOSITEPolarity()
{
    // THE FAULT THAT ONE CHANNEL CANNOT SEE, and the reason real dual-channel
    // inputs use opposite polarity rather than simply doubling.
    //
    // Chafe a cable against the 24 V rail and a normally-closed channel reads
    // energised for ever — which reads as SAFE, which means a pressed E-stop does
    // nothing at all. Two channels of the SAME polarity do not help, since one
    // short in a shared loom takes both.
    //
    // Opposite polarity does: the fault that makes one read safe makes the other
    // read demanded, and a disagreement is a fault. That is the difference between
    // redundancy and diversity.
    FSafetyRelay R = Running();
    assert(R.IsEnabled());

    // Channel A is shorted to the supply: it will read energised whatever happens.
    // Then the button is pressed. A alone would say "all well".
    R.Wiring().Wire(true, true);        // A stuck energised, B says stop
    assert(R.Wiring().ChannelA().DemandsStop() == false);
    assert(R.Wiring().ChannelB().DemandsStop() == true);

    // EITHER channel demanding is a stop. Not both — a pair that required
    // agreement would be a system where one broken wire disables the E-stop.
    Run(R, 1);
    assert(!R.IsEnabled());

    // And held there, the discrepancy itself latches a fault.
    Run(R, 240);
    assert(R.Wiring().IsFaulted());
    assert(std::strstr(R.Wiring().FaultReason(), "discrepancy") != nullptr);
    std::printf("  a short that fools one channel is caught by the other's polarity\n");
}

void TestTheDISCREPANCYWindowExistsBecauseContactsAreNotSimultaneous()
{
    // A dual-channel button's two contact blocks are mechanically separate and
    // never switch on the same millisecond. An INSTANT comparison faults on every
    // legitimate press, which is why real relays carry a window — typically 0.5 s,
    // some 3 s.
    FSafetyRelay R(0.5);
    R.Wiring().Release();
    Run(R, 240);
    Run(R, 10, true);
    Run(R, 10, false);
    assert(R.IsEnabled());

    // Channel A opens 50 ms before channel B closes. Entirely normal.
    R.Wiring().Wire(false, false);
    Run(R, 12);                          // 50 ms of disagreement
    assert(!R.Wiring().IsFaulted());
    R.Wiring().Wire(false, true);        // B catches up
    Run(R, 1);
    assert(!R.Wiring().IsFaulted());
    assert(!R.IsEnabled());              // and it did stop, throughout

    // Beyond the window it is a fault, because a contact that never caught up is
    // a contact that is not going to.
    FSafetyRelay S(0.5);
    S.Wiring().Release();
    Run(S, 240);
    S.Wiring().Wire(false, false);
    Run(S, 240);                         // one full second
    assert(S.Wiring().IsFaulted());
    std::printf("  channels may disagree for 50 ms and may not for a second\n");
}

void TestAWELDEDContactorBLOCKSTheReset()
{
    // EXTERNAL DEVICE MONITORING, and FAULTS.md lists this as caught by nothing.
    //
    // A relay that de-energises its output has done its job only if the CONTACTOR
    // actually opened. One whose contacts have welded stays closed with its coil
    // dead, so the motor keeps turning and the relay is looking at its own output
    // rather than at the machine.
    //
    // Real safety contactors carry MIRROR CONTACTS: a normally-closed auxiliary
    // mechanically linked to the mains, so it can only close when they are
    // genuinely open. Wire those into the reset circuit and a welded contactor
    // cannot be reset — the reset becomes a question the machine answers.
    FSafetyRelay R = Running();
    assert(R.IsEnabled() && R.PowerFlows());

    R.Contactors().Weld(true, false);   // K1 sticks shut
    R.Wiring().Press();
    Run(R, 1);

    // The relay believes it removed power. Two contactors in series means one
    // weld is a LATENT fault rather than an immediate danger — which is exactly
    // why there are two, and exactly why the latent one has to be found.
    assert(!R.IsEnabled());
    assert(!R.PowerFlows());            // K2 still opened
    assert(R.Contactors().AnyWelded());

    // Now the button is released and somebody tries to restart the ride.
    R.Wiring().Release();
    Run(R, 10);
    assert(!R.CanReset());
    assert(std::strstr(R.WhyNotReset(), "welded") != nullptr);

    Run(R, 10, true);
    Run(R, 10, false);                  // a perfectly good reset edge
    assert(!R.IsEnabled());             // and it does not complete

    // It stays refused for ever, rather than timing out or being overridable.
    for (int i = 0; i < 20; ++i) { Run(R, 5, true); Run(R, 5, false); }
    assert(!R.IsEnabled());
    std::printf("  a welded contactor blocks the reset, permanently and by wiring\n");
}

void TestBOTHWeldedIsTheDANGEROUSCaseAndItIsVISIBLE()
{
    // One weld is latent. TWO is power reaching the machine after the relay
    // dropped out — the failure the second contactor exists to prevent, and the
    // reason a latent weld must be found before the next one happens.
    //
    // Nothing in this project could previously SAY this: the drives model an
    // output going to zero, not a contact that failed to open.
    FSafetyRelay R = Running();
    R.Contactors().Weld(true, true);
    R.Wiring().Press();
    Run(R, 1);

    assert(!R.IsEnabled());             // the relay did everything right
    assert(R.PowerFlows());             // and the machine is still live
    assert(R.OutputDisagreesWithCommand());
    std::printf("  two welded contactors: the relay is off and the machine is not, and it says so\n");
}

void TestTheResetIsMONITOREDAndCannotBeTiedDown()
{
    // Same anti-tie-down shape as the E-stop in FTrackDrives and CiA 402's fault
    // reset: a rising edge, acted on at RELEASE, and a button already held when
    // the trip happened is not an edge.
    FSafetyRelay R = Running();
    R.Wiring().Press();
    Run(R, 1);
    assert(!R.IsEnabled());

    // Taped down before the button was even released. Ten thousand scans.
    R.Wiring().Release();
    for (int i = 0; i < 10000; ++i) { R.ScanReset(true); R.Scan(Dt); }
    assert(!R.IsEnabled());

    // It takes a person letting go.
    Run(R, 5, false);
    Run(R, 5, true);
    Run(R, 5, false);
    assert(R.IsEnabled());
    std::printf("  the reset needs a release, and 10,000 held scans do nothing\n");
}

void TestSTOPOVERRIDESSTARTInBothOrders()
{
    // Believed true everywhere in this project and asserted nowhere until now.
    //
    // The demand is evaluated first and unconditionally in Scan(), so there is no
    // path by which a reset in the same scan can put the output back on — not
    // because a check forbids it but because the branch is unreachable.
    for (int Order = 0; Order < 2; ++Order)
    {
        FSafetyRelay R = Running();
        assert(R.IsEnabled());

        // A stop demanded and a reset attempted on the same scan, wired in both
        // orders, because "it happens to work" and "it cannot not work" look the
        // same from one direction.
        if (Order == 0)
        {
            R.Wiring().Press();
            R.ScanReset(true);
        }
        else
        {
            R.ScanReset(true);
            R.Wiring().Press();
        }
        R.Scan(Dt);
        assert(!R.IsEnabled());

        R.ScanReset(false);
        R.Scan(Dt);
        assert(!R.IsEnabled());      // and the release does not restart it either
    }
    std::printf("  stop overrides start, in both orders, on the same scan\n");
}

void TestARESTRAINTFailsTheOTHERWay()
{
    // The exception, and it is the exception in every park. Everything else here
    // fails safe by REMOVING power; a lap bar that dropped open on a power failure
    // would be the worst possible failure, so the safe state is LOCKED and it
    // takes positive present energy to unlock.
    //
    // That inverts the rule rather than breaking it: fail-safe was never
    // "de-energise", it is "fail to the state that cannot hurt anybody".
    FRestraintLock L;
    L.SetPowered(true);
    assert(L.IsLocked());               // locked until told otherwise

    L.Command(true);
    assert(L.IsUnlocked());

    // The power goes. The bar does not.
    L.SetPowered(false);
    assert(L.IsLocked());
    assert(L.IsLockedByPowerLoss());    // and a maintainer can tell which

    // Bars down because the ride wants them down reads differently from bars down
    // because they could not move if asked. Two identical-looking states.
    L.Command(false);
    assert(L.IsLocked());
    assert(!L.IsLockedByPowerLoss());
    std::printf("  a restraint needs power to UNLOCK, and says which kind of locked it is\n");
}

void TestLOSINGAPairIsItselfAFault()
{
    // Two devices of DIFFERENT kinds watching the same fact, because two identical
    // sensors share a failure mode and two different ones mostly do not.
    //
    // The part usually missed: if one dies and the other still reads safe, that
    // safe reading is no longer trustworthy — it is a single point of failure
    // wearing the appearance of a checked one.
    FDiversePair P;
    for (int i = 0; i < 240; ++i) { P.Report(true, true); P.Scan(Dt); }
    assert(P.IsSafe());

    // The photo eye fails dark while the proximity switch still says all is well.
    for (int i = 0; i < 240; ++i) { P.Report(true, false); P.Scan(Dt); }
    assert(!P.IsSafe());
    assert(P.HasLostThePair());

    // And it does NOT come back when they agree again, because the disagreement
    // was evidence about the measurement rather than about the gate.
    for (int i = 0; i < 240; ++i) { P.Report(true, true); P.Scan(Dt); }
    assert(!P.IsSafe());
    P.Reset();
    P.Report(true, true);
    P.Scan(Dt);
    assert(P.IsSafe());
    std::printf("  losing one of a diverse pair is a fault, not a fallback to the other\n");
}

void TestTheRELAYSitsUNDERTheDrivesRatherThanBesideThem()
{
    // CONSTRAINT 7, at the bottom of the stack. FTrackDrives already holds the
    // E-stop and its stop categories; this is the wiring UNDER that, and the two
    // must not be able to disagree about whether the machine is live.
    //
    // Asserted against the real drive layer: a relay that has dropped out means
    // the drives' output reaches nothing, whatever the drives think they are
    // commanding.
    FSafetyRelay R = Running();
    FTrackDrives D(2);
    D.Preset(0, 20.0);
    D.Preset(1, 6.0);
    assert(R.PowerFlows());
    assert(D.Output(0) == 20.0);

    // The button goes. Both layers act, independently and for different reasons —
    // the drives because the stop is latched inside them, the contactors because
    // the coil lost its circuit.
    R.Wiring().Press();
    Run(R, 1);
    D.TripEmergencyStop("the same button");     // Cat 0: power away at once

    assert(!R.PowerFlows());
    assert(D.Output(0) == 0.0);
    assert(D.IsEmergencyStopped());

    // ---- AND CAT 1 IS WHERE THE TWO LAYERS HAVE TO AGREE.
    //
    // The operator's button is the only Cat 1 on the ride: a controlled stop with
    // power RETAINED to achieve it, then removed. That is impossible if the same
    // button opens the contactors instantly — there would be nothing left to ramp
    // with, and Cat 1 would be a Cat 0 wearing a label.
    //
    // So the relay carries a DELAYED output, which is what a real one is for, and
    // its delay is the same 5 s deadline the drives already hold. The relay is
    // commanding stop throughout: only PowerFlows() lags.
    FSafetyRelay Delayed;
    Delayed.SetOffDelaySeconds(5.0);
    Delayed.Wiring().Release();
    Run(Delayed, 240);
    Run(Delayed, 10, true);
    Run(Delayed, 10, false);
    assert(Delayed.IsEnabled() && Delayed.PowerFlows());

    Delayed.Wiring().Press();
    Run(Delayed, 1);
    assert(!Delayed.IsEnabled());        // commanded to stop immediately
    assert(Delayed.PowerFlows());        // with power to do it with

    Run(Delayed, 240 * 4);               // four seconds in
    assert(Delayed.PowerFlows());
    Run(Delayed, 240 * 2);               // past the deadline
    assert(!Delayed.PowerFlows());       // and taken away, regardless

    // AND THE HARDER HALF: with the drives somehow still commanding, the machine
    // is still dead, because the contactors are open. That is what makes the
    // safety chain a chain rather than a policy — it does not route through
    // anything that could have a bug in it.
    FSafetyRelay S = Running();
    FTrackDrives E(1);
    E.Preset(0, 20.0);
    S.Wiring().Press();
    Run(S, 1);
    assert(E.Output(0) == 20.0);        // the drive is commanding away
    assert(!S.PowerFlows());            // and nothing is turning
    std::printf("  the relay is UNDER the drives: their output reaches nothing when it drops\n");
}

} // namespace

int main()
{
    std::printf("Tier 1: the safety chain, as wiring\n\n");

    TestABROKENWireIsAPRESSEDButton();
    TestASHORTToTheSupplyIsCaughtByOPPOSITEPolarity();
    TestTheDISCREPANCYWindowExistsBecauseContactsAreNotSimultaneous();
    TestAWELDEDContactorBLOCKSTheReset();
    TestBOTHWeldedIsTheDANGEROUSCaseAndItIsVISIBLE();
    TestTheResetIsMONITOREDAndCannotBeTiedDown();
    TestSTOPOVERRIDESSTARTInBothOrders();
    TestARESTRAINTFailsTheOTHERWay();
    TestLOSINGAPairIsItselfAFault();
    TestTheRELAYSitsUNDERTheDrivesRatherThanBesideThem();

    std::printf("\ntest_safetyio: all assertions passed.\n");
    return 0;
}
