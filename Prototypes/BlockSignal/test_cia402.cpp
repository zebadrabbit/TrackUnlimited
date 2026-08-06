// Asserts for Cia402.h — the drive state machine, as specified.
//
//   clang++ -std=c++17 -Wall -Wextra -o test_cia402 test_cia402.cpp && ./test_cia402

#include "Cia402.h"

#include <cassert>
#include <cstdio>

namespace
{

// Bring a drive up the way a PLC does, one rung at a time.
FCia402Drive Enabled()
{
    FCia402Drive D;
    D.Write(Cia402Cw::Shutdown);      // NotReady -> SwitchOnDisabled
    D.Write(Cia402Cw::Shutdown);      // -> ReadyToSwitchOn
    D.Write(Cia402Cw::SwitchOnCmd);   // -> SwitchedOn
    D.Write(Cia402Cw::EnableOp);      // -> OperationEnabled
    return D;
}

void TestOnlyOneStateProducesTorque()
{
    // The entire reason a drive has eight states rather than a bool.
    FCia402Drive D;
    assert(D.State() == ECia402State::NotReadyToSwitchOn);
    assert(!D.ProducesTorque());

    D.Write(Cia402Cw::Shutdown);
    assert(D.State() == ECia402State::SwitchOnDisabled);
    assert(!D.ProducesTorque());

    D.Write(Cia402Cw::Shutdown);
    assert(D.State() == ECia402State::ReadyToSwitchOn);
    assert(!D.ProducesTorque());

    D.Write(Cia402Cw::SwitchOnCmd);
    assert(D.State() == ECia402State::SwitchedOn);
    // POWERED AND NOT ENABLED — a real state this project could not express
    // before, and the one an operator sees when a drive is "on" but the ride
    // still will not move.
    assert(!D.ProducesTorque());

    D.Write(Cia402Cw::EnableOp);
    assert(D.State() == ECia402State::OperationEnabled);
    assert(D.ProducesTorque());
    std::printf("  four rungs to torque, and only the last one has any\n");
}

void TestTheHandshakeCannotBeSKIPPED()
{
    // A PLC cannot jump from Switch-on-disabled to Operation-enabled by writing
    // 0x000F once. It writes Shutdown, WAITS for the statusword, writes Switch
    // on, WAITS, then enables. Skipping a rung silently does nothing — which is
    // the classic mistake, and modelling it is the point.
    FCia402Drive D;
    D.Write(Cia402Cw::Shutdown);          // -> SwitchOnDisabled
    assert(D.State() == ECia402State::SwitchOnDisabled);

    for (int i = 0; i < 50; ++i) { D.Write(Cia402Cw::EnableOp); }
    // 0x000F has bit0 set, so from SwitchOnDisabled it never reaches
    // ReadyToSwitchOn — the drive sits there for ever and produces nothing.
    assert(!D.ProducesTorque());
    assert(D.State() != ECia402State::OperationEnabled);
    std::printf("  writing 0x000F at a disabled drive does nothing, for ever\n");
}

void TestQuickStopIsACTIVELOW()
{
    // The bit being CLEAR asserts it. So a zeroed controlword — a dead master, a
    // broken wire, a a cable pulled — commands a quick stop rather than nothing.
    // That is de-energise-to-trip expressed in a fieldbus word.
    FCia402Drive D = Enabled();
    assert(D.ProducesTorque());
    assert((D.Statusword() & Cia402Sw::QuickStop) != 0);   // 1 = NOT quick stopping

    D.Write(Cia402Cw::QuickStopCmd);
    assert(D.State() == ECia402State::QuickStopActive);
    assert(!D.ProducesTorque());
    // Bit 5 CLEARED is what says a quick stop is executing.
    assert((D.Statusword() & Cia402Sw::QuickStop) == 0);

    for (int i = 0; i < 100; ++i) { D.Write(Cia402Cw::QuickStopCmd, 0.01); }
    assert(D.State() == ECia402State::SwitchOnDisabled);
    std::printf("  quick stop is active low: bit CLEAR means it is executing\n");
}

void TestAZeroedControlwordCommandsAQuickStop()
{
    // The consequence worth asserting on its own, because it is the safety
    // property: losing the master does not leave a drive running.
    FCia402Drive D = Enabled();
    assert(D.ProducesTorque());
    D.Write(0x0000);
    assert(!D.ProducesTorque());
    std::printf("  a zeroed controlword stops the drive rather than holding it\n");
}

void TestAFaultResetNeedsARISINGEDGE()
{
    // Holding 0x0080 does nothing. An operator who wedges the reset button waits
    // for ever, which is a real complaint about real drives and is exactly the
    // mistake worth letting somebody make here rather than on a ride.
    FCia402Drive D = Enabled();
    D.SetFaultReaction(ECia402FaultReaction::Coast);

    // THE CASE THAT MATTERS: the reset is ALREADY HELD when the fault happens —
    // a wedged or taped button. The first draft of this test pressed it after
    // the fault, which is a perfectly good rising edge, and it cleared exactly as
    // it should have. Same anti-tie-down shape as the monitored E-stop reset.
    D.Write(Cia402Cw::EnableOp | Cia402Cw::FaultReset);   // held, on a healthy drive
    D.RaiseFault();
    D.Write(Cia402Cw::EnableOp | Cia402Cw::FaultReset, 0.1);
    assert(D.State() == ECia402State::Fault);
    assert(!D.ProducesTorque());
    assert((D.Statusword() & Cia402Sw::Fault) != 0);

    // HELD, not edged. A thousand scans of it change nothing.
    for (int i = 0; i < 1000; ++i)
    {
        D.Write(Cia402Cw::EnableOp | Cia402Cw::FaultReset, 0.01);
    }
    assert(D.State() == ECia402State::Fault);

    // Release, then press: that is an edge, and it clears.
    D.Write(Cia402Cw::EnableOp, 0.01);
    D.Write(Cia402Cw::EnableOp | Cia402Cw::FaultReset, 0.01);
    assert(D.State() == ECia402State::SwitchOnDisabled);
    std::printf("  a held fault reset does nothing; an edge clears it\n");
}

void TestAFaultIsNotINSTANT()
{
    // The drive runs its CONFIGURED reaction and only then settles in Fault. A
    // launch that faults mid-push is still doing something for a moment, and
    // which something is a property of the drive rather than an assumption.
    {
        FCia402Drive D = Enabled();
        D.SetFaultReaction(ECia402FaultReaction::Coast);
        D.RaiseFault();
        assert(D.State() == ECia402State::FaultReactionActive);
        D.Write(Cia402Cw::EnableOp, 0.0);
        assert(D.State() == ECia402State::Fault);   // coast settles at once
    }
    {
        FCia402Drive D = Enabled();
        D.ReactionSeconds = 0.5;
        D.SetFaultReaction(ECia402FaultReaction::SlowDownRamp);
        D.RaiseFault();
        D.Write(Cia402Cw::EnableOp, 0.2);
        assert(D.State() == ECia402State::FaultReactionActive);   // still ramping
        assert(!D.ProducesTorque());
        D.Write(Cia402Cw::EnableOp, 0.4);
        assert(D.State() == ECia402State::Fault);
    }
    std::printf("  a coast settles at once, a ramp takes its configured time\n");
}

void TestTheFirstFaultCauseIsKept()
{
    // Same latching rule as the E-stop and the drive fault: a second cause must
    // not overwrite the first, because what went first is the only thing worth
    // knowing.
    FCia402Drive D = Enabled();
    D.SetFaultReaction(ECia402FaultReaction::SlowDownRamp);
    D.RaiseFault();
    const ECia402State After = D.State();
    D.RaiseFault();
    assert(D.State() == After);
    std::printf("  a second fault does not restart the reaction\n");
}

void TestTheStatuswordIsDERIVED()
{
    // Assembled from the state rather than stored beside it, so the two cannot
    // disagree. A stored word is a second copy of the truth and this project has
    // spent a lot of effort removing those.
    FCia402Drive D = Enabled();
    const std::uint16_t W = D.Statusword();
    assert((W & Cia402Sw::ReadyToSwitchOn) != 0);
    assert((W & Cia402Sw::SwitchedOn) != 0);
    assert((W & Cia402Sw::OperationEnabled) != 0);
    assert((W & Cia402Sw::Fault) == 0);
    assert((W & Cia402Sw::VoltageEnabled) != 0);

    D.SetTargetReached(true);
    D.SetInternalLimit(true);
    assert((D.Statusword() & Cia402Sw::TargetReached) != 0);
    assert((D.Statusword() & Cia402Sw::InternalLimit) != 0);
    std::printf("  the statusword is derived from the state, never stored beside it\n");
}

void TestLosingVoltageDropsOutFromAnywhere()
{
    // The supply going away is not a request and has no preconditions.
    FCia402Drive D = Enabled();
    D.Write(Cia402Cw::DisableVoltage);
    assert(D.State() == ECia402State::SwitchOnDisabled);
    assert(!D.ProducesTorque());
    std::printf("  losing voltage drops out from anywhere\n");
}

} // namespace

int main()
{
    std::printf("CiA 402: the drive state machine, as specified\n\n");

    TestOnlyOneStateProducesTorque();
    TestTheHandshakeCannotBeSKIPPED();
    TestQuickStopIsACTIVELOW();
    TestAZeroedControlwordCommandsAQuickStop();
    TestAFaultResetNeedsARISINGEDGE();
    TestAFaultIsNotINSTANT();
    TestTheFirstFaultCauseIsKept();
    TestTheStatuswordIsDERIVED();
    TestLosingVoltageDropsOutFromAnywhere();

    std::printf("\ntest_cia402: all assertions passed.\n");
    return 0;
}
