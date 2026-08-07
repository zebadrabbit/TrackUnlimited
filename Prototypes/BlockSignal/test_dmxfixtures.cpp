// Asserts for DmxFixtures.h — the effect device layer.
//
//   clang++ -std=c++17 -Wall -Wextra -O2 -o test_dmxfixtures test_dmxfixtures.cpp && ./test_dmxfixtures
//
// FShowBus decides what fires. This is what it fires into, and every assertion
// here is about the DEVICE refusing rather than the cue being wrong.

#include "DmxFixtures.h"
#include "ShowBus.h"

#include <cassert>
#include <cstdio>

namespace
{

const double Dt = 1.0 / 240.0;

void TestADDRESSESAreOneBasedAndTheStartCodeIsNotAChannel()
{
    // The classic DMX mistake. Slot 0 is the START CODE — 0x00 for dimmer data —
    // and it is not a channel, while every fixture's DIP switches and menu count
    // from 1. Modelled rather than skipped, because the resulting off-by-one moves
    // an entire rig by one address and looks like a wiring fault.
    FDmxUniverse U;
    assert(U.StartCode() == 0x00);

    U.Set(1, 255);
    assert(U.Get(1) == 255);
    assert(U.Get(2) == 0);

    U.Set(512, 128);
    assert(U.Get(512) == 128);
    U.Set(513, 200);                    // past the end
    assert(U.Get(513) == 0);
    U.Set(0, 200);                      // the start code is not addressable
    assert(U.Get(0) == 0);

    // A fixture patched at 100 owns slots 100..103 for four channels. Doubly
    // 1-based — within the universe and within the fixture — which is exactly
    // what real patching is.
    const FDmxFixture F{"wash", 100, 4, EDmxDataLoss::Shutdown};
    assert(F.SlotOf(1) == 100);
    assert(F.SlotOf(4) == 103);
    assert(F.LastSlot() == 103);
    std::printf("  addresses are 1-based and slot 0 is the start code, not a channel\n");
}

void TestTheREFRESHCeilingIsReal()
{
    // DMX HAS NO FADES. A packet is a set of absolute values; the console computes
    // every intermediate one and streams them at frame rate. So a cue's resolution
    // is a FRAME, and the frame rate has a hard ceiling.
    //
    // A full 513-slot packet is about 22.7 ms — roughly 44 packets a second — and
    // shorter packets go proportionally faster.
    const double Full = FDmxUniverse::PacketSeconds();
    assert(Full > 0.020 && Full < 0.025);
    assert(FDmxUniverse::MaxRefreshHz() > 40.0);
    assert(FDmxUniverse::MaxRefreshHz() < 48.0);

    // A small rig sends fewer slots and refreshes far faster, which is a real and
    // useful thing to know when a strobe cue looks chunky.
    assert(FDmxUniverse::MaxRefreshHz(24) > 150.0);

    std::printf("  a full universe is %.1f ms (%.0f Hz); 24 slots is %.0f Hz\n",
                Full * 1000.0, FDmxUniverse::MaxRefreshHz(),
                FDmxUniverse::MaxRefreshHz(24));
}

void TestOverlappingFixturesAreREFUSED()
{
    // The most common patching error there is, and its symptom is a light that
    // flickers when a completely different one is cued — which nobody traces back
    // to addressing. Report, never repair: the fix is a different address, and a
    // patch that shuffled fixtures to make room would hide a rig nobody can
    // maintain.
    FDmxPatch P;
    assert(P.Add({"hazer", 1, 3, EDmxDataLoss::Shutdown}));
    assert(!P.Add({"flame", 3, 2, EDmxDataLoss::Shutdown}));    // 3 is already the hazer's
    assert(P.Add({"flame", 4, 2, EDmxDataLoss::Shutdown}));
    assert(P.Num() == 2);

    // And out of the universe entirely.
    assert(!P.Add({"strip", 511, 8, EDmxDataLoss::Shutdown}));
    assert(!P.Add({"bad", 0, 1, EDmxDataLoss::Shutdown}));

    assert(P.HighestSlotUsed() == 5);
    std::printf("  overlapping addresses are refused, not shuffled out of the way\n");
}

void TestDATALOSSIsSelectableAndBothChoicesAreDefensible()
{
    // What a fixture does when the data stops — cable pulled, console crashed,
    // switch rebooted. Real units make it selectable, and the MDG ATMe hazer is
    // one that does.
    //
    // Shutdown is right for haze and light: the effect stops. HoldLast is right
    // for architectural fixtures nobody wants going dark, and WRONG for anything
    // hazardous — a flame fixture holding its last value is a flame nobody is
    // commanding.
    FDmxPatch P;
    assert(P.Add({"hazer", 1, 3, EDmxDataLoss::Shutdown}));
    assert(P.Add({"arch", 10, 3, EDmxDataLoss::HoldLast}));

    FDmxUniverse U;
    U.Set(1, 200); U.Set(2, 180); U.Set(3, 255);
    U.Set(10, 90); U.Set(11, 90); U.Set(12, 90);

    P.OnDataLoss(U);
    assert(U.Get(1) == 0 && U.Get(2) == 0 && U.Get(3) == 0);
    assert(U.Get(10) == 90 && U.Get(11) == 90 && U.Get(12) == 90);
    std::printf("  data loss: the hazer shuts down and the architectural fixture holds\n");
}

void TestARMPLUSLEVELNeedsBothChannels()
{
    // THE CHARACTERISTIC SHAPE OF EFFECT GEAR, and it exists precisely so that one
    // stray channel cannot start an effect.
    //
    // The MDG ATMe hazer is three channels: unit power, haze output 0-255, and a
    // separate haze ENABLE that must be held above half. Two of the three have to
    // be right before anything comes out — so a single bad slot, a patch collision
    // or a console glitch produces nothing rather than a room full of haze.
    FEffectDevice::FSpec Spec;
    Spec.bHazardous = true;
    FEffectDevice Hazer(Spec);

    FEffectRequest R;
    R.bShowRequest = true;
    R.Level = 1.0;
    R.bSafetyPermissive = true;
    R.bArmed = false;                    // level but no arm
    assert(!Hazer.Scan(Dt, R));

    R.bArmed = true;
    R.Level = 0.0;                       // arm but no level
    assert(!Hazer.Scan(Dt, R));

    R.Level = 1.0;
    assert(Hazer.Scan(Dt, R));           // both, and only then
    assert(Hazer.TimesInhibited() > 0);

    // The DMX thresholds themselves, as real gear specifies them. Above half is
    // "on" for an enable; the Salamander's igniter wants above 99%, deliberately
    // near the top of the range so a stray value cannot reach it.
    assert(!DmxEnabled(127));
    assert(DmxEnabled(128));
    assert(!DmxAbove(252, 0.99));
    assert(DmxAbove(255, 0.99));
    std::printf("  arm plus level: neither channel alone produces anything\n");
}

void TestFLAMEIsNOTFiredByTheShowController()
{
    // NFPA 160 puts a safety-rated flame effect controller and a fail-safe
    // POSITIVE MANUAL ENABLE — a human-held enable, actively asserted — between
    // the request and the gas.
    //
    // So a show controller does not request flame and wait to be told yes. It
    // sends its cue into a circuit that may simply be open, AND DOES NOT KNOW.
    // That is the same shape as FShowBus's hazard permissive, one layer down, and
    // asserting both together is the point: the cue fires, the fixture does not.
    FEffectDevice::FSpec Spec;
    Spec.bHazardous = true;
    Spec.WarmupSeconds = 0.0;
    FEffectDevice Flame(Spec);

    FShowBus Bus;
    Bus.AddTrigger({7, ERideEventKind::SensorTrip, 3, 1, true});
    // The permissive defaults CLOSED, which is a show layer that comes up unable
    // to fire pyro — the right way round.
    assert(!Bus.IsHazardPermissiveOpen());

    std::vector<FRideEvent> Events;
    FRideEvent E;
    E.Kind = ERideEventKind::SensorTrip;
    E.Channel = 3;
    E.To = 1;
    E.Scan = 100;
    Events.push_back(E);

    const std::vector<FShowFiring> Fired = Bus.Deliver(Events);
    assert(Fired.size() == 1);
    assert(Fired[0].bInhibited);         // it fired, into a circuit going nowhere

    // And the fixture agrees, from its own side and for its own reason: nobody is
    // holding the manual enable.
    FEffectRequest R;
    R.bShowRequest = true;
    R.Level = 1.0;
    R.bArmed = true;
    R.bSafetyPermissive = false;         // no human holding the enable
    assert(!Flame.Scan(Dt, R));

    // A person holds it. Only now is there gas.
    R.bSafetyPermissive = true;
    assert(Flame.Scan(Dt, R));
    std::printf("  the cue fires into an open circuit and does not know; the fixture does not\n");
}

void TestTheIGNITERWarmupCannotBeHurried()
{
    // A Le Maitre Salamander needs roughly 10 s of igniter heat-up before it will
    // fire at all. A cue during that window is DROPPED rather than queued: firing
    // a flame effect ten seconds after the music wanted it is worse than not
    // firing it.
    FEffectDevice::FSpec Spec;
    Spec.bHazardous = true;
    Spec.WarmupSeconds = 10.0;
    FEffectDevice Flame(Spec);

    FEffectRequest R;
    R.bShowRequest = true;
    R.Level = 1.0;
    R.bArmed = true;
    R.bSafetyPermissive = true;

    assert(Flame.IsWarming());
    for (int i = 0; i < 240 * 9; ++i) { assert(!Flame.Scan(Dt, R)); }
    assert(Flame.IsWarming());
    assert(Flame.TimesDropped() > 2000);

    for (int i = 0; i < 240 * 2; ++i) { Flame.Scan(Dt, R); }
    assert(Flame.IsReady() || Flame.IsRecovering());
    std::printf("  a 10 s igniter warm-up drops %zu cues rather than queueing them\n",
                Flame.TimesDropped());
}

void TestTheDUTYCYCLEIsEnforcedByTheDEVICE()
{
    // THE THING NOTHING HERE COULD PREVIOUSLY EXPRESS. A cheap fogger stops
    // accepting fire commands while its heater recovers; a Salamander's solenoid
    // is rated for 30 s continuous with 0.5-1 s bursts recommended.
    //
    // Enforced by the FIXTURE, not by the cue layer. An operator holding a fader
    // down does not get 30 seconds of flame because they asked for it — and
    // putting this rule in the cue layer would let a bad cue destroy hardware,
    // which is the wrong place for it.
    FEffectDevice::FSpec Spec;
    Spec.bHazardous = true;
    Spec.MaxBurstSeconds = 1.0;
    Spec.RecoverySeconds = 5.0;
    FEffectDevice Fogger(Spec);

    FEffectRequest R;
    R.bShowRequest = true;
    R.Level = 1.0;
    R.bArmed = true;
    R.bSafetyPermissive = true;

    int FiredScans = 0;
    for (int i = 0; i < 240 * 3; ++i) { if (Fogger.Scan(Dt, R)) { ++FiredScans; } }

    // One second of fire out of three seconds of asking, then it cut itself off.
    assert(FiredScans >= 236 && FiredScans <= 244);
    assert(Fogger.TimesDutyCut() == 1);
    assert(Fogger.IsRecovering());

    // And it stays refused through the recovery, however hard it is asked. Two of
    // the five seconds already went by inside the loop above, so two more are
    // still comfortably inside it.
    for (int i = 0; i < 240 * 2; ++i) { assert(!Fogger.Scan(Dt, R)); }
    assert(Fogger.IsRecovering());

    // Then it works again, and cuts itself off again — because the operator never
    // let go, which is exactly the case the duty limit is there for.
    for (int i = 0; i < 240 * 3; ++i) { Fogger.Scan(Dt, R); }
    assert(Fogger.TimesDutyCut() == 2);
    std::printf("  a 1 s duty limit fires for %d scans then cuts itself off for 5 s\n",
                FiredScans);
}

void TestANonHazardousFixtureNeedsNoneOfThat()
{
    // A camera and a wash light cannot injure anybody, so gating them would invent
    // a safety relationship that does not exist — the same rule FShowBus already
    // applies to the on-ride camera.
    FEffectDevice::FSpec Spec;
    Spec.bHazardous = false;
    FEffectDevice Wash(Spec);

    FEffectRequest R;
    R.bShowRequest = true;
    R.Level = 1.0;
    R.bArmed = false;                    // no arm channel at all
    R.bSafetyPermissive = false;         // and no permissive
    assert(Wash.Scan(Dt, R));
    assert(Wash.TimesInhibited() == 0);
    std::printf("  a fixture that cannot injure is not gated, and that is deliberate\n");
}

} // namespace

int main()
{
    std::printf("The effect device layer: DMX, modelled literally and internally\n\n");

    TestADDRESSESAreOneBasedAndTheStartCodeIsNotAChannel();
    TestTheREFRESHCeilingIsReal();
    TestOverlappingFixturesAreREFUSED();
    TestDATALOSSIsSelectableAndBothChoicesAreDefensible();
    TestARMPLUSLEVELNeedsBothChannels();
    TestFLAMEIsNOTFiredByTheShowController();
    TestTheIGNITERWarmupCannotBeHurried();
    TestTheDUTYCYCLEIsEnforcedByTheDEVICE();
    TestANonHazardousFixtureNeedsNoneOfThat();

    std::printf("\ntest_dmxfixtures: all assertions passed.\n");
    return 0;
}
