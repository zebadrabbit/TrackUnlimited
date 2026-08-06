// Asserts for ShowBus.h — the show layer's only connection to the ride.
//
//   clang++ -std=c++17 -Wall -Wextra -o test_showbus test_showbus.cpp && ./test_showbus

#include "ShowBus.h"

#include <cassert>
#include <cstdio>

namespace
{

FRideEvent Ev(ERideEventKind K, int Ch, int To, std::uint64_t Scan = 1)
{
    FRideEvent E; E.Kind = K; E.Channel = Ch; E.To = To; E.Scan = Scan; return E;
}
FShowTrigger Trig(int Fixture, ERideEventKind K, int Ch, int To, bool bHaz = false)
{
    FShowTrigger T;
    T.Fixture = Fixture; T.Kind = K; T.Channel = Ch; T.WhenTo = To; T.bHazardous = bHaz;
    return T;
}

void TestATrainPassesASensorAndTheFogFires()
{
    // The developer's own example, and the whole shape of Tier 3.
    FShowBus Bus;
    Bus.AddTrigger(Trig(10, ERideEventKind::SensorTrip, 3, 1));   // fog on sensor 3

    const auto Fired = Bus.Deliver({Ev(ERideEventKind::SensorTrip, 3, 1)});
    assert(Fired.size() == 1);
    assert(Fired[0].Fixture == 10);
    assert(!Fired[0].bInhibited);
    std::printf("  a train passes a sensor and the fog fires\n");
}

void TestEveryExampleIsTheSameSubscription()
{
    // Pyro, camera, platform lighting and a scenic effect are one mechanism with
    // different fixtures on the end. That the input needed no new plumbing is
    // the strongest evidence the tier boundary sits in the right place.
    FShowBus Bus;
    Bus.SetHazardPermissive(true);
    Bus.AddTrigger(Trig(1, ERideEventKind::SensorTrip, 3, 1, /*hazardous*/ true));  // pyro
    Bus.AddTrigger(Trig(2, ERideEventKind::SensorTrip, 3, 1));                      // camera
    Bus.AddTrigger(Trig(3, ERideEventKind::StationPhase, 0, 3));                    // audio
    Bus.AddTrigger(Trig(4, ERideEventKind::BlockState, 5, 1));                      // scenic

    auto Fired = Bus.Deliver({Ev(ERideEventKind::SensorTrip, 3, 1)});
    assert(Fired.size() == 2);              // pyro and camera share the trip

    Fired = Bus.Deliver({Ev(ERideEventKind::StationPhase, 0, 3),
                         Ev(ERideEventKind::BlockState, 5, 1)});
    assert(Fired.size() == 2);
    std::printf("  pyro, camera, audio and scenery are one subscription\n");
}

void TestAHazardousFixtureIsINHIBITEDNotRefused()
{
    // NOT A REQUEST-AND-GRANT HANDSHAKE. A real show controller does not ask for
    // pyro and wait to be told yes — it sends the cue into a firing circuit with
    // arming, continuity and a key in series, and it fires into one that may
    // simply be open AND DOES NOT KNOW.
    //
    // So a fixture whose permissive is closed still MATCHES and still fires; it
    // is the circuit that goes nowhere. Modelled as inhibited rather than as a
    // refusal, because the show layer genuinely cannot tell the difference.
    FShowBus Bus;
    Bus.AddTrigger(Trig(1, ERideEventKind::SensorTrip, 0, 1, /*hazardous*/ true));

    assert(!Bus.IsHazardPermissiveOpen());   // closed until something opens it
    auto Fired = Bus.Deliver({Ev(ERideEventKind::SensorTrip, 0, 1)});
    assert(Fired.size() == 1);               // it FIRED
    assert(Fired[0].bInhibited);             // into an open circuit

    Bus.SetHazardPermissive(true);
    Fired = Bus.Deliver({Ev(ERideEventKind::SensorTrip, 0, 1)});
    assert(!Fired[0].bInhibited);
    std::printf("  a hazardous fixture fires into a circuit that may be open\n");
}

void TestTheHazardPermissiveDefaultsCLOSED()
{
    // A show layer that came up able to fire pyro would have the default
    // backwards. Asserted on its own because it is the safety property.
    FShowBus Bus;
    Bus.AddTrigger(Trig(1, ERideEventKind::BlockState, 0, 1, true));
    const auto Fired = Bus.Deliver({Ev(ERideEventKind::BlockState, 0, 1)});
    assert(Fired[0].bInhibited);
    std::printf("  the hazard permissive defaults CLOSED\n");
}

void TestANonHazardousFixtureIsNeverGated()
{
    // A camera cannot injure anybody, so it has no permissive at all — the
    // purest Tier 3 case. Gating it would be inventing a safety relationship
    // that does not exist, and this project does not draw lamps for controls it
    // has not modelled.
    FShowBus Bus;
    Bus.SetHazardPermissive(false);
    Bus.AddTrigger(Trig(7, ERideEventKind::SensorTrip, 2, 1, /*hazardous*/ false));
    const auto Fired = Bus.Deliver({Ev(ERideEventKind::SensorTrip, 2, 1)});
    assert(Fired.size() == 1);
    assert(!Fired[0].bInhibited);
    std::printf("  a camera is never gated — it cannot injure anybody\n");
}

void TestNothingFiresOnTheWrongChannelOrState()
{
    FShowBus Bus;
    Bus.AddTrigger(Trig(1, ERideEventKind::SensorTrip, 3, 1));
    assert(Bus.Deliver({Ev(ERideEventKind::SensorTrip, 4, 1)}).empty());   // wrong channel
    assert(Bus.Deliver({Ev(ERideEventKind::SensorTrip, 3, 0)}).empty());   // wrong state
    assert(Bus.Deliver({Ev(ERideEventKind::BlockState, 3, 1)}).empty());   // wrong kind
    std::printf("  wrong channel, wrong state or wrong kind fires nothing\n");
}

void TestEventsAreEvaluatedInSCANORDER()
{
    // Scan order IS control-system behaviour. A bus that sorted or batched
    // events would be inventing an ordering the ride never had, and a recorded
    // show would then replay differently from the ride that produced it.
    FShowBus Bus;
    Bus.AddTrigger(Trig(1, ERideEventKind::SensorTrip, 0, 1));
    Bus.AddTrigger(Trig(2, ERideEventKind::SensorTrip, 1, 1));

    const auto Fired = Bus.Deliver({Ev(ERideEventKind::SensorTrip, 1, 1, 5),
                                    Ev(ERideEventKind::SensorTrip, 0, 1, 5)});
    assert(Fired.size() == 2);
    assert(Fired[0].Fixture == 2);   // the order they arrived, not sorted
    assert(Fired[1].Fixture == 1);
    std::printf("  events fire in scan order, never sorted or batched\n");
}

void TestTheBusHasNoWayToAffectTheRide()
{
    // THE PROPERTY THAT MAKES CONSTRAINT 7 STRUCTURAL RATHER THAN POLICED.
    //
    // This is not asserted by running something — it is asserted by the shape of
    // the class, and the test exists to make that shape deliberate rather than
    // accidental. Deliver() takes events and returns firings. There is no
    // reference to a ride, no callback out into caller code, and nothing a
    // trigger can name that is not a fixture id.
    //
    // A show script cannot dispatch a train because there is nowhere to put the
    // request, not because something checks whether it may.
    FShowBus Bus;
    Bus.AddTrigger(Trig(1, ERideEventKind::BlockState, 0, 1, true));
    const auto Fired = Bus.Deliver({Ev(ERideEventKind::BlockState, 0, 1)});

    // The only things that leave this class: which fixture, when, and whether
    // the circuit was live. Nothing addressable on the ride.
    assert(Fired[0].Fixture == 1);
    assert(Fired[0].Scan == 1);
    assert(Fired[0].bInhibited);
    std::printf("  the bus has no way to affect the ride — constraint 7 by shape\n");
}

} // namespace

int main()
{
    std::printf("ShowBus: the show layer's only connection to the ride\n\n");

    TestATrainPassesASensorAndTheFogFires();
    TestEveryExampleIsTheSameSubscription();
    TestAHazardousFixtureIsINHIBITEDNotRefused();
    TestTheHazardPermissiveDefaultsCLOSED();
    TestANonHazardousFixtureIsNeverGated();
    TestNothingFiresOnTheWrongChannelOrState();
    TestEventsAreEvaluatedInSCANORDER();
    TestTheBusHasNoWayToAffectTheRide();

    std::printf("\ntest_showbus: all assertions passed.\n");
    return 0;
}
