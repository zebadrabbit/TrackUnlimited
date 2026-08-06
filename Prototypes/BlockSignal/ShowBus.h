// TrackUnlimited: the show layer's only connection to the ride.
// Plain C++17, no dependencies.
//
// ===================== SHOW IS READ-ONLY =====================
//
// This is the whole of Tier 3's interface, and it is deliberately a dead end:
// events come IN and fixture commands go OUT, and there is no method here by
// which a show effect can ask the ride for anything. That is not a policy this
// class enforces — it is a shape it has. There is nowhere to put the request.
//
// DMX512 agrees at the wire level: a controller broadcasts 512 channel values
// per universe and there is no return path. Read the ride, write the fixtures.
//
//   sensors, blocks, drives ──► FShowBus ──► DMX / pyro / fog / cameras
//                                   ▲
//                     hard permissive (hazardous outputs only)
//
// ===================== EVERY EXAMPLE IS THE SAME SUBSCRIPTION =====================
//
//   a train passes a sensor          → fire the pyro and the fog
//   a train passes a sensor          → fire the on-ride camera
//   a station reaches Loading        → platform lighting, audio
//   a block goes occupied            → a scenic effect
//
// One mechanism, different fixtures on the end. That the input needed no new
// plumbing — it is the state-transition stream that already exists — is the
// strongest evidence the tier boundary was drawn in the right place.
//
// ===================== WHY DETERMINISM DOES NOT MATTER HERE =====================
//
// Nothing downstream of this reaches the ride, so a show effect that hangs,
// crashes or fires differently every run cannot move a train. That is
// structural rather than policy, and it is what makes "the ride runs identically
// with the show layer absent" provable rather than asserted.

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

// What kind of thing changed. Deliberately the same set the transition log
// already emits — this is a subscription over an existing stream, not a new
// source of truth beside it.
enum class ERideEventKind
{
    BlockState,     // channel = block index
    SensorTrip,     // channel = sensor index
    StationPhase,   // channel = platform index
    DriveState,     // channel = drive index
    TrainMotion,    // channel = train index
};

struct FRideEvent
{
    ERideEventKind Kind = ERideEventKind::BlockState;
    int Channel = 0;
    int From = 0;
    int To = 0;
    std::uint64_t Scan = 0;
};

// One fixture's trigger: fire when this channel of this kind reaches this state.
//
// HAZARDOUS IS A PROPERTY OF THE FIXTURE, not of the trigger. Pyro, flame, CO2
// and moving scenery can injure; light, audio, haze and a camera cannot. The
// distinction decides whether a permissive is required, and it belongs to the
// thing on the end of the wire.
struct FShowTrigger
{
    int Fixture = 0;
    ERideEventKind Kind = ERideEventKind::SensorTrip;
    int Channel = 0;
    int WhenTo = 1;
    bool bHazardous = false;
};

// What fired, this scan. Returned rather than dispatched through a callback so
// a caller can assert on it — and so nothing in this file ever calls out into
// code it does not control, which is how a show script gets to fail freely.
struct FShowFiring
{
    int Fixture = 0;
    std::uint64_t Scan = 0;
    bool bInhibited = false;   // matched, and the permissive was open
};

class FShowBus
{
public:
    void AddTrigger(const FShowTrigger& T) { Triggers.push_back(T); }
    std::size_t NumTriggers() const { return Triggers.size(); }

    // THE HARD PERMISSIVE, and it gates only hazardous fixtures.
    //
    // Note what this is NOT: a request-and-grant handshake. A real show
    // controller does not ask for pyro and wait to be told yes — it sends the
    // cue into a firing circuit that has arming, continuity and a key in series,
    // and it fires into one that may simply be open and DOES NOT KNOW. So a
    // fixture whose permissive is closed still MATCHES and still fires; it is
    // the circuit that goes nowhere. That is modelled as `bInhibited` rather
    // than as a refusal, because the show layer genuinely cannot tell.
    void SetHazardPermissive(bool bOpen) { bHazardOk = bOpen; }
    bool IsHazardPermissiveOpen() const { return bHazardOk; }

    // One scan's worth of events in, one scan's worth of firings out.
    //
    // Events arrive in scan order and are evaluated in the order they arrive,
    // because scan order IS control-system behaviour and a bus that sorted or
    // batched them would be inventing an ordering the ride never had.
    std::vector<FShowFiring> Deliver(const std::vector<FRideEvent>& Events)
    {
        std::vector<FShowFiring> Fired;
        for (const FRideEvent& E : Events)
        {
            for (const FShowTrigger& T : Triggers)
            {
                if (T.Kind != E.Kind || T.Channel != E.Channel || T.WhenTo != E.To)
                {
                    continue;
                }
                FShowFiring F;
                F.Fixture = T.Fixture;
                F.Scan = E.Scan;
                F.bInhibited = T.bHazardous && !bHazardOk;
                Fired.push_back(F);
            }
        }
        return Fired;
    }

private:
    std::vector<FShowTrigger> Triggers;
    // Hazardous fixtures are inhibited until something explicitly opens the
    // permissive. A show layer that came up able to fire pyro would have the
    // default backwards.
    bool bHazardOk = false;
};

// ponytail: no cue lists, no timelines, no offsets, no language. Those are how
// you AUTHOR show logic and they are answerable once something consumes this.
// What is here is the boundary — the one thing Tier 3 needs before any of that
// question arises, and the one thing that is painful to retrofit.
//
// Also not here: a camera trigger type. A camera trip IS a sensor, a third kind
// beside the block boundaries and the stop marks, so it arrives as SensorTrip
// like everything else. Its shutter LEAD is surveyed into where the trip is
// placed rather than computed from speed — the same idiom as the stop mark
// consuming train length at survey rather than in the dispatcher.
