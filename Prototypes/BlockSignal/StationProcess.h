// TrackUnlimited: what has to happen at a platform before a train may leave it.
//
// A STATION IS A PROCESS, NOT A PLACE. A train arrives, riders get off, riders
// get on, restraints are closed and checked, the platform is confirmed clear, and
// only THEN may a dispatch happen. The block signalling is the last link in that
// chain and not the whole of it: a real dispatch permissive is an AND of the
// interlocking and every one of these, and the interlocking is usually the term
// that is already satisfied while an operator is still walking the train.
//
// Until this existed the model dispatched a train the instant the block ahead
// went clear, which is a ride with no riders in it.
//
// ===================== EVERY GATE IS AN INPUT =====================
//
// Not a timer. On a real ride each of these is a physical contact:
//
//   in position        the stop mark - the switch this project already has
//   restraints locked  a sensor per car, ANDed; the train reports one signal
//   platform clear     airgate switches and the operators' all-clear
//   unload / load done what the platform staff confirm
//
// Minimum dwell exists on real rides, but it is a throughput target rather than
// what gates the dispatch. Model the gates and the dwell falls out; model the
// dwell and the gates never get built. FAutoStationCrew at the bottom of this
// file is the stand-in that flips these inputs on timers, and it is clearly
// separate BECAUSE it is the part that goes away when there are riders.
//
// ===================== READY IS NOT LATCHED =====================
//
// The phase walks forward as steps complete, but IsReadyToDispatch is evaluated
// from the LIVE inputs every scan, and a restraint popping open in Ready drops it
// again the same frame. That is what a permissive is on a real PLC - a
// continuously evaluated condition, not a state you arrive at and keep. A station
// that latched Ready would report "ready" on the panel while the machine refused
// to dispatch, which is exactly the disagreement a panel exists to prevent.
//
// ================= THIS IS ONE POSITION, NOT ONE PLATFORM =================
//
// Deliberate, and it is what makes the big operations expressible later. A high
// throughput ride holds SEVERAL trains on one load platform - Space Mountain
// holds three - and dispatches them individually, because a rider who needs
// longer to board must not hold up the two trains in front. So each position runs
// its OWN sequence: its own restraint check, its own all-clear, its own dwell.
// Three positions is three of these, in series, front one leaving first.
//
// One process per platform could not say that, and bolting positions on later
// would be a rewrite. See SIGNALLING.md for the rest of that shape - the storage
// buffer, the turnout and the maintenance spur - none of which is built.
//
// ===================== LOAD AND UNLOAD MAY BE SEPARATE =====================
//
// High-throughput rides split the platform in two: riders get OFF at one and ON
// at another, with track between them, and the train drives from one to the
// other. They are different rooms, often deliberately themed as different scenes.
// So they are separate zones and separate BLOCKS - the entire point is emptying
// one train while another is still being filled - and the roles differ in what
// they require:
//
//   Unload    riders out and the platform clear. NOT restraints locked: the
//             train moves to the load platform with them open, ready to board.
//   Load      riders in, restraints locked, platform clear.
//   Combined  all of it, in one place, which is what a small ride has.
//
// Reference: station operation, restraint checks and dispatch procedure as
// described in Weisenberger, "Coasters 101" (3rd ed.), chapters 6 and 9.

#pragma once

#include <cstddef>

enum class EStationRole
{
    Combined,   // riders off and on in one place
    Unload,     // riders off only; the train leaves empty, restraints open
    Load,       // riders on only
};

enum class EStationPhase
{
    Empty,        // nothing here
    Arriving,     // a train is in the zone but not yet at its mark
    Unloading,    // stopped, restraints released, riders leaving
    Loading,      // riders boarding
    Securing,     // restraints closing, operators checking, gates shutting
    Ready,        // every station condition met; waiting on the interlocking only
    Departing,    // dispatched, still clearing the platform
};

// The contacts, as a PLC would read them on a scan. Each is something a switch or
// a person actually asserts; none of them is a peek at the simulation.
struct FStationInputs
{
    // A train is somewhere in this zone. Distinguishes an empty platform from one
    // with a train still rolling in, which are different lamps and different
    // phases even though neither is ready for anything.
    bool bTrainPresent = false;

    // At its mark and stopped. THE STOP MARK SENSOR, which is already built and
    // is the reason nothing here needs a position.
    bool bTrainInPosition = false;

    bool bUnloadComplete = false;    // platform staff: the train is empty
    bool bLoadComplete = false;      // everyone seated, airgates shut
    bool bRestraintsLocked = false;  // every car reporting locked, ANDed
    bool bPlatformClear = false;     // operators' all-clear
};

class FStationProcess
{
public:
    explicit FStationProcess(EStationRole InRole = EStationRole::Combined)
        : Role(InRole)
    {
    }

    EStationRole GetRole() const { return Role; }
    EStationPhase GetPhase() const { return Phase; }

    // One scan. Walks the phase forward as steps complete and back when a
    // condition is lost, both from the live inputs.
    void Update(const FStationInputs& In)
    {
        Last = In;

        if (!In.bTrainPresent)
        {
            // Gone. Everything resets, because the next train has to do all of it
            // again — a station that remembered the last train's checks would
            // dispatch the next one on them.
            Phase = EStationPhase::Empty;
            bDispatchedThisTrain = false;
            return;
        }

        if (!In.bTrainInPosition)
        {
            // Present but not at the mark: either still arriving, or leaving after
            // a dispatch. Same inputs, opposite meanings, and the latch is the only
            // thing that can tell them apart.
            Phase = bDispatchedThisTrain ? EStationPhase::Departing : EStationPhase::Arriving;
            return;
        }

        // In position. Walk the sequence.
        switch (Phase)
        {
        case EStationPhase::Empty:
        case EStationPhase::Arriving:
            Phase = Role == EStationRole::Load ? EStationPhase::Loading
                                               : EStationPhase::Unloading;
            break;
        case EStationPhase::Unloading:
            if (In.bUnloadComplete)
            {
                // An UNLOAD-ONLY platform is finished the moment the train is
                // empty. It does not close restraints — the train runs to the load
                // platform with them open, which is the whole point of splitting
                // the two.
                Phase = Role == EStationRole::Unload ? EStationPhase::Securing
                                                     : EStationPhase::Loading;
            }
            break;
        case EStationPhase::Loading:
            if (In.bLoadComplete)
            {
                Phase = EStationPhase::Securing;
            }
            break;
        case EStationPhase::Securing:
        case EStationPhase::Ready:
            // BOTH directions, in one test, deliberately. Ready is re-derived every
            // scan, so a restraint opening or an operator withdrawing the all-clear
            // takes it straight back to Securing.
            Phase = ConditionsMet(In) ? EStationPhase::Ready : EStationPhase::Securing;
            break;
        case EStationPhase::Departing:
            // Back on the mark after a dispatch. A train that rolled back onto its
            // own stop sensor has not un-dispatched itself; it stays departing
            // until it is genuinely gone.
            break;
        }

        if (Phase == EStationPhase::Ready)
        {
            bDispatchedThisTrain = true;
        }
    }

    // THE STATION'S HALF OF THE DISPATCH PERMISSIVE, and it ANDs with the block
    // interlocking rather than replacing it. Either one saying no is a no.
    //
    // Note what it is NOT: the operator's dispatch button. That is a request, and
    // this is permission. A caller running manual dispatch checks both; a caller
    // running automatic checks only this.
    //
    // DEPARTING COUNTS, and leaving it out DEADLOCKED THE CIRCUIT. The conditions
    // are re-derived every scan, so the instant a released train rolls off its stop
    // mark it is no longer "in position" and the sequence is no longer complete —
    // the permissive drops, the dispatcher re-commands the brake, the train stops
    // with its tail still over its own mark, and it sits there for the rest of the
    // session with nothing reporting anything wrong. Measured: the launch drive
    // never saw a single train in seven minutes.
    //
    // So the READINESS is continuous and the RELEASE is latched, which sounds like
    // a contradiction and is not. Everything up to the moment of release is
    // re-checked every scan and any of it can take the permission away. After it,
    // the train is moving, and there is no such thing as un-dispatching a moving
    // train — on real hardware or here.
    bool IsReadyToDispatch() const
    {
        return Phase == EStationPhase::Ready || Phase == EStationPhase::Departing;
    }

    // WHAT THE PANEL SHOWS, and the reason the inputs are modelled one at a time
    // instead of as a single "platform ready" bool. "The station is not ready" is
    // useless to an operator standing on it; "restraints not locked" is where to
    // go and look. First unmet condition, in the order the sequence needs them.
    const char* WhatIsHolding() const
    {
        // Nothing holds a train that has permission, which includes one already on
        // its way out. Asked first so a departing train reads as going rather than
        // as blocked by whatever step it is technically no longer doing.
        if (IsReadyToDispatch())     { return ""; }
        if (!Last.bTrainPresent)     { return "no train"; }
        if (!Last.bTrainInPosition)  { return "train not in position"; }
        if (NeedsUnload() && !Last.bUnloadComplete)   { return "unloading"; }
        if (NeedsLoad() && !Last.bLoadComplete)       { return "loading"; }
        if (NeedsRestraints() && !Last.bRestraintsLocked) { return "restraints not locked"; }
        if (!Last.bPlatformClear)    { return "platform not clear"; }
        return "";   // ready; anything still holding the train is the interlocking
    }

    // Which contacts this role actually cares about. Exposed because a panel that
    // shows a greyed-out "restraints" lamp on an unload platform is telling the
    // truth, and one that shows it red is not.
    bool NeedsUnload() const { return Role != EStationRole::Load; }
    bool NeedsLoad() const { return Role != EStationRole::Unload; }
    bool NeedsRestraints() const { return Role != EStationRole::Unload; }

private:
    bool ConditionsMet(const FStationInputs& In) const
    {
        if (NeedsUnload() && !In.bUnloadComplete)         { return false; }
        if (NeedsLoad() && !In.bLoadComplete)             { return false; }
        if (NeedsRestraints() && !In.bRestraintsLocked)   { return false; }
        return In.bPlatformClear;
    }

    EStationRole Role = EStationRole::Combined;
    EStationPhase Phase = EStationPhase::Empty;
    FStationInputs Last;

    // Set once this train has been given permission, so a train off its mark can
    // be told apart from one that has not reached it. Cleared when the train goes.
    bool bDispatchedThisTrain = false;
};

// THE PART THAT GOES AWAY WHEN THERE ARE RIDERS.
//
// Nothing in this project simulates a person, so something has to assert the
// contacts a person would. This runs them on dwell timers, which is precisely the
// modelling shortcut the class above exists to avoid — so it is a separate object
// that WRITES the inputs, rather than timers built into the process where they
// would be impossible to remove later.
//
// The numbers are throughput targets rather than physics. A real operation
// measures them and the whole business of running a coaster is making them
// smaller; treat them as the tuning knob they are.
struct FAutoStationCrew
{
    double UnloadSeconds = 6.0;
    double LoadSeconds = 12.0;
    double SecureSeconds = 4.0;

    // Advance the crew and write what they have finished into In. Call once per
    // scan, after bTrainPresent and bTrainInPosition have been set from the
    // sensors — those two are real and are not the crew's to invent.
    void Serve(const FStationProcess& Station, FStationInputs& In, double DeltaSeconds)
    {
        const EStationPhase P = Station.GetPhase();
        if (P == EStationPhase::Empty || P == EStationPhase::Departing)
        {
            // Reset between trains. Without this the next train arrives to find
            // the last one's work already done and dispatches immediately.
            Elapsed = 0.0;
            In.bUnloadComplete = false;
            In.bLoadComplete = false;
            In.bRestraintsLocked = false;
            In.bPlatformClear = false;
            LastPhase = P;
            return;
        }
        if (P != LastPhase)
        {
            Elapsed = 0.0;
            LastPhase = P;
        }
        if (P == EStationPhase::Arriving)
        {
            return;   // nobody does anything until it stops
        }
        Elapsed += DeltaSeconds;

        switch (P)
        {
        case EStationPhase::Unloading:
            if (Elapsed >= UnloadSeconds) { In.bUnloadComplete = true; }
            break;
        case EStationPhase::Loading:
            if (Elapsed >= LoadSeconds) { In.bLoadComplete = true; }
            break;
        case EStationPhase::Securing:
            // Restraints first, then the walk-round that produces the all-clear.
            // In that order because the check is OF the restraints, and a crew
            // that cleared the platform before they were locked would be signing
            // off work it had not done.
            if (Elapsed >= SecureSeconds * 0.5) { In.bRestraintsLocked = true; }
            if (Elapsed >= SecureSeconds)       { In.bPlatformClear = true; }
            break;
        default:
            break;
        }
    }

    double DwellSeconds() const { return Elapsed; }

private:
    double Elapsed = 0.0;
    EStationPhase LastPhase = EStationPhase::Empty;
};
