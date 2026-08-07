// TrackUnlimited Tier 1: the safety chain, as wiring.
// Plain C++17, no dependencies.
//
// `CLAUDE.md` constraint 7: safety is C++, not scriptable, not overridable, and
// has no debug bypass that ships. `FTrackDrives` already holds the E-stop and its
// stop categories. What was missing is everything BELOW that — the wiring a real
// installation uses to make a stop trustworthy, which this project had been
// modelling as plain booleans.
//
// ===================== DE-ENERGISE TO TRIP =====================
//
// The single idea the whole discipline rests on, and it inverts the intuition.
// A safety input is not "a button that reports pressed". It is a CIRCUIT that
// carries current while everything is well, and a stop is DEMANDED BY THE ABSENCE
// OF CURRENT.
//
// So a broken wire, a pulled connector, a failed power supply and a pressed
// button are all THE SAME SIGNAL, and that is the entire point: the failure modes
// of the wiring land on the safe side by construction, rather than being enumerated
// and handled. Nothing here has to know why the current stopped.
//
// It is the same shape as three things already in this project — the fail-safe
// brake that bites when commanded to zero, CiA 402's active-low quick stop where a
// zeroed controlword commands a stop, and the block counter's falling edge. This
// is where the idea is finally named.
//
// ===================== WHY TWO CHANNELS OF OPPOSITE POLARITY =====================
//
// One channel fails to danger in one specific way: a short to the supply. Chafe a
// cable against a 24 V rail and a normally-closed contact reads energised for
// ever, which reads as SAFE, which means a pressed E-stop does nothing.
//
// Two channels of the SAME polarity do not help — one short is as likely to take
// both, since they run in the same loom. Two channels of OPPOSITE polarity do: the
// fault that makes one read safe makes the other read demanded, and a disagreement
// is a fault. That is why real dual-channel inputs are wired this way rather than
// simply doubled, and it is the difference between redundancy and diversity.
//
// ===================== AND WHY THE COMPARISON IS NOT INSTANT =====================
//
// A dual-channel button's two contact blocks are mechanically separate and never
// switch on the same millisecond. An instant comparison faults on every legitimate
// press. So real safety relays carry a DISCREPANCY WINDOW — typically 0.5 s, some
// 3 s — inside which the channels may disagree, and beyond which they may not.
//
// Units: seconds.

#pragma once

#include <cstddef>
#include <vector>

// How a channel is wired, which decides what "safe" reads as on the wire.
//
// NormallyClosed: current flows when all is well. Energised = safe.
// NormallyOpen:   current flows when the stop is DEMANDED. Energised = demand.
//
// A dual-channel input uses one of each, so no single electrical fault can put
// both into the state that means "carry on".
enum class EChannelPolarity
{
    NormallyClosed,
    NormallyOpen,
};

// One wire, and what it currently carries. `bEnergised` is the ELECTRICAL fact —
// deliberately not called `bSafe` or `bPressed`, because it is neither until the
// polarity says which.
struct FSafetyChannel
{
    EChannelPolarity Polarity = EChannelPolarity::NormallyClosed;
    bool bEnergised = true;

    // What this channel is asking for. A broken wire on a normally-closed channel
    // reads exactly like a pressed button, which is the whole design.
    bool DemandsStop() const
    {
        return Polarity == EChannelPolarity::NormallyClosed ? !bEnergised : bEnergised;
    }
};

// A dual-channel safety input: an E-stop button, a gate switch, a light curtain.
//
// TWO CHANNELS OF OPPOSITE POLARITY, compared every scan, with a window inside
// which they are allowed to disagree.
class FDualChannelInput
{
public:
    explicit FDualChannelInput(double InDiscrepancySeconds = 0.5)
        : DiscrepancyLimit(InDiscrepancySeconds)
    {
        A.Polarity = EChannelPolarity::NormallyClosed;
        B.Polarity = EChannelPolarity::NormallyOpen;
        A.bEnergised = true;      // healthy loop
        B.bEnergised = false;     // no demand
    }

    // The two wires, as the input card reads them. Raw electrical states, because
    // that is what a card has — it does not know what the far end is doing.
    void Wire(bool bChannelAEnergised, bool bChannelBEnergised)
    {
        A.bEnergised = bChannelAEnergised;
        B.bEnergised = bChannelBEnergised;
    }

    // Everything both channels are physically capable of, in one call, for a
    // caller simulating a person rather than a fault.
    void Press() { Wire(false, true); }
    void Release() { Wire(true, false); }

    // Once per scan. The discrepancy timer is the only state here.
    void Scan(double DeltaSeconds)
    {
        const bool bDisagree = A.DemandsStop() != B.DemandsStop();
        if (bDisagree)
        {
            Disagreeing += DeltaSeconds;
            if (Disagreeing >= DiscrepancyLimit && !bFaulted)
            {
                bFaulted = true;
                Fault = "dual-channel discrepancy: the two channels do not agree";
            }
        }
        else
        {
            Disagreeing = 0.0;
        }
    }

    // ===================== THE ONE ANSWER =====================
    //
    // A stop is demanded if EITHER channel says so, or if the pair has faulted.
    //
    // Either, not both — and that is not a redundancy failure, it is the point.
    // Two channels are there so that one of them still works, so one channel
    // asking for a stop is a stop. A pair that required agreement would be a
    // system where one broken wire disables the E-stop, which is the failure
    // dual-channel wiring exists to prevent.
    bool DemandsStop() const
    {
        return A.DemandsStop() || B.DemandsStop() || bFaulted;
    }

    bool IsFaulted() const { return bFaulted; }
    const char* FaultReason() const { return bFaulted ? Fault : ""; }
    double DisagreeingFor() const { return Disagreeing; }

    // Cleared by a person, and only while the channels actually agree — clearing a
    // discrepancy that is still present is clearing it without having fixed it.
    bool ClearFault()
    {
        if (A.DemandsStop() != B.DemandsStop()) { return false; }
        bFaulted = false;
        Fault = "";
        Disagreeing = 0.0;
        return true;
    }

    const FSafetyChannel& ChannelA() const { return A; }
    const FSafetyChannel& ChannelB() const { return B; }

private:
    FSafetyChannel A;
    FSafetyChannel B;
    double DiscrepancyLimit = 0.5;
    double Disagreeing = 0.0;
    bool bFaulted = false;
    const char* Fault = "";
};

// ===================== EXTERNAL DEVICE MONITORING =====================
//
// The other half of a safety relay, and the one people leave out.
//
// A relay that de-energises its output has done its job only if the CONTACTOR
// downstream actually opened. A contactor whose contacts have welded shut stays
// closed with its coil de-energised, so the motor keeps turning and the relay has
// no idea — it is looking at its own output, not at the machine.
//
// The fix is mechanical and is why safety contactors are specified with
// MIRROR CONTACTS: a normally-closed auxiliary that is mechanically linked to the
// main contacts, so it can only close when they are genuinely open. Wire those
// aux contacts in series into the RESET circuit and a welded contactor cannot be
// reset — the reset is a question the machine has to answer, not a button.
//
// `FAULTS.md` lists the welded contactor as caught by nothing. This is the thing
// it was waiting for.
class FContactorPair
{
public:
    // Two contactors in series, so one welding does not by itself pass power.
    // Redundancy on the OUTPUT side, matching the two channels on the input side.
    void Command(bool bClose)
    {
        bCommanded = bClose;
        if (!bWeldedK1) { bK1Closed = bClose; }
        if (!bWeldedK2) { bK2Closed = bClose; }
    }

    // A contactor that will not open again. Injectable, because a fault nothing
    // can produce is a fault nothing has tested.
    void Weld(bool bK1, bool bK2)
    {
        bWeldedK1 = bK1;
        bWeldedK2 = bK2;
        if (bK1) { bK1Closed = true; }
        if (bK2) { bK2Closed = true; }
    }

    // POWER REACHES THE MOTOR only through both. One welded contactor is a latent
    // fault rather than an immediate danger, which is exactly why there are two and
    // exactly why the latent one has to be found before the second fails.
    bool PowerFlows() const { return bK1Closed && bK2Closed; }

    // THE MIRROR CONTACTS, wired NC and mechanically linked. True only when BOTH
    // main contacts are genuinely open — which a welded one never is.
    bool FeedbackSaysOpen() const { return !bK1Closed && !bK2Closed; }

    bool IsCommandedClosed() const { return bCommanded; }
    bool AnyWelded() const { return bWeldedK1 || bWeldedK2; }

private:
    bool bCommanded = false;
    bool bK1Closed = false, bK2Closed = false;
    bool bWeldedK1 = false, bWeldedK2 = false;
};

// The safety relay itself: a dual-channel input, a monitored reset, and a
// contactor pair it watches.
//
// This is the box between the E-stop button and the motor, and it is where the
// wiring rules above become one boolean.
class FSafetyRelay
{
public:
    explicit FSafetyRelay(double DiscrepancySeconds = 0.5) : Input(DiscrepancySeconds) {}

    FDualChannelInput& Wiring() { return Input; }
    const FDualChannelInput& Wiring() const { return Input; }
    FContactorPair& Contactors() { return Output; }
    const FContactorPair& Contactors() const { return Output; }

    // ===================== INSTANTANEOUS AND DELAYED OUTPUTS =====================
    //
    // A real safety relay has both, and this project needs both because
    // FTrackDrives already distinguishes IEC 60204-1's two stop categories.
    //
    //   Zero (the default) is an INSTANTANEOUS output: the contactors drop the
    //   moment the demand arrives. That is Cat 0, which is STO — voltage away,
    //   coast.
    //
    //   Non-zero is a DELAYED output: the relay commands the stop at once and
    //   holds the contactors closed for this long, so the drive has power with
    //   which to perform a controlled stop before it is taken away. That is
    //   Cat 1, which is SS1.
    //
    // WITHOUT THIS, CAT 1 IS IMPOSSIBLE. If the same button opened the contactors
    // immediately there would be nothing left to ramp with, and "controlled stop
    // with power retained to achieve it" would be a Cat 0 wearing a label — which
    // is the exact confusion the stop-category work exists to end. The relay is
    // where the retention physically lives.
    //
    // The relay is ALREADY COMMANDING STOP throughout the delay. IsEnabled() goes
    // false immediately; only PowerFlows() lags. Those being two different
    // questions is the whole of it.
    void SetOffDelaySeconds(double Seconds) { OffDelay = Seconds > 0.0 ? Seconds : 0.0; }
    double OffDelaySeconds() const { return OffDelay; }

    // The reset button, scanned as a level. A RISING EDGE is required and the
    // action happens on RELEASE — the same monitored-reset shape as the E-stop in
    // FTrackDrives and CiA 402's fault reset, and for the same reason: a button
    // taped down must not be able to reset anything.
    void ScanReset(bool bPressedNow) { bResetHeld = bPressedNow; }

    // Once per scan, after the inputs are wired and the reset is scanned.
    void Scan(double DeltaSeconds)
    {
        Input.Scan(DeltaSeconds);

        // ---- STOP OVERRIDES START, and this ordering is the assertion.
        //
        // The demand is evaluated first and unconditionally. There is no path
        // below by which a reset, a start or anything else can put the output back
        // on in the same scan a stop is demanded — not because a check forbids it
        // but because the branch that would have to run is unreachable.
        if (Input.DemandsStop())
        {
            bEnabled = false;
            bLatched = true;
            bSeenResetLow = false;   // a reset held through a trip is not an edge
        }

        // ---- The monitored reset, and it only runs when everything is well.
        const bool bWasHeld = bResetHeldLast;
        bResetHeldLast = bResetHeld;

        if (!bResetHeld) { bSeenResetLow = true; }
        const bool bReleasedNow = bWasHeld && !bResetHeld;

        if (bLatched && bReleasedNow && bSeenResetLow && CanReset())
        {
            bLatched = false;
            bEnabled = true;
        }

        // The output follows the latch. The CONTACTORS follow the output, except
        // through a delayed drop-out, where they lag by the configured time.
        if (bEnabled)
        {
            bDropping = false;
            Dropping = 0.0;
            Output.Command(true);
        }
        else
        {
            // THE DELAY APPLIES TO A TRANSITION, NOT TO A STATE, and the first
            // version got that wrong in the one way that matters: armed from
            // construction, a relay that had NEVER BEEN ENABLED held its contactors
            // closed for the first five seconds of its life.
            //
            // A relay powers up latched with its contacts open. There is nothing to
            // wind down from, because nothing was ever turning. Only a drop-out
            // FROM enabled starts the clock, which is why this needs to know what
            // the last scan looked like rather than only what this one does.
            //
            // It was caught because the contactor feedback then refused the very
            // first reset — the mirror contacts correctly reporting that the mains
            // had never opened. One safety mechanism catching another's bug is
            // worth more than the assertion that noticed.
            if (bEnabledLastScan)
            {
                bDropping = OffDelay > 0.0;
                Dropping = 0.0;
            }
            else if (bDropping)
            {
                Dropping += DeltaSeconds;
                // AT THE DEADLINE, REGARDLESS. A real SS1 relay does not ask the
                // drive whether it finished ramping; it opens the contactor when
                // the timer expires. Same rule as FTrackDrives::SetCat1DelaySeconds
                // — the same physical relay described from two sides.
                if (Dropping >= OffDelay) { bDropping = false; }
            }
            Output.Command(bDropping);
        }
        bEnabledLastScan = bEnabled;
    }

    // WHY THE RESET WILL NOT COMPLETE, or nullptr if it will. Ordered so the first
    // answer is the one to act on, exactly as FPlcUnit::WhyNotRun is.
    const char* WhyNotReset() const
    {
        if (Input.DemandsStop())        { return "a stop is still demanded"; }
        if (Input.IsFaulted())          { return Input.FaultReason(); }
        if (!Output.FeedbackSaysOpen())
        {
            // The mirror contacts say a main contact is still closed while the
            // coil is de-energised. That is a WELDED CONTACTOR, and it is the one
            // thing a reset must never be able to talk its way past.
            return "contactor feedback: contacts have not opened (welded?)";
        }
        return nullptr;
    }

    bool CanReset() const { return WhyNotReset() == nullptr; }

    // The one output. True means power may reach the machine.
    bool IsEnabled() const { return bEnabled; }
    bool IsLatched() const { return bLatched; }

    // What the machine actually gets, which is not the same question — a welded
    // contactor passes power the relay believes it removed.
    bool PowerFlows() const { return Output.PowerFlows(); }

    // The disagreement worth surfacing: the relay thinks it is off and the machine
    // is still live. Nothing else in this project could previously say it.
    bool OutputDisagreesWithCommand() const { return !bEnabled && Output.PowerFlows(); }

private:
    FDualChannelInput Input;
    FContactorPair Output;
    bool bEnabled = false;
    bool bLatched = true;      // a relay powers up latched: somebody resets it
    double OffDelay = 0.0;
    double Dropping = 0.0;
    bool bDropping = false;        // inside a delayed drop-out right now
    bool bEnabledLastScan = false;
    bool bResetHeld = false;
    bool bResetHeldLast = false;
    bool bSeenResetLow = false;
};

// ===================== A RESTRAINT FAILS THE OTHER WAY =====================
//
// Everything above fails safe by REMOVING power. A restraint is the exception in
// this project and the exception in every park: a lap bar that dropped open when
// the power failed would be the worst possible failure, so the safe state is
// LOCKED and it takes positive, present energy to unlock.
//
// That inverts the rule rather than breaking it. "Fail-safe" was never
// "de-energise"; it is "fail to the state that cannot hurt anybody", and for a
// brake that is applied while for a restraint it is shut. Getting the two round
// the wrong way is a real and repeated industry mistake, which is why this is a
// separate type rather than a flag on the one above.
class FRestraintLock
{
public:
    // Unlocking is a COMMAND PLUS POWER, and both must be present. Either absent
    // and the bar stays down.
    void Command(bool bWantUnlocked) { bCommandedUnlock = bWantUnlocked; }
    void SetPowered(bool bOn) { bPowered = bOn; }

    bool IsUnlocked() const { return bCommandedUnlock && bPowered; }
    bool IsLocked() const { return !IsUnlocked(); }

    // What a maintainer needs to tell two identical-looking states apart: bars
    // down because the ride wants them down, versus bars down because the power
    // is gone and they could not move if asked.
    bool IsLockedByPowerLoss() const { return bCommandedUnlock && !bPowered; }

private:
    bool bCommandedUnlock = false;
    bool bPowered = true;
};

// ===================== PAIRED DIVERSE SENSORS =====================
//
// Two devices of DIFFERENT KINDS watching the same fact — a proximity switch and
// a photo eye on the same gate, say. Diversity rather than duplication, because
// two identical sensors share a failure mode and two different ones mostly do not.
//
// LOSS OF THE PAIR IS ITSELF A FAULT, which is the part usually missed. If one
// dies and the other still reads "safe", the safe reading is no longer trustworthy
// — it is a single point of failure wearing the appearance of a checked one. Real
// installations degrade to a stop rather than to a single channel.
class FDiversePair
{
public:
    void Report(bool bFirstSaysSafe, bool bSecondSaysSafe)
    {
        bFirst = bFirstSaysSafe;
        bSecond = bSecondSaysSafe;
        bSeen = true;
    }

    void Scan(double DeltaSeconds, double DisagreementLimit = 0.5)
    {
        if (!bSeen) { return; }
        if (bFirst != bSecond)
        {
            Disagreeing += DeltaSeconds;
            if (Disagreeing >= DisagreementLimit) { bLostPair = true; }
        }
        else
        {
            Disagreeing = 0.0;
        }
    }

    // Safe means BOTH say safe and the pair is intact. A disagreement is not
    // resolved by voting or by trusting the safer one: two sensors that disagree
    // have told you the measurement is unreliable, and that is the finding.
    bool IsSafe() const { return bSeen && bFirst && bSecond && !bLostPair; }
    bool HasLostThePair() const { return bLostPair; }
    void Reset() { bLostPair = false; Disagreeing = 0.0; }

private:
    bool bFirst = false, bSecond = false, bSeen = false, bLostPair = false;
    double Disagreeing = 0.0;
};

// ponytail: no OSSD pulse testing, no cross-monitoring between separate relays, no
// PL/SIL calculation. Those are how a real installation PROVES a category rather
// than implements one, and they need a failure-rate model this project has no
// business inventing. What is here is the behaviour; the certification arithmetic
// is a different document and not one an open-source coaster sim should publish.
