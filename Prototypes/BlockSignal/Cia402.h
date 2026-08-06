// TrackUnlimited: the drive state machine, CiA 402 verbatim.
// Plain C++17, no dependencies.
//
// NOT AN APPROXIMATION OF A DRIVE. CiA 402 (IEC 61800-7-201/301, carried over
// EtherCAT as CoE) is the actual interface behind Beckhoff, KEB, SEW, Elmo and
// the rest, so implementing it as specified IS the real thing rather than a
// model of it. A controlword and a statusword are what a PLC and a VFD say to
// each other on real hardware.
//
// ===================== WHY THIS, WHEN THE DRIVES WORK =====================
//
// FTrackDrives already ramps, slips and faults, and IsReady() already means
// "output has reached command and nothing is faulted". That is behaviourally
// close to Operation enabled and it is not the same thing: it cannot say WHY a
// drive is not ready, cannot express a drive that is powered but not enabled,
// and has nowhere to put the enable handshake a real PLC has to perform.
//
// This project's pattern is that making a device real is what turns a
// correct-looking model into one that can disagree with itself. That is what
// happened with commanded-vs-output-vs-actual, with per-group restraints, and
// with the controller. Eight states is the same move on the drive.
//
// ===================== WHAT IS DELIBERATELY NOT HERE =====================
//
// Fault registers by code, S-curve ramp parameters, torque and current limiting,
// braking resistor versus regenerative front end, and the V/f-versus-vector
// control topologies. All real, all catalogue detail, and none of it changes a
// state transition. The state machine is the part the PLC talks to.

#pragma once

#include <cstdint>

// ===================== THE EIGHT STATES =====================
//
// Only ONE of them produces torque, which is the whole reason the machine has
// eight rather than a bool.
enum class ECia402State
{
    NotReadyToSwitchOn,   // powering up, self-testing
    SwitchOnDisabled,     // alive, no DC bus or not permitted
    ReadyToSwitchOn,      // bus up, waiting to be switched on
    SwitchedOn,           // output stage on, NOT yet enabled — no torque
    OperationEnabled,     // THE ONLY STATE WITH TORQUE
    QuickStopActive,      // executing a quick stop ramp
    FaultReactionActive,  // running its configured reaction, not yet settled
    Fault,                // settled, latched, awaiting a rising-edge reset
};

// Controlword 6040h, the bits that matter.
namespace Cia402Cw
{
    constexpr std::uint16_t SwitchOn        = 1 << 0;
    constexpr std::uint16_t EnableVoltage   = 1 << 1;
    // ACTIVE LOW. A 1 here means "not quick stopping" — the bit is a permission
    // that is removed rather than a command that is given, so a broken wire or a
    // zeroed word commands a quick stop. That is de-energise-to-trip, in a
    // fieldbus word.
    constexpr std::uint16_t QuickStop       = 1 << 2;
    constexpr std::uint16_t EnableOperation = 1 << 3;
    constexpr std::uint16_t FaultReset      = 1 << 7;

    // The handshake a PLC actually performs, in order.
    constexpr std::uint16_t Shutdown        = 0x0006;  // -> Ready to switch on
    constexpr std::uint16_t SwitchOnCmd     = 0x0007;  // -> Switched on
    constexpr std::uint16_t EnableOp        = 0x000F;  // -> Operation enabled
    constexpr std::uint16_t DisableVoltage  = 0x0000;
    constexpr std::uint16_t QuickStopCmd    = 0x0002;  // voltage on, quick stop asserted
}

// Statusword 6041h.
namespace Cia402Sw
{
    constexpr std::uint16_t ReadyToSwitchOn  = 1 << 0;
    constexpr std::uint16_t SwitchedOn       = 1 << 1;
    constexpr std::uint16_t OperationEnabled = 1 << 2;
    constexpr std::uint16_t Fault            = 1 << 3;
    constexpr std::uint16_t VoltageEnabled   = 1 << 4;
    constexpr std::uint16_t QuickStop        = 1 << 5;  // ACTIVE LOW, as above
    constexpr std::uint16_t SwitchOnDisabled = 1 << 6;
    constexpr std::uint16_t Warning          = 1 << 7;
    constexpr std::uint16_t Remote           = 1 << 9;
    constexpr std::uint16_t TargetReached    = 1 << 10;
    constexpr std::uint16_t InternalLimit    = 1 << 11;
}

// What the drive does on its way into Fault (605Eh). It is NOT instant, and that
// matters: a launch that faults mid-push does something for a moment before it
// settles, and which something is a configured property of the drive.
enum class ECia402FaultReaction
{
    Coast,        // output off immediately. The Cat 0 shape.
    SlowDownRamp, // decelerate on the normal ramp, then off. The Cat 1 shape.
    QuickStopRamp,
};

class FCia402Drive
{
public:
    ECia402State State() const { return Current; }

    // ---- Statusword, assembled from the state rather than stored beside it, so
    // the two cannot disagree. A stored word is a second copy of the truth.
    std::uint16_t Statusword() const
    {
        std::uint16_t W = 0;
        switch (Current)
        {
        case ECia402State::NotReadyToSwitchOn:
            break;
        case ECia402State::SwitchOnDisabled:
            W |= Cia402Sw::SwitchOnDisabled;
            break;
        case ECia402State::ReadyToSwitchOn:
            W |= Cia402Sw::ReadyToSwitchOn | Cia402Sw::QuickStop | Cia402Sw::VoltageEnabled;
            break;
        case ECia402State::SwitchedOn:
            W |= Cia402Sw::ReadyToSwitchOn | Cia402Sw::SwitchedOn
               | Cia402Sw::QuickStop | Cia402Sw::VoltageEnabled;
            break;
        case ECia402State::OperationEnabled:
            W |= Cia402Sw::ReadyToSwitchOn | Cia402Sw::SwitchedOn
               | Cia402Sw::OperationEnabled | Cia402Sw::QuickStop | Cia402Sw::VoltageEnabled;
            break;
        case ECia402State::QuickStopActive:
            // Bit 5 CLEARED is what says a quick stop is executing. Active low.
            W |= Cia402Sw::ReadyToSwitchOn | Cia402Sw::SwitchedOn
               | Cia402Sw::OperationEnabled | Cia402Sw::VoltageEnabled;
            break;
        case ECia402State::FaultReactionActive:
            W |= Cia402Sw::ReadyToSwitchOn | Cia402Sw::SwitchedOn
               | Cia402Sw::OperationEnabled | Cia402Sw::Fault;
            break;
        case ECia402State::Fault:
            W |= Cia402Sw::Fault;
            break;
        }
        if (bRemote)        { W |= Cia402Sw::Remote; }
        if (bTargetReached) { W |= Cia402Sw::TargetReached; }
        if (bInternalLimit) { W |= Cia402Sw::InternalLimit; }
        return W;
    }

    // Only Operation enabled produces torque. Everything else in this file exists
    // to decide this one question, exactly as it does on a real drive.
    bool ProducesTorque() const { return Current == ECia402State::OperationEnabled; }

    // What a panel actually shows, beside the state.
    void SetTargetReached(bool b) { bTargetReached = b; }
    void SetInternalLimit(bool b) { bInternalLimit = b; }
    void SetRemote(bool b)        { bRemote = b; }

    void SetFaultReaction(ECia402FaultReaction R) { Reaction = R; }
    ECia402FaultReaction FaultReaction() const { return Reaction; }

    // ---- The drive's own faults. Raised by the hardware, not by the PLC.
    void RaiseFault()
    {
        if (Current == ECia402State::Fault || Current == ECia402State::FaultReactionActive)
        {
            return;   // latched; the first cause is the one worth keeping
        }
        Current = ECia402State::FaultReactionActive;
        ReactionElapsed = 0.0;
    }

    // ---- One scan. The controlword arrives, the state machine runs.
    void Write(std::uint16_t Controlword, double DeltaSeconds = 0.0)
    {
        // FAULT RESET NEEDS A RISING EDGE on bit 7. Holding 0x0080 does nothing
        // at all, which is the classic real-world mistake — an operator wedges
        // the reset and wonders why the drive never comes back. Modelled rather
        // than smoothed over, because letting somebody make that mistake here is
        // cheaper than letting them make it on a ride.
        const bool bResetNow = (Controlword & Cia402Cw::FaultReset) != 0;
        const bool bResetEdge = bResetNow && !bResetWasHigh;
        bResetWasHigh = bResetNow;

        // The fault reaction runs for its own time before the drive settles. A
        // launch that faults mid-push is still doing something for a moment, and
        // WHICH something is configured rather than assumed.
        if (Current == ECia402State::FaultReactionActive)
        {
            ReactionElapsed += DeltaSeconds;
            const double Needed = Reaction == ECia402FaultReaction::Coast ? 0.0 : ReactionSeconds;
            if (ReactionElapsed >= Needed) { Current = ECia402State::Fault; }
            return;
        }

        if (Current == ECia402State::Fault)
        {
            if (bResetEdge) { Current = ECia402State::SwitchOnDisabled; }
            return;
        }

        // Quick stop is ACTIVE LOW: the bit being CLEAR asserts it. A zeroed
        // controlword — a dead master, a broken wire — therefore commands a quick
        // stop rather than nothing, which is de-energise-to-trip in a fieldbus.
        const bool bVoltage   = (Controlword & Cia402Cw::EnableVoltage) != 0;
        const bool bQuickStop = (Controlword & Cia402Cw::QuickStop) == 0;
        const bool bSwitchOn  = (Controlword & Cia402Cw::SwitchOn) != 0;
        const bool bEnableOp  = (Controlword & Cia402Cw::EnableOperation) != 0;

        if (Current == ECia402State::NotReadyToSwitchOn)
        {
            Current = ECia402State::SwitchOnDisabled;   // self-test complete
            return;
        }

        // Losing voltage drops straight out, from anywhere. It is the supply
        // going away, not a request.
        if (!bVoltage)
        {
            Current = ECia402State::SwitchOnDisabled;
            return;
        }

        if (Current == ECia402State::QuickStopActive)
        {
            QuickStopElapsed += DeltaSeconds;
            if (QuickStopElapsed >= ReactionSeconds)
            {
                Current = ECia402State::SwitchOnDisabled;
            }
            return;
        }

        if (bQuickStop && Current == ECia402State::OperationEnabled)
        {
            Current = ECia402State::QuickStopActive;
            QuickStopElapsed = 0.0;
            return;
        }
        if (bQuickStop)
        {
            Current = ECia402State::SwitchOnDisabled;
            return;
        }

        // THE HANDSHAKE, and it is strictly one step at a time. A PLC cannot go
        // from Switch-on-disabled to Operation-enabled by writing 0x000F once:
        // it writes Shutdown, WAITS for Ready to switch on, writes Switch on,
        // WAITS for Switched on, and only then enables. Skipping a rung is the
        // other classic mistake, and it silently does nothing.
        switch (Current)
        {
        case ECia402State::SwitchOnDisabled:
            if (bVoltage && !bSwitchOn) { Current = ECia402State::ReadyToSwitchOn; }
            break;
        case ECia402State::ReadyToSwitchOn:
            if (bSwitchOn) { Current = ECia402State::SwitchedOn; }
            break;
        case ECia402State::SwitchedOn:
            if (!bSwitchOn)     { Current = ECia402State::ReadyToSwitchOn; }
            else if (bEnableOp) { Current = ECia402State::OperationEnabled; }
            break;
        case ECia402State::OperationEnabled:
            if (!bSwitchOn)     { Current = ECia402State::ReadyToSwitchOn; }
            else if (!bEnableOp) { Current = ECia402State::SwitchedOn; }
            break;
        default:
            break;
        }
    }

    // How long a fault reaction or a quick stop ramp takes.
    double ReactionSeconds = 0.5;

private:
    ECia402State Current = ECia402State::NotReadyToSwitchOn;
    ECia402FaultReaction Reaction = ECia402FaultReaction::SlowDownRamp;
    double ReactionElapsed = 0.0;
    double QuickStopElapsed = 0.0;
    bool bResetWasHigh = false;
    bool bRemote = true;
    bool bTargetReached = false;
    bool bInternalLimit = false;
};

// ponytail: the STO/SS1 mapping onto IEC 61800-5-2 is one line of commentary
// rather than a second mechanism, because it already exists. FTrackDrives'
// EStopCategory::Zero is STO — power removed, the drive coasts, which is
// FaultReaction::Coast. EStopCategory::One is SS1 — ramp under control, then
// remove power, which is FaultReaction::SlowDownRamp followed by voltage going
// away. The two standards describe the same two paths and this project already
// built both.
