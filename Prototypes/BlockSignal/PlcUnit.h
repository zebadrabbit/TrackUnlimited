// TrackUnlimited: the controller, as a machine in the cabinet.
// Plain C++17, no dependencies.
//
// `FRideSignals` plus `ServeHolds` is the PLC PROGRAM. Until this existed there
// was no PLC — the behaviour was there and the machine running it was implicit,
// which is exactly the gap the drives had before FTrackDrives and the sensors
// had before FTrackSensors. In both of those cases making the device real is
// what turned a correct-looking model into one that could disagree with itself
// and say so.
//
// ===================== THIS IS THE STANDARD PLC =====================
//
// NOT the safety chain. Real installations run a safety PLC beside a standard
// one, and here the safety chain is the E-stop inside FTrackDrives, where it
// stays. That split is constraint 7 made structural rather than promised:
//
//   THIS MACHINE CAN FAULT AND THE RIDE STILL STOPS SAFELY.
//
// It can withhold permission to run. It has no authority to prevent a stop, and
// there is deliberately no path here by which it could acquire one — a stop
// works because the brakes are fail-safe and because the E-stop overrides drive
// OUTPUT, neither of which routes through this class.
//
// ===================== WHAT A REAL ONE HAS =====================
//
//   a key switch          RUN / PROGRAM / STOPPED
//   a watchdog            a missed scan deadline is a FAULT, not a late scan
//   a program identity    installations care intensely about WHICH program is
//                         loaded; it is the centre of what gets certified
//   power                 and block state is not trusted across a power cycle
//
// Every one of those is a real operational state this project could not express.

#pragma once

#include <cstdint>

enum class EPlcMode
{
    // Powered, program loaded, doing nothing. The state a controller sits in
    // before an operator turns the key.
    Stopped,

    // Program mode: a program may be downloaded, and THE RIDE CANNOT RUN. The
    // honest home for "maintenance is working on it", which until now had
    // nowhere to be.
    Program,

    // Executing. Outputs live, if everything below is satisfied.
    Run,
};

class FPlcUnit
{
public:
    // ===================== POWER =====================
    //
    // Powering up does NOT restore the world as it was. Block occupancy is
    // derived from edges the controller watched happen, and a controller that
    // was off watched nothing — so on power-up it knows where no train is.
    //
    // Real practice is a manual course-clear walkdown and an operator reset, and
    // that is modelled rather than assumed: a fresh controller refuses RUN until
    // somebody has walked the course and said so. Anything else is a machine
    // inventing occupancy it cannot possibly know.
    void PowerOn()
    {
        bPowered = true;
        bCourseClear = false;    // nobody has walked it yet
        Mode = EPlcMode::Stopped;
        bFaulted = false;
        Fault = "";
        ScanSeconds = 0.0;
    }

    void PowerOff()
    {
        bPowered = false;
        bCourseClear = false;
        Mode = EPlcMode::Stopped;
    }

    bool IsPowered() const { return bPowered; }

    // The operator's walkdown: the course is empty and I have looked. Only
    // meaningful while stopped — asserting it on a running ride would be
    // claiming to have walked track with trains on it.
    bool DeclareCourseClear()
    {
        if (!bPowered || Mode == EPlcMode::Run) { return false; }
        bCourseClear = true;
        return true;
    }
    bool IsCourseClear() const { return bCourseClear; }

    // ===================== PROGRAM IDENTITY =====================
    //
    // What is loaded, against what the layout implies. Both are digests derived
    // from the block and zone walk rather than typed, so they cannot drift apart
    // by anybody forgetting to update one.
    //
    // A MISMATCH REFUSES RUN, and that is the most immediately useful thing this
    // class does: a program built for a different track is the failure behind
    // "I changed the code and the editor is still doing the old thing", which
    // has bitten this project before and had no detector at all.
    void LoadProgram(std::uint64_t Identity)
    {
        Loaded = Identity;
        bProgramLoaded = true;
    }
    void SetLayoutIdentity(std::uint64_t Identity) { Layout = Identity; }
    bool IsProgramLoaded() const { return bProgramLoaded; }
    bool ProgramMatchesLayout() const { return bProgramLoaded && Loaded == Layout; }
    std::uint64_t ProgramIdentity() const { return Loaded; }

    // ===================== THE SCAN, AND ITS WATCHDOG =====================
    //
    // A missed scan deadline is a FAULT on real hardware, not a late scan. The
    // detector already existed one layer up — the accumulator drops a backlog it
    // cannot work off — and reporting that as a note rather than a trip was the
    // controller having a symptom with nowhere to put it.
    //
    // LATCHED. A watchdog that cleared itself when the next scan came in on time
    // is a watchdog nobody ever sees, and the point of it is that somebody looks.
    void Scan(double DeltaSeconds, bool bOverran)
    {
        if (!bPowered) { return; }
        ScanSeconds = DeltaSeconds;
        ++Scans;
        if (bOverran && !bFaulted)
        {
            bFaulted = true;
            Fault = "watchdog: scan deadline missed";
        }
    }

    bool IsFaulted() const { return bFaulted; }
    const char* FaultReason() const { return bFaulted ? Fault : ""; }
    double LastScanSeconds() const { return ScanSeconds; }
    std::uint64_t ScanCount() const { return Scans; }

    // Cleared by a person, like every other latch here. Refused while running,
    // because clearing a fault on a machine that is still executing is clearing
    // it without having looked.
    bool ClearFault()
    {
        if (Mode == EPlcMode::Run) { return false; }
        bFaulted = false;
        Fault = "";
        return true;
    }

    // ===================== THE KEY SWITCH =====================
    //
    // Refused rather than silently ignored, and the reason is readable, because
    // "the ride will not start" with no explanation is the single most common
    // complaint about real ride control.
    bool RequestMode(EPlcMode Wanted)
    {
        if (!bPowered) { return false; }

        // LEAVING Run is always allowed. A mode change that could be refused
        // would be a stop with a precondition, and stops do not have those.
        if (Wanted != EPlcMode::Run)
        {
            Mode = Wanted;
            return true;
        }
        if (WhyNotRun() != nullptr) { return false; }
        Mode = EPlcMode::Run;
        return true;
    }

    EPlcMode GetMode() const { return Mode; }

    // Why this controller will not go to RUN, or nullptr if it will. Ordered so
    // the first answer is the one to act on.
    const char* WhyNotRun() const
    {
        if (!bPowered)               { return "no control power"; }
        if (bFaulted)                { return Fault; }
        if (!bProgramLoaded)         { return "no program loaded"; }
        if (!ProgramMatchesLayout()) { return "program does not match this layout"; }
        if (!bCourseClear)           { return "course not declared clear"; }
        return nullptr;
    }

    // ===================== THE ONE OUTPUT =====================
    //
    // Whether the program's commands reach the ride. Everything above exists to
    // decide this single boolean.
    //
    // It permits; it does not compel. A false here means the controller is not
    // commanding anything — which stops a ride only because a device with no
    // command falls to its safe state, exactly as it would with the controller
    // unplugged. The stop does not come from here and must not.
    bool OutputsEnabled() const
    {
        return bPowered && Mode == EPlcMode::Run && !bFaulted && ProgramMatchesLayout();
    }

private:
    bool bPowered = false;
    bool bCourseClear = false;
    bool bProgramLoaded = false;
    bool bFaulted = false;
    const char* Fault = "";
    EPlcMode Mode = EPlcMode::Stopped;
    std::uint64_t Loaded = 0;
    std::uint64_t Layout = 0;
    double ScanSeconds = 0.0;
    std::uint64_t Scans = 0;
};

// ponytail: no I/O modules as physical cards. More fidelity, no new behaviour —
// the process image is already the sensors and the drives, and modelling the
// backplane would be describing a cabinet rather than simulating one. Add it if
// something ever needs to fault a single card.
