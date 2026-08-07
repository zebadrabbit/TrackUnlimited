// TrackUnlimited Phase 3.5: the session, and the document inside it.
// Plain C++17, no engine dependency, same conventions as the other prototypes.
//
// The shell's two questions — WHAT MODE ARE WE IN and IS THERE UNSAVED WORK —
// and they are one piece because they interact: leaving a mode with unsaved work
// is the moment a shell either protects somebody's evening or loses it.
//
// Engine-free on purpose. Every rule below is a rule about state, and putting it
// in a UMG widget would make it untestable and would scatter it across whichever
// button happened to be pressed.
//
// ===================== DIRTY IS A COMPARISON, NOT A FLAG =====================
//
// The obvious implementation is a bool set by every edit. It is wrong in a way
// everybody has felt: undo back to where you started, and the file still claims
// to be modified. Then you save a file identical to the one on disk, or you
// discard changes that do not exist.
//
// So dirty is `current serialisation != serialisation at last save`. That is only
// affordable because this project already decided the SAVE FORMAT IS THE IDENTITY
// (`TrackHistory.h` snapshots against it), so the comparison is a string compare
// against something that already exists.
//
// ===================== AUTOSAVE NEVER TOUCHES YOUR FILE =====================
//
// Writing over somebody's document on a timer is data loss with extra steps: it
// destroys the last known-good state in order to preserve a state they did not
// ask for. Autosave writes a SIDECAR, and the sidecar is offered on next launch
// rather than applied.
//
// Offered, not applied, for the same reason: a recovery that opened itself would
// silently discard whatever the person did deliberately after the crash.
//
// Units: seconds.

#pragma once

#include <cstddef>
#include <string>

// Where the application is. Four states and a boot, because the modes were
// decided by the program-flow card and they are what the shell's navigation is.
enum class EAppMode
{
    // Before anything is loaded. Not a menu — the moment where a crashed
    // session's sidecar is discovered, which has to happen before anybody can
    // open anything and overwrite the evidence.
    Boot,

    MainMenu,

    // The editing surface: the Details panel's shipping replacement. Numeric and
    // parametric, per constraint 1 — the viewport is a preview here as everywhere.
    Build,

    // The ride running under its control system, with the generated panel.
    Operate,

    // On-ride. A camera, and nothing else.
    Ride,
};

// APP MODE, not just "mode" — this header is compiled into the engine
// alongside a control panel that already has a local `ModeName` for the
// PLC's key switch. A global that a local shadows is legal and is a trap:
// it compiles, and the day somebody moves a line it silently means the
// other thing.
inline const char* AppModeName(EAppMode M)
{
    switch (M)
    {
    case EAppMode::Boot:     return "BOOT";
    case EAppMode::MainMenu: return "MENU";
    case EAppMode::Build:    return "BUILD";
    case EAppMode::Operate:  return "OPERATE";
    default:                 return "RIDE";
    }
}

// What a caller is asked when leaving a mode would lose work. Returned rather
// than resolved, because this layer must not invent an answer on somebody's
// behalf — the shell puts the question on screen and comes back.
enum class ELeaveRequest
{
    Allowed,
    NeedsConfirmation,   // unsaved work; ask, do not decide
    Refused,             // never, for a reason the caller can read
};

class FSession
{
public:
    // ===================== THE DOCUMENT =====================

    // Called with the current serialised document, every time the shell wants to
    // know. Cheap enough because the save format already exists as a string —
    // this is a comparison, not a second serialiser.
    void Observe(const std::string& CurrentSaveText) { Current = CurrentSaveText; }

    bool IsDirty() const { return Current != Saved; }

    // A save takes the text it wrote, so dirty becomes false by COMPARISON rather
    // than by being told. A caller that saved something different from what it
    // showed cannot accidentally mark the session clean.
    void DidSave(const std::string& Path, const std::string& WrittenText)
    {
        DocPath = Path;
        Saved = WrittenText;
        ++Saves;
        SinceAutosave = 0.0;
        bSidecarPending = false;
    }

    void DidOpen(const std::string& Path, const std::string& Text)
    {
        DocPath = Path;
        Saved = Text;
        Current = Text;
        SinceAutosave = 0.0;
        bSidecarPending = false;
    }

    // A new document has no path, so "save" means "save as" — which the shell has
    // to know before it puts a file dialog up or fails to.
    void DidCreateNew(const std::string& Text)
    {
        DocPath.clear();
        Saved = Text;
        Current = Text;
        SinceAutosave = 0.0;
        bSidecarPending = false;
    }

    const std::string& Path() const { return DocPath; }
    bool HasPath() const { return !DocPath.empty(); }
    bool NeedsSaveAs() const { return !HasPath(); }
    std::size_t SaveCount() const { return Saves; }

    // ===================== AUTOSAVE =====================
    //
    // Once per frame. Returns true on the frame a sidecar should be written, and
    // the caller writes it — this layer owns WHEN, not HOW, because file I/O is
    // the engine's and the policy is not.
    bool TickAutosave(double DeltaSeconds)
    {
        if (!bAutosaveEnabled || !IsDirty())
        {
            // NOT DIRTY, NOT WRITTEN. An autosave that fired on a clean document
            // would rewrite an identical sidecar every interval for as long as the
            // application is open, which on a laptop is a disc spinning up all
            // afternoon for nothing.
            SinceAutosave = 0.0;
            return false;
        }
        SinceAutosave += DeltaSeconds;
        if (SinceAutosave < AutosaveSeconds) { return false; }
        SinceAutosave = 0.0;
        bSidecarPending = true;
        ++Autosaves;
        return true;
    }

    void SetAutosaveSeconds(double S) { AutosaveSeconds = S > 0.0 ? S : 0.0; }
    void SetAutosaveEnabled(bool bOn) { bAutosaveEnabled = bOn; }
    std::size_t AutosaveCount() const { return Autosaves; }

    // A sidecar exists on disc and has not been superseded by a real save. That is
    // exactly the condition a crash leaves behind.
    bool HasPendingSidecar() const { return bSidecarPending; }

    // ===================== CRASH RECOVERY =====================
    //
    // OFFERED, NEVER APPLIED. A recovery that opened itself would silently discard
    // whatever the person did deliberately after the crash — and the case where
    // somebody relaunches specifically to start again is not rare.
    //
    // Discovered at BOOT, before anything can be opened, because opening a
    // document is what would overwrite the evidence.
    void FoundSidecarAtBoot(const std::string& SidecarPath, const std::string& Text)
    {
        RecoveryPath = SidecarPath;
        RecoveryText = Text;
    }
    bool HasRecovery() const { return !RecoveryPath.empty(); }
    const std::string& RecoveryFile() const { return RecoveryPath; }

    // The person said yes. The recovered text becomes the CURRENT document and the
    // session is DIRTY — because it is: what was recovered has never been saved,
    // and a recovery that presented itself as clean would let somebody close it
    // and lose it a second time.
    bool AcceptRecovery()
    {
        if (!HasRecovery()) { return false; }
        Current = RecoveryText;
        Saved = "";                 // nothing on disc matches this
        bSidecarPending = true;
        RecoveryPath.clear();
        RecoveryText.clear();
        return true;
    }

    void DeclineRecovery()
    {
        RecoveryPath.clear();
        RecoveryText.clear();
    }

    // ===================== THE MODE MACHINE =====================

    EAppMode Mode() const { return Current_Mode; }

    // May we leave the current mode for that one, and what does the caller owe the
    // person first?
    ELeaveRequest MayEnter(EAppMode Wanted) const
    {
        if (Wanted == Current_Mode) { return ELeaveRequest::Allowed; }

        // BOOT IS NOT RE-ENTERABLE. It exists to discover a crash sidecar before
        // anything can overwrite it, and going back there later would mean
        // re-asking a question already answered.
        if (Wanted == EAppMode::Boot) { return ELeaveRequest::Refused; }

        // And nothing leaves Boot until the recovery question is settled. An
        // application that let somebody open a file with a recovery still pending
        // would answer it for them by overwriting the sidecar.
        if (Current_Mode == EAppMode::Boot && HasRecovery())
        {
            return ELeaveRequest::Refused;
        }

        // GOING BACK TO THE MENU DISCARDS THE DOCUMENT, so it is the one transition
        // that needs asking about. Build to Operate and back does not: the document
        // is still open and still in memory.
        if (Wanted == EAppMode::MainMenu && IsDirty())
        {
            return ELeaveRequest::NeedsConfirmation;
        }

        // Riding or operating needs something to ride. Not a confirmation — there
        // is nothing to confirm, only nothing to do.
        if ((Wanted == EAppMode::Operate || Wanted == EAppMode::Ride)
            && Current_Mode == EAppMode::MainMenu)
        {
            return ELeaveRequest::Refused;
        }
        return ELeaveRequest::Allowed;
    }

    const char* WhyNotEnter(EAppMode Wanted) const
    {
        switch (MayEnter(Wanted))
        {
        case ELeaveRequest::Allowed: return nullptr;
        case ELeaveRequest::NeedsConfirmation: return "there is unsaved work";
        default: break;
        }
        if (Wanted == EAppMode::Boot) { return "boot happens once"; }
        if (Current_Mode == EAppMode::Boot) { return "a recovered session is waiting to be accepted or declined"; }
        return "open a track first";
    }

    // Enter, with the confirmation already obtained if one was needed. A caller
    // that passes bConfirmed without having asked is lying to this class, and this
    // class cannot tell — which is why the shell's confirm dialog is the thing to
    // get right, and why MayEnter is separate from Enter rather than a bool
    // parameter on one call.
    bool Enter(EAppMode Wanted, bool bConfirmed = false)
    {
        const ELeaveRequest R = MayEnter(Wanted);
        if (R == ELeaveRequest::Refused) { return false; }
        if (R == ELeaveRequest::NeedsConfirmation && !bConfirmed) { return false; }
        Current_Mode = Wanted;
        return true;
    }

    // ===================== EDITS ARE A MODE QUESTION =====================
    //
    // CONSTRAINT 1, ONE LEVEL UP. The viewport is a read-only preview, and in
    // Operate and Ride the whole application is: a ride that is running is not a
    // ride being edited, and an edit landing mid-lap would change the geometry
    // under a train.
    //
    // Structural rather than a rule anybody has to remember — the shell asks this
    // before it accepts anything, so a panel that forgot would simply not work
    // rather than corrupting a running ride.
    bool EditsAllowed() const { return Current_Mode == EAppMode::Build; }

private:
    std::string DocPath;
    std::string Saved;       // what is on disc
    std::string Current;     // what is in front of the person
    std::string RecoveryPath;
    std::string RecoveryText;

    EAppMode Current_Mode = EAppMode::Boot;

    double AutosaveSeconds = 120.0;   // two minutes, a knob
    double SinceAutosave = 0.0;
    bool bAutosaveEnabled = true;
    bool bSidecarPending = false;

    std::size_t Saves = 0;
    std::size_t Autosaves = 0;
};

// ponytail: no undo stack here — TrackHistory.h already owns that, and dirty is
// a comparison against the saved text precisely so the two do not have to know
// about each other. No recent-files list, no window layout, no per-document
// settings: those are preferences rather than session state, and a shell that
// mixed them would make "did this change need saving?" ambiguous.
