// Asserts for SessionState.h — the shell's session and document rules.
//
//   clang++ -std=c++17 -Wall -Wextra -O2 -o test_sessionstate test_sessionstate.cpp && ./test_sessionstate
//
// Every assertion here is about NOT LOSING SOMEBODY'S WORK. That is the one thing
// a shell has to get right, and it is the thing that is cheapest to design in and
// most expensive to retrofit.

#include "SessionState.h"

#include <cassert>
#include <cstdio>
#include <string>

namespace
{

const double Dt = 1.0 / 60.0;

// A session with a document open and the boot question settled.
FSession Open(const std::string& Text = "{track: a}")
{
    FSession S;
    S.Enter(EAppMode::MainMenu);
    S.DidOpen("C:/rides/thing.tutrack", Text);
    S.Observe(Text);
    S.Enter(EAppMode::Build);
    return S;
}

void TestDIRTYIsAComparisonNotAFlag()
{
    // THE ONE EVERYBODY HAS FELT. A bool set by every edit means undoing back to
    // where you started leaves the file still claiming to be modified — so you
    // save a file identical to the one on disc, or you are warned about
    // discarding changes that do not exist.
    //
    // Only affordable because this project already decided THE SAVE FORMAT IS THE
    // IDENTITY, so the comparison is against a string that already exists.
    FSession S = Open("{track: a}");
    assert(!S.IsDirty());

    S.Observe("{track: b}");
    assert(S.IsDirty());

    // Undo. The text is what it was, so the document is not modified — no flag to
    // clear and nothing to remember to clear it.
    S.Observe("{track: a}");
    assert(!S.IsDirty());
    std::printf("  undoing back to the saved text clears dirty, because dirty is a comparison\n");
}

void TestASaveIsMarkedCleanByWHATItWROTE()
{
    // A save takes the text it actually wrote. A caller that serialised something
    // different from what it displayed therefore cannot accidentally mark the
    // session clean — the comparison catches it on the next Observe.
    FSession S = Open("{track: a}");
    S.Observe("{track: b}");
    assert(S.IsDirty());

    S.DidSave("C:/rides/thing.tutrack", "{track: b}");
    assert(!S.IsDirty());
    assert(S.SaveCount() == 1);

    // And a save that wrote something else is still dirty, correctly, the moment
    // the shell looks again.
    S.Observe("{track: c}");
    S.DidSave("C:/rides/thing.tutrack", "{track: WRONG}");
    S.Observe("{track: c}");
    assert(S.IsDirty());
    std::printf("  a save is clean by what it WROTE, so a mismatched write stays dirty\n");
}

void TestANewDocumentNeedsSaveAS()
{
    // "Save" on something with no path means "save as", and the shell has to know
    // that before it puts a dialog up or fails to put one up.
    FSession S;
    S.Enter(EAppMode::MainMenu);
    S.DidCreateNew("{}");
    S.Observe("{}");
    assert(!S.HasPath());
    assert(S.NeedsSaveAs());

    S.DidSave("C:/rides/new.tutrack", "{}");
    assert(S.HasPath());
    assert(!S.NeedsSaveAs());
    std::printf("  a new document reports that save means save-as\n");
}

void TestAUTOSAVENeverTouchesTheDocumentAndOnlyRunsWhenDirty()
{
    // Writing over somebody's file on a timer is data loss with extra steps: it
    // destroys the last known-good state in order to preserve one they did not
    // ask for. So autosave writes a SIDECAR — this layer owns WHEN, the shell
    // owns HOW, and the document's own path is never involved.
    FSession S = Open("{track: a}");
    S.SetAutosaveSeconds(10.0);

    // NOT DIRTY, NOT WRITTEN. An autosave firing on a clean document would rewrite
    // an identical sidecar every interval for as long as the application is open.
    for (int i = 0; i < 60 * 60; ++i) { assert(!S.TickAutosave(Dt)); }
    assert(S.AutosaveCount() == 0);

    S.Observe("{track: b}");
    int Fired = 0;
    for (int i = 0; i < 60 * 25; ++i) { if (S.TickAutosave(Dt)) { ++Fired; } }
    assert(Fired == 2);                        // 25 s at a 10 s interval
    assert(S.HasPendingSidecar());

    // A real save supersedes the sidecar: there is now something on disc that
    // matches, so the recovery evidence is no longer evidence of anything.
    S.DidSave("C:/rides/thing.tutrack", "{track: b}");
    assert(!S.HasPendingSidecar());
    assert(S.Path() == "C:/rides/thing.tutrack");
    std::printf("  autosave fires %d times in 25 s while dirty, never while clean, never over the file\n",
                Fired);
}

void TestRecoveryIsOFFEREDNeverAPPLIED()
{
    // A recovery that opened itself would silently discard whatever the person did
    // deliberately after the crash — and relaunching specifically to start again is
    // not a rare case.
    FSession S;
    assert(S.Mode() == EAppMode::Boot);
    S.FoundSidecarAtBoot("C:/rides/thing.autosave", "{track: recovered}");
    assert(S.HasRecovery());

    // AND NOTHING LEAVES BOOT UNTIL IT IS ANSWERED. Opening a file with a recovery
    // still pending would answer the question by overwriting the sidecar.
    assert(!S.Enter(EAppMode::MainMenu));
    assert(S.Mode() == EAppMode::Boot);
    assert(S.WhyNotEnter(EAppMode::MainMenu) != nullptr);

    S.DeclineRecovery();
    assert(!S.HasRecovery());
    assert(S.Enter(EAppMode::MainMenu));
    std::printf("  a pending recovery blocks boot until somebody answers it\n");
}

void TestAnACCEPTEDRecoveryIsDIRTY()
{
    // Because it is. What was recovered has never been saved, and a recovery that
    // presented itself as clean would let somebody close it and lose the same work
    // a second time — which is the worst possible outcome for a feature whose
    // entire job is not losing it.
    FSession S;
    S.FoundSidecarAtBoot("C:/rides/thing.autosave", "{track: recovered}");
    assert(S.AcceptRecovery());
    S.Observe("{track: recovered}");

    assert(S.IsDirty());
    assert(S.HasPendingSidecar());
    assert(!S.HasRecovery());
    assert(S.Enter(EAppMode::MainMenu, /*bConfirmed*/ false) == false);   // and it warns
    std::printf("  an accepted recovery is dirty, so closing it warns rather than losing it twice\n");
}

void TestLeavingToTheMENUWithUnsavedWorkASKS()
{
    // The one transition that discards the document, so the one that needs asking
    // about. Refused-until-confirmed rather than silently allowed, and NOT
    // silently refused either — this layer returns the question and the shell puts
    // it on screen.
    FSession S = Open("{track: a}");
    S.Observe("{track: b}");

    assert(S.MayEnter(EAppMode::MainMenu) == ELeaveRequest::NeedsConfirmation);
    assert(!S.Enter(EAppMode::MainMenu));                  // unconfirmed: refused
    assert(S.Mode() == EAppMode::Build);
    assert(S.Enter(EAppMode::MainMenu, true));             // confirmed: allowed
    assert(S.Mode() == EAppMode::MainMenu);

    // Clean, it never asks, because there is nothing to ask about.
    FSession C = Open("{track: a}");
    assert(C.MayEnter(EAppMode::MainMenu) == ELeaveRequest::Allowed);
    assert(C.Enter(EAppMode::MainMenu));
    std::printf("  going back to the menu with unsaved work asks; clean, it does not\n");
}

void TestBUILDToOPERATEDoesNotAskBecauseNothingIsDiscarded()
{
    // The document is still open and still in memory. A shell that asked here
    // would train people to click through the dialog that matters.
    FSession S = Open("{track: a}");
    S.Observe("{track: b}");
    assert(S.IsDirty());

    assert(S.Enter(EAppMode::Operate));
    assert(S.Enter(EAppMode::Ride));
    assert(S.Enter(EAppMode::Build));
    assert(S.IsDirty());
    std::printf("  build to operate to ride and back never asks: nothing is discarded\n");
}

void TestEDITSAreAMODEQuestion()
{
    // CONSTRAINT 1, ONE LEVEL UP. A ride that is running is not a ride being
    // edited, and an edit landing mid-lap would change the geometry under a train.
    //
    // Structural rather than remembered: the shell asks this before accepting
    // anything, so a panel that forgot would simply not work rather than
    // corrupting a running ride.
    FSession S = Open();
    assert(S.EditsAllowed());

    S.Enter(EAppMode::Operate);
    assert(!S.EditsAllowed());
    S.Enter(EAppMode::Ride);
    assert(!S.EditsAllowed());
    S.Enter(EAppMode::Build);
    assert(S.EditsAllowed());

    // And nowhere else either.
    FSession M;
    assert(!M.EditsAllowed());                 // Boot
    M.Enter(EAppMode::MainMenu);
    assert(!M.EditsAllowed());
    std::printf("  edits are allowed in BUILD and in no other mode\n");
}

void TestYouCannotRIDEWhatIsNotOpen()
{
    // Not a confirmation — there is nothing to confirm, only nothing to do. And
    // the reason is readable, because "the button does nothing" is the single most
    // common complaint about any shell.
    FSession S;
    S.Enter(EAppMode::MainMenu);
    assert(S.MayEnter(EAppMode::Ride) == ELeaveRequest::Refused);
    assert(S.MayEnter(EAppMode::Operate) == ELeaveRequest::Refused);
    assert(std::string(S.WhyNotEnter(EAppMode::Ride)) == "open a track first");

    // And boot happens once.
    assert(S.MayEnter(EAppMode::Boot) == ELeaveRequest::Refused);
    std::printf("  riding needs something to ride, and boot happens once\n");
}

} // namespace

int main()
{
    std::printf("The session: what mode, and is there unsaved work\n\n");

    TestDIRTYIsAComparisonNotAFlag();
    TestASaveIsMarkedCleanByWHATItWROTE();
    TestANewDocumentNeedsSaveAS();
    TestAUTOSAVENeverTouchesTheDocumentAndOnlyRunsWhenDirty();
    TestRecoveryIsOFFEREDNeverAPPLIED();
    TestAnACCEPTEDRecoveryIsDIRTY();
    TestLeavingToTheMENUWithUnsavedWorkASKS();
    TestBUILDToOPERATEDoesNotAskBecauseNothingIsDiscarded();
    TestEDITSAreAMODEQuestion();
    TestYouCannotRIDEWhatIsNotOpen();

    std::printf("\ntest_sessionstate: all assertions passed.\n");
    return 0;
}
