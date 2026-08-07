// Asserts for Settings.h — settings and input bindings.
//
//   clang++ -std=c++17 -Wall -Wextra -O2 -o test_settings test_settings.cpp && ./test_settings

#include "Settings.h"

#include <cassert>
#include <cstdio>
#include <string>

namespace
{

FSettings Fresh()
{
    FSettings S;
    S.Declare("graphics.shadows", "true");
    S.Declare("graphics.scale", "1.0");
    S.Declare("sim.hz", "240");
    S.Declare("ui.showTelemetry", "true");
    return S;
}

void TestADEFAULTIsNotAValue()
{
    // THE RULE THAT DECIDES WHETHER A CONFIG SYSTEM AGES WELL. Writing every
    // setting to the file means the default can never change again: everybody who
    // ever launched the application has the old one written down as though they
    // chose it, so a better default reaches new users only.
    FSettings S = Fresh();
    assert(S.Get("sim.hz") == "240");
    assert(S.IsAtDefault("sim.hz"));
    assert(S.NumExplicit() == 0);

    // A saved file contains NOTHING but what was chosen.
    assert(S.Save().find("sim.hz") == std::string::npos);

    S.Set("graphics.shadows", "false");
    assert(S.WasSetExplicitly("graphics.shadows"));
    assert(S.Save().find("graphics.shadows = false") != std::string::npos);
    assert(S.Save().find("sim.hz") == std::string::npos);
    std::printf("  a settings file holds only what was chosen, never the defaults\n");
}

void TestChangingADEFAULTReachesEverybodyWhoNeverTouchedIt()
{
    // The payoff, and the whole reason for the rule above. Somebody's file is
    // loaded, the application ships a better default, and it takes effect —
    // without touching the one setting they did choose.
    FSettings Old = Fresh();
    Old.Set("graphics.shadows", "false");
    const std::string File = Old.Save();

    // A later build: the sim rate default improves and the shadow default flips.
    FSettings New;
    New.Declare("graphics.shadows", "false");
    New.Declare("graphics.scale", "1.0");
    New.Declare("sim.hz", "480");
    New.Declare("ui.showTelemetry", "true");
    New.Load(File);

    assert(New.Get("sim.hz") == "480");            // improved, and it reached them
    assert(New.Get("graphics.shadows") == "false");// their choice, still theirs
    assert(New.WasSetExplicitly("graphics.shadows"));
    std::printf("  a better default reaches everybody who never chose otherwise\n");
}

void TestRESETIsADeletionNotAWrite()
{
    // Writing the default's current text on reset would freeze today's value into
    // the file and undo the entire point one setting at a time.
    FSettings S = Fresh();
    S.Set("sim.hz", "120");
    assert(S.Get("sim.hz") == "120");

    S.Reset("sim.hz");
    assert(S.Get("sim.hz") == "240");
    assert(S.IsAtDefault("sim.hz"));
    assert(S.Save().find("sim.hz") == std::string::npos);
    std::printf("  resetting a setting deletes it rather than writing today's default\n");
}

void TestUNKNOWNKeysSURVIVEARoundTrip()
{
    // A config written by a NEWER version contains keys this build has never heard
    // of. Dropping them on load and rewriting the file DESTROYS THEM — so running
    // an older build once, for any reason, silently resets everything the newer
    // one added.
    //
    // It costs a map, and it is the difference between "I ran the old version to
    // check something" being harmless and being an afternoon.
    const std::string FromNewer =
        "graphics.shadows = false\n"
        "raytracing.bounces = 4\n"
        "vr.comfortVignette = true\n";

    FSettings Old = Fresh();
    Old.Load(FromNewer);
    assert(Old.NumUnknownKept() == 2);
    assert(Old.Get("graphics.shadows") == "false");

    // The old build changes something and saves. The newer version's settings are
    // still in the file, untouched.
    Old.Set("sim.hz", "120");
    const std::string Rewritten = Old.Save();
    assert(Rewritten.find("raytracing.bounces = 4") != std::string::npos);
    assert(Rewritten.find("vr.comfortVignette = true") != std::string::npos);
    assert(Rewritten.find("sim.hz = 120") != std::string::npos);

    // And the newer build reads its own settings back intact.
    FSettings New = Fresh();
    New.Declare("raytracing.bounces", "1");
    New.Declare("vr.comfortVignette", "false");
    New.Load(Rewritten);
    assert(New.Get("raytracing.bounces") == "4");
    assert(New.GetBool("vr.comfortVignette"));
    assert(New.GetNumber("sim.hz") == 120.0);
    std::printf("  running an older build keeps the newer one's settings intact\n");
}

void TestAHandEditedFileSurvivesBeingMessy()
{
    // A settings file is one of the few things a person edits by hand when
    // something has gone wrong, so a stray blank line or a comment must not cost
    // them the file. That is why it is not JSON.
    FSettings S = Fresh();
    S.Load("# my settings\n"
           "\n"
           "  graphics.scale   =   1.5  \n"
           "this line has no equals sign\n"
           "= orphan value\n"
           "sim.hz=60\n");
    assert(S.Get("graphics.scale") == "1.5");
    assert(S.Get("sim.hz") == "60");
    assert(S.NumExplicit() == 2);
    std::printf("  whitespace, comments, blank lines and junk are survived, not fatal\n");
}

// ---------------------------------------------------------------- bindings

void TestREBINDINGReplacesRatherThanAdds()
{
    // An action bound to two keys is almost never what somebody meant by dragging
    // a key onto it — and if it is, they can bind an alias action.
    FInputMap M;
    M.Bind("Dispatch", "Space");
    M.Bind("Dispatch", "Enter");
    assert(M.Num() == 1);
    assert(M.KeyFor("Dispatch") == "Enter");

    M.Unbind("Dispatch");
    assert(M.Num() == 0);
    assert(M.KeyFor("Dispatch").empty());
    std::printf("  rebinding an action moves its key rather than adding a second\n");
}

void TestCONTEXTIsWhyOneKeyCanMeanTwoThings()
{
    // The part people leave out, and it is why [Space] can be dispatch on the
    // operator panel and something else entirely while riding without either being
    // a mistake.
    FInputMap M;
    M.Bind("Dispatch", "Space", "operate");
    M.Bind("LookBack", "Space", "ride");
    assert(!M.HasConflicts());
    assert(M.KeyFor("Dispatch", "operate") == "Space");
    assert(M.KeyFor("LookBack", "ride") == "Space");

    // Same context, same key: a real conflict.
    M.Bind("EStop", "Space", "operate");
    assert(M.HasConflicts());
    assert(M.Conflicts().size() == 1);
    assert(M.Conflicts()[0].Key == "Space");
    std::printf("  one key means two things in two contexts, and one thing in one\n");
}

void TestAGLOBALBindingConflictsWithEVERYTHING()
{
    // THE CASE PEOPLE MISS, because it looks fine in whichever context they happen
    // to be looking at. A global binding is live everywhere, so it fights every
    // context's use of that key.
    FInputMap M;
    M.Bind("Dispatch", "Space", "operate");
    M.Bind("LookBack", "Space", "ride");
    assert(!M.HasConflicts());

    M.Bind("Screenshot", "Space");             // global
    const std::vector<FBindingConflict> C = M.Conflicts();
    assert(C.size() == 2);                     // it fights BOTH
    std::printf("  a global binding conflicts with every context, not just its own\n");
}

void TestAConflictingBindingIsSTOREDAndREPORTED()
{
    // Refusing it would leave somebody stuck part-way through remapping: they
    // press a key, it is rejected, and the action they wanted to move it off still
    // holds it. Real remapping UIs let the clash exist and show it — which is also
    // the report-never-repair rule this project applies everywhere.
    FInputMap M;
    M.Bind("Dispatch", "Space", "operate");
    M.Bind("EStop", "Space", "operate");

    assert(M.Num() == 2);                      // both stored
    assert(M.KeyFor("EStop", "operate") == "Space");
    assert(M.HasConflicts());

    // And resolving it is the person moving the other one, which works.
    M.Bind("Dispatch", "Enter", "operate");
    assert(!M.HasConflicts());
    std::printf("  a clashing binding is stored and shown, so remapping can be done in any order\n");
}

} // namespace

int main()
{
    std::printf("Settings and input bindings\n\n");

    TestADEFAULTIsNotAValue();
    TestChangingADEFAULTReachesEverybodyWhoNeverTouchedIt();
    TestRESETIsADeletionNotAWrite();
    TestUNKNOWNKeysSURVIVEARoundTrip();
    TestAHandEditedFileSurvivesBeingMessy();

    TestREBINDINGReplacesRatherThanAdds();
    TestCONTEXTIsWhyOneKeyCanMeanTwoThings();
    TestAGLOBALBindingConflictsWithEVERYTHING();
    TestAConflictingBindingIsSTOREDAndREPORTED();

    std::printf("\ntest_settings: all assertions passed.\n");
    return 0;
}
