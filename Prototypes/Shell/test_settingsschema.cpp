// Asserts for SettingsSchema.h — the table a settings menu is generated from.
//
//   clang++ -std=c++17 -Wall -Wextra -o test_settingsschema test_settingsschema.cpp
//
// The point of a generated menu is that the table and the screen cannot drift.
// These are the checks that make the table itself trustworthy, because once the
// screen is generated the table is the only thing left to get wrong.

#include "SettingsSchema.h"
#include "Settings.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <string>

namespace
{

// EVERY DEFAULT MUST SATISFY ITS OWN CONSTRAINT, and this is the assertion worth
// having above all the others: a scalar whose default is outside its own slider,
// or a choice whose default is not one of its options, is a control that cannot
// display the value it starts on. The symptom is a slider pinned at one end or a
// dropdown showing blank, and neither reads as "the table is wrong".
void TestEveryDefaultIsReachable()
{
    std::printf("Every default satisfies its own constraint\n");
    int Scalars = 0, Choices = 0;

    for (const FSettingEntry& E : SettingsSchema())
    {
        if (E.Kind == ESettingKind::Scalar)
        {
            ++Scalars;
            const double V = std::atof(E.Default.c_str());
            assert(!E.Default.empty() && "a scalar with no default has nothing to show");
            assert(E.Min < E.Max && "a scalar whose range is empty cannot be dragged");
            assert(E.Step > 0.0);
            if (!(V >= E.Min && V <= E.Max))
            {
                std::printf("  BAD %s default %g outside [%g, %g]\n",
                            E.Key.c_str(), V, E.Min, E.Max);
            }
            assert(V >= E.Min && V <= E.Max);
        }
        else if (E.Kind == ESettingKind::Choice)
        {
            ++Choices;
            // NO EXCEPTIONS LEFT. This carried one for video.resolution, whose
            // options came from the monitor at runtime. Dropping that setting
            // makes the rule absolute: every choice offers its own default.
            assert(!E.Options.empty() && "a choice with no options is a dead control");
            bool bFound = false;
            for (const std::string& O : E.Options) { bFound = bFound || O == E.Default; }
            if (!bFound)
            {
                std::printf("  BAD %s default '%s' is not one of its options\n",
                            E.Key.c_str(), E.Default.c_str());
            }
            assert(bFound);
        }
        else if (E.Kind == ESettingKind::Bool)
        {
            assert((E.Default == "true" || E.Default == "false")
                   && "a bool default must read back as a bool");
        }
    }
    std::printf("  %d scalars and %d choices, every default reachable\n", Scalars, Choices);
}

// A KEY IS A PROMISE TO A CONFIG FILE. Two entries sharing one means whichever
// is written last silently wins, and the loser is a control that does nothing.
void TestKeysAreUniqueAndLabelled()
{
    std::printf("Keys are unique, and everything has a label\n");
    std::set<std::string> Seen;
    for (const FSettingEntry& E : SettingsSchema())
    {
        assert(!E.Key.empty());
        assert(!E.Label.empty() && "an unlabelled setting is a control nobody can identify");
        if (Seen.count(E.Key))
        {
            std::printf("  DUPLICATE %s\n", E.Key.c_str());
        }
        assert(Seen.insert(E.Key).second);
    }
    std::printf("  %zu settings, no collisions\n", Seen.size());
}

// EVERY PAGE HAS SOMETHING ON IT. A tab that opens onto nothing is a promise the
// application does not keep, and it happens the moment somebody adds a page
// enumerator before the settings that justify it.
void TestEveryPageIsPopulated()
{
    std::printf("Every page has entries\n");
    std::map<int, int> Count;
    for (const FSettingEntry& E : SettingsSchema()) { ++Count[static_cast<int>(E.Page)]; }

    const ESettingPage Pages[] = {ESettingPage::Video, ESettingPage::Graphics,
                                  ESettingPage::Audio, ESettingPage::Controls,
                                  ESettingPage::Simulation};
    for (ESettingPage P : Pages)
    {
        const int N = Count[static_cast<int>(P)];
        std::printf("  %-11s %d\n", SettingPageName(P), N);
        assert(N > 0);
    }
}

// AN ENGINE-OWNED SETTING MUST NAME ITS PROPERTY, or applying it is a guess made
// from the key — which is the coupling this table exists to remove.
void TestOwnershipIsCoherent()
{
    std::printf("Ownership is coherent\n");
    for (const FSettingEntry& E : SettingsSchema())
    {
        if (E.Owner == ESettingOwner::Engine)
        {
            assert(!E.Engine.empty() && "an engine setting with no property is unappliable");
        }
        else
        {
            assert(E.Engine.empty() && "only engine settings name an engine property");
        }
        // A key binding belongs to the input map and nowhere else: stored in
        // FSettings as well, it would have two homes and drift between them.
        if (E.Kind == ESettingKind::Key)
        {
            assert(E.Owner == ESettingOwner::Input);
        }
    }
    std::printf("  engine entries all name a property; bindings all belong to the input map\n");
}

// AND THE WHOLE POINT: declaring the schema into a store leaves every setting
// readable and the store still EMPTY of explicit values. That is "a default is
// not a value" holding across a schema rather than one key at a time — the file
// stays empty until somebody actually changes something.
void TestDeclaringLeavesNothingWritten()
{
    std::printf("Declaring the schema writes nothing\n");
    FSettings S;
    DeclareSchemaDefaults(S);

    for (const FSettingEntry& E : SettingsSchema())
    {
        assert(S.Get(E.Key) == E.Default);
    }

    // NOT "the file is empty" — Save() writes a header comment explaining what
    // the file is, which is worth having and would make that assertion lie. The
    // claim is that no KEY is in it.
    const std::string Text = S.Save();
    for (const FSettingEntry& E : SettingsSchema())
    {
        if (Text.find(E.Key) != std::string::npos)
        {
            std::printf("  WRITTEN WITHOUT BEING SET: %s\n", E.Key.c_str());
        }
        assert(Text.find(E.Key) == std::string::npos);
    }
    std::printf("  %zu settings declared, none written\n", SettingsSchema().size());

    // Change one, and exactly one appears.
    S.Set("audio.master", "0.5");
    const std::string After = S.Save();
    assert(After.find("audio.master") != std::string::npos);
    assert(After.find("audio.ride") == std::string::npos);
    std::printf("  after changing one: it is written, and its neighbours are not\n");
}

// RESETTING A PAGE IS A DELETION, and this is the assertion the settings screen
// turned out to need. Its first version reset a row by writing the default's TEXT
// back through the same path an edit takes — which freezes today's default into
// somebody's file and undoes "a default is not a value" one setting at a time,
// silently, for the one person who pressed the button.
//
// The store has always known how to do this properly. The screen has to ask for
// it, and the difference is invisible until the default changes a year later.
void TestResettingAPageIsADeletion()
{
    std::printf("Resetting a page deletes rather than writes\n");
    FSettings S;
    DeclareSchemaDefaults(S);

    for (const FSettingEntry& E : SettingsSchema())
    {
        if (E.Page == ESettingPage::Audio && E.Kind == ESettingKind::Scalar)
        {
            S.Set(E.Key, "0.25");
        }
    }
    assert(S.NumExplicit() > 0);

    for (const FSettingEntry& E : SettingsSchema())
    {
        if (E.Page == ESettingPage::Audio) { S.Reset(E.Key); }
    }
    assert(S.NumExplicit() == 0 && "a reset page must leave nothing written");

    const std::string Text = S.Save();
    for (const FSettingEntry& E : SettingsSchema())
    {
        if (E.Page == ESettingPage::Audio)
        {
            assert(Text.find(E.Key) == std::string::npos);
        }
    }

    // AND THE POINT OF THE DELETION: a better default reaches somebody who has
    // reset. Writing the text back would have pinned them to the old one for ever,
    // which is precisely the state they were trying to leave.
    S.Declare("audio.master", "0.6");
    assert(S.Get("audio.master") == "0.6");
    std::printf("  page reset writes nothing, and picks up a later default\n");
}

// SEEDING THE INPUT MAP FROM THIS TABLE MUST NOT PRODUCE A CONFLICT. Two rows
// naming one key give a page that shows the same keystroke against two actions,
// and the application binds whichever comes last — the exact ambiguity that has
// already bitten this project twice, once with [Backspace] against the E-stop and
// once with the editor's own function keys.
//
// FInputMap already reports conflicts and nothing was holding the table to it.
void TestBindingsDoNotCollide()
{
    std::printf("No two actions claim one key\n");
    FInputMap Map;
    int Bound = 0;
    for (const FSettingEntry& E : SettingsSchema())
    {
        if (E.Kind != ESettingKind::Key) { continue; }
        assert(!E.Default.empty() && "a binding row with no key is a blank control");
        // One context, because every binding this shell has is global today. That
        // is what makes a duplicate a genuine collision rather than two contexts
        // legitimately reusing a key.
        Map.Bind(E.Key, E.Default);
        ++Bound;
    }
    for (const FBindingConflict& C : Map.Conflicts())
    {
        std::printf("  COLLISION on [%s]: %s and %s\n",
                    C.Key.c_str(), C.ActionA.c_str(), C.ActionB.c_str());
    }
    assert(!Map.HasConflicts());
    std::printf("  %d bindings, no two on one key\n", Bound);
}

} // namespace

int main()
{
    std::printf("SettingsSchema: the table a settings menu is generated from\n\n");

    TestEveryDefaultIsReachable();
    TestKeysAreUniqueAndLabelled();
    TestEveryPageIsPopulated();
    TestOwnershipIsCoherent();
    TestDeclaringLeavesNothingWritten();
    TestResettingAPageIsADeletion();
    TestBindingsDoNotCollide();

    std::printf("\ntest_settingsschema: all assertions passed.\n");
    return 0;
}
