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
            // video.resolution is the deliberate exception: its options come
            // from the monitor at runtime, so an empty default means "whatever
            // the display is already using" rather than a missing value.
            if (E.Options.empty())
            {
                assert(E.Default.empty() && "no options means no default to pick from");
                continue;
            }
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

} // namespace

int main()
{
    std::printf("SettingsSchema: the table a settings menu is generated from\n\n");

    TestEveryDefaultIsReachable();
    TestKeysAreUniqueAndLabelled();
    TestEveryPageIsPopulated();
    TestOwnershipIsCoherent();
    TestDeclaringLeavesNothingWritten();

    std::printf("\ntest_settingsschema: all assertions passed.\n");
    return 0;
}
