// Asserts for FirstRun.h — the first five minutes.
//
//   clang++ -std=c++17 -Wall -Wextra -O2 -o test_firstrun test_firstrun.cpp && ./test_firstrun
//
// This is mostly text, and the assertions are about COVERAGE and TONE rather than
// arithmetic. That is deliberate: the failure mode for on-ramp content is not
// being wrong, it is being absent — a field with no tooltip, a panel with no
// empty state — and absence is exactly what a walk over the real lists catches.

#include "FirstRun.h"

#include <cassert>
#include <cstdio>
#include <string>

namespace
{

void TestEVERYFieldHasHelpAndAUnit()
{
    // THE CHECK THAT IS THE POINT. A hand-kept list of help text drifts the first
    // time somebody adds a field, silently, and the drift lands on a beginner.
    //
    // So this walks the EDITOR'S OWN FIELD LIST rather than a copy — adding a
    // field without help fails the build. Same rule as the process image
    // describing itself and the reference figures being reproducible.
    for (std::size_t f = 0; f < static_cast<std::size_t>(EEditField::Count); ++f)
    {
        const EEditField F = static_cast<EEditField>(f);
        const FFieldHelp H = HelpFor(F);

        assert(std::string(H.Tooltip).size() > 30);          // a sentence, not a label
        assert(std::string(FieldName(F)).size() > 0);

        // A numeric field has a typical range; a tick box and a dropdown do not,
        // and pretending otherwise would put "0 to 0" under a checkbox.
        const bool bNumeric = F != EEditField::ZoneKind && F != EEditField::StartsNewDevice;
        assert(H.bHasRange == bNumeric);
        if (H.bHasRange) { assert(H.TypicalMax > H.TypicalMin); }

        // UNITS ARE ALWAYS SHOWN WHERE THERE IS ONE — UI_CONVENTIONS. Every field
        // with a DIMENSION carries its unit; `Turns` is a count and genuinely has
        // none, which is a distinction rather than a gap: "2.5 turns turns" is
        // what putting a unit on it produces.
        const bool bDimensionless = F == EEditField::Turns || !bNumeric;
        assert(bDimensionless == std::string(FieldUnit(F)).empty());
    }
    std::printf("  all %zu editor fields carry a tooltip, and every numeric one a range and a unit\n",
                static_cast<std::size_t>(EEditField::Count));
}

void TestTheRADIUSTooltipSaysWhichWayRoundItGoes()
{
    // The single most confusing field for a beginner, because the intuition is
    // backwards: a BIGGER number is a GENTLER turn. Somebody who thinks bigger
    // means tighter types 5 and gets a hairpin the validator refuses.
    const std::string T = HelpFor(EEditField::Radius).Tooltip;
    assert(T.find("SMALLER IS TIGHTER") != std::string::npos);

    // And curvature explains itself in terms of radius, because that is the one
    // people already have a feel for.
    const std::string C = HelpFor(EEditField::CurvatureStart).Tooltip;
    assert(C.find("radius") != std::string::npos);
    assert(C.find("0 is dead straight") != std::string::npos);
    std::printf("  the radius tooltip says SMALLER IS TIGHTER, because the intuition is backwards\n");
}

void TestTheBRAKESpeedTooltipSaysWhatTheNumberACTUALLYMeans()
{
    // A brake's authored number is the speed it RELEASES at, not the speed it
    // stops at — brakes rest closed. Somebody who assumes otherwise types 0 and
    // wonders why the train never leaves.
    const std::string T = HelpFor(EEditField::ZoneSpeed).Tooltip;
    assert(T.find("RELEASES") != std::string::npos);
    assert(T.find("rest closed") != std::string::npos);
    std::printf("  the device speed tooltip says a brake's number is its RELEASE speed\n");
}

void TestEVERYPanelHasAnEmptyState()
{
    // A blank panel is a panel somebody assumes is broken.
    for (std::size_t p = 0; p < static_cast<std::size_t>(EPanelKind::Count); ++p)
    {
        const std::string S = EmptyStateFor(static_cast<EPanelKind>(p));
        assert(S.size() > 40);

        // AND IT SAYS WHAT TO DO, not just what is absent. "No data" is a status;
        // "add a straight to begin" is a next step.
        assert(S.find("appear") != std::string::npos
            || S.find("Add") != std::string::npos
            || S.find("Start") != std::string::npos
            || S.find("running") != std::string::npos);
    }
    std::printf("  all %zu panels have an empty state, and each says what to do next\n",
                static_cast<std::size_t>(EPanelKind::Count));
}

void TestTheProgramPanelEmptyStateDoesNOTReadAsSomethingMissing()
{
    // THE ALIENATION RISK, EXACTLY WHERE IT LANDS. The control layer is a LAYER OF
    // CHOICE — nobody should ever have to open it to have a working, exciting
    // ride. An empty program panel saying "no program loaded" would read as
    // something broken or something owed, which is the opposite of true.
    const std::string S = EmptyStateFor(EPanelKind::ControlProgram);
    assert(S.find("never have to") != std::string::npos);
    assert(S.find("running") != std::string::npos);
    assert(S.find("one block") != std::string::npos);      // and the override is per block

    // It does not say "no", "none", "empty" or "missing" anywhere.
    for (const char* Bad : {"No program", "none", "empty", "missing"})
    {
        assert(S.find(Bad) == std::string::npos);
    }
    std::printf("  the program panel's empty state reads as working, not as owed\n");
}

void TestTheDragAnswerDoesNotBeginBySayingNo()
{
    // People arriving from Planet Coaster will try to drag the track and need a
    // straight answer. "Constraint 1" is not it, and neither is a refusal.
    const std::string A = WhyCannotIDragTheTrack();

    // It leads with what the view IS rather than what it is not.
    assert(A.find("picture of your track") != std::string::npos);

    // It concedes the obvious objection rather than pretending it away.
    assert(A.find("sounds like more work") != std::string::npos);

    // It gives the reason a rider would care about, not an architectural one.
    assert(A.find("rider cannot feel the join") != std::string::npos);
    assert(A.find("G-force") != std::string::npos);

    // No jargon in the first sentence. A clothoid can wait.
    const std::string First = A.substr(0, A.find('\n'));
    for (const char* Jargon : {"clothoid", "curvature", "parametric", "C2", "spline"})
    {
        assert(First.find(Jargon) == std::string::npos);
    }

    assert(std::string(ViewportHint()).find("Why?") != std::string::npos);
    std::printf("  the drag answer leads with what the view IS, and concedes the objection\n");
}

void TestNEWTrackIsNeverAnEmptyList()
{
    // The first edit should be CHANGING A NUMBER ON SOMETHING THAT ALREADY RUNS,
    // not authoring geometry from nothing — a completely different and much
    // harder first task.
    assert(NumTemplates() >= 3);

    std::size_t Blank = 0;
    for (std::size_t i = 0; i < NumTemplates(); ++i)
    {
        const FTemplate T = TemplateAt(i);
        assert(std::string(T.Name).size() > 0);
        assert(std::string(T.Description).size() > 30);

        // EVERY TEMPLATE SAYS WHAT TO TRY FIRST, and it is a concrete edit with a
        // consequence you can watch — not "explore the editor".
        assert(std::string(T.WhatToTryFirst).size() > 30);
        if (std::string(T.Name) == "Blank") { ++Blank; }
    }

    // A TEMPLATE NAMES A PRESET RATHER THAN CARRYING ITS OWN GEOMETRY. Five
    // worked examples already ship, every one MEASURED before it went in — a
    // parallel set of starter layouts would be a second set of tracks to keep
    // working, drifting from the ones the docs quote.
    assert(TemplateAt(0).Preset == ETemplatePreset::Reference);
    assert(TemplateAt(1).Preset == ETemplatePreset::TwoTrainCircuit);
    assert(TemplateAt(2).Preset == ETemplatePreset::OutAndBack);
    assert(TemplateAt(3).Preset == ETemplatePreset::Showcase);
    // BLANK IS LAST, and that is the assertion rather than "blank is at 4" — the
    // showcase was inserted ahead of it and this line went on naming index 3,
    // which is how a positional check drifts. Somebody who wants an empty track
    // knows where to look; somebody who does not should not meet it first.
    assert(TemplateAt(NumTemplates() - 1).Preset == ETemplatePreset::Blank);

    // Exactly one blank option, and it is NOT the default.
    assert(Blank == 1);
    assert(std::string(TemplateAt(DefaultTemplate()).Name) != "Blank");
    assert(DefaultTemplate() < NumTemplates());
    std::printf("  %zu templates, one of them blank, and the default is not it\n", NumTemplates());
}

void TestTheFirstTryIsAlwaysAConcreteEditWithAVisibleConsequence()
{
    // "Explore the editor" is not guidance. Each line names a field to change and
    // a panel that will show the result — which is what makes the first minute a
    // loop rather than a stare.
    for (std::size_t i = 0; i < NumTemplates(); ++i)
    {
        const std::string W = TemplateAt(i).WhatToTryFirst;
        assert(W.find("Change") != std::string::npos || W.find("Make") != std::string::npos
            || W.find("Add") != std::string::npos);
    }

    // And the launched circuit's suggestion points at the profile panel by name,
    // because "it will stall" is only useful if you know where it says so.
    const std::string L = TemplateAt(1).WhatToTryFirst;
    assert(L.find("profile panel") != std::string::npos);
    assert(L.find("stalls") != std::string::npos);
    std::printf("  every template's first suggestion is a named field and a visible result\n");
}

} // namespace

int main()
{
    std::printf("The first five minutes\n\n");

    TestEVERYFieldHasHelpAndAUnit();
    TestTheRADIUSTooltipSaysWhichWayRoundItGoes();
    TestTheBRAKESpeedTooltipSaysWhatTheNumberACTUALLYMeans();
    TestEVERYPanelHasAnEmptyState();
    TestTheProgramPanelEmptyStateDoesNOTReadAsSomethingMissing();
    TestTheDragAnswerDoesNotBeginBySayingNo();
    TestNEWTrackIsNeverAnEmptyList();
    TestTheFirstTryIsAlwaysAConcreteEditWithAVisibleConsequence();

    std::printf("\ntest_firstrun: all assertions passed.\n");
    return 0;
}
