// Asserts for SegmentEditorModel.h — the segment editor, minus the widgets.
//
//   clang++ -std=c++17 -Wall -Wextra -O2 -o test_segmenteditormodel test_segmenteditormodel.cpp && ./test_segmenteditormodel

#include "SegmentEditorModel.h"

#include <cassert>
#include <cstdio>

namespace
{

FEditSegment Make(EEditKind K)
{
    FEditSegment S;
    S.Kind = K;
    S.Set(EEditField::Length, 20.0);
    S.Set(EEditField::Roll, 0.0);
    return S;
}

bool Visible(const FSegmentEditor& E, EEditField F)
{
    for (const FFieldView& V : E.Fields()) { if (V.Field == F) { return V.bVisible; } }
    return false;
}

double Shown(const FSegmentEditor& E, EEditField F)
{
    for (const FFieldView& V : E.Fields()) { if (V.Field == F) { return V.Value; } }
    return 0.0;
}

bool Differs(const FSegmentEditor& E, EEditField F)
{
    for (const FFieldView& V : E.Fields()) { if (V.Field == F) { return V.bDiffers; } }
    return false;
}

void TestEachKINDShowsItsOwnFields()
{
    // EditConditionHides, reimplemented. Pick Arc and you get Radius; pick
    // Clothoid and you get curvature endpoints.
    FSegmentEditor E;
    E.SetSegments({Make(EEditKind::Straight), Make(EEditKind::Arc),
                   Make(EEditKind::Clothoid), Make(EEditKind::Helix)});

    E.Select({0});
    assert(Visible(E, EEditField::Length) && !Visible(E, EEditField::Radius));
    assert(!Visible(E, EEditField::CurvatureStart));

    E.Select({1});
    assert(Visible(E, EEditField::Radius) && Visible(E, EEditField::Length));
    assert(!Visible(E, EEditField::Turns));

    E.Select({2});
    assert(Visible(E, EEditField::CurvatureStart) && Visible(E, EEditField::CurvatureEnd));
    assert(!Visible(E, EEditField::Radius));

    // A HELIX HAS NO LENGTH FIELD, because it is authored by radius, climb and
    // turns — its length is derived, and offering the field would be offering to
    // overconstrain it.
    E.Select({3});
    assert(Visible(E, EEditField::Radius) && Visible(E, EEditField::Turns));
    assert(!Visible(E, EEditField::Length));

    // Roll is on everything, because any segment can be banked.
    for (std::size_t i = 0; i < 4; ++i) { E.Select({i}); assert(Visible(E, EEditField::Roll)); }
    std::printf("  each kind shows its own fields, and a helix has no length field\n");
}

void TestHIDDENIsNOTDeleted()
{
    // THE ONE THAT ACTUALLY BITES. The naive implementation clears a value when
    // its field stops being relevant — and then somebody flips Arc to Straight to
    // look at something, flips back, and their radius is gone.
    //
    // Hidden is not deleted. The value was never the widget's to own.
    FSegmentEditor E;
    E.SetSegments({Make(EEditKind::Arc)});
    E.Select({0});
    E.SetValue(0, EEditField::Radius, 35.0);
    assert(Shown(E, EEditField::Radius) == 35.0);

    E.SetKind(0, EEditKind::Straight);
    assert(!Visible(E, EEditField::Radius));      // gone from the panel
    assert(E.At(0).Get(EEditField::Radius) == 35.0);   // and still in the data

    E.SetKind(0, EEditKind::Arc);
    assert(Visible(E, EEditField::Radius));
    assert(Shown(E, EEditField::Radius) == 35.0);      // exactly where they left it
    std::printf("  flipping Arc to Straight and back keeps the radius\n");
}

void TestADeviceSPEEDOnPlainTrackIsNotAField()
{
    // The one visibility rule that depends on a VALUE rather than on the kind,
    // and it is exactly what EditConditionHides did on bStartsNewDevice.
    FSegmentEditor E;
    E.SetSegments({Make(EEditKind::Straight)});
    E.Select({0});
    assert(Visible(E, EEditField::ZoneKind));
    assert(!Visible(E, EEditField::ZoneSpeed));
    assert(!Visible(E, EEditField::StartsNewDevice));

    E.At(0).Zone = 3;                              // it is a brake now
    assert(Visible(E, EEditField::ZoneSpeed));
    assert(Visible(E, EEditField::StartsNewDevice));
    std::printf("  device speed appears only once there is a device\n");
}

void TestMULTISELECTShowsTheIntersectionAndFlagsDifferences()
{
    // A field only ONE of the selected segments uses is not editable across the
    // selection — writing it would silently give an arc's radius to a straight.
    FSegmentEditor E;
    E.SetSegments({Make(EEditKind::Arc), Make(EEditKind::Straight), Make(EEditKind::Arc)});

    E.Select({0, 1});
    assert(Visible(E, EEditField::Length));        // both have it
    assert(!Visible(E, EEditField::Radius));       // only one does

    E.Select({0, 2});
    assert(Visible(E, EEditField::Radius));        // both arcs

    // AND A DIFFERING VALUE IS FLAGGED RATHER THAN SHOWING THE FIRST ONE. Showing
    // the first is a lie people then edit on top of — they see 35, adjust it to
    // 36, and silently overwrite the other segment's 12.
    E.SetValue(0, EEditField::Radius, 35.0);
    E.SetValue(2, EEditField::Radius, 12.0);
    assert(Differs(E, EEditField::Radius));
    assert(Shown(E, EEditField::Radius) == 0.0);   // no value is offered at all

    E.SetValue(2, EEditField::Radius, 35.0);
    assert(!Differs(E, EEditField::Radius));
    assert(Shown(E, EEditField::Radius) == 35.0);
    std::printf("  multi-select shows the intersection, and differing values show as neither\n");
}

void TestTheUNDOMergeKeyIsFieldANDSegment()
{
    // TrackHistory says explicitly that the key for "typing 30.5 is one undo
    // step" is the UI's to supply. Get it wrong one way and undoing a number
    // takes five presses; the other way and undo skips work somebody wanted back.
    FSegmentEditor E;
    E.SetSegments({Make(EEditKind::Arc), Make(EEditKind::Arc)});

    // Typing "30.5" into one field: the first keystroke is a new step, the rest
    // merge.
    assert(E.BeginEdit(0, EEditField::Radius));    // "3"
    assert(!E.BeginEdit(0, EEditField::Radius));   // "0"
    assert(!E.BeginEdit(0, EEditField::Radius));   // "."
    assert(!E.BeginEdit(0, EEditField::Radius));   // "5"

    // Tabbing to a different field is a new step.
    assert(E.BeginEdit(0, EEditField::Length));

    // And the SAME field on a DIFFERENT segment is a different thing entirely —
    // merging those would make undo jump between segments.
    assert(E.BeginEdit(1, EEditField::Length));
    assert(!E.BeginEdit(1, EEditField::Length));
    std::printf("  typing a number is one undo step; tabbing away starts another\n");
}

void TestANYTHINGThatIsNotTypingBREAKSTheRun()
{
    // Without this, typing a value, clicking away to check something, and coming
    // back to adjust it produces ONE undo step covering both — and undoing the
    // adjustment throws away the original too.
    FSegmentEditor E;
    E.SetSegments({Make(EEditKind::Arc), Make(EEditKind::Arc)});

    assert(E.BeginEdit(0, EEditField::Radius));
    assert(!E.BeginEdit(0, EEditField::Radius));

    E.BreakEditRun();                              // they clicked away
    assert(E.BeginEdit(0, EEditField::Radius));    // same field, new step

    // Structural edits break it for free, so no caller has to remember.
    assert(!E.BeginEdit(0, EEditField::Radius));
    E.Duplicate(0);
    assert(E.BeginEdit(0, EEditField::Radius));

    assert(!E.BeginEdit(0, EEditField::Radius));
    E.Remove(2);
    assert(E.BeginEdit(0, EEditField::Radius));
    std::printf("  clicking away, duplicating or deleting all start a fresh undo step\n");
}

void TestTheSELECTIONFollowsTheSegmentsItPointsAt()
{
    // A selection is INDICES, so inserting or removing above it moves what it
    // points at. Left unfixed, deleting segment 2 while segment 5 is selected
    // silently selects a different piece of track — and the next edit lands on
    // something nobody chose.
    FSegmentEditor E;
    E.SetSegments({Make(EEditKind::Straight), Make(EEditKind::Arc), Make(EEditKind::Straight),
                   Make(EEditKind::Arc), Make(EEditKind::Straight), Make(EEditKind::Arc)});
    E.Select({5});
    E.SetValue(5, EEditField::Radius, 99.0);

    E.Remove(2);
    assert(E.Selection().size() == 1);
    assert(E.Selection()[0] == 4);                          // followed it down
    assert(E.At(E.Selection()[0]).Get(EEditField::Radius) == 99.0);

    E.Insert(0, Make(EEditKind::Straight));
    assert(E.Selection()[0] == 5);                          // and back up
    assert(E.At(E.Selection()[0]).Get(EEditField::Radius) == 99.0);

    // DELETING THE SELECTED ONE DROPS IT rather than leaving the selection
    // pointing at whatever slid into its place.
    E.Select({5});
    E.Remove(5);
    assert(E.Selection().empty());
    std::printf("  the selection follows its segment, and is dropped when that segment is\n");
}

void TestDuplicatePutsTheCopyAFTERTheOriginal()
{
    // The list is in travel order, and a duplicate is nearly always the next
    // piece of track. Before it would make somebody drag every copy.
    FSegmentEditor E;
    E.SetSegments({Make(EEditKind::Arc), Make(EEditKind::Straight)});
    E.SetValue(0, EEditField::Radius, 35.0);
    E.Duplicate(0);

    assert(E.Num() == 3);
    assert(E.At(1).Kind == EEditKind::Arc);
    assert(E.At(1).Get(EEditField::Radius) == 35.0);
    assert(E.At(2).Kind == EEditKind::Straight);

    // And reordering works on the whole list.
    assert(E.Move(2, 0));
    assert(E.At(0).Kind == EEditKind::Straight);
    assert(!E.Move(0, 0));
    assert(!E.Move(9, 0));
    std::printf("  a duplicate lands after its original, in travel order\n");
}

// ===================== THE TWO FIELDS THAT COULD NOT BE REACHED =====================
//
// Until 2026-08-21 the runtime editor could change numbers on segments that
// already existed and nothing else, which is a TUNING PANEL rather than an
// editor. `EEditField` had no Kind entry, so a blank template plus [I] gave
// straights for ever and every curve on every shipped track came from a preset
// or from the developer-only Details panel. PROJECT_PLAN gives Phase 1 the gate
// "build an arbitrary coaster from scratch in-editor", and the SHIPPING path
// had never met it with the whole Phase 1 list ticked.
void TestKINDAndROLLSTARTAreEditableAtAll()
{
    // ---- KIND IS OFFERED ON EVERY KIND, or the way out of Straight depends on
    // already not being in it.
    const EEditKind All[] = {EEditKind::Straight, EEditKind::Arc,
                             EEditKind::Clothoid, EEditKind::Helix};
    for (EEditKind K : All)
    {
        FSegmentEditor E;
        E.SetSegments({Make(K)});
        E.Select({0});
        assert(Visible(E, EEditField::Kind));
        assert(Visible(E, EEditField::RollStart));
        assert(Visible(E, EEditField::Roll));
    }
    std::printf("  kind and both ends of the roll are offered on all four kinds\n");

    // ---- IT READS BACK AS THE KIND IT IS. Kind lives in `Kind` rather than in
    // `Value[]`, and is reached through the same Get/Set as everything else so
    // the intersection and the differs flag work on it -- so the two have to
    // agree, and this is what would catch them not agreeing.
    FSegmentEditor E;
    E.SetSegments({Make(EEditKind::Arc)});
    E.Select({0});
    assert(Shown(E, EEditField::Kind) == static_cast<double>(EEditKind::Arc));
    E.SetValue(0, EEditField::Kind, static_cast<double>(EEditKind::Helix));
    assert(E.At(0).Kind == EEditKind::Helix);
    assert(Shown(E, EEditField::Kind) == static_cast<double>(EEditKind::Helix));

    // ---- AND CHANGING KIND THROUGH THE FIELD KEEPS WHAT IS HIDDEN, which is the
    // one edit that would break `hidden is not deleted` if it were written as a
    // widget clearing what it stops showing. Helix has no radius offered; the
    // arc's radius must still be there on the way back.
    E.SetValue(0, EEditField::Kind, static_cast<double>(EEditKind::Arc));
    E.SetValue(0, EEditField::Radius, 42.0);
    E.SetValue(0, EEditField::Kind, static_cast<double>(EEditKind::Clothoid));
    assert(!Visible(E, EEditField::Radius));
    E.SetValue(0, EEditField::Kind, static_cast<double>(EEditKind::Arc));
    assert(Shown(E, EEditField::Radius) == 42.0);
    std::printf("  changing kind through the FIELD keeps the radius too\n");

    // ---- ROLL IS A PAIR, AND ONLY THE END OF IT WAS WRITABLE. That is what made
    // a hand-authored bank START at zero on every segment and STEP at each
    // joint. The two are independent values, which is the whole assertion.
    E.SetValue(0, EEditField::RollStart, 12.0);
    E.SetValue(0, EEditField::Roll, 30.0);
    assert(Shown(E, EEditField::RollStart) == 12.0);
    assert(Shown(E, EEditField::Roll) == 30.0);
    std::printf("  roll start and roll end are two values, not one\n");

    // ---- MULTI-SELECT FLAGS A DIFFERING KIND rather than showing the first
    // one's, which is what stops somebody flattening eight segments onto
    // whichever kind they happened to be looking at.
    FSegmentEditor M;
    M.SetSegments({Make(EEditKind::Arc), Make(EEditKind::Arc)});
    M.Select({0, 1});
    assert(!Differs(M, EEditField::Kind));
    M.SetValue(1, EEditField::Kind, static_cast<double>(EEditKind::Straight));
    assert(Differs(M, EEditField::Kind));
    std::printf("  a selection of two kinds reports differing rather than the first\n");
}

void TestUNITSAreCarriedByTheFieldNotTheWidget()
{
    // UI_CONVENTIONS: units are always shown, and authored fields stay in the
    // model's own metres and degrees, NEVER converted on the way in. That is the
    // same rule the save format follows, and it is what makes an imperial display
    // a display-layer change later rather than a data migration.
    assert(std::string(FieldUnit(EEditField::Length)) == "m");
    assert(std::string(FieldUnit(EEditField::Roll)) == "deg");
    assert(std::string(FieldUnit(EEditField::CurvatureStart)) == "1/m");
    assert(std::string(FieldUnit(EEditField::ZoneSpeed)) == "m/s");
    assert(std::string(FieldUnit(EEditField::StartsNewDevice)).empty());

    // Every field has a name, so no widget invents one.
    for (std::size_t f = 0; f < static_cast<std::size_t>(EEditField::Count); ++f)
    {
        assert(std::string(FieldName(static_cast<EEditField>(f))).size() > 0);
    }
    std::printf("  every field carries its own name and unit, in the model's own units\n");
}

} // namespace

int main()
{
    std::printf("The segment editor, minus the widgets\n\n");

    TestEachKINDShowsItsOwnFields();
    TestHIDDENIsNOTDeleted();
    TestADeviceSPEEDOnPlainTrackIsNotAField();
    TestMULTISELECTShowsTheIntersectionAndFlagsDifferences();
    TestTheUNDOMergeKeyIsFieldANDSegment();
    TestANYTHINGThatIsNotTypingBREAKSTheRun();
    TestTheSELECTIONFollowsTheSegmentsItPointsAt();
    TestDuplicatePutsTheCopyAFTERTheOriginal();
    TestKINDAndROLLSTARTAreEditableAtAll();
    TestUNITSAreCarriedByTheFieldNotTheWidget();

    std::printf("\ntest_segmenteditormodel: all assertions passed.\n");
    return 0;
}
