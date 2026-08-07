// TrackUnlimited Phase 3.5: the segment editor, minus the widgets.
// Plain C++17, no engine dependency.
//
// The largest single piece of UI work in the project, and the one that turns
// TrackUnlimited from an Unreal project into a program. The Details panel gave
// three things for free that a shipping editor has to rebuild, and two of them
// are logic rather than layout — so they are here, where they can be wrong in a
// way a test notices.
//
// ===================== 1. PER-KIND FIELD VISIBILITY =====================
//
// Pick Arc and you get Radius; pick Clothoid and you get curvature endpoints.
// That is `EditConditionHides`, reimplemented.
//
// THE PART THAT IS EASY TO GET WRONG IS NOT WHICH FIELDS SHOW. It is what happens
// to the value behind a field that stops showing. The naive implementation clears
// it — and then somebody flips Arc to Straight to look at something, flips back,
// and their radius is gone. Hidden is not deleted, and this is where that gets
// decided once instead of per widget.
//
// ===================== 2. THE UNDO MERGE KEY =====================
//
// `TrackHistory.h` is snapshot-based and UI-agnostic, and it says explicitly that
// the merge key for "typing 30.5 is one undo step" is the UI's to supply. This is
// the UI supplying it.
//
// Get it wrong in one direction and every keystroke is an undo step, so undoing a
// number takes five presses. Get it wrong in the other and an edit to a different
// field merges into the previous one, so undo skips work the person wanted back.
//
// ===================== 3. MULTI-SELECT =====================
//
// Which is mostly about what to show when the values DIFFER. Showing the first
// one is a lie that people edit on top of.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

// The authored vocabulary, as the editor sees it.
enum class EEditKind
{
    Straight,
    Arc,
    Clothoid,
    Helix,
};

// Every field the editor can show. Deliberately a flat list rather than one per
// kind, because a field means the same thing wherever it appears and the save
// format already stores it once.
enum class EEditField
{
    Length,
    Radius,
    CurvatureStart,
    CurvatureEnd,
    ClimbAngle,
    Turns,
    Roll,
    ZoneKind,
    ZoneSpeed,
    StartsNewDevice,
    Count
};

inline const char* FieldName(EEditField F)
{
    switch (F)
    {
    case EEditField::Length:          return "Length";
    case EEditField::Radius:          return "Radius";
    case EEditField::CurvatureStart:  return "Curvature start";
    case EEditField::CurvatureEnd:    return "Curvature end";
    case EEditField::ClimbAngle:      return "Climb angle";
    case EEditField::Turns:           return "Turns";
    case EEditField::Roll:            return "Roll";
    case EEditField::ZoneKind:        return "Device";
    case EEditField::ZoneSpeed:       return "Device speed";
    default:                          return "Starts a new device";
    }
}

// UNITS ARE ALWAYS SHOWN — `UI_CONVENTIONS.md`. Authored fields stay in the
// model's own metres and degrees and are NEVER converted on the way in, which is
// the same rule the save format follows and what makes an imperial display a
// display-layer change later.
inline const char* FieldUnit(EEditField F)
{
    switch (F)
    {
    case EEditField::Length:
    case EEditField::Radius:         return "m";
    case EEditField::CurvatureStart:
    case EEditField::CurvatureEnd:   return "1/m";
    case EEditField::ClimbAngle:
    case EEditField::Roll:           return "deg";
    case EEditField::ZoneSpeed:      return "m/s";
    default:                         return "";
    }
}

// ===================== WHICH FIELDS A KIND USES =====================
//
// The one place this is decided. A widget that asked the question itself would
// be a second answer, and the two would drift the first time a kind gained a
// field.
inline bool KindUsesField(EEditKind K, EEditField F)
{
    switch (F)
    {
    case EEditField::Length:
        // A helix is authored by radius, climb and TURNS — its length is derived,
        // so offering a length field would be offering to overconstrain it.
        return K != EEditKind::Helix;
    case EEditField::Radius:
        return K == EEditKind::Arc || K == EEditKind::Helix;
    case EEditField::CurvatureStart:
    case EEditField::CurvatureEnd:
        return K == EEditKind::Clothoid;
    case EEditField::ClimbAngle:
    case EEditField::Turns:
        return K == EEditKind::Helix;
    case EEditField::Roll:
    case EEditField::ZoneKind:
        return true;                       // every segment can be banked and zoned
    case EEditField::ZoneSpeed:
    case EEditField::StartsNewDevice:
        return true;                       // shown only when a device is set — below
    default:
        return false;
    }
}

// One segment's authored values, as the editor holds them.
//
// EVERY FIELD IS STORED, WHATEVER THE KIND. That is the whole point: a radius
// typed on an arc survives being flipped to a straight and back, because the
// value was never the widget's to own.
struct FEditSegment
{
    EEditKind Kind = EEditKind::Straight;
    double Value[static_cast<std::size_t>(EEditField::Count)] = {};
    int Zone = 0;                          // 0 = plain track

    double Get(EEditField F) const { return Value[static_cast<std::size_t>(F)]; }
    void Set(EEditField F, double V) { Value[static_cast<std::size_t>(F)] = V; }
};

// ===================== WHAT TO SHOW =====================

// A field's state in the panel right now.
struct FFieldView
{
    EEditField Field = EEditField::Length;
    bool bVisible = false;
    bool bDiffers = false;       // multi-select, and they are not all the same
    double Value = 0.0;          // meaningless when bDiffers
};

class FSegmentEditor
{
public:
    void SetSegments(const std::vector<FEditSegment>& In) { Seg = In; }
    std::size_t Num() const { return Seg.size(); }
    const FEditSegment& At(std::size_t i) const { return Seg[i]; }
    FEditSegment& At(std::size_t i) { return Seg[i]; }

    void Select(const std::vector<std::size_t>& Indices) { Sel = Indices; }
    const std::vector<std::size_t>& Selection() const { return Sel; }

    // What the panel draws. One entry per field, in a stable order, with
    // visibility resolved and multi-select differences flagged.
    std::vector<FFieldView> Fields() const
    {
        std::vector<FFieldView> Out;
        for (std::size_t f = 0; f < static_cast<std::size_t>(EEditField::Count); ++f)
        {
            const EEditField F = static_cast<EEditField>(f);
            FFieldView V;
            V.Field = F;
            if (Sel.empty()) { Out.push_back(V); continue; }

            // MULTI-SELECT SHOWS THE INTERSECTION. A field only one of the
            // selected segments uses is not editable across the selection,
            // because writing it would silently give an arc's radius to a
            // straight.
            bool bAll = true;
            bool bFirst = true;
            double Common = 0.0;
            for (std::size_t i : Sel)
            {
                if (i >= Seg.size()) { continue; }
                if (!VisibleOn(Seg[i], F)) { bAll = false; break; }
                const double This = Seg[i].Get(F);
                if (bFirst) { Common = This; bFirst = false; }
                else if (This != Common) { V.bDiffers = true; }
            }
            V.bVisible = bAll && !bFirst;
            V.Value = V.bDiffers ? 0.0 : Common;
            Out.push_back(V);
        }
        return Out;
    }

    // ===================== HIDDEN IS NOT DELETED =====================
    //
    // Changing a kind changes what is SHOWN and nothing else. Somebody flipping
    // Arc to Straight to look at something and back must find their radius
    // exactly where they left it — and the naive implementation, which clears the
    // value when the field stops being relevant, loses it silently.
    void SetKind(std::size_t Index, EEditKind K)
    {
        if (Index < Seg.size()) { Seg[Index].Kind = K; }
    }

    // ===================== THE UNDO MERGE KEY =====================
    //
    // TrackHistory is snapshot-based and says explicitly that the key for
    // "typing 30.5 is one undo step" is the UI's to supply. This supplies it.
    //
    // Two edits merge when they are the same FIELD on the same SEGMENT with
    // nothing in between. Get it wrong one way and undoing a number takes five
    // presses; get it wrong the other and undo skips work somebody wanted back.
    struct FEditKey
    {
        std::size_t Segment = 0;
        EEditField Field = EEditField::Length;
        bool bValid = false;

        bool SameAs(const FEditKey& O) const
        {
            return bValid && O.bValid && Segment == O.Segment && Field == O.Field;
        }
    };

    // Returns true when this edit should START A NEW undo step, false when it
    // should merge into the previous one.
    bool BeginEdit(std::size_t Index, EEditField F)
    {
        FEditKey K;
        K.Segment = Index;
        K.Field = F;
        K.bValid = true;
        const bool bNew = !K.SameAs(Last);
        Last = K;
        return bNew;
    }

    // ANYTHING THAT IS NOT TYPING BREAKS THE RUN. Selecting a different segment,
    // undoing, saving, adding or removing a segment — after any of those, the
    // next keystroke is a fresh undo step even if it lands on the same field.
    //
    // Without this, typing a value, clicking away to check something, and coming
    // back to adjust it produces ONE undo step covering both — and undoing the
    // adjustment throws away the original too.
    void BreakEditRun() { Last = FEditKey(); }

    // ---- Editing, which is where a run gets broken for free.

    void SetValue(std::size_t Index, EEditField F, double V)
    {
        if (Index < Seg.size()) { Seg[Index].Set(F, V); }
    }

    void Insert(std::size_t At, const FEditSegment& S)
    {
        if (At > Seg.size()) { At = Seg.size(); }
        Seg.insert(Seg.begin() + static_cast<long>(At), S);
        Reindex(At, +1);
        BreakEditRun();
    }

    void Remove(std::size_t At)
    {
        if (At >= Seg.size()) { return; }
        Seg.erase(Seg.begin() + static_cast<long>(At));
        Reindex(At, -1);
        BreakEditRun();
    }

    // Duplicating puts the copy AFTER the original, because the list is in travel
    // order and a duplicate is nearly always the next piece of track.
    void Duplicate(std::size_t At)
    {
        if (At < Seg.size()) { Insert(At + 1, Seg[At]); }
    }

    bool Move(std::size_t From, std::size_t To)
    {
        if (From >= Seg.size() || To >= Seg.size() || From == To) { return false; }
        const FEditSegment S = Seg[From];
        Seg.erase(Seg.begin() + static_cast<long>(From));
        Seg.insert(Seg.begin() + static_cast<long>(To), S);
        BreakEditRun();
        return true;
    }

private:
    // A DEVICE SPEED ON PLAIN TRACK IS NOT A FIELD, which is the one visibility
    // rule that depends on a value rather than on the kind — and it is exactly
    // what `EditConditionHides` did on `bStartsNewDevice`.
    static bool VisibleOn(const FEditSegment& S, EEditField F)
    {
        if (!KindUsesField(S.Kind, F)) { return false; }
        if ((F == EEditField::ZoneSpeed || F == EEditField::StartsNewDevice) && S.Zone == 0)
        {
            return false;
        }
        return true;
    }

    // A SELECTION IS INDICES, so inserting or removing above it moves what it
    // points at. Left unfixed, deleting segment 2 while segment 5 is selected
    // silently selects a different piece of track — and the next edit lands on
    // something nobody chose.
    void Reindex(std::size_t At, int Delta)
    {
        std::vector<std::size_t> Next;
        for (std::size_t I : Sel)
        {
            if (Delta < 0 && I == At) { continue; }        // the one that went
            Next.push_back(I >= At ? static_cast<std::size_t>(static_cast<long>(I) + Delta) : I);
        }
        Sel = Next;
    }

    std::vector<FEditSegment> Seg;
    std::vector<std::size_t> Sel;
    FEditKey Last;
};

// ponytail: no copy/paste buffer and no field-level validation. Paste is the
// clipboard's problem and validation already exists in TrackValidate, which the
// diagnostics panel surfaces — putting a second copy behind each field would be
// two answers to one question.
