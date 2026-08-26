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
    // A vertical curve, from 2026-08-26: pitch change, tightest radius, and
    // whether it eases in, eases out or holds. The kind that lets a hill be
    // authored anywhere but a preset.
    Pitch,
};

// Every field the editor can show. Deliberately a flat list rather than one per
// kind, because a field means the same thing wherever it appears and the save
// format already stores it once.
enum class EEditField
{
    // KIND IS FIRST, because it decides what the rest of this list means. It
    // is also the field that was MISSING until 2026-08-21, which made the
    // runtime editor a tuning panel rather than an editor: a blank template
    // plus [I] gave straights and there was no way to author anything else.
    Kind,
    Length,
    Radius,
    CurvatureStart,
    CurvatureEnd,
    ClimbAngle,
    Turns,
    // THE VERTICAL CURVE'S TWO: how far the nose comes up or down, and its
    // shape (ease in / ease out / constant), which is picked, not typed. Its
    // radius is the Radius row every turn already has.
    PitchDelta,
    PitchEase,
    // ROLL IS A PAIR, and only the END of it was reachable. A bank authored
    // one segment at a time therefore started at 0 every time and STEPPED at
    // each joint -- which TrackValidate then correctly complained about, the
    // tool being honest about a hole it had no way to fill.
    RollStart,
    Roll,
    ZoneKind,
    ZoneSpeed,
    ZoneAccel,
    ZoneDecel,
    ZoneBrakeDecel,
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
    case EEditField::PitchDelta:      return "Pitch change";
    case EEditField::PitchEase:       return "Pitch shape";
    case EEditField::Kind:            return "Kind";
    case EEditField::RollStart:       return "Roll start";
    case EEditField::Roll:            return "Roll end";
    case EEditField::ZoneKind:        return "Device";
    case EEditField::ZoneSpeed:       return "Device speed";
    case EEditField::ZoneAccel:       return "Accel";
    case EEditField::ZoneDecel:       return "Decel";
    case EEditField::ZoneBrakeDecel:  return "Brake pad";
    case EEditField::StartsNewDevice: return "Starts a new device";
    case EEditField::Count:           break;
    }
    // Exhaustive rather than defaulted, for the reason written out in FirstRun.h:
    // a default handed three device fields the tick box's name and help, and did
    // it silently because that is what a default is for.
    return "";
}

// UNITS ARE ALWAYS SHOWN WHERE THERE IS ONE — `UI_CONVENTIONS.md`.
//
// `Turns` is the exception and it is a real one rather than an oversight: a count
// is DIMENSIONLESS, and "2.5 turns turns" is what putting a unit on it produces.
// So the rule is that every field with a DIMENSION carries its unit, which the
// first-run suite asserts by walking this list.
//
// Authored fields stay in the model's own metres and degrees and are NEVER converted on the way in, which is
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
    case EEditField::PitchDelta:
    case EEditField::RollStart:
    case EEditField::Roll:           return "deg";
    case EEditField::ZoneSpeed:      return "m/s";
    case EEditField::ZoneAccel:
    case EEditField::ZoneDecel:
    case EEditField::ZoneBrakeDecel: return "m/s2";
    // DIMENSIONLESS, EACH FOR ITS OWN REASON, and listed rather than defaulted so
    // that a new field with a dimension cannot inherit "no unit" by silence.
    case EEditField::Turns:            // a count
    case EEditField::Kind:             // a choice
    case EEditField::PitchEase:        // a choice
    case EEditField::ZoneKind:         // a choice
    case EEditField::StartsNewDevice:  // a tick box
    case EEditField::Count:          return "";
    }
    return "";
}

// ===================== A CHOICE IS NOT A NUMBER =====================
//
// Kind, device and the tick box are PICKED rather than typed, and that one fact
// decides three separate things: the row cycles on click instead of taking
// focus, it carries no unit, and it has no typical range to suggest.
//
// ANSWERED ONCE HERE BECAUSE IT WAS ANSWERED TWICE BEFORE. The panel kept its
// own list and the help suite kept another, both spelled out by hand, and they
// agreed only because the set had not changed since either was written. Adding
// Kind made them disagree immediately -- the suite asserted a dropdown ought to
// have a typical range, which is the shape of every drift this file is written
// against.
inline bool IsChoiceField(EEditField F)
{
    return F == EEditField::Kind
        || F == EEditField::PitchEase
        || F == EEditField::ZoneKind
        || F == EEditField::StartsNewDevice;
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
        // so offering a length field would be offering to overconstrain it. A
        // pitch curve likewise: pitch change and radius fix its length.
        return K != EEditKind::Helix && K != EEditKind::Pitch;
    case EEditField::Radius:
        return K == EEditKind::Arc || K == EEditKind::Helix || K == EEditKind::Pitch;
    case EEditField::CurvatureStart:
    case EEditField::CurvatureEnd:
        return K == EEditKind::Clothoid;
    case EEditField::ClimbAngle:
    case EEditField::Turns:
        return K == EEditKind::Helix;
    case EEditField::PitchDelta:
    case EEditField::PitchEase:
        return K == EEditKind::Pitch;
    case EEditField::Kind:
    case EEditField::RollStart:
    case EEditField::Roll:
    case EEditField::ZoneKind:
        return true;                       // every segment has a kind, a bank and a zone
    case EEditField::ZoneSpeed:
    case EEditField::ZoneAccel:
    case EEditField::ZoneDecel:
    case EEditField::ZoneBrakeDecel:
    case EEditField::StartsNewDevice:
        return true;                       // shown only when a device is set — below
    case EEditField::Count:
        break;
    }
    // ===================== NO `default:`, FOR THE THIRD TIME =====================
    //
    // A default here would have quietly answered "no" for Kind and RollStart,
    // and the two fields added to make the editor able to AUTHOR would have
    // been invisible with nothing failing. The same shape shipped in HelpFor
    // and gave three device fields the tick box's tooltip for weeks.
    return false;
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

    // KIND IS STORED ONCE, in `Kind`, and read through the SAME accessor as
    // every other field — so multi-select, the intersection and the
    // "differs" flag all work on it without a special case anywhere above.
    // A copy in `Value[]` would be a second source of truth for the one field
    // that decides what all the others mean.
    double Get(EEditField F) const
    {
        if (F == EEditField::Kind) { return static_cast<double>(Kind); }
        return Value[static_cast<std::size_t>(F)];
    }
    void Set(EEditField F, double V)
    {
        if (F == EEditField::Kind)
        {
            Kind = static_cast<EEditKind>(static_cast<int>(V));
            return;
        }
        Value[static_cast<std::size_t>(F)] = V;
    }
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
        // A DEVICE'S RATES ARE DEVICE FIELDS TOO. The pad is what makes a brake a
        // brake -- tyres are a setpoint driven toward from EITHER side, so a block
        // brake with no pad accelerates a train it was authored to slow.
        const bool bDeviceField = F == EEditField::ZoneSpeed
            || F == EEditField::ZoneAccel || F == EEditField::ZoneDecel
            || F == EEditField::ZoneBrakeDecel || F == EEditField::StartsNewDevice;
        if (bDeviceField && S.Zone == 0)
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
