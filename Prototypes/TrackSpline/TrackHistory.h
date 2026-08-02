// TrackUnlimited Phase 1: undo/redo for the authored track document.
// Plain C++17, no engine dependency, same conventions as TrackSpline.h.
//
// Snapshots, not commands.
//
// The command pattern is the textbook answer and it is the wrong one here. It
// needs every edit to supply a do AND an inverse, and the failure mode when the
// two disagree is silent corruption discovered several undos later. A document
// is an ordered list of small POD structs — a 100-segment layout is a few
// kilobytes — so copying it whole costs nothing and cannot be asymmetric. Reach
// for commands the day a document is large enough for that to hurt, which is
// not a day this project is going to have.
//
// Equality goes through the SAVE FORMAT rather than a hand-written operator==,
// and that is deliberate on two counts. It is the correct semantics — two
// documents are the same exactly when they save the same — and it cannot rot:
// `Torsion` and `RollMode` were both added to this model after the fact, and a
// hand-written comparison would have silently ignored them and quietly dropped
// undo steps. It also means fields a kind does not use (a helix's climbAngle on
// an arc) correctly compare equal, because they are not part of the document's
// meaning.
//
// Units: whatever FTrackDocument holds. This file never looks inside a segment.

#pragma once

#include "TrackIO.h"

#include <cstddef>
#include <string>
#include <vector>

class FTrackHistory
{
public:
    // MaxDepth counts stored states, not edits, so the floor is 2: one to be in
    // and one to go back to.
    explicit FTrackHistory(const FTrackDocument& Initial, std::size_t InMaxDepth = 200)
        : MaxDepth(InMaxDepth < 2 ? 2 : InMaxDepth)
    {
        Entries.push_back(Make(Initial, "initial", std::string()));
        Index = 0;
        SavedIndex = 0;
    }

    // Record a new state. Returns false and does nothing if the document is
    // unchanged — an editor that commits on every field callback would
    // otherwise fill the stack with undo steps that undo nothing, and the user
    // presses Ctrl+Z five times before anything moves.
    //
    // MergeKey coalesces: consecutive commits carrying the same non-empty key
    // REPLACE each other rather than stacking. Typing "30.5" into a radius field
    // is five keystrokes and should be one undo step. The caller owns the key —
    // something like "seg3.radius", cleared when focus leaves — because only the
    // UI knows when an edit is finished. No timers, no guessing.
    bool Commit(const FTrackDocument& Next, const std::string& Label,
                const std::string& MergeKey = std::string())
    {
        std::string Canonical;
        std::string Error;
        if (!WriteTrackJson(Next, Canonical, Error))
        {
            // A document that cannot be written cannot be compared or restored,
            // so it does not go on the stack. The caller still holds it; this
            // just declines to promise it can be got back.
            return false;
        }
        if (Canonical == Entries[Index].Canonical)
        {
            return false;
        }

        const bool bMerge = !MergeKey.empty() && Index > 0
                            && Entries[Index].MergeKey == MergeKey;
        if (bMerge)
        {
            Entries.resize(Index + 1); // a merged edit still kills the redo branch
            Entries[Index].Doc = Next;
            Entries[Index].Canonical = Canonical;
            Entries[Index].Label = Label;
            // The state at Index is not the content that was saved any more,
            // even though the index did not move.
            if (SavedIndex == Index)
            {
                SavedIndex = NoSaved;
            }
            return true;
        }

        // Committing after an undo discards the redo branch. If the saved state
        // lived on that branch it is gone, and the document is dirty from here
        // on however far back you undo.
        if (SavedIndex != NoSaved && SavedIndex > Index)
        {
            SavedIndex = NoSaved;
        }
        Entries.resize(Index + 1);
        Entries.push_back(Make(Next, Label, MergeKey));
        Index = Entries.size() - 1;
        Trim();
        return true;
    }

    bool CanUndo() const { return Index > 0; }
    bool CanRedo() const { return Index + 1 < Entries.size(); }

    const FTrackDocument& Undo()
    {
        if (CanUndo())
        {
            --Index;
        }
        return Current();
    }

    const FTrackDocument& Redo()
    {
        if (CanRedo())
        {
            ++Index;
        }
        return Current();
    }

    const FTrackDocument& Current() const { return Entries[Index].Doc; }

    // What the menu item should say. The undo label describes the edit that
    // PRODUCED the current state, which is the one about to be reversed.
    const std::string& UndoLabel() const { return Entries[Index].Label; }
    const std::string& RedoLabel() const
    {
        static const std::string None;
        return CanRedo() ? Entries[Index + 1].Label : None;
    }

    void MarkSaved() { SavedIndex = Index; }

    // Undoing back to the state that was written to disk makes the document
    // clean again — it is byte-for-byte what is on disk, so claiming otherwise
    // would nag the user into a pointless save. This is why the saved position
    // is tracked as an index rather than a bool.
    bool IsDirty() const { return SavedIndex != Index; }

    std::size_t Depth() const { return Entries.size(); }
    std::size_t Position() const { return Index; }

private:
    struct FEntry
    {
        FTrackDocument Doc;
        std::string Canonical; // the save format, used as the identity of a state
        std::string Label;
        std::string MergeKey;
    };

    static FEntry Make(const FTrackDocument& Doc, const std::string& Label,
                       const std::string& MergeKey)
    {
        FEntry E;
        E.Doc = Doc;
        E.Label = Label;
        E.MergeKey = MergeKey;
        std::string Error;
        WriteTrackJson(Doc, E.Canonical, Error);
        return E;
    }

    void Trim()
    {
        if (Entries.size() <= MaxDepth)
        {
            return;
        }
        const std::size_t Drop = Entries.size() - MaxDepth;
        Entries.erase(Entries.begin(), Entries.begin() + static_cast<std::ptrdiff_t>(Drop));
        Index = Index >= Drop ? Index - Drop : 0;
        // If the saved state fell off the bottom, the document can never be
        // proven clean again. Saying "unsaved" forever is the safe direction;
        // the alternative is telling someone their work is safe when the
        // evidence for that has been discarded.
        if (SavedIndex == NoSaved || SavedIndex < Drop)
        {
            SavedIndex = NoSaved;
        }
        else
        {
            SavedIndex -= Drop;
        }
    }

    static const std::size_t NoSaved = static_cast<std::size_t>(-1);

    std::vector<FEntry> Entries;
    std::size_t MaxDepth;
    std::size_t Index = 0;
    std::size_t SavedIndex = 0;
};

// ponytail: no grouping of several edits into one transaction, and no undo of
// selection or view state. Both are real editor features and neither is a data
// question — add the transaction wrapper when there is an operation that edits
// more than one segment at once, which the closure solver will be the first to
// need.
