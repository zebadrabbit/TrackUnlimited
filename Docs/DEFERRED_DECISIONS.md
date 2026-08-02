# Deferred Decisions

Choices that came up during work, are **not blocking** progress, and are the developer's to make rather than something to assume. Each entry says what the choice is, what was done in the meantime, and what it would cost to change later.

Anything genuinely blocking does not belong here — it belongs in a question.

Last updated: 2026-08-02.

---

## 1. Helix entry convention — should the editor auto-insert the pitch transition?

**Status:** open. Raised by implementing the helix segment.

A helix segment is constant curvature plus constant torsion. That is a true helix, but its **axis orientation is inherited from the incoming frame** — the track must already be pitched at the helix's climb angle when the segment starts, or the axis comes out tilted. Real track does this anyway, with a transition into the helix.

- **Author supplies the transition.** Explicit, matches how every other segment behaves, no hidden geometry.
- **Editor inserts it.** `MakeHelix` (or the editor) emits pitch-up → helix → pitch-down as a compound. Friendlier; means one authored "segment" is several stored segments, which touches the data model and undo granularity.

**Meanwhile:** the author supplies it. `MakeHelix` returns one segment and its comment states the requirement; a test builds the transition explicitly.
**Cost of deciding later:** low. It is additive — a compound helper can be added without changing the stored representation.

---

## 2. Does `Torsion` belong in the authored vocabulary, or only as a derived field?

**Status:** open.

`FTrackSegment::Torsion` is what makes a helix expressible. But torsion is not a quantity anyone authors — you author radius, climb angle and turns, and `MakeHelix` converts. The question is whether the Phase 1 editor ever shows a torsion field.

- **Hide it.** Editor offers straight / arc / clothoid / helix, each with natural parameters, and torsion is computed. Keeps the UI honest to how track is actually designed.
- **Expose it.** More general — any constant-curvature-constant-torsion curve becomes authorable. Almost certainly a footgun.

**Meanwhile:** the field exists on the struct because the integrator needs it; only `MakeHelix` sets it.
**Cost of deciding later:** none for the data model. Purely a UI question, and it belongs with the numeric entry card.

---

## 3. Which roll mode should the editor offer by *default*?

**Status:** open. Raised by resolving the roll-mode question below — the data model answer does not settle the UI one.

`ERollMode::PathRelative` is the struct default because it is defined everywhere and it keeps every existing track meaning exactly what it did. That is a compatibility choice, not an authoring one. What a new segment should arrive as in the editor is separate:

- **Default path-relative.** Never undefined, never surprises anyone through an inversion. Costs the author a mental conversion on the single most common thing they will do, which is bank a turn.
- **Default world bank.** Matches what "bank" means to a human, and the common case — a banked turn on rolling terrain — comes out right with no thought. Wrong for inversions, where it is undefined and the author must notice and switch.
- **Default by segment type.** Arc and clothoid arrive world-referenced; anything the author builds an inversion from arrives path-relative. Best behaviour, and the only option that needs the editor to carry an opinion about what a segment is *for*.

**Meanwhile:** path-relative, from the struct default. No editor exists yet to have a different opinion.
**Cost of deciding later:** low now, high after tracks are saved. The mode is stored per segment, so changing the default silently changes the meaning of every segment authored under the old one. Decide before the save format ships.

---

## Resolved

### World-referenced roll mode, or keep rotation-minimising only? — **both, per segment** (2026-08-02)

`FTrackSegment::RollMode`. `PathRelative` is unchanged and stays the default; `WorldBank` means the value is the bank a spirit level would read, and is resolved per sample against the walked frame rather than baked in when typed — so an edit upstream cannot silently unbank a turn downstream.

It was cheap because roll never perturbs the path: it is applied last, as a rotation about the tangent, so solving a world bank is one `atan2` against a frame already in hand. `FTrackFrame::Roll` stays path-relative, and nothing downstream learned about roll modes.

Neither option in the original framing was right alone. World bank is **undefined pointing straight up**, so it cannot be the only mode; path-relative makes the author compute their own banking, so it should not be. Full measurements, including what the two new blind spots cost, are in `PHASE0_FINDINGS.md` under the `Roll = 0` entry.
