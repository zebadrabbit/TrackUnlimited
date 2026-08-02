# Deferred Decisions

Choices that came up during work, are **not blocking** progress, and are the developer's to make rather than something to assume. Each entry says what the choice is, what was done in the meantime, and what it would cost to change later.

Anything genuinely blocking does not belong here — it belongs in a question.

Last updated: 2026-08-02.

---

## 1. World-referenced roll mode, or keep rotation-minimising only?

**Status:** open. Flagged in `PHASE0_FINDINGS.md` under track spline limitations as "a real Phase 1 decision, not a bug".

The path frame is exactly rotation-minimising, so `Roll = 0` does **not** mean "level with the horizon" on non-planar track. Climb 45° / turn left 90° / descend 45° at roll 0 throughout exits at −54.736° of world bank; three right angles gives exactly −90°. The geometry and its felt G are correct either way — this is about what an author *means* when they type a roll value.

- **Keep RMF only.** Roll is always relative to the parallel-transported frame. Simple, already true, and what the physics wants. Authors must compute the world bank they want.
- **Add a world-referenced mode.** A segment could declare its roll as "relative to horizon" and the editor solves for the RMF roll that achieves it. More intuitive for authoring a banked turn; more machinery, and meaningless when inverted.

**Meanwhile:** RMF only, unchanged.
**Cost of deciding later:** low while tracks are hand-authored. Rises once the editor UI labels a field "bank", because that label implies an answer.

---

## 2. Helix entry convention — should the editor auto-insert the pitch transition?

**Status:** open. Raised by implementing the helix segment (see item below in "Resolved").

A helix segment is constant curvature plus constant torsion. That is a true helix, but its **axis orientation is inherited from the incoming frame** — the track must already be pitched at the helix's climb angle when the segment starts, or the axis comes out tilted. Real track does this anyway, with a transition into the helix.

- **Author supplies the transition.** Explicit, matches how every other segment behaves, no hidden geometry.
- **Editor inserts it.** `MakeHelix` (or the editor) emits pitch-up → helix → pitch-down as a compound. Friendlier; means one authored "segment" is several stored segments, which touches the data model and undo granularity.

**Meanwhile:** the author supplies it. `MakeHelix` returns one segment and its comment states the requirement; a test builds the transition explicitly.
**Cost of deciding later:** low. It is additive — a compound helper can be added without changing the stored representation.

---

## 3. Does `Torsion` belong in the authored vocabulary, or only as a derived field?

**Status:** open.

`FTrackSegment::Torsion` is what makes a helix expressible. But torsion is not a quantity anyone authors — you author radius, climb angle and turns, and `MakeHelix` converts. The question is whether the Phase 1 editor ever shows a torsion field.

- **Hide it.** Editor offers straight / arc / clothoid / helix, each with natural parameters, and torsion is computed. Keeps the UI honest to how track is actually designed.
- **Expose it.** More general — any constant-curvature-constant-torsion curve becomes authorable. Almost certainly a footgun.

**Meanwhile:** the field exists on the struct because the integrator needs it; only `MakeHelix` sets it.
**Cost of deciding later:** none for the data model. Purely a UI question, and it belongs with the numeric entry card.
