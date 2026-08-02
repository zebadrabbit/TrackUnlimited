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

## Resolved

### Which roll mode does the editor default to? — **path-relative, for every kind** (2026-08-02)

Not "world bank for arcs, path-relative for the rest", which was the earlier provisional call. Switching the default on segment kind means changing a segment's kind silently changes what its roll number *means*, and that trade is worse than one extra dropdown. Path-relative is defined everywhere; the author picks "Bank (from horizon)" when they want it, and the dropdown says so in words.

### Does `Torsion` belong in the authored vocabulary? — **yes, show it** (2026-08-02)

It is not a quantity anyone reaches for first — you author radius, climb and turns, and `MakeHelix` converts — but it is worth knowing and worth being able to set directly, because it is the one field that turns a planar curve into a corkscrewing one. The loop side-step investigation needed exactly that and had to reach past the authored vocabulary to get it. Editable on the kinds where it is genuinely an input, and shown as a derived read-only value on a helix, where it is an output.

### World-referenced roll mode, or keep rotation-minimising only? — **both, per segment** (2026-08-02)

`FTrackSegment::RollMode`. `PathRelative` is unchanged and stays the default; `WorldBank` means the value is the bank a spirit level would read, and is resolved per sample against the walked frame rather than baked in when typed — so an edit upstream cannot silently unbank a turn downstream.

It was cheap because roll never perturbs the path: it is applied last, as a rotation about the tangent, so solving a world bank is one `atan2` against a frame already in hand. `FTrackFrame::Roll` stays path-relative, and nothing downstream learned about roll modes.

Neither option in the original framing was right alone. World bank is **undefined pointing straight up**, so it cannot be the only mode; path-relative makes the author compute their own banking, so it should not be. Full measurements, including what the two new blind spots cost, are in `PHASE0_FINDINGS.md` under the `Roll = 0` entry.
