# Authoring a track

How track is created in TrackUnlimited, and why it is created that way.

![The authored segment vocabulary](../Brand/github/authoring-1280x420.png)

## Contents

- [The rule](#the-rule)
- [The segment vocabulary](#the-segment-vocabulary)
- [Roll vs bank](#roll-vs-bank)
- [The heartline](#the-heartline)
- [Validation reports, it does not repair](#validation-reports-it-does-not-repair)
- [Closing a circuit](#closing-a-circuit)
- [Undo and redo](#undo-and-redo)
- [The file format](#the-file-format)
- [What the editor actually is](#what-the-editor-actually-is)
- [Importing from NoLimits 2](#importing-from-nolimits-2)

---

## The rule

**A track is an ordered list of typed parametric segments.** Each one is defined by values or
expressions you type. There is no click-drag-place viewport interaction for track geometry, and there
will not be one.

This is not a style preference. It is the decision the rest of the architecture rests on:

- **Exactness.** A radius of 32 m is 32 m, not 31.9 m because that is where the mouse was.
- **It diffs.** The segment list serialises directly to a text format, so a change is a readable diff
  instead of a binary blob.
- **The editor is free.** Unreal's Details panel already has typed fields, per-kind field visibility,
  add/remove/reorder, copy/paste, multi-select and its own undo. Reimplementing that in Slate would
  be a great deal of code to arrive back where we started.
- **It composes with the geometry.** Curvature-continuous track is what you get when curvature is the
  thing you author; it is something you have to chase when position is.

The 3D view, the live speed and G traces, and the banking preview are all read-only views driven off
that data.

## The segment vocabulary

| Kind | What you type | Notes |
|---|---|---|
| `Straight` | length | `κ = 0` |
| `Arc` | length, radius | `κ = const`. **+ve radius is a left turn, −ve a right turn** |
| `Clothoid` | length, κ start, κ end | `κ` varies linearly. The transition curve |
| `Helix` | radius, climb angle, turns | Constant curvature plus constant torsion. Length is derived |
| `Raw` | curvature endpoints, torsion | Everything the vocabulary cannot say yet |

Every kind additionally carries **roll start**, **roll end** and a **roll mode**.

**Clothoid endpoints are curvature (1/m), not radii, and this is deliberate.** An easement out of a
straight has an endpoint with *no* radius, and `κ = 0` says that as an ordinary value where radius
would need infinity or a sentinel. The cost is that a transition into R30 reads as
`0.033333333333333333` in the file; the editor should show radius with a "straight" option and
convert.

**`Raw` is information, not a wart.** Right now the authored vocabulary is yaw-only — no `Make*`
helper builds *pitch* curvature — so every hill in the reference layout is a pair of `Raw` segments.
A track made entirely of `Raw` is a track nobody can meaningfully edit, and that is exactly what the
file is telling you.

**Helix entry is the author's job.** A helix inherits its axis orientation from the incoming frame,
so the track must already be pitched at the climb angle when the segment starts — real track does
this anyway, with a transition into the helix. Whether the editor should auto-insert that transition
is an open question; see [`DEFERRED_DECISIONS.md`](DEFERRED_DECISIONS.md).

## Roll vs bank

**These are not synonyms**, and the difference is a per-segment mode (`ERollMode`).

- **Roll** is measured from the rotation-minimising path frame. Defined everywhere, including
  inverted and vertical track. This is what the integrator sees, and what `FTrackFrame::Roll`
  reports.
- **Bank** is measured from the horizon — what a spirit level reads. Undefined pointing straight up.

`Roll = 0` is **not** level on non-planar track. A corner hill authored at roll 0 exits 54.736° off
level path-relative and 5.7e-14 world-referenced. Say which one you mean.

Both modes exist because neither is defined everywhere a coaster goes. `PathRelative` is the default
for every kind — switching the default on segment kind would mean changing a kind silently changes
what its roll number *means*, and that trade is worse than one extra dropdown. World bank is resolved
per sample against the walked frame rather than baked in when typed, so editing an upstream hill
cannot silently unbank a turn downstream.

## The heartline

Banking and ride-camera calculations are computed around the **heartline** — an offset reference line
roughly at rider chest height (1.1 m above the rail centreline for a sit-down train) — rather than
around the rail centreline. That is what makes felt-G through a banked turn physically correct rather
than merely plausible.

Arc length, and therefore every `S` value in the model, is measured along the heartline.

## Validation reports, it does not repair

`TrackValidate.h` runs over the *authored* list before anything is built from it, because the geometry
cannot tell you a radius was typed into a curvature field — it can only produce the consequences.

It also does **self-clearance**: nothing else in the model could see the track passing through itself,
and it took *riding* the vertical slice to find out. Its loop is built from pure pitch curvature,
which makes it exactly planar, so the ascending and descending legs pass 0.189 m apart against rails
1.215 m wide. Every segment individually valid, every joint continuous, every G reading plausible,
rider through solid steel.

That is the third member of a family worth naming: felt G cannot see roll rate, the authored list
cannot see the roll the geometry contributes, and neither can see two parts of the track occupying
the same space. Each needed its own check.

## Closing a circuit

The one real cost of authoring curvature is that you arrive wherever you arrive. `TrackClose.h`
handles it:

- **`MeasureClosure`** reports the gap per axis, and gives **height its own field** — because that is
  the one that does not announce itself. A track ending 8.5 m low still looks closed in plan view,
  which is exactly how the vertical slice originally shipped.
- **Per-axis targets.** "Closed" is not one question. A circuit wants all three axes; a
  point-to-point layout wants only Z, because a station-to-brake-run ride is not supposed to end back
  in the station's footprint.
- **`SolveClosure`** is damped Gauss-Newton over **authored** parameters that the caller explicitly
  frees — so a solved arc keeps the radius you typed instead of gaining a curvature field that
  disagrees with it, and nothing moves that you did not offer. Radius freedoms have their sign locked
  and magnitude floored, because crossing zero would reverse a turn and pass through infinite
  curvature.
- On failure the document is left **unchanged** by default.

Two limits are measured rather than fixed: horizontal straights cannot move height at all (zero
gradient), and freeing one half of a balanced hill breaks the balance that made it level.

## Undo and redo

Snapshots, not commands. The command pattern needs every edit to supply a do *and* an inverse, and
the failure mode when those disagree is silent corruption discovered several undos later. A document
is an ordered list of small POD structs, so copying it whole costs nothing and cannot be asymmetric.

Two details worth knowing:

- **Equality goes through the save format**, not a hand-written `operator==`. Two documents are the
  same exactly when they save the same. This cannot rot — `Torsion` and `RollMode` were both added
  after the fact, and a field-by-field comparison would have silently ignored them and dropped undo
  steps.
- **Dirty state is a position, not a flag**, so undoing back to what is on disk correctly reports
  clean. If the depth cap discards the saved state it reports dirty forever — telling someone their
  work is safe when the evidence was thrown away is the wrong direction to be wrong in.

Typing "30.5" into a field is one undo step, not five, via a caller-supplied merge key. The UI owns
that, because only the UI knows when an edit is finished.

## The file format

Versioned, diffable JSON. The rule is **store what was typed, never what was derived**.

A helix is `radius 20, climbing 15°, 1.5 turns` — not the `length 195.14, curvature 0.04665, torsion
0.01250` the integrator runs on. Storing the expansion costs roughly one segment per metre, rides
identically, and contains the radius **nowhere**, so it could never be edited again. Measured: a
helix radius change is **one line of diff**; against a stored expansion it would rewrite every one of
roughly 195 segments' fitted curvature endpoints.

- **Angles are degrees** — in the file *and* in the authored struct, the only place in the codebase
  that is not radians. Nobody types `0.26179938779914941`; they type `15`. Keeping degrees in the
  struct is what makes the round trip exact: the conversion runs one direction only, in
  `BuildSegment`, so degrees → radians → degrees never happens and cannot lose an ulp.
- **Curvature stays 1/m**, for the reason given above.
- Field and key names carry the unit, so a hand edit cannot get it wrong.
- Defaults are omitted rather than written, which is what let `Torsion` and `RollMode` be added to an
  existing model without touching a stored line.
- **Unknown keys load and are ignored** (forward compatibility); an unknown segment **kind** or a
  future format version is **refused outright**, because silently skipping geometry leaves a file
  that loads, looks fine, and is a different ride.

Measured: worst position error 0.0 m over a six-kind layout round trip, and re-saving a loaded file is
byte-identical — so opening and closing a track cannot churn a diff. Eleven malformed inputs are
rejected with a reason, and the writer refuses to emit a non-finite value rather than write `nan`.

## What the editor actually is

Unreal's Details panel over `TArray<FTUTrackSegment>`, with a live viewport preview and the
ride-profile traces drawn along the track. That is not a placeholder for a bespoke Slate UI — it is
the numeric entry surface, and it satisfies the first architectural constraint exactly: numbers in a
list, no drag handles anywhere near the viewport.

Powered track is per segment, so a lift is however many segments in a row say "Lift" and moving one is
an edit rather than a recompile. Arc lengths accumulate over the segments the track actually
*accepted*, so a rejected degenerate segment cannot silently shift every zone boundary after it.

Every edit rebuilds and reports total length, C² continuity, height gap and closest self-approach —
each of which was previously a defect that took *riding* the track to find.

## Importing from NoLimits 2

`Prototypes/NL2Csv/` reads and writes NoLimits 2's documented tab-separated spline export, so real NL2
layouts can be driven through this model and our tracks opened in NL2 to compare G and speed traces.

**This is a validation and test-fixture path, not an authoring path.** An imported track is thousands
of derived micro-segments with the original segment vocabulary unrecoverable — it is not something
anyone edits, and it must not grow into a back door around parametric authoring. Do not commit NL2
park files or exports of real rides.
