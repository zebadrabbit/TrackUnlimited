# The prototypes

Five standalone prototypes under `Prototypes/`, all plain C++17 with **no engine dependency**. They
build and run without an Unreal install, which makes them the lowest-friction way into this codebase —
and keeps the maths honest by preventing it from quietly depending on engine behaviour.

They are not throwaways. `TrackSpline.h`, `TrainPhysics.h` and `BlockSignal.h` are the **canonical
designs**, compiled *into* the engine by `TrackUnlimited.Build.cs` rather than copied — so the
standalone assert suites test exactly the code that ships.

## Building and running

A compiler is the only requirement.

```sh
cd Prototypes/TrackSpline
clang++ -std=c++17 -Wall -Wextra -O2 -o test_trackspline test_trackspline.cpp && ./test_trackspline
```

Same shape for each of the others. **Run from inside the prototype's own directory** — the tests
include their headers by relative path.

Tests are plain `assert`s with no framework: add to the existing file and call your function from
`main`. `BlockSignal` and `TrackSpline` finish in well under a second; `TrainPhysics` takes about six,
because it simulates tens of thousands of ticks and each one evaluates the track — pass `-O2` if you
are iterating on it.

> Read [`PHASE0_FINDINGS.md`](PHASE0_FINDINGS.md) before changing any of these headers. It records
> what is verified, with numbers, and — more usefully — which limitations are deliberate. Several
> obvious-looking fixes have already been tested and shown to make things worse.
>
> A `ponytail:` comment marks a deliberate simplification whose ceiling and upgrade path are named
> right there in the comment. `grep -rn "ponytail:" Prototypes/` lists them all. Check whether
> something is one of these before filing it as an oversight.

---

## `TrackSpline/` — geometry

The curvature-continuous track model. Segments are curvature profiles over arc length; geometry comes
from integrating a moving orthonormal frame along that profile, so C² continuity is a property of the
data rather than something fitted.

| File | What it holds |
|---|---|
| `TrackSpline.h` | The segment model, the frame integrator, felt G, roll vs world bank |
| `TrackIO.h` | The authored data model and the diffable JSON save format |
| `TrackValidate.h` | Diagnostics, including self-clearance. Reports, never repairs |
| `TrackClose.h` | Circuit closure: measure the gap, then solve it |
| `TrackHistory.h` | Undo/redo as snapshots, with the save format as identity |
| `TrackProfile.h` | A generic cross-section — gauge, rail and spine dimensions, tie spacing |

**Proves:** curvature continuity holds across joints without fitting; heartline-relative banking
produces sensible felt G; the round trip through the save format is exact.

## `TrainPhysics/` — motion

1D train motion along that track. Gravity is an **exact energy exchange** rather than an integrated
force, so a frictionless circuit conserves energy at any timestep — a deliberately bad 1/30 s tick and
a fine 1/300 s one agree on final speed to 1e-6 around a full loop, where a force-integrating model
drifts. Rolling resistance follows the actual normal load; air
drag is lumped; one zone type covers lift, launch, brake and station.

| File | What it holds |
|---|---|
| `TrainPhysics.h` | The motion model, zones, the train's length |
| `RideProfile.h` | The whole ride measured at edit time — speed, three G axes, roll rate, height |

**Proves:** energy conservation is exact rather than approximate; friction stopping distance matches
the closed form; drag decays speed exponentially at the predicted rate; a train with length behaves
differently from a point mass in the direction a designer expects — length softens sharp features and
steadies inversions.

## `BlockSignal/` — signalling

The `CLEAR → OCCUPIED → BUFFER(x) → CLEAR` state machine and the dispatch permissive logic, with unit
tests. See [`SIGNALLING.md`](SIGNALLING.md) for what the states mean and why the buffer exists.

**Proves:** the permissive logic denies the cases it should, including the wrap-around case where a
lookahead spanning the whole circuit would include the asking train's own block.

## `NL2Csv/` — validation fixtures

Reads and writes NoLimits 2's documented tab-separated spline export — position plus front/left/up
unit vectors, one row per sample, which is an `FTrackFrame` sequence and so reads with a `strtod`
loop. Real NL2 layouts can therefore be driven through this model, and our tracks opened in NL2 to
compare G and speed traces against an independent simulator.

The writer exists so the reader is testable with no NL2 install.

**This is a validation path, not an authoring path** — see
[`AUTHORING.md#importing-from-nolimits-2`](AUTHORING.md#importing-from-nolimits-2). Do not commit NL2
park files or exports of real rides; `Prototypes/NL2Csv/Tracks/` is gitignored for that reason.

## `NL2Telemetry/` — live comparison

Captures NoLimits 2's live telemetry, so the model can be checked against a running independent
simulator rather than only against closed forms. `calibrate.cpp` fits `RollingResistance` and `DragK`
against a recorded speed trace.

**Also proves a negative, which is why it earns its place:** on the recording available, the two
coefficients cannot be separated. The correlation between their predictors — `N·g` and `v²` — is
**0.975** with a condition number around 2000, so no fit can attribute the energy loss to one rather
than the other; the split between them is a free parameter, not a result. Visible as `DragK` flipping
sign across train lengths while the residual barely moves. The cause is the ride, which tops out at
44.5 km/h over 234 m, and drag needs speed to have anything to be measured against. `DragK` therefore
stays at its derived value, and a *fast* reference recording is the actionable next step.

It did move one default, though. Pin `DragK` and rolling resistance alone is well conditioned: three
recordings converge on **0.022–0.026** against a shipped 0.006, with a residual of 0.0005 m/s² — which
says the model's *shape* was right and its coefficient was not. Read alongside the fact that 0.006 had
been justified as steel-on-steel, where a coaster actually runs **polyurethane** on steel at
0.01–0.03, `RollingResistance` was corrected to **0.024**. See
[`PHASE0_FINDINGS.md`](PHASE0_FINDINGS.md).

---

## Conventions

Metres, radians, seconds. A **right-handed** frame where `Tangent × Lateral = Up`, and `+Lateral` is
the rider's **left**. Unreal is centimetres and left-handed. Convert units *and* flip handedness at
the port boundary, never inside the maths — see
[`ARCHITECTURE.md#units-and-handedness`](ARCHITECTURE.md#units-and-handedness).

Use `FTrackFrame::Tangent` for direction. **Never finite-difference `EvaluateAt`**, and treat it as
O(track length) per call.

**Check new type names against all of Unreal, not just against this repo.** These headers are
engine-free but they are compiled *into* the engine, so a name only needs to be unique against the
whole engine. `FFrame` collided with the Blueprint VM's and became `FTrackFrame`; `FField` collided
with `UObject/Field.h`'s and became `FTrackIOField`. Standalone clang compiles both cleanly, so the
collision only appears at the port.
