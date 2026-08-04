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
include their headers by relative path. `TrainPhysics` has two suites, `test_trainphysics.cpp` and
`test_twotrains.cpp`; build them separately.

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
| `TrainPhysics.h` | The motion model, zones, the train's length, runtime zone retargeting |
| `RideProfile.h` | The whole ride measured at edit time — speed, three G axes, roll rate, height |
| `test_twotrains.cpp` | Two trains on the real preset geometry — the only file crossing all three layers |

**Proves:** energy conservation is exact rather than approximate; friction stopping distance matches
the closed form; drag decays speed exponentially at the predicted rate; a train with length behaves
differently from a point mass in the direction a designer expects — length softens sharp features and
steadies inversions.

`test_twotrains.cpp` is the integration check, and it is where a block brake can finally demonstrate
itself: with one train nothing is ever ahead of it. It transcribes
`ATUCoasterRide::TwoTrainCircuitLayout()` — the actor cannot be included from here, same precedent as
`reference_figures.cpp` — and proves the outer pre-station brake **stops a moving train** (7.64 m/s
down to 0 in 4.7 m), **holds it while the block ahead is occupied** (3.88 s, stopped *and* red, which
is a different claim from merely stopped), and **releases it** once the train ahead clears.

It also proves the layout is a **closed circuit** — position, heading and roll at the seam, plus the
total turning being one lap rather than the 446° it used to be — that a **held train parks inside its
own block** instead of straddling the seam, and that the circuit **carries four trains**: 1 through 4,
seven minutes each, every train lapping, zero violations, never a shared block.

Its last test is a line-for-line stand-in for `ATUCoasterRide::Tick`, because that function cannot be
compiled without Unreal and its dispatch policy has nowhere else to be checked: ten minutes of ride,
both trains circulating through the seam under their own power, no deadlock, no shared block, zero
violations. **Change the actor's tick order and change that test with it.**

It also **prints the circuit's canonical figures** on the way out, the same job
`reference_figures.cpp` does for the reference layout — so a number quoted anywhere about this preset
can be checked by running one thing:

```
30 segments, 1288.02 m, C2 yes
seam 0.000000 m, 0.000084 deg heading, 0.000000 deg roll
top 136.8 km/h   vertical -0.53 .. +3.08   lateral 0.15
crest 48.5 m   clearance 11.68 m   peak roll rate 17.4 deg/s   105 s
8 blocks, 5 of them able to hold a train, so 4 trains
```

## `BlockSignal/` — signalling

The `CLEAR → OCCUPIED → BUFFER(x) → CLEAR` state machine and the dispatch permissive logic, with unit
tests. See [`SIGNALLING.md`](SIGNALLING.md) for what the states mean and why the buffer exists.

| File | What it holds |
|---|---|
| `BlockSignal.h` | The per-block state machine and the permissive. Knows about blocks and nothing else |
| `RideSignals.h` | The mapping layer: arc length → block index, each train's nose-and-tail range, one permissive keyed to the destination |

`RideSignals.h` consumes **doubles and a train index, not an `FTrain`** — `RearS`, `FrontS`, `dt`.
That keeps it independent of the physics, lets the assert suite drive it with bare numbers, and makes
the Unreal actor a caller like any other. Neither `BlockSignal.h` nor `TrainPhysics.h` was modified to
support it.

**Proves:** the permissive denies the cases it should, including the wrap-around case where a
lookahead spanning the whole circuit would include the asking train's own block; that one range diff
handles a straddle, a rollback and a lap-end teleport with no special cases; and that a train stopped
short of the station does *not* deny its own dispatch through a block it is standing in — which it
would do permanently, since nothing would ever clear it.

**Proves, for two trains,** the three identity failures that used to be silent: that a train entering
a block another is standing in is *counted* rather than suppressed, that one train moving on does not
release a block another is parked in, and that a permissive asked on B's behalf is not answered with
A's blocks. All three were confirmed to bite by mutation — reverting each identity check kills exactly
one test and no others. The single-argument forms **deny** on a multi-train instance rather than
aliasing to train 0, so a caller that forgot its index fails closed instead of reproducing the whole
class of bug.

**Proves, for a circuit,** that a reversed rear/front pair is read as a **seam straddle** — the train
holding the last block and the first — rather than sorted into a claim on the entire ring; that the
seam is not a hiding place, so a train arriving across it still collides with and is still denied by
one parked in the first block; and that a point-to-point layout's reversed pair is still sorted,
because a strip has no seam. Three more mutations, three more kills.

**The permissive derives its braking distance** when `SetHoldingBlocks` is supplied: it clears from
the destination to the next block that can *hold* a train, because a train let into a block with no
device in it is committed until it reaches somewhere it can stop. A fixed count cannot say that — on
the closed circuit, whose free runs are 696 m and 184 m, the count rule let **four trains collide**
(14 violations at lookahead 1, 18 at lookahead 2) and the derived rule runs all four clean.

**Records two limits rather than fixing them**, both asserted so they stay known quantities: a block
crossed entirely within one `dt` is never entered and never arms its overlap (needs a block under
0.5 m at 60 Hz and 30 m/s), and the enter-before-exit ordering is *not* observable with a single
train — reversing the two loops fails no assertion, and the comment says so rather than implying the
suite covers it.

Two contracts it cannot enforce from the inside, so they are the caller's: `Tick` is **once per
frame, not once per train** (overlaps live on blocks, and there is no clock in here to notice), and
update order within a frame is observable — but it **fails closed**, reporting a following train half
a frame early rather than half a frame late.

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
