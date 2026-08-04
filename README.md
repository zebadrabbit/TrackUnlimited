<p align="center">
  <img src="Brand/github/hero-1280x400.png" alt="TrackUnlimited — an open-source successor to NoLimits 2" width="100%">
</p>

<p align="center">
  <a href="LICENSE"><img alt="Licence: MIT" src="https://img.shields.io/badge/licence-MIT-7FD8FF?style=flat-square"></a>
  <img alt="Unreal Engine 5.8" src="https://img.shields.io/badge/Unreal%20Engine-5.8-E8F2F8?style=flat-square">
  <img alt="Phase 2" src="https://img.shields.io/badge/phase-2%20%C2%B7%20physics%20%26%20ride%20feel-FFB020?style=flat-square">
  <img alt="C++17" src="https://img.shields.io/badge/prototypes-C%2B%2B17%2C%20no%20engine%20needed-4ADE80?style=flat-square">
</p>

# TrackUnlimited

A free, open-source roller coaster simulator built in Unreal Engine 5, aimed squarely at the gap
[NoLimits 2](https://www.nolimitscoaster.com/) has left open for a decade: engineering-grade
precision, a modern interface, and a real block-signalling and ride-control system that neither
NoLimits 2 nor Planet Coaster has attempted.

Track is **curvature-continuous by construction**, physics is **energy-exact**, and the G-forces are
the ones a rider would actually feel — not a plausible-looking curve.

---

## The reference layout

![The reference layout, drawn as a side elevation with dimensions and callouts](Brand/github/layout-1280x560.png)

This drawing is not an illustration. It is produced by compiling the project's own prototype headers
against the layout in [`Source/TrackUnlimited/TUCoasterRide.cpp`](Source/TrackUnlimited/TUCoasterRide.cpp)
and running the ride profile over the result. Sixteen typed segments, authored as numbers, riding
inside the engine and reading out on screen.

Full figures and how to reproduce them: [`Docs/REFERENCE_LAYOUT.md`](Docs/REFERENCE_LAYOUT.md).

---

## Features

### Track is data, not dragging

![The authored segment vocabulary: straight, arc, clothoid, helix, raw](Brand/github/authoring-1280x420.png)

A track is an ordered list of typed parametric segments, each defined by values or expressions. The
3D view is a read-only preview, not the editing surface. This is a deliberate rejection of
direct-manipulation editing, and it buys three things at once: exact geometry, an editor that is
Unreal's own Details panel rather than a bespoke UI, and a save format that produces a one-line diff
when you change a helix radius.

→ [`Docs/AUTHORING.md`](Docs/AUTHORING.md)

### Curvature continuity you get for free

Every segment carries curvature varying linearly over arc length — a straight is `κ = 0`, a
constant-radius curve is `κ = const`, a clothoid is `κ` linear — and geometry comes from integrating
a moving orthonormal frame along that profile. C² continuity is therefore a **property of the
representation**, not something fitted afterwards, and transition curves are the native case instead
of special handling. The reference layout is continuous to 1e-9 across all fifteen joints, with
nothing solved to make it so.

→ [`Docs/ARCHITECTURE.md`](Docs/ARCHITECTURE.md)

### Physics that conserves what it should

Gravity is applied as an exact energy exchange rather than an integrated force, so a frictionless
circuit conserves energy at **any** timestep — halve the timestep and the answer does not drift.
Rolling resistance follows the actual normal load, so it rises in a valley and falls toward zero at
airtime. The train has length, so a crest is paid for at the whole train's mean height rather than
the lead car's — which is why the back car gets thrown harder than the front over an *asymmetric*
airtime hill, and why over a symmetric one it does not.

→ [`Docs/ARCHITECTURE.md#physics`](Docs/ARCHITECTURE.md#physics)

### Real block signalling

![The block state machine: CLEAR, OCCUPIED, BUFFER(x), CLEAR](Brand/github/signalling-1280x420.png)

Not "train present / absent". Each block is a state machine that withholds CLEAR for a configurable
safety overlap after a train has physically left it — the real-railway *overlap* concept. A train
occupies a *range* of blocks, nose to tail, so it holds two while it straddles a boundary.

**Two trains run one circuit, interlocked.** A train is held by commanding the device under it to
zero, never by declining to simulate it — so a station is not a special case, and a block brake holds
a train mid-course while the block ahead is occupied and releases it when it clears. Where a train may
be held is itself checked: a friction brake can stop a train and never start one, a launch can start
one and never stop one, and a block that is too short to stop what it receives is a trim brake
whatever it is labelled.

Manual dispatch is designed but not built — the interlocks are meant to apply identically whichever
way the timing is decided, which is precisely why manual is not a second code path.

→ [`Docs/SIGNALLING.md`](Docs/SIGNALLING.md)

### A control panel generated from the coaster's own data — *designed, not yet built*

Walk the same ordered block and segment list that drives the geometry and the physics: each block
emits an indicator, each powered segment emits a VFD module with target speed, motor feedback,
torque and ramp rate. The point is to make the causal chain **visible** — a sensor trips, an
indicator lights, the logic evaluates, an actuator responds — rather than collapsing it into an
invisible if-statement. This one is a settled design with nothing implemented behind it yet; every
other feature on this page is running code.

→ [`Docs/SIGNALLING.md#the-generated-control-panel`](Docs/SIGNALLING.md#the-generated-control-panel)

### Measured, not asserted

Every claim on this page has a number behind it, and the numbers come from running the code. The
ride profile measures the whole ride at edit time — speed, all three felt-G axes, roll rate and
height, sampled by arc length — so an author learns a hill is too tall *before* watching a train
fail to crest it.

---

## Status

**Phase 2 — Physics & Ride Feel.** Phases 0 and 1 are complete.

What exists today, all of it engine-free and assert-tested under `Prototypes/`, with a thin Unreal
actor over the top:

| | |
|---|---|
| Track geometry | curvature profile over arc length; straight, arc, clothoid, helix |
| Authored data model | typed segment list, diffable JSON, exact round trip |
| Validation | reports, never repairs — including self-clearance |
| Circuit closure | damped Gauss-Newton over the parameters you free |
| Undo / redo | snapshots, with the save format as identity |
| Train physics | energy-exact motion, zones, a train with length |
| Ride profile | the whole ride measured at edit time |
| Block signalling | state machine, overlap, permissives — and **two** running trains that trip them |
| Block brakes | hold, and release on a permissive — with the layout checked for whether it *can* stop a train there |
| NL2 interop | CSV and live telemetry — validation fixtures, not an authoring path |
| Starter layouts | four worked examples of the vocabulary, each measured before shipping |
| In-engine slice | builds against UE 5.8, rides, reads out speed, G and block state |

→ [`Docs/ROADMAP.md`](Docs/ROADMAP.md) for what each phase ships and what is left.

---

## Try it in ten seconds

The riskiest maths is standalone C++17 with no engine dependency, so you can build and run it
**without an Unreal install** — the lowest-friction way into this codebase.

```sh
cd Prototypes/TrackSpline
clang++ -std=c++17 -Wall -Wextra -O2 -o test_trackspline test_trackspline.cpp && ./test_trackspline
```

Same shape for `Prototypes/BlockSignal` and `Prototypes/TrainPhysics`. Run each from inside its own
directory — the tests include their headers by relative path.

For the full engine build, see [`CONTRIBUTING.md`](CONTRIBUTING.md).

→ [`Docs/PROTOTYPES.md`](Docs/PROTOTYPES.md) for what each prototype proves.

---

## Documentation

| Document | What it covers |
|---|---|
| [`Docs/ARCHITECTURE.md`](Docs/ARCHITECTURE.md) | Track representation, physics model, meshing, rendering, save format — and the constraints that are not up for negotiation |
| [`Docs/AUTHORING.md`](Docs/AUTHORING.md) | The segment vocabulary, roll vs bank, validation, closure, undo, the file format |
| [`Docs/SIGNALLING.md`](Docs/SIGNALLING.md) | Block states, buffer/overlap, dispatch permissives, the generated control panel |
| [`Docs/CONTROL_ARCHITECTURE.md`](Docs/CONTROL_ARCHITECTURE.md) | **Design only, nothing built:** how a scriptable control and show layer could be shaped, and the real standards it would follow |
| [`Docs/PROTOTYPES.md`](Docs/PROTOTYPES.md) | The five standalone prototypes: what each proves, how to build and run them |
| [`Docs/ROADMAP.md`](Docs/ROADMAP.md) | Phases, shippable artifacts, current status |
| [`Docs/REFERENCE_LAYOUT.md`](Docs/REFERENCE_LAYOUT.md) | The canonical measured figures for the reference layout |
| [`Docs/PROJECT_PLAN.md`](Docs/PROJECT_PLAN.md) | The full plan: vision, market context, all five pillars, risks |
| [`Docs/PHASE0_FINDINGS.md`](Docs/PHASE0_FINDINGS.md) | What was proven, what was **dis**proved, and the known-limitations ledger |
| [`Docs/GLOSSARY.md`](Docs/GLOSSARY.md) | Heartline, clothoid, holonomy, felt G — what the words mean on a real coaster |
| [`Docs/DEFERRED_DECISIONS.md`](Docs/DEFERRED_DECISIONS.md) | Open choices, what was done in the meantime, and what changing costs |
| [`CHANGELOG.md`](CHANGELOG.md) | What has landed, newest first |

Three places, three jobs: the repo docs are the **design reference**,
[Trello](https://trello.com/b/Uzqm38o7/coaster-sim-nolimits-successor) is **live task status**, and
GitHub Issues is the **inbound channel** for bugs and proposals.

---

## Contributing

Design discussion and prototyping help are worth more right now than polished PRs. Start with
[`CONTRIBUTING.md`](CONTRIBUTING.md), and read [`Docs/PHASE0_FINDINGS.md`](Docs/PHASE0_FINDINGS.md)
before changing any prototype header — several obvious-looking fixes have already been measured and
found to make things worse, and that page says which and why.

Working with an AI coding agent? [`CLAUDE.md`](CLAUDE.md) is the onboarding file: the non-negotiable
constraints, the current phase, and the vocabulary. Read it before generating code here.

## Licence

MIT — see [`LICENSE`](LICENSE). This repository contains project code and content only, **not**
Unreal Engine source; you will need your own Unreal Engine installation, per Epic's
[Unreal Engine EULA](https://www.unrealengine.com/eula/unreal). No real manufacturer trademarks or
ride designs are used anywhere in this project.
