<p align="center">
  <img src="Brand/github/hero-1280x400.png" alt="TrackUnlimited — an open-source successor to NoLimits 2" width="100%">
</p>

<p align="center">
  <a href="LICENSE"><img alt="Licence: MIT" src="https://img.shields.io/badge/licence-MIT-7FD8FF?style=flat-square"></a>
  <img alt="Unreal Engine 5.8" src="https://img.shields.io/badge/Unreal%20Engine-5.8-E8F2F8?style=flat-square">
  <img alt="Phase 4" src="https://img.shields.io/badge/phase-4%20%C2%B7%20track%20meshing%20%26%20supports-FFB020?style=flat-square">
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

**A station is a process, not a place.** Arrive, unload, load, secure, all-clear, dispatch — and the
permissive is an AND of every one of those *and* the interlocking, which on a working ride is usually
the term that went green first while an operator was still walking the train. Every gate is a
contact rather than a timer: restraints and gates are **commanded** devices with a travel time and
per-group sensors, so "commanded closed but car 3 is not locked" is a thing the model can say.

Manual dispatch applies the same interlocks — it changes who decides the timing and never whether the
safety logic can be bypassed, which is precisely why it is not a second code path. The button must be
released between trains, so a taped control runs nothing.

→ [`Docs/SIGNALLING.md`](Docs/SIGNALLING.md)

### A control panel generated from the coaster's own data

Walk the same ordered block and segment list that drives the geometry and the physics: each block
emits an indicator, each powered segment emits a VFD module with commanded speed, motor feedback and
torque, each platform a sequence readout saying what is holding it. Nothing is authored per coaster
and nothing is cached — add a block to a layout and an indicator appears, because there is nowhere
else for it to come from.

Corrected against photographs of three real operator consoles, which changed three things: the blocks
became a **schematic** rather than a table, the operating and stop controls got the green and red
fields every real panel uses, and `ADVANCE` was separated from `DISPATCH`. It has an **operator view
and a maintenance view**, because a real installation has both and motor current belongs to exactly
one of them.

The point is to make the causal chain **visible** — a sensor trips, an indicator lights, the logic
evaluates, an actuator responds — rather than collapsing it into an invisible if-statement.

→ [`Docs/SIGNALLING.md#the-generated-control-panel`](Docs/SIGNALLING.md#the-generated-control-panel)

### Failures, and what notices them

Restraint groups that will not close, gates that jam, sensors that die, stick on or chatter on a loose
connection — injectable, deterministic, and each one measured against **what the safety design
actually catches**. A dead block sensor makes the two independent detection methods disagree, and
neither can say which is wrong, which is the property a second method is bought for rather than a
better single one.

The matrix is kept honest in both directions: it records the failures nothing currently catches.

→ [`Docs/FAULTS.md`](Docs/FAULTS.md)

### Measured, not asserted

Every claim on this page has a number behind it, and the numbers come from running the code. The
ride profile measures the whole ride at edit time — speed, all three felt-G axes, roll rate and
height, sampled by arc length — so an author learns a hill is too tall *before* watching a train
fail to crest it.

---

## Status

**Phase 4 — Track Meshing & Supports.** Phases 0 through 3.75 are complete, **Phase 3.5's shell is
substantially done**, and the whole of it **compiles and runs in the engine**: the track is solid
geometry standing on supports and footings, catwalks run where the evacuation model says they do,
and the control system drives all of it.

What is left in this phase is **art** — the structure is still on a placeholder material, and cars
are still cubes. The geometry underneath is parametric and stays that way; a modelled rail would
have to be re-modelled for every layout.

What exists today, all of it engine-free and assert-tested under `Prototypes/`, with a thin Unreal
actor over the top:

| | |
|---|---|
| Track geometry | curvature profile over arc length; straight, arc, clothoid, helix |
| Authored data model | typed segment list, diffable JSON, exact round trip |
| Validation | reports, never repairs — including self-clearance |
| Circuit closure | damped Gauss-Newton over the parameters you free — and one layout that closes by shape instead, exactly |
| Undo / redo | snapshots, with the save format as identity |
| Train physics | energy-exact motion, zones, a train with length |
| Ride profile | the whole ride measured at edit time |
| Block signalling | state machine, overlap, permissives — and up to **four** running trains that trip them |
| Block brakes | hold, and release on a permissive — with the layout checked for whether it *can* stop a train there |
| Braking distance | derived from the layout: a dispatch clears to the next block that can actually stop the train |
| NL2 interop | CSV and live telemetry — validation fixtures, not an authoring path |
| Sensors | proximity switches with no idea which train is on them, and a train counter over them |
| Diverse redundancy | the counter runs alongside the interlocking; a disagreement stops the ride |
| Drives | one VFD per powered run — commanded, output and motor feedback can all disagree |
| Emergency stop | inside the drives, latched, IEC 60204-1 stop categories, monitored 0-1-0 reset |
| Stations | a process per platform position, with commanded restraints and gates |
| Control panel | generated from the same walk, operator and maintenance views |
| Ride envelopes | the profile judged against duration-dependent acceleration limits |
| Fault injection | stuck restraints and gates, dead/stuck/chattering sensors — with a detection matrix |
| Event log | every state transition, timestamped, to the panel and to disk |
| Starter layouts | five worked examples of the vocabulary, each measured before shipping — two of them closed circuits |
| Track meshing | rails, spine and ties swept from the same curvature profile — 309k triangles on the reference circuit, capped ends, asserted watertight |
| Supports | placed where the rules allow and REFUSED with a reason where they do not — through an inversion, under grade, or fouling the track. **Drawn**, with a spread footing under every column and a footer plate under track too low for one |
| Catwalks | the deck and guardrail an evacuation route is actually walked along — one toggle, because a walkway without a rail is a fall hazard rather than a cheaper walkway. Track too steeply banked to walk on is reported and still drawn |
| Device audit | what a layout's devices will actually do, judged against the ride it produces, with the numbers in the sentence |
| Application shell | boot, main menu and track browser, Build / Operate / Ride, settings that persist, save and open, autosave and crash recovery |
| Runtime editing | typed numeric entry, insert and remove, multi-select showing the intersection, and undo — no drag handles, ever |
| The console | pressable: dispatch, auto/manual, E-stop, reset — and the gates, harness and walk-round when you are the crew rather than the simulated one |
| In-engine | builds against UE 5.8.1 with zero warnings; rides, meshes, signals, and reads out speed, G and block state |

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
| [`Docs/CONTROL_ARCHITECTURE.md`](Docs/CONTROL_ARCHITECTURE.md) | The three tiers and the rule that requests never travel down — safety in C++ and not scriptable, for the same reason it is not in a real park |
| [`Docs/TIER2_OVERRIDES.md`](Docs/TIER2_OVERRIDES.md) | The expression language for per-block overrides. Strict IEC 61131-3 spelling, and an override can only ever make a permissive MORE restrictive |
| [`Docs/DIRECTION_AND_ROUTES.md`](Docs/DIRECTION_AND_ROUTES.md) | **Design only:** what a reverse section and a track merge actually cost — direction as two signs, and route interlocking instead of a block list |
| [`Docs/COASTER_TYPES.md`](Docs/COASTER_TYPES.md) | **Design only:** what "which kind of coaster" costs, sorted by that rather than by what a rider notices — and why a type is a preset and never a branch |
| [`Docs/REFERENCES.md`](Docs/REFERENCES.md) | Outside work this project relies on and what each contributed, including what was deliberately *not* taken |
| [`Docs/PROTOTYPES.md`](Docs/PROTOTYPES.md) | The seven standalone prototypes and 35 assert suites: what each proves, how to build and run them |
| [`Docs/ROADMAP.md`](Docs/ROADMAP.md) | Phases, shippable artifacts, current status |
| [`Docs/REFERENCE_LAYOUT.md`](Docs/REFERENCE_LAYOUT.md) | The canonical measured figures for the reference layout |
| [`Docs/PROJECT_PLAN.md`](Docs/PROJECT_PLAN.md) | The full plan: vision, market context, all five pillars, risks |
| [`Docs/PHASE0_FINDINGS.md`](Docs/PHASE0_FINDINGS.md) | What was proven, what was **dis**proved, and the known-limitations ledger |
| [`Docs/GLOSSARY.md`](Docs/GLOSSARY.md) | Heartline, clothoid, holonomy, felt G — what the words mean on a real coaster |
| [`Docs/FAULTS.md`](Docs/FAULTS.md) | Injecting failures, and the matrix of what the safety design actually catches |
| [`Docs/UI_CONVENTIONS.md`](Docs/UI_CONVENTIONS.md) | Framework, resolution, colour, layout and units — the rules every panel is built against |
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
