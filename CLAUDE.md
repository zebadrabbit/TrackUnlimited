# CLAUDE.md

Instructions for Claude Code (or any other AI coding agent) working in this repository. Read this before writing or changing anything.

## What this project is

TrackUnlimited: a free, open-source, Unreal Engine 5 roller coaster simulator built as a successor to NoLimits 2 — engineering-grade physics and track precision, a modern approachable UI, and a real block-signaling/ride-control system neither NoLimits 2 nor Planet Coaster has. Solo-developer-led. PC flatscreen first; VR is a deliberate later phase.

## Read first

- `Docs/PROJECT_PLAN.md` — the full plan: vision, market context, all five product pillars, MVP/full scope, technical architecture, phased roadmap, solo-dev execution strategy, legal/licensing, risks, and immediate next steps. Read this in full before making any architectural decision — most "why isn't this built like X" questions are already answered there.
- `Docs/PHASE0_FINDINGS.md` — what the prototypes actually proved, with numbers, and the known-limitations ledger. Read this before changing either prototype header. Several plausible-looking "fixes" have already been tested and shown to make things worse; that page says which, and why.
- Project board (Trello): https://trello.com/b/Uzqm38o7/coaster-sim-nolimits-successor — the live, day-to-day source of truth for what's in progress/done. This repo's docs are the design reference; Trello is where task status actually lives and gets moved around. If you have Trello MCP access in this session, check it for current status before assuming a task list from the docs is up to date.

## Non-negotiable architectural constraints

These were deliberate decisions made after discussion, not defaults — do not "helpfully" change them without raising it first.

1. **No viewport click-drag-place track editing, ever.** Track segments are numeric/formula-driven parametric data (straight, constant-radius curve, clothoid transition, helix), each defined by typed values/expressions. The 3D view is a read-only preview, not the editing surface. This was an explicit rejection of the Coaster Forge / Satisfactory-style direct-manipulation model — do not add drag handles "for convenience" or "for a quick prototype."
2. **C++ for physics and signaling.** Blueprint is fine for editor tooling, UI, and rapid iteration on non-critical systems, but the energy-based motion model (Section 5, "Physics simulation") and the block-signaling state machine (Section 5, "Signaling and ride control") must be C++.
3. **No dependency on Train and Rail System (Polygon Jelly) or any other paid marketplace asset in committed/shipped code.** It's reference material the developer used while prototyping locally — never to be redistributed or have its code/assets copied into this repo.
4. **No Unreal Engine source in this repo, ever.** Project code and content only, per the UE EULA. Contributors get the engine themselves via Epic Games Launcher or their own linked GitHub access.
5. **No real manufacturer trademarks or exact ride designs** (Vekoma, Intamin, B&M, RMC, etc.) without explicit permission. Use original/generic naming and designs.
6. **MIT license** on everything committed here (see `LICENSE`).

## Current phase: Phase 2 — Physics & Ride Feel

**Phase 0 and Phase 1 are complete** (2026-08-02). Two cards remain open in the Phase 1 list and both are deliberate deferrals rather than unfinished work: the loop side-step (the authored vocabulary cannot express a rideable one — five fixes measured and rejected, see `PHASE0_FINDINGS.md`) and ride-profile trace legibility (an aesthetics pass that wants designing, not accreting).

What exists now, all of it engine-free and assert-tested under `Prototypes/`, with a thin UE actor over the top:

- **Track geometry** — curvature profile over arc length; straight, arc, clothoid, helix (via constant torsion). `TrackSpline.h`
- **Authored data model and diffable JSON** — stores what was typed, never what was derived. `TrackIO.h`
- **Validation** — report, never repair. `TrackValidate.h`
- **Circuit closure solver** — damped Gauss-Newton over authored parameters. `TrackClose.h`
- **Undo/redo** — snapshots, with the save format as identity. `TrackHistory.h`
- **Train physics and the ride profile** — the whole ride measured at edit time. `TrainPhysics/`
- **Block signalling, wired to the ride, for N trains** — the state machine in `BlockSignal/BlockSignal.h`, and `BlockSignal/RideSignals.h` mapping each train's nose-and-tail arc length onto it. `RideSignals` takes **doubles and a train index, not an `FTrain`**; keep it that way, it is why the whole layer is testable without the physics. The actor derives blocks from the contiguous-zone walk it already did. Three reads carry train identity — the entry test, the exit release, and the permissive — and each one is what stopped a specific silent fail-open; `Tick` is **once per frame, not once per train**. Holding is done by **gating the zone**, never by declining to integrate.
- **Block brakes that actually hold and release** — `FTrain::SetZoneTargetSpeed` retargets a zone at runtime and `FindHoldZoneAt` says where a train may be parked: a device needs **both** authorities, so a trim brake (cannot start a train) and a launch (cannot stop one) are both refused. Whether it can stop the train in the block it has is a separate, layout-only question — `v²/2a` against the block length.
- **A permissive that derives its braking distance** — `FRideSignals::SetHoldingBlocks` says which blocks can hold a train, and a dispatch then clears from the destination *to the next one of those*, because a train in a block with no device in it is committed. The fixed `Lookahead` count is now extra headway rather than the safety rule. **This is what makes the circuit carry four trains instead of colliding at four**, measured both ways.
- **NL2 CSV and telemetry** — validation fixtures, not authoring paths. `NL2Csv/`, `NL2Telemetry/`
- **Starter layouts** — three worked examples of the vocabulary behind a `Preset` dropdown: flat rig, out-and-back, reference. Each was measured before shipping. `ATUCoasterRide::PresetLayout`
- **The canonical figures, reproducible** — `Prototypes/TrackSpline/reference_figures.cpp` prints everything `Docs/REFERENCE_LAYOUT.md` publishes. Run it before quoting a number from that page.

The editor surface is Unreal's Details panel over `TArray<FTUTrackSegment>`, with a live viewport preview and ride-profile traces. That is not a placeholder for a Slate UI — see the numeric-entry card for why.

**Phase 2's bar is "feels right to an NL2 veteran."** The thing this file used to send you at — the train being a point with no length — is **done**: `FTrainConfig::TrainLength`, nine sample points, gravity reading the train's *mean* height. Two claims that went with it have since been measured and were wrong, so do not repeat them. The back car is only thrown harder on an **asymmetric** hill, and on a symmetric one there is no difference at all. And train length does **not** explain the `DragK` fit — the two coefficients were never separable, correlation 0.975, so that split is a free parameter rather than a result.

Two trains on one circuit is **done** (2026-08-03) and the actor runs them, so do not go looking for it. `Prototypes/TrainPhysics/test_twotrains.cpp` is the proof, and the last test in it is a line-for-line stand-in for `ATUCoasterRide::Tick` — that exists because the actor cannot be compiled without Unreal, so its dispatch policy has nowhere else to be checked. **If you change the actor's tick order, change that test with it or the policy is unverified.**

**A real ride's brake count is not its block count.** Three different things sit on the track and all look like a brake: **block brakes** (interlocking — the only ones that bound a block and the only ones the capacity formula counts), **evacuation zones** (getting riders *off*, which needs a walkway and an egress route, not just a way to stop), and **safety catches** (rollback or a defect; passive, fail closed). A large ride can have ~25 evacuation zones and a handful of catches while running far fewer trains than either count. Neither of the last two is modelled at all.

What is actually worth starting on:

1. **A fast reference recording from NoLimits 2.** Rolling resistance is calibrated and corrected; drag is not, and cannot be, because the existing recording tops out at 44.5 km/h and drag needs speed to be measured against. This one needs the developer at the keyboard, not an agent.
2. **Something that catches a rollback.** `FTrainConfig::bAllowRollback` simulates a train running backwards and **nothing stops it**, because an anti-rollback catch is hardware `FTrackZone` cannot describe — it has powered track and nothing else. A rollback is reported by `FRideProfile` and then simply continues. That is honest for a model with no such device, but it means the one failure mode the physics can actually produce has no safety system in front of it.
3. **An authored stop offset.** A held train now brakes to a *position* rather than to zero-where-it-stands, and parks mid-device. Mid-device is right for a station and arbitrary for a 130 m block brake, so an authored offset is the next step — but it is a preference now, not a defect. It was a defect: parking at the zone's start left a dwelling train holding two blocks and **deadlocked the circuit at three trains**, silently. Read that entry in `PHASE0_FINDINGS.md` before changing where a train stops.

**Trains run real laps, and `bCircuit` is MEASURED, never authored.** The actor checks position, heading and roll at the seam on every rebuild and only wraps a layout that passes; the other three presets still return to the station by teleport, which is correct for track whose two ends are hundreds of metres apart. Read the lap entry in `PHASE0_FINDINGS.md` before touching it — `S -= TotalLength` is one of four changes, and the other three (distance travelled, the sample walk across the seam, and what a reversed rear/front pair *means*) are each a silent failure if missed.

**A circuit closes by SHAPE, not by solver, and the shape is load-bearing.** An oval closes analytically when both turns are the same way, exactly 180°, same radius, same easement; the only condition left is that the return leg's horizontal extent equals the two collinear outbound legs'. That condition means **every metre spent outbound is paid for twice**, which is why the turnaround sits at the top of the hill rather than at each end — measured, the obvious layout came to 1717 m and stalled in every variant. See `PHASE0_FINDINGS.md`.

Before quoting any number from this project, check whether something already measures it — `reference_figures.cpp` for the reference layout, `test_twotrains.cpp` for the closed circuit (it prints its canonical figures on the way out), the assert suites for the models, `PHASE0_FINDINGS.md` for what has already been disproved. Several plausible fixes on that page were measured and rejected, and re-proposing one without saying why yours differs wastes everybody's time.

## Phase 0 — Prototype (complete)

Kept because the *reasoning* is still binding, not because the work is outstanding. Every item below is done; the notes attached to them are decisions that still hold.

1. ~~Evaluate Coaster Forge (Fab/Dualstate Games) — a build-vs-adapt decision for the procedural track meshing layer.~~ **DECIDED: build.** Coaster Forge is a commercial product and cannot be redistributed under MIT, so "adapt" was never available — constraint 3 below already said as much. Buying it would have bought reference reading, not a shortcut. The one pattern the plan wanted from it, zone-based speed control, was arrived at independently and verified against NoLimits 2 (`FTrackZone` in `Prototypes/TrainPhysics/`). Phase 4 meshing is written from scratch against `Prototypes/TrackSpline/TrackProfile.h`. **This closes the last Phase 0 gate.**
2. ~~Prototype the curvature-continuous spline math standalone.~~ **DONE** — `Prototypes/TrackSpline/`. Note the representation is **curvature-profile-over-arc-length**, not the cubic Hermite/B-spline model the plan originally called for: a segment carries curvature varying linearly over arc length (straight `k=0`, arc `k=const`, clothoid `k` linear) and geometry comes from integrating a moving orthonormal frame. C² continuity is therefore a property of the data, not something fitted. This supersedes the earlier wording — **do not "restore" a Hermite/control-point formulation.**
3. ~~Prototype the block-occupancy + buffer state machine as a plain C++ class with unit tests.~~ **DONE** — `Prototypes/BlockSignal/`.
4. ~~Vertical slice: a single hand-authored spline, a cart that follows it with real physics, a basic camera.~~ **DONE** — `Source/TrackUnlimited/`. Use `FTrackFrame::Tangent` for direction; never finite-difference `EvaluateAt`, and treat it as O(track length) per call. That warning applies to anything differentiating the geometry, including the closure solver, which is why its finite-difference step is deliberately coarse.

Added beyond the original plan: `Prototypes/TrainPhysics/` — the Section 5 energy-based motion model, built standalone because the vertical slice needs it. Gravity is an exact energy exchange, not an integrated force; powered sections are tractive accelerations that pass through the same energy accounting. **Do not "simplify" a zone back into a post-hoc clamp on speed** — that formulation manufactures energy on a gradient, which is measured and recorded in `Docs/PHASE0_FINDINGS.md`.

`Prototypes/BlockSignal/BlockSignal.h`, `Prototypes/TrackSpline/TrackSpline.h` and `Prototypes/TrainPhysics/TrainPhysics.h` are the **canonical designs to port** into UE5 C++ — not references to reimplement from scratch. `Docs/PHASE0_FINDINGS.md` has a measured port checklist.

Also added: `Prototypes/NL2Csv/` — reads and writes NoLimits 2's documented tab-separated spline export, so real NL2 layouts can be driven through the model and our tracks opened in NL2 to compare G and speed traces. This is a **validation and test-fixture path, not an authoring path**, and it does not soften constraint #1 above: an imported track is thousands of derived micro-segments with the original segment vocabulary unrecoverable, so it is not something anyone edits. Do not let it grow into a back door around parametric authoring, and do not commit NL2 park files or exports of real rides (`Prototypes/NL2Csv/Tracks/` is gitignored for that reason).

One rule from this phase that outlived it, because it has now bitten twice: **these headers are engine-free but they are compiled INTO the engine, so a type name only needs to be unique against all of UE.** `FFrame` collided with the Blueprint VM's and became `FTrackFrame`; `FField` collided with `UObject/Field.h`'s and became `FTrackIOField`. Standalone clang compiles both cleanly, so the collision only appears at the port. Check a new type name against the engine before, not after.

## Key vocabulary (used throughout the docs and should be used consistently in code/comments)

`Docs/GLOSSARY.md` explains these and the geometry terms (curvature profile, clothoid, parallel transport, holonomy, torsion, Darboux vector, felt G) in terms of what they are on a real coaster. The list below is the short form for code consistency; the glossary is for understanding what the words mean.

- **Block buffer / overlap** — the safety-margin state a block holds after a train physically exits it, before it reports CLEAR. Named after the real-railway "overlap" signaling concept.
- **Heartline** — the reference line (not the rail centerline) that banking and ride-camera calculations are computed around, so felt-G through banked turns is physically correct.
- **Roll vs bank** — not synonyms, and the difference is a per-segment mode (`ERollMode`). **Roll** is measured from the rotation-minimising path frame: defined everywhere including inverted and vertical track, and what the integrator sees. **Bank** is measured from the horizon — what a spirit level reads — and is undefined pointing straight up. Say which one you mean; `Roll = 0` is not level on non-planar track.
- **Dispatch permissive** — the logic gate that allows a station/launch to release a train, based on downstream block clearance (and, for high-speed sections, braking-distance lookahead).
- **VFD module** — the generated control-panel element for a powered segment (lift chain, tire-drive launch): target frequency/speed, actual motor feedback, torque/current draw, ramp rate.

## Conventions in prototype code

The prototypes are metres, radians, seconds, and a **right-handed** frame where `Tangent × Lateral = Up` and `+Lateral` is the rider's **left**. UE5 is centimetres and **left-handed** with `+Y` to the right.

Convert units *and* flip handedness at the port boundary, never inside the math. The flip is not a single sign: UE's `Right` is `-M(Lateral)` where `M(x,y,z) = (x,-y,z)`. Getting this wrong mirrors the entire track and produces geometry that still looks self-consistent — `Docs/PHASE0_FINDINGS.md` has the measured residuals.

## When in doubt

This is a solo-developer, free/open-source project with no funding cushion — favor small, shippable, testable increments over broad refactors. If an implementation choice isn't covered by `Docs/PROJECT_PLAN.md` or this file, flag it and ask rather than assuming; the architectural decisions recorded here were made deliberately, through real back-and-forth, and getting them right mattered enough to write down.
