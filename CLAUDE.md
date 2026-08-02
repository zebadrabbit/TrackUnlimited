# CLAUDE.md

Instructions for Claude Code (or any other AI coding agent) working in this repository. Read this before writing or changing anything.

## What this project is

TrackUnlimited: a free, open-source, Unreal Engine 5 roller coaster simulator built as a successor to NoLimits 2 — engineering-grade physics and track precision, a modern approachable UI, and a real block-signaling/ride-control system neither NoLimits 2 nor Planet Coaster has. Solo-developer-led. PC flatscreen first; VR is a deliberate later phase.

## Read first

- `Docs/PROJECT_PLAN.md` — the full plan: vision, market context, all five product pillars, MVP/full scope, technical architecture, phased roadmap, solo-dev execution strategy, legal/licensing, risks, and immediate next steps. Read this in full before making any architectural decision — most "why isn't this built like X" questions are already answered there.
- Project board (Trello): https://trello.com/b/Uzqm38o7/coaster-sim-nolimits-successor — the live, day-to-day source of truth for what's in progress/done. This repo's docs are the design reference; Trello is where task status actually lives and gets moved around. If you have Trello MCP access in this session, check it for current status before assuming a task list from the docs is up to date.

## Non-negotiable architectural constraints

These were deliberate decisions made after discussion, not defaults — do not "helpfully" change them without raising it first.

1. **No viewport click-drag-place track editing, ever.** Track segments are numeric/formula-driven parametric data (straight, constant-radius curve, clothoid transition, helix), each defined by typed values/expressions. The 3D view is a read-only preview, not the editing surface. This was an explicit rejection of the Coaster Forge / Satisfactory-style direct-manipulation model — do not add drag handles "for convenience" or "for a quick prototype."
2. **C++ for physics and signaling.** Blueprint is fine for editor tooling, UI, and rapid iteration on non-critical systems, but the energy-based motion model (Section 5, "Physics simulation") and the block-signaling state machine (Section 5, "Signaling and ride control") must be C++.
3. **No dependency on Train and Rail System (Polygon Jelly) or any other paid marketplace asset in committed/shipped code.** It's reference material the developer used while prototyping locally — never to be redistributed or have its code/assets copied into this repo.
4. **No Unreal Engine source in this repo, ever.** Project code and content only, per the UE EULA. Contributors get the engine themselves via Epic Games Launcher or their own linked GitHub access.
5. **No real manufacturer trademarks or exact ride designs** (Vekoma, Intamin, B&M, RMC, etc.) without explicit permission. Use original/generic naming and designs.
6. **MIT license** on everything committed here (see `LICENSE`).

## Current phase: Phase 0 — Prototype

See `Docs/PROJECT_PLAN.md` Section 6 (Roadmap) and the Trello board's "Phase 0 — Prototype" list for full detail. Concrete next actions, in rough order:

1. Evaluate Coaster Forge (Fab/Dualstate Games) and other existing Unreal spline-physics prior art hands-on — a time-boxed (~1-2 week) build-vs-adapt decision for the procedural track meshing layer. Do this before committing engineering time to meshing.
2. Prototype the curvature-continuous spline math standalone, outside full engine integration (even a plain script) — cubic Hermite/B-spline segments with explicit first/second-derivative continuity, clothoid/Euler-spiral transitions, heartline-relative banking.
3. Prototype the block-occupancy + buffer state machine as a plain C++ class with unit tests: `CLEAR → OCCUPIED → BUFFER(x) → CLEAR`. This is a from-scratch C++ rebuild of a design the developer already built and shipped once in Blueprint (see prior projects referenced in `Docs/PROJECT_PLAN.md` Section 8) — treat it as lower-risk than the rest of the architecture, not a cold start.
4. Vertical slice: a single hand-authored spline, a cart that follows it with real physics, a basic camera. Prove the core ride feel before building any editor UI.

Do not start Phase 1 (Track Editor MVP) work until the Phase 0 spline-math and block-state-machine prototypes are de-risked — see `Docs/PROJECT_PLAN.md` Section 10, "Immediate Next Steps."

## Key vocabulary (used throughout the docs and should be used consistently in code/comments)

- **Block buffer / overlap** — the safety-margin state a block holds after a train physically exits it, before it reports CLEAR. Named after the real-railway "overlap" signaling concept.
- **Heartline** — the reference line (not the rail centerline) that banking and ride-camera calculations are computed around, so felt-G through banked turns is physically correct.
- **Dispatch permissive** — the logic gate that allows a station/launch to release a train, based on downstream block clearance (and, for high-speed sections, braking-distance lookahead).
- **VFD module** — the generated control-panel element for a powered segment (lift chain, tire-drive launch): target frequency/speed, actual motor feedback, torque/current draw, ramp rate.

## When in doubt

This is a solo-developer, free/open-source project with no funding cushion — favor small, shippable, testable increments over broad refactors. If an implementation choice isn't covered by `Docs/PROJECT_PLAN.md` or this file, flag it and ask rather than assuming; the architectural decisions recorded here were made deliberately, through real back-and-forth, and getting them right mattered enough to write down.
