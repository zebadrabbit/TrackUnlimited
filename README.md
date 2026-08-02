# TrackUnlimited

A free, open-source, Unreal Engine 5 roller coaster simulator — built as a successor to [NoLimits 2](https://www.nolimitscoaster.com/), aiming for its engineering-grade precision (curvature-continuous track, real physics, accurate G-forces) with a modern, approachable interface and a real block-signaling/ride-control system neither NoLimits 2 nor Planet Coaster has attempted.

**Status:** Phase 0 — the three hardest cores are prototyped and passing; in-engine vertical slice next.

## Prototypes

The riskiest math got built first, standalone. All three are plain C++17 with no engine dependency, so they build and run **without an Unreal install** — the lowest-friction way into this codebase.

- [`Prototypes/TrackSpline/`](Prototypes/TrackSpline/) — curvature-continuous track geometry. Segments are curvature profiles over arc length (straight, arc, clothoid), so C² continuity is a property of the data rather than something fitted. Heartline-relative banking and felt G.
- [`Prototypes/TrainPhysics/`](Prototypes/TrainPhysics/) — 1D train motion along that track. Gravity is applied as an exact energy exchange rather than an integrated force, so a frictionless circuit conserves energy at any timestep. Rolling resistance that follows the normal load, air drag, and one zone type covering lift, launch, brake and station.
- [`Prototypes/BlockSignal/`](Prototypes/BlockSignal/) — the `CLEAR → OCCUPIED → BUFFER(x) → CLEAR` block state machine and dispatch permissive logic.

```sh
cd Prototypes/TrackSpline
clang++ -std=c++17 -Wall -Wextra -o test_trackspline test_trackspline.cpp && ./test_trackspline
```

What they proved, what they disproved, and what is still unknown: [`Docs/PHASE0_FINDINGS.md`](Docs/PHASE0_FINDINGS.md).

## What makes this different

- **Numeric/formula-driven track authoring.** No click-drag-place viewport editing — track segments are defined by typed parameters, not sculpted by hand.
- **Real block signaling.** Occupancy + safety-overlap buffer state machine (`CLEAR → OCCUPIED → BUFFER(x) → CLEAR`), automatic dispatch with manual override, safety interlocks that apply identically in both modes.
- **A generated ride control panel.** Not a cosmetic prop — a control panel built directly from each coaster's own block/segment data, with VFD modules for powered segments, and a visible sensor → indicator → actuator causal chain.
- **PC flatscreen first.** VR is a deliberate later phase, not a day-one requirement.

## Project links

- Full project plan: [`Docs/PROJECT_PLAN.md`](Docs/PROJECT_PLAN.md) — vision, market context, technical architecture, roadmap, risks.
- Phase 0 findings: [`Docs/PHASE0_FINDINGS.md`](Docs/PHASE0_FINDINGS.md) — verified results, known limitations, UE5 port checklist.
- Changelog: [`CHANGELOG.md`](CHANGELOG.md).
- Project board: [Trello](https://trello.com/b/Uzqm38o7/coaster-sim-nolimits-successor).

Three places, three jobs: the repo docs are the **design reference**, Trello is **live task status**, and GitHub Issues is the **inbound channel** for bugs and proposals.

## License

MIT — see [`LICENSE`](LICENSE). Note: this repository contains project code and content only, **not** Unreal Engine source. You'll need your own Unreal Engine installation (via Epic Games Launcher or your own linked GitHub access) to build this project — see Epic's [Unreal Engine EULA](https://www.unrealengine.com/eula/unreal) for why.

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md).

## Working with an AI coding agent

[`CLAUDE.md`](CLAUDE.md) has onboarding instructions for Claude Code (or any other AI coding agent) working in this repo — the non-negotiable architectural constraints, current phase and next actions, and key vocabulary. Read it before generating code here.
