# TrackUnlimited

A free, open-source, Unreal Engine 5 roller coaster simulator — built as a successor to [NoLimits 2](https://www.nolimitscoaster.com/), aiming for its engineering-grade precision (curvature-continuous track, real physics, accurate G-forces) with a modern, approachable interface and a real block-signaling/ride-control system neither NoLimits 2 nor Planet Coaster has attempted.

**Status:** Pre-Phase 0 — planning complete, prototyping not yet started.

## What makes this different

- **Numeric/formula-driven track authoring.** No click-drag-place viewport editing — track segments are defined by typed parameters, not sculpted by hand.
- **Real block signaling.** Occupancy + safety-overlap buffer state machine (`CLEAR → OCCUPIED → BUFFER(x) → CLEAR`), automatic dispatch with manual override, safety interlocks that apply identically in both modes.
- **A generated ride control panel.** Not a cosmetic prop — a control panel built directly from each coaster's own block/segment data, with VFD modules for powered segments, and a visible sensor → indicator → actuator causal chain.
- **PC flatscreen first.** VR is a deliberate later phase, not a day-one requirement.

## Project links

- Full project plan: [`Docs/PROJECT_PLAN.md`](Docs/PROJECT_PLAN.md) — vision, market context, technical architecture, roadmap, risks.
- Project board: tracked in Trello (ask the maintainer for the link, or see your own board if this is your fork).

## License

MIT — see [`LICENSE`](LICENSE). Note: this repository contains project code and content only, **not** Unreal Engine source. You'll need your own Unreal Engine installation (via Epic Games Launcher or your own linked GitHub access) to build this project — see Epic's [Unreal Engine EULA](https://www.unrealengine.com/eula/unreal) for why.

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md).

## Working with an AI coding agent

[`CLAUDE.md`](CLAUDE.md) has onboarding instructions for Claude Code (or any other AI coding agent) working in this repo — the non-negotiable architectural constraints, current phase and next actions, and key vocabulary. Read it before generating code here.
