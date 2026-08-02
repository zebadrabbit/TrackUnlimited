# Contributing

Thanks for taking a look at this project. It's early — pre-Phase 0 — so the most valuable contributions right now are design discussion and prototyping help, not polished PRs.

## Before you start

Read [`Docs/PROJECT_PLAN.md`](Docs/PROJECT_PLAN.md) first. It covers the vision, the five product pillars, the technical architecture (in particular the numeric/formula-driven track authoring model and the block-signaling/ride-control design), the phased roadmap, and known risks. Most "why isn't this built like X" questions are answered there.

## Ground rules

- **C++ for correctness-critical systems** (physics, block signaling), Blueprint for editor tooling, UI, and rapid iteration on non-critical systems.
- **No direct-manipulation track editing.** Track segments are parametric and numeric/formula-driven by design — this isn't a style preference, it's a core architectural decision. See the plan's "Track editor UX" section before proposing viewport-drag editing.
- **No Unreal Engine source in this repo**, per the Engine EULA. Project code and content only.
- **No real manufacturer trademarks or exact ride designs** (Vekoma, Intamin, B&M, RMC, etc.) without explicit permission. Use original/generic naming.
- **This repo does not depend on or redistribute Train and Rail System (Polygon Jelly)** or any other paid marketplace asset. It's fine as a personal reference while prototyping locally, but code/assets from it must never be committed here.

## Getting set up

1. Install Unreal Engine 5 (latest stable) via the Epic Games Launcher.
2. Clone this repo. It uses Git LFS for binary assets (`.uasset`, `.umap`, `.fbx`, textures) — install [Git LFS](https://git-lfs.com/) before cloning, or run `git lfs install` and `git lfs pull` after.
3. Open the `.uproject` file once one exists (Phase 0 hasn't produced one yet — see the Trello board's "Phase 0 — Prototype" list for current status).

## Filing issues / proposing changes

Open an issue describing the problem or idea before sending a PR for anything non-trivial — this is a solo-led project with a specific architectural philosophy, and it's better to align before you write code than after.
