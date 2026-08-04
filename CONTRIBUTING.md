# Contributing

Thanks for taking a look at this project. It's early — Phase 0 — so the most valuable contributions right now are design discussion and prototyping help, not polished PRs.

## Before you start

Read [`Docs/PROJECT_PLAN.md`](Docs/PROJECT_PLAN.md) first. It covers the vision, the five product pillars, the technical architecture (in particular the numeric/formula-driven track authoring model and the block-signaling/ride-control design), the phased roadmap, and known risks. Most "why isn't this built like X" questions are answered there.

## Ground rules

- **C++ for correctness-critical systems** (physics, block signaling), Blueprint for editor tooling, UI, and rapid iteration on non-critical systems.
- **No direct-manipulation track editing.** Track segments are parametric and numeric/formula-driven by design — this isn't a style preference, it's a core architectural decision. See the plan's "Track editor UX" section before proposing viewport-drag editing.
- **No Unreal Engine source in this repo**, per the Engine EULA. Project code and content only.
- **No real manufacturer trademarks or exact ride designs** (Vekoma, Intamin, B&M, RMC, etc.) without explicit permission. Use original/generic naming.
- **This repo does not depend on or redistribute Train and Rail System (Polygon Jelly)** or any other paid marketplace asset. It's fine as a personal reference while prototyping locally, but code/assets from it must never be committed here.

## Working on the prototypes (no Unreal install needed)

This is the easiest way to contribute, and it covers the three hardest parts of the project. All three prototypes are standalone C++17 with no engine dependency — a compiler is the only requirement.

```sh
cd Prototypes/BlockSignal
clang++ -std=c++17 -Wall -Wextra -o test_blocksignal test_blocksignal.cpp && ./test_blocksignal
```

Same shape for `Prototypes/TrackSpline` and `Prototypes/TrainPhysics`. Run from inside the prototype's own directory — the tests include their headers by relative path. Tests are plain `assert`s with no framework; add to the existing file and call your function from `main`. `BlockSignal` and `TrainPhysics` each hold two suites (`test_blocksignal`/`test_ridesignals`, `test_trainphysics`/`test_twotrains`); build them separately.

`BlockSignal` and `TrackSpline` finish in well under a second. `TrainPhysics` takes about 6, because it simulates tens of thousands of ticks and each one evaluates the track — pass `-O2` if you are iterating on it.

Read [`Docs/PHASE0_FINDINGS.md`](Docs/PHASE0_FINDINGS.md) before changing either header. It records what is already verified (with numbers) and, more importantly, which limitations are deliberate — several obvious-looking "fixes" have already been tested and found to make things worse.

A `ponytail:` comment marks a deliberate simplification whose ceiling and upgrade path are named right there in the comment. `grep -rn "ponytail:" Prototypes/` lists them all. Please check whether something is one of these before filing it as an oversight.

## Getting set up (full engine)

1. Install **Unreal Engine 5.8** via the Epic Games Launcher. The `.uproject` pins `EngineAssociation` to 5.8; another version will prompt you to convert the project.
2. Clone this repo. It uses Git LFS for binary assets (`.uasset`, `.umap`, `.fbx`, textures, HDRIs) — run `git lfs install` before cloning, or `git lfs install && git lfs pull` after.
3. Open `TrackUnlimited.uproject` at the repo root. Both plugins it enables ship with the engine, so there is nothing extra to install.
4. `.mcp.json` points at the maintainer's local AI-agent server. It is harmless and you can ignore it.

## Filing issues / proposing changes

Open an issue describing the problem or idea before sending a PR for anything non-trivial — this is a solo-led project with a specific architectural philosophy, and it's better to align before you write code than after.
