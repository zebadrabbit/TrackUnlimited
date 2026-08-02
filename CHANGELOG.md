# Changelog

Notable changes to this project, newest first. Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versioning follows [Semantic Versioning](https://semver.org/).

Nothing is released yet. Until the first public build (Phase 6 — see [`Docs/PROJECT_PLAN.md`](Docs/PROJECT_PLAN.md) Section 6), everything lands under **Unreleased**, and the project stays at 0.x with no compatibility promise of any kind — including for the save format.

## [Unreleased]

### Added

- **Track spline prototype** (`Prototypes/TrackSpline/`). Curvature-continuous track geometry as standalone C++17 with no engine dependency. Segments are curvature profiles over arc length — straight, constant-radius arc, clothoid transition — integrated into geometry by an exponential-midpoint scheme that keeps the frame orthonormal by construction. Heartline-relative banking, rail centreline derivation, and felt lateral/vertical G. 14 assert-based tests.
- **Block signalling prototype** (`Prototypes/BlockSignal/`). The `CLEAR → OCCUPIED → BUFFER(x) → CLEAR` per-block state machine with configurable safety overlap, plus dispatch permissive logic with wrapping multi-block lookahead. Standalone C++17, 7 assert-based tests.
- **`Docs/PHASE0_FINDINGS.md`** — what the prototypes proved (with numbers), what was disproved, the known-limitations ledger, and the measured UE5 port checklist.
- UE5 project skeleton: `TrackUnlimited.uproject`, `Config/`, and an empty `TrackUnlimited` runtime module.
- This changelog.

### Changed

- **Track representation is curvature-profile-over-arc-length, superseding the cubic Hermite / B-spline model** originally specified in `PROJECT_PLAN.md` Section 5. C² continuity becomes a property of the data rather than something fitted, and clothoids are the native case rather than special-cased. Rationale in `Prototypes/TrackSpline/TrackSpline.h`; consequences in `Docs/PHASE0_FINDINGS.md`.
- `PROJECT_PLAN.md` Section 6 no longer describes the Phase 1 editor as having "draggable handles" — that contradicted the project's first architectural constraint, and the data model has no control points to drag.
- The segment vocabulary now records that **a helix is not expressible as a linear curvature profile** and needs its own segment type. Authored naively it produces a flat tilted circle, not a helix.
- `README.md`, `CONTRIBUTING.md` and `CLAUDE.md` updated for a repo that now contains working code, including build instructions for the prototypes and the UE 5.8 engine pin.

### Fixed

- Path acceleration was resolved onto the *banked* lateral axis instead of the path's, which would have left a banked turn never quite cancelling its own lateral G — wrong in a way that throws no error.
- Roll sign was inverted: a right-hand rotation about the tangent drops the rider's right side, so `+bank` leaned *out* of a left turn.
- `FTrack::AddSegment` now rejects zero-length, negative and NaN segment lengths. A zero-length segment whose endpoint curvatures matched its neighbours could otherwise satisfy `IsCurvatureContinuous` at both joints while bridging a real discontinuity — a measured 0.76 G lateral step reported as continuous.
- `FBlockController::CanDispatch` denies a zero lookahead. It previously returned an unconditional dispatch permissive that checked nothing — verified across every state vector for N ≤ 6, including with every block occupied.
- `FBlockController::Tick` ignores zero, negative and NaN deltas. A NaN would latch a block in `BUFFER` permanently with no later well-formed tick able to free it; a negative delta grew a countdown past its configured ceiling.
- Test coverage on the vertical axis, found by mutation testing: four broken variants of the spline header passed the original suite, including one that inverted the entire track (loop apex at z = -16 instead of +16) while leaving every G reading numerically identical. All previously-surviving mutants are now killed.

### Security

- Removed a generated `AndroidFileServer` `SecurityToken` from `Config/DefaultEngine.ini` before it reached history. This is a PC-first project with no Android target, so the whole section was dead weight.
- `ModelContextProtocol` is now restricted to `Editor` targets in the `.uproject`. It is marked `NoRedist` with unrestricted runtime modules, so it would otherwise compile into a packaged Shipping build.
