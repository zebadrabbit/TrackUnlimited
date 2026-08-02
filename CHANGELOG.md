# Changelog

Notable changes to this project, newest first. Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versioning follows [Semantic Versioning](https://semver.org/).

Nothing is released yet. Until the first public build (Phase 6 — see [`Docs/PROJECT_PLAN.md`](Docs/PROJECT_PLAN.md) Section 6), everything lands under **Unreleased**, and the project stays at 0.x with no compatibility promise of any kind — including for the save format.

## [Unreleased]

### Added

- **Train physics prototype** (`Prototypes/TrainPhysics/`). 1D train motion constrained to the track, standalone C++17. Gravity is applied as an exact energy exchange rather than an integrated force, so a frictionless circuit conserves energy to 5e-14 m/s across a 333× range of timestep. Rolling resistance follows the normal load from the track's felt G, air drag is a lumped `v²` term, and one zone shape covers lift chain, tyre launch, brake run and station — they differ only in target speed and in how hard they may push or hold back. Reports fore-aft G alongside the track's lateral and vertical. 18 assert-based tests, 22 of 22 mutants killed.
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

- **Powered sections could manufacture energy on a gradient.** A zone was a per-*time* clamp on speed applied after a per-*distance* energy exchange, so on a slope the two ratcheted against each other: a lift chain rated at 3.0 m/s² — physically unable even to hold the train against a 30° grade's 4.9 — climbed 19.5 m anyway, inventing 191 J/kg out of nothing at every timestep tested. Zones are now tractive accelerations resolved before the energy step, so `MaxAccel` means what its comment always claimed. An underpowered lift now correctly fails to move the train, and an adequate one holds chain speed uphill at exactly `sin(grade)` G fore-aft.
- **A train at rest on a gradient never moved.** Position was integrated from entry speed alone, so zero speed meant zero travel, meant no height change, meant zero speed — forever. Placing a cart at the top of a drop, the vertical slice's most obvious first experiment, produced nothing at all. Position now carries the acceleration term, and a train released from rest reaches `sqrt(2g·Δz)` at every grade tested.
- Fore-aft G mixed conventions: lateral and vertical were apparent G, but tangential returned raw `dv/dt`, putting phantom load on every slope in the ride. A freely rolling train now reads exactly 0 G, as it should — gravity is accelerating it, not pushing against it.
- A train coming to rest could creep at ~1e-9 m/s forever, because `S1 - S0` is not bit-identical to the computed advance once the distance travelled is dwarfed by the position.
- `FTrain::AddZone` now rejects malformed zones rather than storing them — an inverted span silently never fires, and a negative `MaxDecel` turned a brake run into an unbounded launch.
- Path acceleration was resolved onto the *banked* lateral axis instead of the path's, which would have left a banked turn never quite cancelling its own lateral G — wrong in a way that throws no error.
- Roll sign was inverted: a right-hand rotation about the tangent drops the rider's right side, so `+bank` leaned *out* of a left turn.
- `FTrack::AddSegment` now rejects zero-length, negative and NaN segment lengths. A zero-length segment whose endpoint curvatures matched its neighbours could otherwise satisfy `IsCurvatureContinuous` at both joints while bridging a real discontinuity — a measured 0.76 G lateral step reported as continuous.
- `FBlockController::CanDispatch` denies a zero lookahead. It previously returned an unconditional dispatch permissive that checked nothing — verified across every state vector for N ≤ 6, including with every block occupied.
- `FBlockController::Tick` ignores zero, negative and NaN deltas. A NaN would latch a block in `BUFFER` permanently with no later well-formed tick able to free it; a negative delta grew a countdown past its configured ceiling.
- Test coverage on the vertical axis, found by mutation testing: four broken variants of the spline header passed the original suite, including one that inverted the entire track (loop apex at z = -16 instead of +16) while leaving every G reading numerically identical. All previously-surviving mutants are now killed.

### Security

- Removed a generated `AndroidFileServer` `SecurityToken` from `Config/DefaultEngine.ini` before it reached history. This is a PC-first project with no Android target, so the whole section was dead weight.
- `ModelContextProtocol` is now restricted to `Editor` targets in the `.uproject`. It is marked `NoRedist` with unrestricted runtime modules, so it would otherwise compile into a packaged Shipping build.
