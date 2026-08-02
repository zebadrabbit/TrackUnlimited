# Phase 0 Findings

Phase 0 is a de-risking phase, so its real deliverable is *knowledge*, not code. This page is where that knowledge lives: what the three standalone prototypes proved, what they disproved, and what is still unknown. The prototypes themselves are small; the reason to trust them is on this page.

Last updated: 2026-08-02.

## The prototypes

All four are plain C++17 with no engine dependency, so they build and run without an Unreal install. That is deliberate — this is the lowest-friction way to work on the hardest parts of the project, and it keeps the math honest by preventing it from quietly depending on engine behaviour.

| Prototype | Proves | Files |
|---|---|---|
| Track spline | Curvature-continuous track geometry, clothoid transitions, heartline-relative banking, felt-G | [`Prototypes/TrackSpline/`](../Prototypes/TrackSpline/) |
| Train physics | Energy-conserving 1D motion along the track, load-dependent resistance, powered/braked zones, fore-aft G | [`Prototypes/TrainPhysics/`](../Prototypes/TrainPhysics/) |
| Block signalling | `CLEAR → OCCUPIED → BUFFER(x) → CLEAR` occupancy plus dispatch permissive with lookahead | [`Prototypes/BlockSignal/`](../Prototypes/BlockSignal/) |
| NL2 CSV | Reading and writing NoLimits 2's sampled-frame spline export, so real layouts can be driven through the model and our tracks opened in NL2 | [`Prototypes/NL2Csv/`](../Prototypes/NL2Csv/) |
| NL2 telemetry | The NL2 telemetry wire protocol, and a recorder that correlates a live ride against an imported track — the route to NL2's own G and speed numbers | [`Prototypes/NL2Telemetry/`](../Prototypes/NL2Telemetry/) |

```sh
cd Prototypes/TrackSpline
clang++ -std=c++17 -Wall -Wextra -o test_trackspline.exe test_trackspline.cpp && ./test_trackspline.exe
```

Same shape for the other four. Run from inside the prototype's own directory — the test includes its header by bare name.

Keep the `.exe` on the output name. `-o test_trackspline` emits an **extensionless** binary: Git Bash runs it fine, but PowerShell and Explorer do not recognise it as executable and pop the "how do you want to open this file?" dialog instead. The `.gitignore` covers both spellings.

`NL2Telemetry` additionally needs a socket library and is the only prototype that requires NoLimits 2 to be running:

```sh
cd Prototypes/NL2Telemetry
clang++ -std=c++17 -Wall -Wextra -o record.exe record.cpp -lws2_32
./record.exe --seconds 90
```

The design rationale for the spline representation is in the header comment at the top of [`TrackSpline.h`](../Prototypes/TrackSpline/TrackSpline.h), deliberately kept next to the code so it cannot drift from it. This page does not repeat it.

## Decision: curvature profiles, not Hermite splines

`PROJECT_PLAN.md` Section 5 originally specified "cubic Hermite or B-spline segments with explicit control over first- and second-derivative continuity". The prototype does something equivalent but simpler, and the plan has been updated to match.

A segment is a **curvature profile over arc length** — curvature varies linearly across it, so a straight is `k = 0`, a constant-radius arc is `k = const`, and a clothoid is `k` linear. Geometry is produced by integrating a moving orthonormal frame along that profile.

The payoff is that C² continuity stops being something you fit for and becomes a property of the data: if curvature is continuous across a joint, the geometry is. Nothing is solved, nothing is fitted, and the clothoid is the *native* case rather than a special-cased transition. It also serialises directly to the typed-value segment list the numeric editor needs.

This has one real cost, recorded under limitations below: there is no endpoint targeting.

## What was proven

Verified by independent reimplementation against closed-form references, not by the prototypes' own tests agreeing with themselves.

**Geometry and accuracy**

- The integrator is **exactly second order**. Ten successive step halvings on a quarter circle give error ratios converging to 4.0000, observed order 2.000.
- At the shipped 0.01 m step, position error is **bounded, not accumulating**: worst 0.000417 mm anywhere over 500 m at `k = 1/20`. On a realistic 615 m / 23-segment layout (station, eased lift, crest, pull-out, R=8 vertical loop, two banked turns, brake run) the worst error is 0.0025 mm.
- Error growth over distance is **linear**, about 0.027 mm per km — 0.055 mm over a 2 km layout. Steel track manufacturing tolerance is of order 1 mm, so there are roughly three orders of magnitude of margin.
- **Clothoid geometry matches the Fresnel closed form**: worst 0.000085 mm (R=100, L=80), 0.000282 mm (R=30, L=24), 0.003021 mm at a punishing R=3, L=6.
- **Arc length is exactly nominal** — 500.000000001 m over a nominal 500 m. A physics model integrating speed against `S` sees no drift. The per-step chord deficit is cancelled because the scheme applies the exact turn angle, which instead inflates the realised radius by ~2e-7 m; the effect on felt G is around 1e-8 G.
- **Orientation carries zero discretisation error** on planar segments (heading error at roundoff, 1e-15 rad), because midpoint sampling integrates a linear curvature ramp exactly. All discretisation error lives in position; the rider frame is effectively exact.
- The frame **stays orthonormal without re-orthonormalisation**: `|T|-1` is 5.4e-11 after a million steps (10 km). `Dot(T, L)` is exactly zero at every length tested.
- Full vertical loops **close exactly** — 0.000000 mm closure error at R = 4, 6, 8 and 12 m.

**Physics**

- The apparent-gravity convention is correct: specific force `f = a - g_vec`, so adding `(0, 0, +g)` is right. A banked turn at `atan(v²/gR)` gives lateral **exactly 0.0** and vertical `sqrt(1 + (v²/gR)²)`, agreeing with an independently derived closed form to 1.5e-13.
- Pitch-axis G is exact to 9 decimal places: valley and crest match `1 ± v²k/g`, the airtime threshold sits at `k = g/v²`, and the top of an R=8 loop at 20 m/s reads +4.098581 G — correctly reporting the rider pressed *into* the seat while inverted, with the apex at z = +16.000001 m against a true 2R = 16.
- **Banking rotates the rider frame around the heartline without perturbing the path, exactly.** A 0 → 2π barrel roll on a straight leaves the heartline bit-identical, and on a curved segment the max difference between rolled and unrolled paths across 201 samples is 0.000e+00 m.
- The heartline is **not conflated with the rail centreline**. `HeartlineHeight` never enters the G calculation. Through a banked quarter turn the rail's horizontal radius measures `R + h·sin(bank)` exactly; using that rail radius for the rider would misreport lateral G by 0.0135 g where the correct heartline answer is 0.
- The path frame is **exactly parallel-transported** (rotation-minimising), max 1.5e-08 rad/m. This is what makes roll a clean independent parameter.
- Omitting tangential acceleration is **exactly harmless** for the two reported axes — `dv/dt` acts along `T`, and `T·L = T·U = 0` exactly, even through a full inversion. The lateral and vertical numbers are complete for an accelerating or braking train.

**Train physics**

- **Gravity really is exact.** A frictionless loop returns to its launch speed with an error of 5e-14 m/s, and that figure is flat across a 333× range of timestep — 1/15 s reads the same as 1/5000 s. The per-step height deltas telescope exactly, which is also why `EvaluateAt`'s documented ~1e-9 m non-smoothness never reaches the speed.
- Constant-acceleration motion is reproduced **exactly**, not approximately: carrying the acceleration term in the position update makes the energy form algebraically identical to `v1 = v0 + a·dt` on straight track, so a zone lands on its target speed to 1e-9 rather than hunting around it.
- **The normal load is roll-invariant, and measurably so.** A banked turn and the same turn unbanked cost bit-identical speed (difference 0.000e+00) — banking rotates the rider around the heartline, it does not change how hard the wheels press on the rail. Both cost 1.6515× a straight of equal length, against `sqrt(1+r²)` = 1.688 at entry.
- `NormalG = sqrt(lateral² + vertical²)` matches an independent world-space computation of the wheel normal load to 1.2e-13.
- The fore-aft convention is correct including the braking sign: a freely rolling train reads **exactly 0 G** on any slope, and holding chain speed up a 30° grade reads +0.5000 G — `sin(30°)`, falling out of the model rather than being asserted.
- Closed forms reproduced: drag-only decay `v = v0·e^(-k·x)`, friction stopping distance `v0²/(2·Crr·g)`, loop apex speed `sqrt(v0² - 4gR)`, and release-from-rest reaching `sqrt(2g·Δz)` at 2°, 11.5° and 30° grades.
- **22 of 22 mutants killed.** The first pass left 13 of 16 alive — every survivor clustered in the zone controller and the normal-load shape, because all the zone tests ran on level track where the competing formulations coincide.

**Signalling**

- All tests pass clean under `-Wall -Wextra`. Class invariants held across **1,200,000 randomly fuzzed API calls** over 20,000 controllers: zero violations of the buffer invariants, of drain-to-CLEAR, or of `CanDispatch` against an independent reimplementation.
- Driven correctly it is sound end to end: a simulated 10-minute run (6 blocks, 3 trains, 3 s overlap, 1/60 s tick, every move gated on `CanDispatch`) produced **453 moves with zero violations**, with an independent audit confirming no two trains were ever co-resident.
- Buffer countdown drift is a non-issue: 2.4M ticks accumulate -2.7e-07 s. The visible effect is at most one extra tick, and a buffer can only expire on a tick boundary anyway.

**NL2 CSV import/export**

NoLimits 2 publishes a [tab-separated spline export](https://nolimitscoaster.com/nolimits2/help/pages/fileformats.html) — position plus front/left/up unit vectors, one row per sample. That is an `FTrackFrame` sequence, so it reads with a `strtod` loop, and it is the cheapest route to the two calibration unknowns at the bottom of this page. The writer exists so the reader is testable with no NL2 install, and so our tracks can be opened in NL2 for comparison.

- **The axis map is a rotation, not a handedness flip.** NL2 is right-handed with +Y up; `(x, y, z) → (x, -z, y)` has determinant +1, so cross products survive untouched and `Front × Left = Up` lands on `Tangent × Lateral = Up` with no sign correction. This is *not* the UE5 port's `-M(Lateral)` rule — that one exists because UE is left-handed. Applying it here would mirror the whole track into geometry that still looks self-consistent.
- **Round-trip fidelity on a 529.8 m / 25-segment layout** (30° lift to +41.7 m, crest, drop, pull-out, R=8 loop, two 45°-banked turns, brake run) sampled at 0.25 m: total length off by **4.1e-06 m**, worst position error **2.6 mm**, worst felt-G error **7.5e-03 G**, rms felt-G error **6.8e-04 G**.
- **Single elements reconstruct far tighter**, because they have no curvature-slope kink to round off: a straight to **6.0e-13 m**, a 45°-banked R=25 arc to **2.7e-07 m** and **6.3e-08 G**, an R=8 loop to **1.0e-06 m** with apex at **+16.000002 m** and **1.9e-11 m** closure.
- **The physics agrees on both tracks.** A train driven over the original and over the reconstruction diverges by at most **1.7e-03 m/s** across a full 2,446-tick circuit, finishing at 23.8055 vs 23.8056 m/s. That is the check the fixture actually exists for.
- **Roll unwraps through the branch cut**: a full 2π barrel roll recovers to **2.7e-10 rad** rather than snapping sign mid-inversion.
- **Error converges with sample spacing** — 1.0 m → 0.1 m takes worst position from 4.2e-02 m to 4.2e-04 m and rms G from 1.4e-02 to 1.8e-04, both roughly second order. Convergence is the assert that separates discretisation from a bug: a sign or frame error would not shrink with spacing.
- **Verified against a real NL2 export**, not only against our own writer. A 466-sample circuit exported at NL2's default 0.5 m node spacing parsed with worst basis-vector length error **6.5e-07**, worst orthogonality **1.1e-06**, and worst `|Front × Left − Up|` **1.6e-06** — so **the handedness reading is confirmed on real data**, not just assumed from the docs. It reconstructed to 232.933 m in 466 segments, curvature-continuous, closing on itself to **12 mm** over the full circuit.
- **NL2 opens a circuit export wherever its spline starts, not at the station.** The first real export began **22.2° into a drop**, which `FTrack` cannot represent — its start frame is pinned level, so the whole track would tilt against gravity. On a closed circuit this costs nothing to fix: rotate the sample list to begin at a level sample (that export had 31 of them, one at exactly 0.000°). Only the arc-length origin moves. An *open* track with a pitched start is still refused, because there is nowhere to rotate to.
- **Two bugs were found by measurement, not by inspection**, and both are worth knowing about because the naive form of each looks right:
  - Recovering the turn angle with `acos(Dot(T0, T1))` is ill-conditioned at the small angles between consecutive samples — `dθ/d(cos) = -1/sin θ` amplifies the last digits of the file's rounded decimal text by `1/θ`. On an R=25 arc at 0.2 m spacing it left a 4.9e-07 1/m curvature error and 3.0e-05 m of endpoint drift. `atan2(|T0 × T1|, T0 · T1)` costs the same, is well conditioned at both ends, and needs no clamp: **110× better**, down to 2.7e-07 m.
  - A writer that accumulates `S += Spacing` drifts, and the drift lands on the last row: a final clamped step of ~1e-12 m emits a row that rounds to the same text as its predecessor, which a reader can only see as two coincident samples. Walking by row index over uniform fractions of the total has neither problem.

**NL2 telemetry**

The CSV spline export is a **Professional-licence** feature (NL2 Editor → `Professional` tab → `Export Track Spline`, default 0.5 m node spacing). The [telemetry interface](https://nolimitscoaster.com/nolimits2/help/pages/telemetry.html) is **not** — it ships in the standard edition (`Main Menu → Setup → Others`, or `--telemetry`, default port 15151) — so it is the route to NL2's own numbers that works regardless of licence tier.

- **Protocol verified against a running copy.** Framing is `'N'` + type(2) + requestId(4) + dataSize(2) + payload + `'L'`, all multi-byte fields big-endian; telemetry is a 76-byte payload. The version handshake round-trips against NL2 **2.6.8.1**, which is what proves the framing before any recording is trusted.
- **The G-force reference frame is undocumented.** The spec gives offsets and units but never says whether G is world or car-local, nor whether it is the force on the rider or the acceleration of the car. So `FNL2Telemetry` leaves those three floats **raw and unmapped** — mapping them as world axes would bake in the very assumption being tested — and `IdentifyGravityAxis` decides it from data instead: a train under 1 m/s feels exactly one g and nothing else, so whichever component parks near ±1 *is* vertical, and its sign says which sense NL2 reports. Position **is** mapped, with the same `FromNL2` the CSV reader uses; if those two ever drift apart nothing correlates.
- **Correlation is on world position, not arc length.** S origins differ between the two sides (the importer rotates a circuit to start level) and the reconstruction is yaw-rotated and translated besides. NL2 world position is the only thing both genuinely share.
- **The tool refuses to conclude from noise.** An early run recorded a stationary game and cheerfully reported "lateral sign AGREES" by comparing our −0.0189 against NL2's +0.0000. A sign comparison now requires both readings above 0.15 g *and* a position match within 2 m, and a recording with no motion is rejected outright. A confident verdict drawn from zeros is worse than no verdict.
- **Bound by the wall clock, not a poll count.** Replies return in well under a millisecond on loopback, so an early poll-budget version burned through "90 seconds" in about half of one and recorded a single sample.

## Validation against NoLimits 2

A 90-second recording of a full lap on a real NL2 circuit (233 m, unbanked, 12.4 m/s top speed), 5,666 samples, correlated against the same track imported from its CSV export. This is the first time anything in this project has been compared to an independent simulator rather than to a closed form.

**The lateral-G sign question is settled, and the documentation reading was wrong.**

- NL2 reports G in the **car's local frame**, decided from data rather than docs: on the lift the gravity axis reads **0.909** against a car-local prediction of cos(pitch) = 0.914 and a world-frame prediction of 1.000.
- It reports the force felt **by the rider**, same sense as our `FeltG`: the gravity axis sits at **+0.9972** at rest over 1,465 samples, next-largest component 0.0028.
- Sign agreement is **627 / 627 samples (100%)** wherever NL2's lateral exceeds 0.3 g. The prototype's convention was right and the documentation reading that suggested otherwise was wrong. **No negation is needed anywhere.**

**Magnitude agreement is far better than the sign check required.** Over 4,298 moving samples, NL2 minus ours:

| axis | mean difference | rms | worst |
|---|---|---|---|
| vertical | −0.0008 g | 0.0089 g | +0.105 g |
| lateral | +0.0019 g | 0.0299 g | +0.271 g |

At the peak lateral event — a 6.35 m radius unbanked turn at 12.35 m/s — NL2 reads **+2.3521 g** and the model reads **+2.3979 g**, a 1.9% difference. The geometry model, the heartline banking model and `FeltG` all agree with a mature commercial simulator to a few hundredths of a g across a whole lap.

**Calibration against NoLimits 2** — first real measurement of the two tuning knobs, fitted over the 1,700 coasting samples between chain release (S = 47.0 m) and the brake run (S = 225.4 m):

| | `RollingResistance` | `DragK` | rms speed error |
|---|---|---|---|
| shipped defaults | 0.006 | 0.00045 | **0.998 m/s** |
| fitted to this ride | 0.0046 | 0.00144 | **0.564 m/s** |

The defaults run this ride consistently *fast* — 11.0 m/s against NL2's 10.6 at mid-circuit, and 11.0 against 9.4 by the end, i.e. the error grows with distance travelled, which is what too little total resistance looks like. See "Still unknown" for why these fitted numbers are **not** being adopted as defaults.

**Test suites are discriminating.** This was checked by mutation rather than assumed: deliberately broken variants of the headers were generated and run against the suites. The first pass found four surviving mutants in the spline suite — including one that put the loop apex at z = **-16** instead of +16 while leaving *every* G reading numerically identical, because the frame flipped in step with the curvature. An entirely inverted track would have reported perfectly self-consistent G. Six asserts closed that gap, and all six previously-surviving mutants are now killed.

## What was disproved

**The helix is not expressible with the current segment vocabulary.** This is a representation gap, not a missing parameter, and the plan's segment list (straight / arc / clothoid / helix) was wrong to imply otherwise.

Authored the obvious way — constant radius plus constant climb — you do not get a bad helix, you get a **tilted flat circle** that returns to its starting height after a full turn (out-of-plane deviation 1.9e-12 m over 300 m; it really is planar). "Constant climb" is *zero* pitch curvature at a fixed pitch angle, yawing about the **world** vertical, not the body up-axis.

A true helix comes out of this integrator only from yaw and pitch of constant magnitude rotating at the torsion rate (`Yaw = k·cos(τs)`, `Pitch = k·sin(τs)`), which a linear ramp cannot express. Approximating it with the available ramps costs roughly one segment per metre. Phase 1 needs either a non-linear profile shape or a world-vertical yaw term in the integrator.

## Known limitations

Deliberate, measured, and written down. A bounded limitation that is recorded is a Phase 0 success, not debt. Shortcuts also carry a `ponytail:` comment in the code naming the ceiling and the upgrade path — `grep -rn "ponytail:" Prototypes/` lists them.

**Track spline**

- **No endpoint targeting.** You author curvature and arrive wherever you arrive; closing a circuit back onto the station is an inverse solve that a control-point model would have given free. Measured: a symmetric oval closes at 0.0000 m, but easements of 20 m vs 8 m leave a 0.93 m gap and radii of 30 m vs 45 m leave a 29.63 m gap — heading closes exactly in every case. This is the one real cost of the representation, and it is a Phase 1 scope item.
- **Roll-rate steps are invisible to the continuity check.** `IsCurvatureContinuous` checks roll *value*, not rate. The banking pattern the header itself endorses — ramp bank over the clothoid, hold through the arc — steps roll angular velocity by 53.7 °/s at four joints per banked turn, and both the check and the G readout report nothing, because felt G has no roll-rate term. A roll-rate metric belongs in the Phase 1 editor.
- **The rider is a point at the heartline**, so roll-rate terms vanish by construction. Correct for the heartline and the right simplification, but the omitted ω²r at 0.5 m above it is 0.031 g at 45 °/s, 0.126 g at 90 °/s, and 0.503 g at 180 °/s. A fast barrel roll that reads smooth at the heartline is half a G of head snap.
- **`Roll = 0` does not mean level with the horizon** on non-planar track. The frame is exactly rotation-minimising, so climb 45° / left 90° / descend 45° at roll 0 throughout exits at -54.736° of world bank (= -arccos(1/√3)); three right angles gives exactly -90°. The geometry and its G are correct — but "keep RMF and document it" vs "add a world-referenced roll mode" is a real Phase 1 decision, not a bug.
- **`EvaluateAt` is O(track length)**, re-integrating from the start on every call — about 0.2 ms on a 1 km track, so a 0.1 m mesh pass over that track is ~2.3 s. The cached arc-length sample table is the named upgrade path. Nothing needs it yet.
- **Do not finite-difference `EvaluateAt`** to recover direction or speed. Step counts are re-discretised per call, so position is not smooth in `S` below ~1e-9 m and a difference quotient stops converging under h ≈ 1e-4. Use the returned `Tangent`, which is exact to 1e-15 rad.
- **Authored values are not validated beyond segment length.** `AddSegment` rejects zero, negative and NaN lengths — the one shared choke point — but `MakeArc(L, 0)` still produces NaN curvature, and a NaN in any field passes `IsCurvatureContinuous` because `fabs(x) > tol` is false for NaN. This is intentional: validation belongs at the Phase 1 editor boundary, where one check covers every way to author a bad segment. Do **not** "fix" `MakeArc(L, 0)` by clamping it to a straight — that was tested and produces a plausible 1.00 G with a clean continuity pass, which is strictly worse than an obvious NaN.
- **At an exact joint, the ending segment supplies roll and curvature** — and that is not reliable in floating point. With authored lengths that do not sum exactly, roughly a quarter of joint queries return the following segment's values instead. It only bites on curvature-discontinuous data. Sample either side of a joint rather than on it.
- **`MaxStep` is a hardcoded 0.01 in metres.** Nothing breaks today, but a port that hands it centimetre lengths gets 100× the steps, with no error and no symptom except being unusably slow. The suite passes at `MaxStep` up to 0.5, so there is ~50× of headroom if performance ever matters.

**Train physics**

- **The train is a point at the heartline; it has no length.** This is the one omission most likely to be *felt*. A real train is 10–15 m long, and its speed over a crest is governed by the whole train's centre of mass, not by the lead car's position — which is exactly why a car at the back gets thrown over an airtime hill harder than the front. Point-mass coaster sims are known to feel wrong for this reason. Defensible for Phase 0 (it does not change the shape of the model — it becomes an average over sample points along the train), but it should not survive Phase 2's "feels right to an NL2 veteran" bar.
- **A train that runs out of energy stops dead rather than rolling back.** Reversal needs a signed velocity through the whole model and the block system has to hear about it. A valley stall is a design error to surface, not a state to simulate — but a real editor will eventually want to show the roll-back.
- **A zone shorter than one step's travel is skipped entirely.** At 40 m/s and 1/60 s that is any zone under 0.67 m. Fine for lift hills and brake runs; a trap for a short trim brake.
- **The energy exchange is exact; the *path* integration is not.** Position carries the acceleration term, which makes it exact under constant acceleration, but gravity varies along a curve, so on curved track there is a residual O(dt²) position error. Energy stays exact regardless, because it is computed from the heights actually visited.
- **`RollingResistance` and `DragK` are tuning knobs, not measurements.** `DragK = 0.00045` is `0.5·ρ·Cd·A/m` evaluated for a loaded 7-car steel train (~8000 kg, CdA ≈ 5.5 m²), and `RollingResistance = 0.006` is a plausible steel-on-steel figure. Both need calibrating against a reference ride before any G trace is quoted as accurate. The physical world needs tuning a minimal model cannot see.
- **The physics and the signalling do not talk to each other yet.** `FTrain` knows nothing about `FBlockController`, so nothing yet trips block occupancy from a train's position, and nothing gates a station release on a dispatch permissive. That wiring is the Phase 3 job, and it is where the two prototypes finally meet.

**Block signalling**

- **One train per block, by definition.** There is no state for "two inside", so after a signalling violation the first train's exit runs `BUFFER → CLEAR` while the second is still inside, the second train's exit gets a 0 s overlap instead of the configured one, and further violations go unreported for the rest of that occupancy. The damage is bounded and self-healing — one injected violation followed by 20 clean laps gives 3 disagreements, all within 3 steps, and none afterwards. This is correct scoping: a violation is an E-stop condition in real ride control, not a recoverable state.
- **Block topology is a hard-coded ring** (`index + 1 mod N`). Verified correct for a circuit. On a shuttle or a transfer spur the last block's permissive consults the first, and the failure direction is fail-*open*. A `bool bIsCircuit` when a non-ring layout first exists; a successor graph only once transfer tracks do. Phase 3.
- **Dispatch mode (automatic vs manual) is not implemented.** Deliberate — it sits above this class and does not change its shape, since the permissive logic is identical in both modes by design.

**NL2 CSV**

- **It is a fixture path, not an authoring path**, and must not become one. An imported track is thousands of derived micro-segments; the original vocabulary (which parts were straight, arc, clothoid) is gone and cannot be recovered from samples. Nobody edits that numerically, so this does not soften constraint #1 in `CLAUDE.md`.
- **The felt-G error is not uniform — it lives at curvature-slope kinks.** Per-interval curvature is averaged onto samples with a (1,2,1)/4 kernel, which rounds off a corner in the curvature profile. The worst point in the test layout is the entry to the R=8 loop, where the ramp's slope drops to zero across one joint; there the error converges only *first* order in spacing while everything else is second order. At 0.25 m under 1% of sampled points exceed 0.005 G. The exactly-conservative alternative (`End[i] = 2·Interval[i] - Start[i]`) is continuous too but has gain -1 per step, so noise becomes a sawtooth that never decays — smoothing is the right trade on sampled data.
- **A pitched start is rejected, a banked start is not.** `FTrack` pins its own start to `T=+X, L=+Y, U=+Z`, so a reconstruction is the original translated and yawed — harmless, since neither touches gravity — but only while the first tangent is horizontal. A pitched start would be rotated flat, tilting the entire track relative to gravity with no visible symptom, and nothing downstream could recover it. Roll does not move the tangent, so a banked start is fine and the opening roll is measured against world up rather than assumed zero.
- **A reconstruction is dense.** The 529.8 m layout at 0.25 m becomes 2,120 segments, and `AdvanceFrom` → `Locate` is an O(N) linear scan per call. Immaterial at 60 Hz — the full-circuit physics test runs in well under a second — but it is the case that would first justify the cached arc-length sample table `TrackSpline.h` already names. Sample at 0.25–1 m unless something needs finer.
- **`.nl2park` and `.nl2elem` are not read.** `.nl2park` is binary and undocumented. `.nl2elem` is XML carrying NL2's own vertex + roll-point representation, which would have to be fitted onto the curvature model anyway. CSV is already sampled frames, so it is strictly less work for strictly more coverage.
- **Which line NL2 exports is unverified.** See "Still unknown" below.

## Vertical slice

`Source/TrackUnlimited/TUCoasterRide.cpp` is the first code to take the prototypes into the engine. `TrackUnlimited.Build.cs` puts `Prototypes/` on the public include path and includes the headers **directly** rather than copying them, so the standalone assert suites test exactly the code that ships. The class is a thin shell: it converts units and handedness at the boundary and does nothing else of consequence.

- **Builds clean** against UE 5.8.1 (MSVC 14.50) with **zero warnings**. The engine-free headers drop into a UE module without name collisions — worth noting given how many short identifiers they define (`FTrack`, `Dot`, `Cross`, `Length`). `FTrackFrame` is deliberately not called `FFrame`, which *would* have collided with the Blueprint VM's type.
- **The handedness flip is verified in the shipped conversion, not just in principle.** Across 2,000 frames of the slice's own 543.7 m layout, the UE basis built by `ToWorldRotation` has worst `|Fwd × Right − Up|` of **1.15e-12**, orthogonality 2.9e-15, and `||Fwd| − 1|` of 5.9e-13 — the same order as the 1.6e-13 measured for the correct rule below, and nowhere near the 2.000000 that the wrong one produces.
- **The hand-authored layout reproduces its design figures exactly**: 16 segments, 543.7 m, curvature-continuous, 34.9 m lift, 100.9 km/h, vertical G +0.70 to +4.25, peak lateral +0.36, and +1.05 G over the loop apex while inverted. Authored numerically as an ordered list of typed segment parameters, with no viewport dragging anywhere — the authoring model the project actually intends.
- **It runs.** Placed in a level and played in-editor, `BuildTrack` logs `16 segments, 543.7 m, curvature-continuous=yes` — identical to what the standalone harness produces from the same code. That closes the loop the whole prototype strategy was built around: the same header, tested in a second without an editor, produces the same geometry inside the engine. The heartline and rail centreline both draw, the ride camera sits on the heartline, and the camera visibly climbs the lift. No errors or warnings from the module, the pawn, or possession.
- **The on-screen telemetry works** — speed, the three G axes, and the brake-run state all read out during a lap, confirmed by eye. Worth recording *how* that was confirmed, because it cost time: screen messages cannot be verified from a screenshot. `UEngine::DrawOnscreenDebugMessages` gates on `!GIsHighResScreenshot && !GIsDumpingMovie && GAreScreenMessagesEnabled`, so every capture path suppresses them and a blank capture proves nothing either way. Check this one on a human's screen. (`AddOnScreenDebugMessage` with `TimeToDisplay = 0.f` is fine, incidentally: the message is drawn *before* the expiry test, so a per-frame re-add persists correctly.)
- **The level is saved** as `/Game/Maps/VerticalSlice`, with the ride pawn at z = 1050 cm. That height is not cosmetic: the layout's lowest point is **−8.498 m**, so anything lower puts everything from the loop onward under the floor.
- **Not yet proven: ride feel.** Geometry and numbers are verified and the developer's first impression riding it was positive, but "feels right to an NL2 veteran" is a judgement no log line can make and no single lap settles. The point-mass train (no length) is the known limitation most likely to show up here — and the NL2 calibration above now gives it independent supporting evidence.

## UE5 port checklist

Measured, not guessed. Both headers are the canonical design to **port**, not references to reimplement.

1. **Handedness must flip, and it is not just a sign.** With `M(x,y,z) = (x,-y,z)`, taking UE `Right = -M(Lateral)` gives a residual of 1.6e-13 against `Fwd × Right = Up`. Using `M(Lateral)` directly as UE's Y gives **2.000000** — exactly inverted. Concretely: the end of a +R=30 left arc sits at prototype `Y = +6.4234 m`, which lands 6.4234 m to the *right* in UE if converted naively.
2. **Units.** Prototype is metres/radians/seconds; UE is centimetres. Convert at the port boundary, never inside the math — and remember `MaxStep`.
3. **Build the `FQuat` from the three basis vectors**, not from `FRotator` angles. The frame is already exactly orthonormal; going through Euler angles reintroduces error and gimbal cases the integrator specifically avoids.
4. **`std::vector` → `TArray`**, `std::size_t` → `int32`. Keep block indices unsigned or validate once at a single entry point: the current unsigned arithmetic is fail-*safe* (a negative lookahead wraps to `SIZE_MAX` and lands on the deny guard), and `int32` loses that property.
5. **Keep these as plain C++ structs**, not `UObject`s. Nothing here needs reflection, and `UPROPERTY` on the hot path costs for nothing.
6. **Live Coding cannot add or remove members on an actor class — full rebuild only.** Measured the hard way: adding one plain member to `ATUCoasterRide` and hot-patching with Ctrl+Alt+F11 logged `Could not find existing class TUCoasterRide … assuming new or modified class`, `Re-instancing TUCoasterRide after reload`, and a warning that *"data type changes may cause packaging to fail"* — then Play fatal-crashed ten seconds later with `Cast of Object /Script/CoreUObject.Default__Object to Actor failed`, an entirely engine-side stack with no project frames in it. Changing a function body is fine; changing the layout is not. Two things make this hard to diagnose: the crash names no project code, and the module DLL's timestamp does **not** move, because Live Coding writes `*.patch_0.*` alongside it. Close the editor and rebuild.
7. **`FTrain` holds a `const FTrack&`.** That is fine in a standalone prototype where both live on the stack, and a hazard in UE where the track will likely be owned by a GC'd object. Decide the ownership model at port time — a weak pointer, or the track living inside the same component — rather than letting a dangling reference happen.

## Still unknown

Phase 0 did not touch these, and no claim on this page covers them.

- **Train length.** See the limitation above — the single most likely source of "this doesn't feel right" once there is something to ride.
- **Distance-based block overlap.** Only the time-based overlap exists; the distance form needs train position and braking distance from the physics model.
- **Calibration is now measured once, not solved.** See "Calibration against NoLimits 2" above: the shipped `RollingResistance` / `DragK` run this ride about 1.0 m/s fast, and a fit halves that. But it is **one train on one 233 m layout**, the fitted `DragK` lands 3.2× above its physically derived value, and there is residual structure in the fit — so the numbers are a data point, not a new default. Repeat on a second, longer, faster ride before changing anything shipped.
- **Whether train length explains the residual.** The fit needed far more drag than physics suggests, which is what modelling a 10–15 m train as a point at the heartline would look like: the real train's speed over a crest is governed by the whole train's centre of mass. This is the first *evidence* for the train-length limitation rather than an argument from first principles, but it is not yet separated from the other candidates (NL2's own resistance model, chain-release behaviour).
- **Which line NL2's CSV export follows** — the heartline or the rail centreline. Telemetry and the CSV are separated by a near-constant **1.365 m** (min 1.315, p90 1.384), so they are definitely *different* reference lines about a heartline-height apart. That does not say which is which, and this track is unbanked (max 1.9°) so its G data cannot distinguish them — the two only diverge through bank. Settle it on a banked ride before quoting a G number from an imported NL2 track.
- **Procedural meshing.** The Coaster Forge build-vs-adapt evaluation is still the open Phase 0 item, and it gates Phase 4.
