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

> **Corrected in Phase 2 (2026-08-02): that fitted PAIR was never separable, and calling it a measurement of two things overstated it.** `Prototypes/NL2Telemetry/calibrate.cpp` refits it properly — the model is *linear* in both coefficients for a coasting train, so the whole thing is a closed-form 2×2 solve rather than the search the first attempt used. The diagnostic that matters: the correlation between the two predictors, `N·g` and `v²`, is **0.975**, and the normal matrix has a condition number around **2000**. They move together across this recording, so nothing can say which of them the energy loss belongs to. The split between them is a free parameter, not a result.
>
> The symptom is visible once you look for it: refitting at train lengths from 0 to 25 m, `DragK` flips sign — −0.00138, −0.00019, +0.00021, −0.00050, +0.00048, +0.00143 — while the residual barely moves. That is noise along a direction the data does not constrain.
>
> **The cause is the ride, not the method.** This recording tops out at 44.5 km/h over 234 m. Drag scales with `v²`, so at these speeds it is a small share of the loss with very little to be measured against. Separating the two needs a *fast* reference ride, and that is now a specific, actionable thing to go and record rather than a vague "calibrate against something".
>
> **Train length does not rescue it.** That was the hypothesis — that a point-mass model had nowhere to put the discrepancy except drag — and it is not supported: the correlation stays at 0.975 across every train length tested. The hypothesis is untested rather than disproved, because this data cannot test it.
>
> **What this recording does support** is one parameter, not two. Pin `DragK` at its derived 0.00045 and rolling resistance alone is well conditioned: **`RollingResistance` = 0.02204**, against a shipped default of 0.006. That is high for steel-on-steel, which most likely says something about this particular NL2 track's own friction settings rather than about steel — one more reason a second, faster reference ride is the next real step. **The defaults remain unchanged.**

> ---
>
> **RESOLVED 2026-08-06. The fast ride was recorded and both coefficients are now measured.** A purpose-built dead-flat straight, launched to **142.5 km/h** and coasting 1350 m — the coast running 142.3 → 97.2 km/h before the map ran out and the brakes went on.
>
> ```
> clean coast: 3426 samples, 142.3 -> 97.2 km/h
>   Crr    = 0.02603      (the 621 m / 30 km/h run gave 0.02602)
>   DragK  = 0.000100     (derived value 0.00045)
>   resid  = 0.00014 m/s^2  on decels of 0.328-0.412   -> a 0.04% fit
>   drag share: 38% at the top, 22% at the bottom
> ```
>
> **`DragK` = 0.000100, and the derived 0.00045 was 4.5× too high.** That derivation assumed a loaded 7-car steel train at CdA ≈ 5.5 m²; either NL2 models a far slicker train or the CdA was generous. The data is not ambiguous about it.
>
> **Three things make it a measurement rather than a fit.** Rolling resistance **replicates to five significant figures** across two independent recordings at 30 and 142 km/h, on different track of different length — that is a constant appearing twice, not a fit agreeing with itself. The **upper and lower halves of the coast, fitted separately, give identical coefficients**; if drag were being absorbed into rolling resistance the halves would trade off against each other. And drag runs **38% of the loss down to 22%**, where the old recording had it at ~10% and falling to nothing — a term that varies that much is a term the fit can see.
>
> **`corr(x,y)` is NOT the diagnostic here, and reading it as one is a trap.** It still shows 0.975 on this recording and the condition number is worse, not better — because on a dead-flat coast `N·g` is very nearly *constant*, so its correlation with anything is numerically meaningless. What settles separability is the drag *share* and whether it varies. `calibrate.cpp` now derives that verdict instead of printing a fixed paragraph, which it had been doing: it told this recording the split was arbitrary and a faster ride was needed, having been written against the 44.5 km/h one.
>
> **The calibrator was wrong first time and would have been believed.** Its coast filter admits any interval implying under 2 m/s², which correctly rejects an 8 m/s² brake and does **not** reject the *ramp into one* — 0.4, 0.6, 1.2 all pass on the way up. Those samples sit at the low end of the `v²` range carrying several times the coast's resistance, so they lift the intercept and flatten the slope: `Crr` 0.0277 against a true 0.02603, `DragK` 0.000086 against 0.000100, residual 0.0509 against 0.00014, and the pinned fit went **negative** — which is the symptom that should have given it away. Fixed with a robust trim on median absolute deviation, because a tighter fixed threshold is the same fragility with a different number in it.
>
> **APPLIED the same day.** `DragK` 0.00045 → **0.000100**, `RollingResistance` 0.024 → **0.026**. The blast radius turned out to be one assertion — the mid-course entry speed on the two-train circuit, 26.5 → 30.39 m/s, which is the most drag-sensitive number in the suites because it is a speed after 872 m of free running with nothing but resistance acting. All eleven prototype suites pass.
>
> **Nothing geometric moved. Every dynamic figure did.** Length, crest, closure, continuity, loop radius and apex height are identical; the ride is simply faster and pulls harder, because the train keeps energy it was previously losing.
>
> | | before | after |
> |---|---|---|
> | reference: top speed | 105.2 km/h | **107.5** |
> | reference: vertical G | +0.66 .. +4.20 | **+0.66 .. +4.52** |
> | reference: lateral G | 0.28 | **0.37** |
> | reference: loop apex felt G | +1.16 | **+1.78** |
> | reference: ride time | 63.4 s | **63.1** |
> | circuit: vertical G | −0.53 .. +3.08 | **−0.93 .. +3.46** |
> | circuit: lateral G | 0.15 | **0.30** |
> | circuit: lap | 105 s | **100** |
>
> The circuit's top speed is unchanged at 136.8 km/h because a launch is *commanded* to a speed rather than reaching one. **The loop is where it shows most**: apex felt G rose by more than half a g, because the train arrives with more speed left. The old figures were not measurement error — they were a correct simulation of a train dragged 4.5× harder than a real one.

**Test suites are discriminating.** This was checked by mutation rather than assumed: deliberately broken variants of the headers were generated and run against the suites. The first pass found four surviving mutants in the spline suite — including one that put the loop apex at z = **-16** instead of +16 while leaving *every* G reading numerically identical, because the frame flipped in step with the curvature. An entirely inverted track would have reported perfectly self-consistent G. Six asserts closed that gap, and all six previously-surviving mutants are now killed.

## What was disproved

**The helix is not expressible with the current segment vocabulary.** This is a representation gap, not a missing parameter, and the plan's segment list (straight / arc / clothoid / helix) was wrong to imply otherwise.

Authored the obvious way — constant radius plus constant climb — you do not get a bad helix, you get a **tilted flat circle** that returns to its starting height after a full turn (out-of-plane deviation 1.9e-12 m over 300 m; it really is planar). "Constant climb" is *zero* pitch curvature at a fixed pitch angle, yawing about the **world** vertical, not the body up-axis.

A true helix comes out of this integrator only from yaw and pitch of constant magnitude rotating at the torsion rate (`Yaw = k·cos(τs)`, `Pitch = k·sin(τs)`), which a linear ramp cannot express. Approximating it with the available ramps costs roughly one segment per metre. Phase 1 needs either a non-linear profile shape or a world-vertical yaw term in the integrator.

> **Closed in Phase 1 (2026-08-02).** Neither of the two options above was needed in full. The curvature *vector* now carries a constant `Torsion` — one field — and the path frame being rotation-minimising is exactly why that suffices: the Frenet frame turns about the tangent at the torsion rate relative to it, so constant curvature plus constant torsion reproduces `Yaw = k·cos(τs)`, `Pitch = k·sin(τs)` directly. `MakeHelix(Radius, ClimbAngle, Turns)` converts the parameters an author actually uses. Verified as geometry rather than as fields: over two turns at R = 20 m and 15° climb, the radius about a **vertical** axis holds 20.000000–20.000001 m and the climb rate `dz/ds` varies by 5.4e-09, against closed forms for rise (67.3430 m) and arc length that match exactly. Straight, arc and clothoid all carry `Torsion = 0` and are unchanged. Two consequences worth knowing: the helix **axis is inherited from the incoming frame**, so the track must already be pitched at the climb angle when the segment starts; and the curvature vector exits rotated by `τL` (2π·sin α per turn), so a helix does not generally end on pure yaw and `IsCurvatureContinuous` had to start comparing *effective* curvature rather than the raw end fields.

> **The helix composes, but only from C++ (2026-08-03).** The entry above says the axis is inherited
> from the incoming frame and the exit leaves on rotated curvature. Both are true, and neither is the
> whole condition. Three independent designs and nine refutation passes settled it.
>
> **The axis needs TWO things, not one.** At the helix start the curvature vector is pure yaw
> `k = cos²α/R`, so the axis is `sin(α)·T + cos(α)·U`. That equals world vertical exactly when the
> tangent is pitched at the climb angle **and** the path frame is untwisted (ψ = 0). Two conditions,
> so one knob provably cannot satisfy them: solving torsion alone drives ψ to 3.8e-16 rad but leaves
> pitch at **+9.906°** against a target −7°, tilts the axis **16.906°**, *and* reopens the joint
> (step 1.204e-02 1/m) because torsion rotates the ramp's own exit curvature off pure yaw.
>
> The failure this produces is nastier than "a bad helix". Fitted in the plane perpendicular to its
> own tilted axis, the naive case is a **perfect circle of exactly the authored radius** — it is a
> perfect helix about the **wrong axis**, tilted **9.42°**, not a deformed one. Anything measuring
> radius in the XY plane reports a plausible-looking error and misdiagnoses it.
>
> **Entry and exit need different tools.** Entry: the helix starts on pure yaw, so
> `MakeClothoid(L, 0, cos²α/R)` lands on it with a joint step of exactly 0 — `ChainCurvature` cannot
> help there and would actively break it, overwriting the start fields and turning a constant-magnitude
> helix into a ramp. Exit: the curvature vector has rotated by `τL` and carries a pitch component no
> `Make*` helper expresses, and `ChainCurvature` is the only thing in the vocabulary that closes it.
> Unchained the exit steps **4.223e-02 1/m** and the ride stalls; chained it is **0.000e+00** and
> completes.
>
> **A full layout does work.** Solving the two interior pitch knots of a three-`Raw` entry bridge by
> 2D Newton — targets: final pitch = climb angle, final ψ = 0 — converges in 3–4 iterations and gives
> a 22-segment, 829.721 m ride: `IsCurvatureContinuous(1e-9)` true with worst joint step
> **0.000e+00**, axis tilt **0.00000°**, plan radius **20.00000 m** (1.379e-08 rms against an authored
> 20), climb held −7.000°, ends at −0.0000 m, completes, self-clearance 11.496 m. An independent
> refuter rebuilt it with different code and a different seed and landed on the same knots to **eight
> decimals**. **No header change; it uses only fields `FTrackSegment` already has.**
>
> **So why is this still a gap?** Because none of it can be *authored*. `BuildSegment` maps one row to
> one segment with no neighbour awareness, so a chained or solved value reaches a document only by
> being typed into a `Raw` segment — storing a **derived** number. Change the helix radius 20 → 22 m
> and the stored value goes stale: joint reopens to **4.523e-03 1/m**, C² false, file still claiming to
> be valid. That is exactly what "store what was typed, never what was derived" exists to prevent.
> The solved bridge has the same disease one level up: editing an upstream turn from k = 0.030 to
> 0.050 without re-solving tilts the axis **9.708°** and drifts the plan radius to 20.18351 m
> (0.993 m rms) while `IsCurvatureContinuous` stays **true** and `ValidateTrack` reports **zero**
> diagnostics. A fourth member of the blind-spot family this page already names.
>
> **The near miss worth recording.** One rejected design added a `MakeHelixEntry` helper that
> overwrote the curvature *direction* instead of rotating it. Because `K = √(Yaw² + Pitch²)` is
> unsigned, an authored **right**-hand descending helix came out **left**-hand ascending — +2.0706
> turns where −2.00 was asked for, rising +42.373 m where −44.152 m was asked for — and it was
> **silent**: `IsCurvatureContinuous` YES, `ValidateTrack` zero diagnostics, because the entry was
> pre-wound from the same wrong phase so the joint closed perfectly onto mirrored geometry. Found only
> because a refuter ran the sign sweep the designer never did. `TrackSpline.h`'s own `CurvatureAt`
> comment already warns that a mirrored helix still looks like a helix.
>
> **Smallest thing that would close it:** a build-time hook that can see a segment's neighbour, plus a
> stored *intent* flag rather than a stored derived value — the shape `ERollMode` already has, where
> `WorldBank` is resolved per sample in `FTrack::Finish` rather than baked in when typed. Until then,
> `Helix` is authorable as a lone segment and composable only from C++.

**Two trains meant no interlocking at all, and it failed silently — FIXED 2026-08-03.** Kept in full
because the three failures below are what the fix is *shaped by*, and any future change to
`FRideSignals` has to keep clearing them. Each is now a named test in `test_ridesignals.cpp`, and all
three were confirmed to bite by mutation: reverting the identity check in the exit loop, in the entry
test, or in the permissive each kills exactly one test and no others.

The fix is a train index on `Update` and `CanDispatchInto`, a per-train held range, and one new rule —
**a block is released only when no OTHER train's range covers it**. The single-argument forms survive
for the layouts that genuinely run one train, and they *deny* on a multi-train instance rather than
aliasing to train 0, because a two-train caller that forgot its index would otherwise drive both
trains through one slot and reproduce the whole failure. What follows is what it replaced.

`FRideSignals`
tracked one train. Its own comment said to widen three members to vectors and that "nothing else
changes" — that is right about the storage and wrong about the meaning. Both obvious ways to run two
trains were measured and **neither reports anything**: no denial, no violation, no counter.

*One instance per train* gives each a private `FBlockController`, so no occupancy is shared at all.
Train B's permissive is **granted** with train A standing in the destination, B's `Update` returns
true ("was clear"), the two are independently confirmed co-resident, and `Violations()` reads **0** on
both. Each train validates against a private fiction of the circuit.

*One shared instance* is worse, because it looks correct. The single tail/nose/occupies triple
re-reads as "wherever the train that called `Update` **last** is", and all three consumers fail open:
each train's exit loop walks the other's old range and releases it, so a block reads CLEAR with a
train parked in it; `CanDispatchInto` skips "blocks this train holds" using that triple, so asked for
B while it describes A it skips **A's** blocks as the asker's own and grants entry into an OCCUPIED
block; and the collision entry is **suppressed** — `bHeldAlready` is true, so `OnTrainEnter`, the only
function that can report a violation, never runs. "Occupied by me" degrades into a test of call order.

Two corrections fall out. The enters-before-exits ordering was commented as "the safe order the day a
second train exists" — **it is not what fails**; identity fails first and fails harder. And `Tick`
acquires a contract it never needed: **once per frame, not once per train**, or a 5 s overlap expires
in 2.5 s with two trains and nothing says so.

**Two rules derived here turn out to be the industry's, and one is too strict (2026-08-04).** Checked
against Weisenberger's *Coasters 101* (3rd ed.), the accessible standard reference. This is the only
outside check either rule has had, so it is worth recording that they match.

- **Capacity.** The book states it directly: a ride must have at least one more block than it has
  trains. That is `trains = blocks − 1`, derived here from five holding places running four trains and
  gridlocking at five, and since generalised to a test over rings of three to eight.
- **What makes a block a block.** The book requires each block to contain a way to stop a train *and*
  a way to get it moving again. That is the both-authorities rule `FindHoldZoneAt` enforces, arrived at
  by watching trains park on trim brakes and never leave.
- **Brakes rest closed.** Real friction brakes are spring-closed and opened by air pressure, so a loss
  of pressure applies them. The dispatcher's "a holding device is shut unless a permissive is live"
  was reasoned from fail-closed logic; the hardware does it mechanically for the same reason.

**Where we are too strict, and deliberately staying so.** The book gives a *third* way to restart a
stopped train, beside a chain and drive tyres: **gravity**, by putting the stop point on a downgrade.
So a plain friction brake on a slope steeper than neutral genuinely does bound a block, and
`FindHoldZoneAt` — which demands `MaxAccel > 0` — would refuse it.

That is the safe direction to be wrong in: we decline to park a train somewhere that would in fact
work, rather than parking one somewhere it would strand. Closing the gap means testing the grade at
the stop point against the neutral slope rather than testing the device, and nothing authored here
needs it yet.

**Neutral slope, and the model reproduces it exactly.** The grade at which a rolling car holds speed,
gravity along the track exactly paying for resistance. At a crawl, where drag vanishes, it is
closed-form: `tan θ = Crr`, so the measured `RollingResistance = 0.024` gives **1.3748°**. Asserted
both ways — a train holds speed on it, gains on anything steeper, loses on anything shallower — which
is a check on the friction model that owes nothing to a layout or a timestep.

**What the model does NOT have, now that there is a reference to check against.** A magnetic brake's
force scales with *velocity* (eddy currents), which is why real ones decelerate more smoothly than
friction; `FTrackZone` has a constant `MaxDecel`, so every brake here is a friction brake. And train
position is known *continuously*, where a real ride infers it from discrete proximity switches
counting a metal flag under each car — the count being how it knows the whole train cleared, which is
the hardware's version of this project's nose-and-tail range.

**A circuit closes by SHAPE, not by solver — and the shape decides the whole ride (2026-08-04).** The
two-train layout closed in *height* and nothing else: it ended **373.794 m** from the station pointing
**86.421°** wrong, because its two turns summed to **446.42°** where a circuit needs 360. The train ran
off the end and was teleported back, which is what the developer saw first time they opened it.

An **oval closes analytically**. Both turns the same way, each exactly 180°, same radius, same
easement — then the lateral displacements (+2R, −2R) cancel, the along-track ones cancel, heading sums
to 360, and one scalar condition is left: *the return leg's horizontal extent equals the two collinear
outbound legs'*. A 180° turn with easements of length E needs `arc = πR − E`, since each easement turns
`E/2R`. No Gauss-Newton, no freedoms, no residual. Measured seam: **0.000000 m, 0.000084°, 0.000000°**.

The interesting part is what that condition *costs*. Because the return leg must match the outbound
legs, **every metre spent outbound is paid for twice.** The obvious layout — whole hill outbound,
turnarounds at each end — came to 1717 m, and it could not be rescued: softening the launch crest to
tame the G *lengthens* it (crest length is angle ÷ peak curvature), which lengthens the circuit, which
costs more energy than a longer launch buys. Every variant stalled, at 914, 930, 954, 1470, 1574 m.

The fix is to **split the hill across the turn** — a top-hat turnaround, which is what real launched
coasters do — and it is cheaper in three currencies at once: the train is slowest at the top so the
turn costs least lateral G, a small radius suffices there, and the drop's horizontal extent lands in
the return leg where the closure condition needs it. That took the circuit to **1288 m** and it gets
round with margin.

Closing it improved almost everything, because the constraints pulled in the same direction:

| | open | closed |
|---|---|---|
| seam | 373.794 m, 86.421° | **0.000000 m, 0.000084°** |
| length | 1072.5 m | 1288.0 m |
| top speed | 136.8 km/h | 136.8 km/h |
| vertical G | −0.35 .. +3.57 | −0.53 .. +3.08 |
| lateral G | 0.54 | **0.15** |
| self-clearance | 7.75 m | **11.68 m** |
| roll-rate diagnostics | 4 | **0** (peak 17.4 °/s) |
| blocks that can hold | 3 | **5** |

Two numbers had to be *hunted* rather than solved. Speed over the top: the first closed layout that
completed crawled the turnaround at **2.70 m/s**, a 37-second dead spot, fixed by lowering the crest
from 57.3 m to 48.5 m for **12.14 m/s**. And roll rate, which scales cleanly with easement length —
34.0, 25.8, 20.8, 17.4 °/s at easements of 26, 34, 42, 50 m — so the easement is 50, landing on the
17.1 °/s target the `AUTHORING.md` mitigation table already set.

**A lap is four changes, and only one of them is `S -= L` (2026-08-04).** With the circuit closed, the
train can wrap instead of stopping at the end — but the wrap is the easy part. `FTrainConfig::bCircuit`
turns it on, and it is measured rather than authored: the actor checks position, heading **and** roll
at the seam and only wraps a layout that passes, because wrapping an open one invents continuity
across whatever gap is there. The other three:

- **Distance travelled cannot be read back off the position.** `Travelled = S1 - S0` is negative by a
  whole lap on the step that crosses the seam, which would reverse the gravity and resistance work for
  that step. On a circuit the advance *is* the distance.
- **`AdvanceFrom` walks backwards across the seam.** The wrapped target is behind the cached frame, so
  the incremental walk would traverse the entire track the wrong way. Crossing samples re-evaluate
  from the start instead — O(track length), but nine calls per lap against the ~33 integrator steps a
  frame the cache exists to avoid. That is why the seam must close in heading and roll and not merely
  in position: a residual there is a pop, once a lap, in the same place every time.
- **A reversed rear/front pair changes meaning.** On a strip it is a caller mistake and gets sorted.
  On a circuit it is a train **straddling the seam**, holding the last block and the first — and
  sorting it claims every block *between* them, which is the whole ring, from one train, silently.
  `FRideSignals` now walks a possibly-wrapped range through one helper, so the entry test, the exit
  release and the occupancy query cannot disagree about what "the range" means. Three mutations
  confirm it: treating the wrap as a swap, ignoring the wrap in `Covers`, and not wrapping the range
  walk each kill exactly one test.

Measured on the closed preset: two trains, ten minutes, **4 and 5 laps**, zero violations, never a
shared block, and ~24,000 frames apiece spent genuinely straddling the seam rather than stepping over
it in one tick.

**A train stopped in the station straddled the seam, and it was NOT cosmetic — it deadlocked the
circuit at three trains (2026-08-04).** Recorded first as a realism nit, then measured and found to be
the thing standing between three trains and a stuck ride.

The station is the first block, so its start *is* the seam. A zone says *reach this speed*, and zero
is reachable at once, so a train commanded to hold stopped within **0.3 m of the zone's start** and
hung its back half into the **last** block. A dwelling train therefore held two blocks. With three
trains on this ring that is exactly enough to close the loop of denials:

```
train0 centre    0.0  rear 1280.5   holds 7 and 0     wants 1,2,3 — train1 in 3
train1 centre  887.1                holds 3           wants 4,5   — train2 in 5
train2 centre 1226.4  rear 1218.9   holds 5 and 6     wants 7     — train0's TAIL
```

Nothing moves, nothing is reported, and `Violations()` reads 0. A circuit of fail-closed rules doing
exactly what it was told.

**The fix needed no new authored concept.** The dispatcher already commands the target every frame, so
it brakes to a *position* instead of switching to zero: `sqrt(2·a·d)` is the fastest a train may be
travelling and still stop in `d`, so commanding that as the target eases it down and parks it
mid-device. A 15 m train centred in a 26 m station spans 5.5–20.5 m — wholly inside its own block, and
the ring gets its slack back. Asserted directly rather than via the deadlock.

Still open, and now only a preference: mid-device is right for a station and arbitrary for a 130 m
block brake. An authored stop offset is the next step if anyone wants the platform marked elsewhere.

**A fixed lookahead cannot express a braking distance, and four trains proved it (2026-08-04).**
`CanDispatch(FromBlock, Lookahead)` clears a *count* of blocks. That is a guess at a distance, and on
a layout whose free runs are **696 m and 184 m** no single count is right for both. Measured on the
closed circuit with the count rule:

| trains | lookahead 1 | lookahead 2 |
|---|---|---|
| 1–3 | clean | clean |
| 4 | **14 violations**, shared blocks | **18 violations**, shared blocks |

The failure is always the same shape: a train is granted a block with no device in it, is therefore
committed, and finds the block beyond it occupied on arrival.

`FRideSignals::SetHoldingBlocks` supplies one bool per block — can a device here stop a train *and*
let it go — and the permissive then clears **from the destination to the next block that can hold the
train**, however many that is. With it, all four run clean. It stays optional: without the list the
old fixed count is used unchanged, which is what every caller did before.

One consequence worth knowing: the two rules then stack. `DispatchLookahead` is now *extra headway*
rather than the safety requirement, its default drops from 2 to **1**, and at 2 the same four trains
spend their time held — the ring is too tight to demand the stopping margin twice.

**A layout carries one fewer train than it has places to stand — and that is THE formula, not an observation about one layout (2026-08-04).** With five holding
places, four trains run clean and **five never move at all** — every train is parked exactly where the
train behind it needs to go. No violation, no deadlock in the code; the ride is simply full. So the
cap is `holding places − 1`, which is what the actor now applies, rather than the number of places
(which permitted the gridlock) or `blocks/(1 + lookahead)` (which is neither necessary nor sufficient
here: it allows 4 at lookahead 1, where the count rule collided, and forbids 3 at lookahead 2, which
runs).

The fifth train also arrived **already in violation**, before moving a metre, for the same reason the
station straddle deadlocked three: placed at its zone's *start*, a 15 m train hangs back over the
boundary, and for the station — whose start is the seam — that means overlapping whatever stands in
the last block. Trains are now placed **mid-device**, which is both where the dispatcher parks them
and the only placement that fits a whole train inside one block.

**Since generalised.** It is not a fact about this circuit; it is the rule, and it now has a test that
says so — rings of three to eight blocks, `N−1` trains circulating with nobody starving, `N` trains
moving not at all, and neither case ever a violation. Driven on bare numbers in
`test_ridesignals.cpp`, because capacity is a property of the signalling with nothing to do with
geometry or physics.

Two things worth stating plainly, because they are what a ride designer would reach for:

- **What counts as a block here is narrow.** A launch cannot stop a train; a trim brake cannot release
  one. Neither is a boundary for capacity however much hardware is bolted to it. Only drive-tyre runs
  and block brakes.
- **Sections buy trains, and trains buy capacity.** That is why a high-throughput ride is carved into
  *more* block sections rather than given longer trains. It also says exactly how to raise this
  circuit's capacity: every block brake added is one more train, and the geometry already has 696 m
  and 184 m of free run with nothing in them.

**But do not read a real ride's brake count as its block count (2026-08-04).** Three different things
sit on the track and all of them look like "a brake":

| | What it is for | Bounds a block? |
|---|---|---|
| **Block brake** | Interlocking: stop, hold, release on a permissive | **yes**, and only this one |
| **Evacuation zone** | Getting *riders off* — needs walkway, access, an egress route | no |
| **Safety catch** | Rollback or a defect; passive, fails closed | no |

A large ride can have on the order of **25 evacuation zones** and a handful of catch brakes while
running far fewer trains than that. Those counts answer *"can everyone get off safely"* and *"what
happens when something fails"*, not *"how many trains fit"*. Only the first row feeds the formula
above, and an earlier version of this page implied otherwise.

**None of the other two is modelled.** `FTrackZone` describes powered track and nothing else — no
evacuation zone, no walkway, no anti-rollback device. The gap that bites first is the catch:
`FTrainConfig::bAllowRollback` simulates a train running backwards and **nothing catches it**, because
a catch is hardware the vocabulary cannot describe. A rollback is reported by `FRideProfile` and then
simply continues, which is the honest behaviour for a model with no such device in it — but it means
the one failure mode the physics *can* produce has no safety system standing in front of it.

**A closed circuit reports its own seam as a self-intersection (2026-08-04).** `AnalyseSelfClearance`
skips sample pairs close together *along* the track, measured linearly. On a circuit the first and
last samples are the same piece of track, so a linear separation calls them `TotalLength` apart, never
skips them, and reports **0.00 m at S=0.0 against S=1717.0** with nothing wrong anywhere. The fix is a
`bCircuit` flag that measures that separation the short way round; it is off by default, because a
point-to-point layout's two ends genuinely are far apart and hiding that would be the opposite error.
Both behaviours are asserted, so neither can quietly become the other.

**A holding device needs BOTH authorities, and the mid-course brake has neither the length nor the
tyres (2026-08-03; the length half was FIXED by closing the circuit, 2026-08-04).** The device rule
below still stands unchanged. The length verdict does not: the return leg had to grow to satisfy the
closure condition, that spare length went into the mid-course brake rather than into dead straight,
and at **130 m against the 58.1 m** it needs to stop a 26.40 m/s train it is now a real block brake.
The circuit gained a queueing position on the far side from the station. The table below is the
measurement on the **open** layout, kept because it is what the rule was derived from. Two separate facts, both measured, and together they decide where a train may be
parked on the two-train preset.

First, the device. `MakeBrake` has `MaxAccel = 0` and `MakeLaunch` has `MaxDecel = 0`, so a friction
brake can stop a train and never release it, and a launch can release one and never stop it. Only a
drive-tyre run — `MakeLift`'s shape — has both. `FTrain::FindHoldZoneAt` therefore requires both and
returns −1 for the other two, which is why gating a launch mid-launch (an *aborted* launch, the
opposite of an interlock) and parking a train on a trim brake (a permanent stop with no symptom) are
both unreachable rather than merely discouraged. The actor gained `ETUSegmentZone::BlockBrake` for
this: identical physics to a lift chain, separate enumerator because block boundaries fall where the
*kind* changes and two holding devices in a row must stay two blocks.

Second, the length, which no zone kind can fix. A block brake must stop the train in the block it
occupies: `v²/2a` against the block length. On the two-train preset, measured per block at the fixed
6 m/s² grip —

| block | device | length | arrives | needs to stop | verdict |
|---|---|---|---|---|---|
| 3 | mid-course | 45.00 m | 28.19 m/s | **66.2 m** | trim only |
| 5 | outer pre-station | 37.50 m | 7.57 m/s | 4.8 m | holds |
| 6 | transfer tyres | 27.00 m | 4.28 m/s | 1.5 m | holds |
| 7 | inner pre-station | 37.50 m | 3.25 m/s | 0.9 m | holds |

So the mid-course brake is authored `Brake` and stays a trim. Making it a `BlockBrake` would build a
device that closes and is then run straight through — **worse than a trim, because it would look like
an interlock.** Lengthening the block past ~70 m is what would make it one, and that is a layout
change with its own re-measurement, not an enum change. This is also the cap on train count: the
number of trains a layout can run is the number of places that can both hold a train and release it,
which is four here and one on every other preset.

**A block brake holds on a gradient exactly, and the physics needed nothing (2026-08-03).** This was
an open question on the block-brake card and the answer is a clean one. A brake zone commanded to
zero holds a train with drift **0.000000000 m over 120 s** at every grade up to −37°, and the
threshold is exactly `g·sinθ`: at −37° (`g·sin37° = 5.9018`) a bite of 5.850 crept **0.0518 m**, 5.900
crept **0.0012 m**, and 5.902 and above held **bit-exactly**. Undersizing produces *creep*, not a
cliff, which is how real hardware presents. The hold is timestep-independent — 1/30, 1/60, 1/120 and
1/600 s all give exactly zero — because it falls out of the energy formulation rather than a clamp.
At the actor's fixed 6.0 m/s² grip the maximum holdable grade is **37.72°**.

The asymmetry is the interesting part: **a friction brake can hold but cannot start.** On level track
a stopped train whose zone was retargeted to 12 m/s moved **0.000 m in 30 s**; on −2° it crawled to
3.17 m/s and was still short; on −6° it reached exactly 12.000 and trimmed. That is why real level
mid-course brakes carry drive tyres and gradient ones do not, and the model reproduces it without
being told to. A released target above what gravity supplies is a number with no effect — on −6°,
targets of 15 and 25 m/s both produce the same **13.415 m/s** exit.

**Zone membership is the train's CENTRE, not its span, and that must not be quietly changed.** Zones
test `S0 = DistanceAlong` against `[StartS, EndS]`. Measured with a 15 m train against a block boundary
and a zone edge both at S = 150 m: the block goes OCCUPIED when the **nose** reaches 150.000 m, but
the brake first bites when the nose is at **157.517 m** — a 7.517 m disagreement, exactly half a
train. Signalling occupancy is a span and zone membership is a point, and they legitimately disagree.
The Phase 2 train-length figures were all measured with centre membership, so changing it silently
re-tunes every one of them.

## Known limitations

Deliberate, measured, and written down. A bounded limitation that is recorded is a Phase 0 success, not debt. Shortcuts also carry a `ponytail:` comment in the code naming the ceiling and the upgrade path — `grep -rn "ponytail:" Prototypes/` lists them.

**Track spline**

- **No endpoint targeting.** You author curvature and arrive wherever you arrive; closing a circuit back onto the station is an inverse solve that a control-point model would have given free. Measured: a symmetric oval closes at 0.0000 m, but easements of 20 m vs 8 m leave a 0.93 m gap and radii of 30 m vs 45 m leave a 29.63 m gap — heading closes exactly in every case. This is the one real cost of the representation, and it is a Phase 1 scope item.

  > **Addressed in Phase 1 (2026-08-02).** `TrackClose.h`, in two halves. `MeasureClosure` reports the gap per axis — and gives **height its own field**, because that is the one that does not announce itself: a track ending 8.5 m low still looks closed in plan view, which is exactly how the vertical slice shipped. `SolveClosure` is damped Gauss-Newton over **authored** parameters the caller explicitly frees. Measured: a 9.093 m gap closes to 3.0e-06 m in 2 iterations and 11 track walks; a 20.98 m gap to 7.0e-06 m in 2. Both are effectively linear problems — changing a straight's length translates everything downstream without rotating it — which is why two iterations is enough rather than a search.
  >
  > It solves for authored values, never derived ones, so a solved arc still has the radius the author typed rather than a curvature field that disagrees with it. Three guards are load-bearing rather than defensive: a radius has its **sign locked and magnitude floored**, because +ve is a left turn and -ve is a right one, so crossing zero would silently reverse a turn *and* pass through the infinite curvature this document already records as producing a clean, straight, every-check-passing segment. The finite-difference step is deliberately coarse at `1e-3·scale`, because the "do not finite-difference `EvaluateAt`" entry below applies to the solver too — a textbook `1e-8` step differentiates the integrator's step-count staircase, not the geometry. And on failure the document is left **unchanged** by default: an author who has lost their own round numbers and still has an open circuit is worse off than before.
  >
  > **Two limits measured, not fixed.** Freedoms only reach the axes they point along: horizontal straights cannot move the endpoint's height *at all* — 8.93 m of height gap survives a converged-looking solve completely untouched, because those parameters have identically zero gradient on that row. (A straight sitting after an unbalanced hill is itself pitched and moves height fine; it is the direction that matters, not the segment kind.) And parameters are independent, so freeing one half of a balanced hill breaks the balance that made it level — the same 8.93 m only reaches 6.57 m that way. **Closing height while keeping the exit level needs coupled freedoms**, which this solver's one-parameter-one-column model cannot express. That is the next piece of work, not a bug in this one.
- **Roll-rate steps are invisible to the continuity check.** `IsCurvatureContinuous` checks roll *value*, not rate. The banking pattern the header itself endorses — ramp bank over the clothoid, hold through the arc — steps roll angular velocity by 53.7 °/s at four joints per banked turn, and both the check and the G readout report nothing, because felt G has no roll-rate term. A roll-rate metric belongs in the Phase 1 editor.

  > **Closed in Phase 1 (2026-08-02).** `TrackValidate.h` reports roll-rate steps at joints, and `AnalyseRollRate` gives peak rate, worst joint step, and the head-snap G a rider actually feels. On the standard banked turn it measures a **56.2 °/s** step and **0.049 g** of head snap at 0.5 m above the heartline, on track that passes `IsCurvatureContinuous` cleanly. Felt G's blindness here is now asserted *exactly* rather than approximately: two tracks with identical curvature and the same roll value at the sample point but **twice** the roll rate return bit-identical `FeltG` — because roll does not perturb the path, so the path frames are identical and the roll-rate term simply is not in the model. That is a structural gap, not a tolerance, and it stays until the train has length.
  >
  > **A second layer of the same blind spot, found while adding world-referenced roll.** `AnalyseRollRate` reads the authored list, so it measures the roll rate somebody *typed*. A `WorldBank` segment holding a constant bank through a climbing turn types nothing and rotates the rider the whole way, because the path frame twists underneath the number: measured **0.0 rad/m authored against 0.0296 rad/m resolved** — 34.0 °/s and 0.018 g of head snap — on a three-quarter helix at a constant 0.5 rad of bank. `AnalyseResolvedRollRate` walks the built track instead of reading the list, and is the report that can see it. Felt G cannot see roll rate; the segment list cannot see geometry. Two different blind spots, stacked, hence two reports.
- **The rider is a point at the heartline**, so roll-rate terms vanish by construction. Correct for the heartline and the right simplification, but the omitted ω²r at 0.5 m above it is 0.031 g at 45 °/s, 0.126 g at 90 °/s, and 0.503 g at 180 °/s. A fast barrel roll that reads smooth at the heartline is half a G of head snap.
- **`Roll = 0` does not mean level with the horizon** on non-planar track. The frame is exactly rotation-minimising, so climb 45° / left 90° / descend 45° at roll 0 throughout exits at -54.736° of world bank (= -arccos(1/√3)); three right angles gives exactly -90°. The geometry and its G are correct — but "keep RMF and document it" vs "add a world-referenced roll mode" is a real Phase 1 decision, not a bug.

  > **Closed in Phase 1 (2026-08-02).** Both, per segment — `FTrackSegment::RollMode` is `PathRelative` (default, unchanged, the only mode defined everywhere) or `WorldBank`, where the value is the bank a spirit level would read. The magnitude above reproduces exactly: the same corner hill at roll 0 exits **54.736°** off level path-relative and **5.7e-14** world-referenced. Note the sign convention, since two now exist — `WorldBankOf` reports the **rider's** bank, positive leaning the same way a positive roll does, so it reads *+*54.736° where the number above reports the path frame's own twist, which is the roll correction needed to cancel it. Same measurement, opposite sign, and the code names which is which.
  >
  > The choice that made it cheap: roll never perturbs the path, so it is applied last, as a rotation about the tangent (`FTrack::Finish`). Solving a world bank is therefore one `atan2` against a frame already in hand, and `FTrackFrame::Roll` stays path-relative for everything downstream — felt G, the cross-section and the ride camera need to know nothing about roll modes. It is resolved **per sample at walk time, not baked in when typed**, so editing an upstream hill cannot silently unbank a turn a hundred metres later.
  >
  > Two things fall out of this and are recorded rather than fixed. Roll values from different modes cannot be subtracted, so a mixed-mode joint reports `RollModeMixed` and skips the roll checks instead of fabricating a step or a clean bill; `IsCurvatureContinuous` refuses such a joint outright, which can cry wolf where the joint happens to be level and is the conservative direction. And `WorldBank` is **undefined pointing straight up** — there is no horizontal to be level with. That needs no special error type, because the symptom is exact and already measured: the level reference flips through the vertical, so holding a world bank across the top of a loop costs **12.6 rad/m** of roll rate against **0.0** for the identical geometry authored path-relative. Author inversions path-relative.
- **`EvaluateAt` is O(track length)**, re-integrating from the start on every call — about 0.2 ms on a 1 km track, so a 0.1 m mesh pass over that track is ~2.3 s. The cached arc-length sample table is the named upgrade path. Nothing needs it yet.
- **Do not finite-difference `EvaluateAt`** to recover direction or speed. Step counts are re-discretised per call, so position is not smooth in `S` below ~1e-9 m and a difference quotient stops converging under h ≈ 1e-4. Use the returned `Tangent`, which is exact to 1e-15 rad.
- **Authored values are not validated beyond segment length.** `AddSegment` rejects zero, negative and NaN lengths — the one shared choke point — but nothing else is checked. This is intentional: validation belongs at the Phase 1 editor boundary, where one check covers every way to author a bad segment. Do **not** "fix" `MakeArc(L, 0)` by clamping it to a straight — that was tested and produces a plausible 1.00 G with a clean continuity pass.

  > **Corrected and closed in Phase 1 (2026-08-02).** The description above was wrong about the mechanism, in a way that understated it. `MakeArc(L, 0)` produces **infinity**, not NaN — `1.0/0.0` is `inf`. What follows is worse than a NaN would be: the lerp inside `CurvatureAt` computes `(inf − inf) · 0 = NaN`; `IsCurvatureContinuous` compares with `fabs(x) > tol`, which is false for NaN, so the joint **passes**; and `Integrate` guards on `Rate > 0.0`, also false for NaN, so it takes the straight-line branch. The measured result is not visibly broken geometry — it is a clean, perfectly **straight** 50 m segment where an arc was authored, with every check reporting success. There is no "obvious NaN" to notice. `Prototypes/TrackSpline/TrackValidate.h` now catches it at the editor boundary, and reports rather than repairs.
- **At an exact joint, the ending segment supplies roll and curvature** — and that is not reliable in floating point. With authored lengths that do not sum exactly, roughly a quarter of joint queries return the following segment's values instead. It only bites on curvature-discontinuous data. Sample either side of a joint rather than on it.
- **`MaxStep` is a hardcoded 0.01 in metres.** Nothing breaks today, but a port that hands it centimetre lengths gets 100× the steps, with no error and no symptom except being unusably slow. The suite passes at `MaxStep` up to 0.5, so there is ~50× of headroom if performance ever matters.
- **The prototypes are engine-free, but they are COMPILED INTO the engine — so their type names must be unique against all of UE.** Hit twice now. `FFrame` collided with the Blueprint VM's and became `FTrackFrame`; `TrackIODetail::FField` collided with UE's global `FField` (`UObject/Field.h`) and became `FTrackIOField`. Both compile perfectly under standalone clang, because standalone clang has never heard of the other name — the collision only appears at the port, and the second one surfaced as forty lines of "cannot convert `std::vector<FField>` to `std::vector<TrackIODetail::FField>`", which is a confusing way to be told a name was shadowed. A `using namespace` inside a function does not protect a detail namespace from a global. Worth checking a new type name against the engine before, not after.
- **Nothing in the model could see the track passing through itself.** Found in Phase 1 (2026-08-02) by *riding* the vertical slice, which is the worst way to find anything. A vertical loop built from pure pitch curvature is exactly planar, so its ascending and descending legs pass **0.097 m** apart on the bare loop and **0.189 m** apart in the slice — against rails that are **1.215 m** wide. Every segment was individually valid, every joint curvature-continuous, every G reading plausible, and the rider went through solid steel. `AnalyseSelfClearance` in `TrackValidate.h` now reports closest approach and the two arc lengths it happens at. Third member of the same family: felt G cannot see roll rate, the authored segment list cannot see the roll geometry contributes, and neither can see two parts of the track occupying the same space.

  > **A planar loop ALWAYS crosses itself, and no shape parameter changes that.** Swept radius 7–13 m against easing 30–98% of the loop's length — twenty combinations — and every one lands between **0.014 m and 0.138 m**. Entry pitch is no lever either: 0° through −40° all give exactly 0.097 m, because pitching the approach rotates the whole loop rigidly and the legs move together. This is not a tuning failure. A planar curve with exactly 2π of turning returns parallel to itself having gone essentially nowhere, so the legs coincide whatever shape they take. **Separation requires a lateral component, full stop.**
  >
  > **And every cheap way of adding one was measured and failed.** Recorded because each looks obviously right until it is tried:
  > - *Constant curvature plus torsion* — a true helix about a horizontal axis, which returns the tangent **exactly** and side-steps cleanly. Unrideable: with curvature constant the loop is circular, and at this entry speed it pulls **+9.49 g**. This is precisely why teardrops exist, arrived at from the opposite direction.
  > - *Teardrop plus torsion* — works for clearance, 0.189 m → **2.25 m** at 0.68 g lateral with the torsion on the crown. But the loop **translates 9 m downward per metre of side-step**, because the helix axis it implies is mostly vertical. Closing that needs the lift growing from 75 m to roughly **123 m**, taking the ride from 43 m tall to 91 m. A different ride, not a tweak.
  > - *A pitch correction after the loop* — does nothing, and measuring first would have said so: the stepped loop already exits at **+0.078°**. The lost height is the loop translating, not the track running downhill.
  > - *Torsion on the entry and exit eases, where the frame is upright and lateral is genuinely horizontal* — the exit ease undoes the entry ease's drift and clearance stays at **0.099 m**.
  > - *Banking it out* — a constant roll cannot cancel lateral G through an inversion where speed runs 26 m/s at the bottom to 10 m/s at the top. Best found was 0.73 g at −15°, against 0.36 g planar.
  >
  > Left unapplied. The vertical slice's job is proving ride feel, which it does; the defect is now **detectable**, which is the actual win. Real coasters plainly solve this, so there is an answer the current authored vocabulary cannot express — most likely an asymmetric entry/exit rather than a symmetric loop with a twist. That belongs to the editor phase and wants its own card.
  >
  > **RESOLVED 2026-08-26, and the guess above was wrong in an instructive way: it is a symmetric loop with a twist — the twist just has to follow the curvature.** Every failed attempt above put a *constant* torsion somewhere, and on this loop the eases are 54 m each around a 2.5 m crown, so "torsion on the crown" was torsion on a point and "torsion on the eases" rotated a curvature vector that was mostly zero. What the vocabulary lacked is torsion as a constant **ratio** of curvature (`TorsionRatio`), which by Lancret's theorem makes the eased loop a *generalised helix*: its tangent keeps one angle ρ to a fixed horizontal axis through the ramps, it projects onto the plane perpendicular to that axis as exactly the planar teardrop it was, and it advances along the axis at cos ρ per metre. Measured on the reference loop with ratio 0.0498 (ρ = 87.15°): tangent returns to **5e-10**, height preserved, legs **3.46 m** apart (from 0.189), every published ride figure unchanged to the second decimal, and peak lateral 0.37 g as before — because the rider's up follows the bend (`ERollMode::FollowsTorsion`); path-relative, the 17.9° of twist reads as lateral G, which is the "roughly triples lateral" the earlier attempts saw. Two things came with it. The path frame leaves the loop twisted by **2π·cos ρ** and stays so, so everything downstream is authored in that frame (`InTwistedFrame`) — a turn typed as plain yaw there pitches by sin(twist) of its turning, and the reference layout dropped **24 m** before that was done, which is the "translates 9 m per metre of side-step" above wearing a different hat. And the corridor check gives the design number: the legs cross about 63 % of the loop apart, so closest approach ≈ 0.63 × side-step, and 5.5 m of side-step clears the 3.0 m corridor with margin.
  >
  > Composing torsion across segments needed a new primitive. Torsion's phase is measured from each segment's own start, so a segment exits with its curvature vector rotated by `Torsion*Length` and the next begins rotating again from zero — three torsioned segments in a row step at **every** joint. `ChainCurvature` hands the vector across; `IsCurvatureContinuous` correctly refused the naive version, which is the check doing exactly what it was built for.

**Train physics**

- **The train is a point at the heartline; it has no length.** This is the one omission most likely to be *felt*. A real train is 10–15 m long, and its speed over a crest is governed by the whole train's centre of mass, not by the lead car's position — which is exactly why a car at the back gets thrown over an airtime hill harder than the front. Point-mass coaster sims are known to feel wrong for this reason. Defensible for Phase 0 (it does not change the shape of the model — it becomes an average over sample points along the train), but it should not survive Phase 2's "feels right to an NL2 veteran" bar.

  > **Closed in Phase 2 (2026-08-02).** `FTrainConfig::TrainLength`, and the prediction above was right about the shape: it is an average over sample points, nine of them, uniformly spaced. The substitution that matters is one line — the gravity term reads the change in the train's **mean** height rather than its centre's, because a rigid body on rails is accelerated by the slope its whole mass sits on. Rolling resistance averages its normal load over the train for the same reason. `TrainLength = 0` is the default and every sample collapses to one, so every number measured before this is bit-identical, asserted exactly rather than within a tolerance.
  >
  > Measured on a crest: a point mass arrives at **19.619 m/s**, a 15 m train at **19.735 m/s**. Straddling the crest its mass is below the crest, so it has not paid the full height. And on an asymmetric crest the spread across the train is **0.73 g** — front car **−1.996 g**, centre **−2.318 g**, back car **−2.723 g**.
  >
  > **The condition this entry did not state: the back-car effect requires an ASYMMETRIC hill.** On a symmetric one there is none at all, and the first version of the test measured exactly that. The front car crests when the centre of mass is half a train short of the crest and the back car crests when it is half a train past — symmetric and frictionless, those are the same height, so the speeds are identical and so is the airtime. It takes a gentle rise and a sharp fall, which is what real airtime hills are, for the mass to be lower when the back crests. Both cases are now asserted, the symmetric one as the control that stops the asymmetric assertion passing for the wrong reason.
  >
  > **Still open, and now visible:** samples clamp at the track ends, so a train hanging off a point-to-point layout has its overhanging mass piled at the endpoint and its mean height is wrong until it is fully on. Harmless where both ends are flat station track, which is every layout here so far. And whether length explains the `DragK` fit landing 3.2× high is now *testable* against the NL2 traces rather than merely suspected — that refit has not been done.
- **A train that runs out of energy stops dead rather than rolling back.** Reversal needs a signed velocity through the whole model and the block system has to hear about it. A valley stall is a design error to surface, not a state to simulate — but a real editor will eventually want to show the roll-back.

  > **Available from Phase 2 (2026-08-02), and off by default.** `FTrainConfig::bAllowRollback`. Signed velocity through the model, and the change is smaller than the entry implies: the position integration already computed a negative advance on a gradient and simply threw it away. Removing that clamp is most of it. The rest is that resistance is charged over the **distance covered** rather than over signed travel — multiply it by a signed displacement and friction becomes an energy *source* the moment a train rolls backwards, which would show up in the most flattering possible way, as a train coming back down a hill faster than it went up.
  >
  > Asserted exactly that way round: frictionless, a train sent at an uncrestable climb at 15.000 m/s comes back past its starting point at **15.000 m/s**. With resistance it takes **12 reversals** to settle, and settles 0.8 m from the true bottom of the valley.
  >
  > **Still off by default, and not out of caution.** Stopping dead is what every number recorded before Phase 2 was measured against, and one test asserts it directly. More importantly the entry above is still right that a valley stall is a *design error to surface* — a train that rolls back oscillates and settles, which an author can easily read as the ride working. What makes rollback useful is the reporting, so `FRideProfile` now distinguishes the two: a stall says "this hill is too tall", a rollback says that **and** that the train is loose on the track heading the wrong way, which is a signalling problem as much as a physics one. The profile stops at the reversal rather than watching the oscillation.
  >
  > One thing this does **not** yet do, and the entry named it: the block system has not heard about any of it. A rolling-back train changes occupancy, and that is Phase 3.
- **A zone shorter than one step's travel is skipped entirely.** At 40 m/s and 1/60 s that is any zone under 0.67 m. Fine for lift hills and brake runs; a trap for a short trim brake.
- **The energy exchange is exact; the *path* integration is not.** Position carries the acceleration term, which makes it exact under constant acceleration, but gravity varies along a curve, so on curved track there is a residual O(dt²) position error. Energy stays exact regardless, because it is computed from the heights actually visited.
- **`RollingResistance` and `DragK` are tuning knobs, not measurements.** `DragK = 0.00045` is `0.5·ρ·Cd·A/m` evaluated for a loaded 7-car steel train (~8000 kg, CdA ≈ 5.5 m²), and `RollingResistance = 0.006` is a plausible steel-on-steel figure. Both need calibrating against a reference ride before any G trace is quoted as accurate. The physical world needs tuning a minimal model cannot see.

  > **`RollingResistance` corrected to 0.024 in Phase 2 (2026-08-02), and the old value was justified against the WRONG MATERIAL.** "A plausible steel-on-steel figure" describes a *railway*. Coaster running wheels are **polyurethane** on steel, typically around 95A, and the hysteresis loss as that elastomer deforms under load is a real and continuous drain. Published figures for polyurethane on steel sit around **0.01–0.03**; steel wheel on steel rail is nearer **0.001–0.002** — so 0.006 was not even correct for the material it named.
  >
  > **Measured, not argued.** Three NoLimits 2 recordings, two coaster types, two layouts, all fitting **0.022–0.026**. The cleanest is a purpose-built 621 m dead-flat coast-down launched at exactly 30 km/h and allowed to stop: `Crr = 0.02602`, residual **0.0005 m/s²** against decelerations of 0.1–0.2 m/s². That is a **0.3% fit**, and its real message is that the model's *shape* — `Crr·N·g + DragK·v²` — was right all along and only this constant was wrong. Reproduced bit-identically across two separate recordings of the same configuration, so the number is stable rather than lucky.
  >
  > It also resolves something recorded earlier as inexplicable: rolling resistance outweighing air drag **3:1 even at 100 km/h**. Absurd for a steel railway wheel; exactly right for polyurethane tyres.
  >
  > **What caught it was a question, not a measurement** — *"if NL2 is used by professionals, how can it just be simulated and not real?"* The honest answer is that NL2 is a previsualisation tool rather than a certification one, and its coefficients need not be engineering values. Following that up meant re-reading our own justification, which is where "steel-on-steel" turned out to be describing the wrong vehicle entirely. The measurement had been sitting there for an hour being interpreted as *NL2 disagreeing with physics*, when it was *our default disagreeing with coasters*.
  >
  > **`DragK` is unchanged at its derived 0.00045.** The fit prefers 0.0001, but at 30 km/h drag is a minority term the data cannot really see, and a physically derived value beats a weakly determined one. Separating it properly needs a much faster reference ride, and may not be worth it: at the corrected `Crr`, drag is a 3:1 minority even at 100 km/h.
  >
  > **Cost: the vertical slice needed re-tuning**, exactly as predicted. At 0.024 its loop crested at **+0.13 g** — hanging on the track rather than held to it. The fix was the drop, not the lift: 12 m → 24 m restores **+1.13 g** and the original profile, and the lift then went 75.11 m → 90.99 m purely to close the ending back to station height, changing no G number at all.
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
- **Procedural meshing technique.** The build-vs-adapt question is settled — see below — but nothing has been meshed yet. Sweeping rail sections along a curvature-profile track is straightforward; **support and footer auto-placement is not**, and the plan already treats it as its own milestone rather than a detail. That is where the Phase 4 risk actually lives, and no amount of prior art was going to remove it.

## Decision: build the meshing layer, do not adapt

`PROJECT_PLAN.md` framed procedural meshing as a build-vs-adapt call against Coaster Forge, and called adapting "the single biggest lever available to compress a multi-year solo effort." That lever does not exist here, and the reason is structural rather than a judgement about the asset.

Coaster Forge is a commercial product. This repository is MIT and publishes every line, so its code and assets can never live in it — constraint 3 in `CLAUDE.md` already said this about paid marketplace assets generally. So the money would have bought *reference reading*, followed by writing an original implementation regardless. On a project with no funding cushion that is a much weaker case than the plan implied, because the plan was implicitly pricing a shortcut that licensing forecloses.

Two things also changed since the plan was written. The zone-based speed control it cited as the pattern worth borrowing was arrived at independently and verified against NoLimits 2 — `FTrackZone` covers station, brake, boost and lift as one shape differing only in target speed and tractive authority. And a cross-section specification now exists in `Prototypes/TrackSpline/TrackProfile.h`, so meshing starts from typed dimensions rather than a blank sheet.

**This closes the last Phase 0 gate.**
