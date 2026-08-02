# Phase 0 Findings

Phase 0 is a de-risking phase, so its real deliverable is *knowledge*, not code. This page is where that knowledge lives: what the two standalone prototypes proved, what they disproved, and what is still unknown. The prototypes themselves are small; the reason to trust them is on this page.

Last updated: 2026-08-01.

## The prototypes

Both are plain C++17 with no engine dependency, so they build and run in about a second without an Unreal install. That is deliberate — this is the lowest-friction way to work on the two hardest parts of the project, and it keeps the math honest by preventing it from quietly depending on engine behaviour.

| Prototype | Proves | Files |
|---|---|---|
| Track spline | Curvature-continuous track geometry, clothoid transitions, heartline-relative banking, felt-G | [`Prototypes/TrackSpline/`](../Prototypes/TrackSpline/) |
| Block signalling | `CLEAR → OCCUPIED → BUFFER(x) → CLEAR` occupancy plus dispatch permissive with lookahead | [`Prototypes/BlockSignal/`](../Prototypes/BlockSignal/) |

```sh
cd Prototypes/TrackSpline
clang++ -std=c++17 -Wall -Wextra -o test_trackspline test_trackspline.cpp && ./test_trackspline
```

Same shape for `BlockSignal`. Run from inside the prototype's own directory — the test includes its header by bare name.

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

**Signalling**

- All tests pass clean under `-Wall -Wextra`. Class invariants held across **1,200,000 randomly fuzzed API calls** over 20,000 controllers: zero violations of the buffer invariants, of drain-to-CLEAR, or of `CanDispatch` against an independent reimplementation.
- Driven correctly it is sound end to end: a simulated 10-minute run (6 blocks, 3 trains, 3 s overlap, 1/60 s tick, every move gated on `CanDispatch`) produced **453 moves with zero violations**, with an independent audit confirming no two trains were ever co-resident.
- Buffer countdown drift is a non-issue: 2.4M ticks accumulate -2.7e-07 s. The visible effect is at most one extra tick, and a buffer can only expire on a tick boundary anyway.

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

**Block signalling**

- **One train per block, by definition.** There is no state for "two inside", so after a signalling violation the first train's exit runs `BUFFER → CLEAR` while the second is still inside, the second train's exit gets a 0 s overlap instead of the configured one, and further violations go unreported for the rest of that occupancy. The damage is bounded and self-healing — one injected violation followed by 20 clean laps gives 3 disagreements, all within 3 steps, and none afterwards. This is correct scoping: a violation is an E-stop condition in real ride control, not a recoverable state.
- **Block topology is a hard-coded ring** (`index + 1 mod N`). Verified correct for a circuit. On a shuttle or a transfer spur the last block's permissive consults the first, and the failure direction is fail-*open*. A `bool bIsCircuit` when a non-ring layout first exists; a successor graph only once transfer tracks do. Phase 3.
- **Dispatch mode (automatic vs manual) is not implemented.** Deliberate — it sits above this class and does not change its shape, since the permissive logic is identical in both modes by design.

## UE5 port checklist

Measured, not guessed. Both headers are the canonical design to **port**, not references to reimplement.

1. **Handedness must flip, and it is not just a sign.** With `M(x,y,z) = (x,-y,z)`, taking UE `Right = -M(Lateral)` gives a residual of 1.6e-13 against `Fwd × Right = Up`. Using `M(Lateral)` directly as UE's Y gives **2.000000** — exactly inverted. Concretely: the end of a +R=30 left arc sits at prototype `Y = +6.4234 m`, which lands 6.4234 m to the *right* in UE if converted naively.
2. **Units.** Prototype is metres/radians/seconds; UE is centimetres. Convert at the port boundary, never inside the math — and remember `MaxStep`.
3. **Build the `FQuat` from the three basis vectors**, not from `FRotator` angles. The frame is already exactly orthonormal; going through Euler angles reintroduces error and gimbal cases the integrator specifically avoids.
4. **`std::vector` → `TArray`**, `std::size_t` → `int32`. Keep block indices unsigned or validate once at a single entry point: the current unsigned arithmetic is fail-*safe* (a negative lookahead wraps to `SIZE_MAX` and lands on the deny guard), and `int32` loses that property.
5. **Keep these as plain C++ structs**, not `UObject`s. Nothing here needs reflection, and `UPROPERTY` on the hot path costs for nothing.

## Still unknown

Phase 0 did not touch these, and no claim on this page covers them.

- **Tangential physics.** The energy model — gravity, rolling friction, air drag, powered segments — is entirely unbuilt. Felt G here is the geometric part only.
- **Distance-to-arc-length inversion** for advancing a train by `v·dt` along the track each tick.
- **Distance-based block overlap.** Only the time-based overlap exists; the distance form needs train position and braking distance from the physics model.
- **Lateral-G sign versus NoLimits 2.** The prototype reports +0.76 for a flat left turn (rider thrown right), consistent with its own vertical convention. NL2's documentation reads as the opposite. This rests on a documentation reading only — nobody checked a running copy. Settle it empirically before G traces, editor graphs and comfort thresholds get built on it; it is one negation now and expensive later.
- **Procedural meshing.** The Coaster Forge build-vs-adapt evaluation is still the open Phase 0 item, and it gates Phase 4.
