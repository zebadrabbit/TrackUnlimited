# Architecture

How TrackUnlimited is built, and why. This is the technical overview that used to live in the
README. The long-form version — with market context, risks and the reasoning behind each pillar — is
[`PROJECT_PLAN.md`](PROJECT_PLAN.md); the measured consequences are in
[`PHASE0_FINDINGS.md`](PHASE0_FINDINGS.md).

## Contents

- [Constraints that are not up for negotiation](#constraints-that-are-not-up-for-negotiation)
- [Engine and language](#engine-and-language)
- [Track representation](#track-representation)
- [Physics](#physics)
- [Signalling and ride control](#signalling-and-ride-control)
- [Procedural meshing](#procedural-meshing)
- [Rendering](#rendering)
- [Save format](#save-format)
- [Sharing and community](#sharing-and-community)
- [Units and handedness](#units-and-handedness)

---

## Constraints that are not up for negotiation

These were decided deliberately, after discussion, and each has already survived at least one
plausible-sounding proposal to reverse it. Raise them before changing them.

1. **No viewport click-drag-place track editing, ever.** Segments are numeric and parametric. The 3D
   view is a preview. See [`AUTHORING.md`](AUTHORING.md).
2. **C++ for physics and signalling.** Blueprint is fine for editor tooling, UI, and iteration on
   non-critical systems. The energy-based motion model and the block state machine are not.
3. **No dependency on paid marketplace assets** in committed or shipped code — including
   Train and Rail System (Polygon Jelly), which was reference material only.
4. **No Unreal Engine source in this repo, ever.** Project code and content only, per the EULA.
5. **No real manufacturer trademarks or exact ride designs** without explicit permission.
6. **MIT licence** on everything committed here.

## Engine and language

Unreal Engine 5.8. C++ for the simulation core, where correctness and speed are non-negotiable;
Blueprint where iteration speed matters more. For a solo developer that split is the whole point —
you do not want to be debugging a Blueprint graph to find out why a train gained energy on a
gradient.

The prototype headers under `Prototypes/` are engine-free C++17, but they are compiled *into* the
engine by `TrackUnlimited.Build.cs` rather than copied. The standalone assert suites therefore test
exactly the code that ships.

> One rule that has now bitten twice: a type name in these headers only needs to be unique **against
> all of Unreal**. `FFrame` collided with the Blueprint VM's and became `FTrackFrame`; `FField`
> collided with `UObject/Field.h`'s and became `FTrackIOField`. Standalone clang compiles both
> cleanly, so the collision only shows up at the port. Check a new type name against the engine
> before, not after.

## Track representation

The core data structure is deliberately **not** a position-only spline. Those produce visible
curvature discontinuities exactly where NoLimits 2's track feels smooth.

A track is a sequence of segments, each of which is a **curvature profile over arc length**:
curvature varies linearly across the segment, so

| kind | curvature |
|---|---|
| straight | `κ = 0` |
| constant-radius curve | `κ = const` |
| clothoid (Euler spiral) | `κ` linear from start to end |
| helix | constant curvature plus constant torsion |

Geometry comes from integrating a moving orthonormal frame along that profile.

The consequence that justifies the whole choice: **C² continuity becomes a property of the data
rather than something fitted after the fact.** If curvature is continuous across a joint, the
geometry is, with nothing to solve. Transition curves — the standard real-world technique for banked
curve entry and exit, and the thing that makes physical track feel smooth instead of jarring — become
the native case rather than special handling.

> This supersedes the cubic Hermite / B-spline formulation originally specified in the plan. Do not
> "restore" a control-point model. The full rationale lives in
> `Prototypes/TrackSpline/TrackSpline.h`, where it cannot drift from the code.

**Roll is independent of position** and defined along the same curvilinear parameter, computed around
an offset **heartline** rather than the rail centreline — which is what keeps felt-G physically
sensible through banked turns. Roll is applied last, as a rotation about the tangent, so it never
perturbs the path.

**The one real cost** of this representation is endpoint targeting: you author curvature and you
arrive wherever you arrive, where a control-point model gets closure for free. That is what
`TrackClose.h` exists for — see [`AUTHORING.md#closing-a-circuit`](AUTHORING.md#closing-a-circuit).

## Physics

The train's motion is a 1D energy/force model constrained to the spline, not Unreal's Chaos
rigid-body solver — which would fight for the precision this project needs. Chaos still has a role
for secondary effects: rider ragdoll, camera shake, debris, scenery collision. Just not the ride.

What the model does:

- **Gravity is an exact energy exchange, not an integrated force.** A frictionless circuit therefore
  conserves energy at any timestep, because gravity's contribution is read off the track rather than
  integrated. A deliberately bad 1/30 s tick and a fine 1/300 s one agree on final speed to **1e-6**
  around a full loop. This is the assertion a force-integrating model fails.
- **Rolling resistance follows the normal load**, read from felt G rather than assumed to be 1 g — so
  it rises in a valley and through a hard banked turn, and falls toward zero at airtime.
- **Air drag** is lumped: deceleration is `DragK · v²`.
- **Zones** cover lift, launch, brake and station with one type. A powered section is a tractive
  acceleration that passes through the same energy accounting as everything else.
- **The train has length.** Nine sample points; the gravity term reads the change in the train's
  *mean* height rather than its centre's, because a rigid body on rails is accelerated by the slope
  its whole mass sits on. `TrainLength = 0` collapses every sample to one, so every result measured
  before this feature is bit-identical.

> Do **not** "simplify" a zone back into a post-hoc clamp on speed. That formulation manufactures
> energy on a gradient, and the amount is measured in [`PHASE0_FINDINGS.md`](PHASE0_FINDINGS.md).

`RideProfile.h` runs the train once over the finished track at edit time and records speed, all three
felt-G axes, roll rate and height, **sampled by arc length rather than by time** — so the data does
not thin out exactly where the ride is fastest and the G is worth looking at. Roll rate is a
first-class channel because no G trace can ever show it: felt G models the rider as a point, and
spinning a point costs nothing.

## Signalling and ride control

Its own C++ subsystem. Summary: each block is a `CLEAR → OCCUPIED → BUFFER(x) → CLEAR` state machine,
dispatch permissives read forward through the block list, and the control panel is *generated* from
the same ordered block and segment data that drives the geometry and physics.

Full detail: [`SIGNALLING.md`](SIGNALLING.md).

## Procedural meshing

Rail, tie and support meshes generated from the spline data using instanced static meshes.
`Prototypes/TrackSpline/TrackProfile.h` holds a generic cross-section — gauge, rail and spine
dimensions, tie spacing — so this starts from typed dimensions rather than a blank sheet.

Support auto-generation (placing and sizing towers to reach the ground) is a genuinely hard
sub-problem and is treated as its own milestone, not an assumed detail.

**Build, do not adapt.** Coaster Forge (Dualstate Games) was evaluated and rejected: it is a
commercial product and cannot be redistributed under MIT, so "adapt" was never actually available.
The one pattern worth borrowing — zone-based speed control — was arrived at independently and
verified against NoLimits 2.

## Rendering

Lumen and Nanite for park environments and lighting. For a solo developer this is the cheapest
available route to a visual bar competitive with a AAA-backed title, because Epic is doing the heavy
lifting.

VR is a deliberate later phase, not a day-one requirement — but the render and camera pipeline should
not actively foreclose it.

## Save format

Versioned, human-diffable JSON rather than an opaque binary. For an open-source project this matters
enormously: text diffs in version control, hand-editing, and third-party tooling.

The governing rule is **store what was typed, never what was derived**. Details and the measured
round-trip results are in [`AUTHORING.md#the-file-format`](AUTHORING.md#the-file-format).

## Sharing and community

There is no Steam Workshop to lean on. Plan for something simple and community-hosted early — even a
GitHub- or itch.io-hosted repository of track files — rather than deferring sharing indefinitely.
NoLimits 2's community strength came directly from Workshop-style sharing, and the death of its
Exchange platform visibly hurt it.

## Units and handedness

The prototypes are **metres, radians, seconds**, in a **right-handed** frame where
`Tangent × Lateral = Up` and `+Lateral` is the rider's **left**. Unreal is **centimetres** and
**left-handed** with `+Y` to the right.

Convert units *and* flip handedness at the port boundary, never inside the maths. The flip is not a
single sign: Unreal's `Right` is `-M(Lateral)` where `M(x,y,z) = (x,-y,z)`. Getting this wrong
mirrors the entire track and produces geometry that still looks self-consistent —
[`PHASE0_FINDINGS.md`](PHASE0_FINDINGS.md) has the measured residuals for both the correct rule and
the wrong one.

Use `FTrackFrame::Tangent` for direction. **Never finite-difference `EvaluateAt`**, and treat it as
O(track length) per call. That warning applies to anything differentiating the geometry, including
the closure solver — which is why its finite-difference step is deliberately coarse.
