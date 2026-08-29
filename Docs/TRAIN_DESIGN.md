# The Train — design sheet

**DESIGNED, NOT SCHEDULED.** Written 2026-08-19 for the Phase 4 card *"Train mesh — cars,
bogies, articulation"*, which existed only as a deferral inside another card from 2026-08-02
until that day. Nothing here is built. Read it before writing the mesh, and read
`PHASE0_FINDINGS.md` before changing any number it quotes.

## The decision: PROCEDURAL, like everything else this project draws

Rails, spine, ties, columns, footings, catwalks and handrails are all generated from
`Prototypes/TrackMesh/`, engine-free and assert-tested. **An imported model would be the
only binary art asset in the repository**, and four separate constraints point the same way:

- **Constraint 6 (MIT).** Everything committed has to be redistributable. A model needs a
  provenance check that generated geometry does not.
- **Constraint 5 (no manufacturer designs).** A generated train is specified by dimensions
  nobody owns. A downloaded one is a question about whose it was.
- **The UI card already made this argument.** Widget Blueprints were flagged as *"the part
  nobody can review"* because binary assets do not diff. An FBX is worse: a car body is the
  most looked-at object in the simulator, and it would be the one thing a contributor cannot
  read a change to.
- **There is no artist.** Solo project, no budget. A parametric train ships this month; a
  modelled one waits on somebody who does not exist yet.

**This is not a decision against art.** A styled body shell replacing the lofted one is a
clean later pass — see *Two passes* below. The chassis never needs redoing.

## Nothing here is a new number

Every dimension already exists. This spec's job is to say which list drives the mesh, not to
invent measurements.

| Quantity | Value | Where it lives | Note |
|---|---|---|---|
| Train length | 15 m (6 m small-batch) | `TrainLengthM` | **derived**, `CarCount × CarLengthM` |
| Cars | 5 (2 small-batch) | `CarCount` | authored — *"nobody specifies a train in metres"* |
| Car length | 3.0 m | `CarLengthM` | **coupled pitch, not body length** |
| Heartline height | 1.1 m | `FTrack::GetHeartlineHeight` | where the rider and the camera sit |
| Gauge | 1.10 m | `FTUTrackStyle::GaugeM` | rail centre to rail centre |
| Rail diameter | 0.115 m | `FTUTrackStyle::RailDiameterM` | what the wheels run on |
| Spine drop | 0.45 m | `FTUTrackStyle::SpineDropM` | negative on an inverted style |
| Body width | 1.4 m | current cube placement | *"a shade under a metre and a half"* |
| Body height | 0.9 m | current cube placement | **load-bearing, see below** |

**0.9 m is not a guess and must not be raised.** The heartline is 1.1 m above the rail
centreline. A body taller than that swallows the point the ride camera sits at, and the
rider spends the lap inside a box. A real car body comes up to about the chest of a seated
rider for exactly the same reason — you have to be able to see out of it.

## THE CURRENT DRAW IS OFF THE WRONG LIST

`Cars` is a `UInstancedStaticMeshComponent` of `/Engine/BasicShapes/Cube.Cube`, one instance
per **physics sample point** — `NumSamplePoints()`, nine — spaced `TrainLengthM / 8`.

Nine is a *physics resolution*: it is how finely gravity reads the train's mean height, and
it is right for that. **It is not the number of cars, and cars are authored.** The evidence
is already in the code comment: dividing the length into nine made the 15 m train look
"accidentally plausible" and the 6 m small-batch vehicle look like "nine playing cards
standing on edge". The boxes were then made to touch, which hid it. The wrong source stayed.

**The mesh is built from `CarCount` and `CarLengthM`. The sample points keep doing physics
and stop doing rendering.** The two lists are allowed to differ and always will — a nine-
point train of five cars is the normal case, and neither number is derivable from the other.

## Anatomy, in the order worth building

### 1. The bogie, and why it comes first

**A coaster train is not a railway train, and the difference is almost entirely the wheels.**
Three sets per bogie, gripping the rail from three directions:

- **Running wheels** — on top of the rail, carrying the weight. Polyurethane, which is not
  cosmetic trivia: `RollingResistance = 0.026` is measured for *polyurethane on steel* and
  the header argues it at length. The wheel that gets drawn should be the wheel the physics
  was fitted to.
- **Side friction wheels** — against the inner face, taking lateral load.
- **Upstop wheels** — underneath, and these are what make the ride possible at all. They are
  why a train survives −0.94 g on the showcase instead of leaving the track.

All three are cylinders at known offsets from a rail whose centreline, diameter and gauge
the profile already carries. **This is the piece that makes it read as a coaster** — from
the chase camera a wheel assembly wrapped around a rail is more of the silhouette than the
fibreglass is, and it is exactly what a spline in space most obviously lacks.

### 2. The chassis

A beam between the two bogies. Boxes and a sweep, no shaping to speak of. Its job is to be
what the body and the bogies both attach to, so articulation has somewhere to happen.

### 3. The body shell

Lofted: one cross-section swept along the car with `SweepTube`, the same call the rails and
the handrails already use. 1.4 m wide, 0.9 m tall, seated into the chassis.

**A TUB, NOT A BOX (2026-08-27).** The first section had a roof, and the first lap bars
closed into it and vanished, then poked through it when raised — reported from a screenshot
within the hour. The section now goes up the outer flank, over a rolled rim, DOWN the inner
wall and across a cabin floor: open on top because riders sit in it. The cabin floor sits
just above the shoulder where the flank first reaches full width, so the cavity cannot poke
out through the taper over the wheels — the rider sits on the wheel wells, as on a real
car. Non-convex, so the end caps are ear-clipped rather than fanned, and the tub is still
watertight and outward-wound (1.428 m³ of shell with its bulkheads, where the box enclosed 2.669). Two seats a
row — squab and backrest, their own buffer because upholstery is not gelcoat — and the lap
bar's hinge is DERIVED from the squab, since a bar hinged at an authored height swings over
nobody.

**TWO ROWS, AND A BULKHEAD AT EACH END (2026-08-28).** One row left half of a 2.7 m shell
empty on screen; `RowsPerCar` is 2 now, spaced evenly along the shell by `RowCentreX` so the
pitch is the shell's rather than a second authored number, and the station reads the same
number for its gates. And the tub's end caps close the SHELL'S THICKNESS and nothing else, so
the first tub was open front and back — a front-row rider had nothing between their knees and
the car ahead. `EndPanelThickM` is a bulkhead at each end, cabin floor to a centimetre under
the rim, let INTO the walls and floor by 2 cm rather than butted against them (two closed
bodies sharing a face weld into one with a doubled edge, the seats' lesson) and set 1 cm in
from the end so its face is not coplanar with the cap. Asserted as geometry: a body triangle
inside the cabin at each end, which the bare tub never has.

**This is the only part with taste in it**, and the only part a later art pass would
replace. Keep it a single function returning a cross-section, so replacing it is one edit.

### 4. Couplers and articulation

**The part that sells it, and it comes almost free.** Each car takes its frame from *its own
arc length*, so through a tight curve the bodies visibly chord across the arc while the
bogies stay on it — the motion nobody manages to fake, and the reason a coaster train looks
alive from outside. It falls out of placing each car at its own S rather than interpolating
one transform for the whole train.

A coupler is a short strut between adjacent chassis ends: `SweepStrut`, which the ties and
the support legs already share.

### 5. Restraints — NOT this card

`FCommandedBank` already models commanded position, travel time and per-group sensing, and
`DrawRestraints` puts debug boxes on screen. The state is real; the geometry is a later
pass, and the hook it needs already exists.

## The camera does not want a BONE. It wants a SEAT.

The obvious move is a socket on the mesh for the ride camera to hang off, one per seating
position. **Do not do that, and the reason is the one this project keeps rediscovering:
it would be a second source of truth for where the rider is.**

Felt G is computed at the **heartline**. If the camera hangs off mesh geometry and the G
readout comes from the heartline, the two can disagree — and the entire reason the heartline
model exists is that a rider's eye position and a rider's felt G are *the same point*. A
socket authored on a rig is a number somebody can nudge; the heartline is derived. There is
also a mechanical objection: the mesh is `UProceduralMeshComponent`, so a bone means going
skeletal, which means an imported asset, which is the thing this document just argued out.

**A seat is DATA, and the frame is DERIVED from it** — exactly as `SeatOffset` already works
today (`RiderPosition × TrainLengthM × 0.5` into `GetFrameAt`, which returns a heartline
frame and needs no mesh at all).

```
FSeat { int Car; int Row; double LateralM; double VerticalM; int FacingSign; }
SeatFrame(train, seat) -> FTrackFrame     // one answer, several consumers
```

**One answer, three consumers**, which is the `GraphRect` and `ConsolePlatformPtr()` rule
again — the camera sits in it, the mesh puts the seat and the restraint there, and
`GEnvelope` judges the G there.

Two things fall out, and neither is speculative:

- **`RiderPosition` should become discrete.** It is a continuous −1..+1 slider, which is the
  same class of mistake as drawing off the sample points: a real ride has row 1 and row 2,
  not 0.37 of a train. Discrete seats give the camera somewhere exact to sit *and* stop the
  readout describing a rider who is half in one car.
- **This is the wing-coaster prerequisite.** `COASTER_TYPES.md` says a laterally offset rider
  feels roll-rate × offset that the centre seat never does, that it is measurable with what
  `GEnvelope` already has, and that **it applies to the outer seats of every wide train**.
  That measurement needs a lateral offset per seat. `LateralM` is it. Three separate cards
  want this object; it is not being built for the camera alone.

## Rules the mesh inherits

- **The rider sits at the heartline; the body sits on the rails.** Already correct in the
  cube placement (`Position − Up × (HeartlineHeight − BodyHeight/2)`), and the whole reason
  the heartline model exists. Must survive the rewrite.
- **Watertight, and outward-wound by signed volume.** The bar the track, the supports and
  the catwalks are all held to. A closed mesh encloses positive volume only when every
  triangle faces outward, so reversing one is measurable rather than a matter of opinion.
- **THE PORT RULE: `M(x,y,z) = (x,−y,z)` is a reflection, and you DO NOT swap indices.**
  Corrected 2026-08-09, after months of every surface being inside out. Two flips — the
  reflection and UE's opposite front-face rule — cancel. Doing one explicitly is what broke
  it. **A thin tube inside out has the same silhouette; a solid body does not**, which is
  precisely how it was finally caught, and a car body is solid.
- **Facing.** `FacingSign` already exists for backward-facing seats. The mesh must not
  assume a nose.
- **A type is a preset, never a branch.** Per `COASTER_TYPES.md`: inverted is `SpineDropM`
  negative, flying is that plus a prone seat. The train mesh reads the style; it must never
  learn that coaster *types* exist.

## Two passes, and the second one is where Blender belongs

**Pass one — card #141, no dependencies.** Chassis, bogies, three wheel sets, a lofted body,
couplers. Procedural, engine-free, assert-tested, MIT-clean. Ships without an artist, and
ends "the ride payoff is grey boxes".

**Pass two — a styled body shell, whenever somebody wants it.** *That* is a modelling job,
and if it is done in Blender the useful output is a **cross-section profile**, not a train:
a handful of points replacing the loft's section function, staying parametric and staying
diffable. A generated `.py` that emits the section beats a `.blend` nobody can review.
Licence-check anything imported against `NOTICE.md` first.

The split matters because pass two must not touch pass one. The chassis, the wheels and the
articulation are engineering; only the shell is taste.

## Explicitly out of scope

Riders; a second train style; per-seat lateral offset
(that is the wing-coaster work, and `CarCount`'s own comment already notes *"a car is not a
seat row yet"*); wheel spin; suspension travel; a train interior for rider-mode look-around
(recorded as blocked on art in the Phase 2 camera card — this unblocks it, it does not
deliver it).
