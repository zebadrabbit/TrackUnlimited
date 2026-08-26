# The reference layout

`ATUCoasterRide::ReferenceLayout()` in
[`Source/TrackUnlimited/TUCoasterRide.cpp`](../Source/TrackUnlimited/TUCoasterRide.cpp) — the
hand-authored layout the vertical slice rides, and the fixture most of this project's numbers come
from.

**This page is the canonical home for its figures.** If another document disagrees with this one,
this one is right and the other is stale — see [the note at the bottom](#a-note-on-stale-figures).

![The reference layout, drawn as a side elevation with dimensions and callouts](../Brand/github/layout-1280x560.png)

The diagram is drawn from the same measurement as the table below, not retyped from it. The brand
pack that renders it is no longer part of the codebase — only the finished figures are tracked, under
`Brand/github/` — but the measurement is: see
[Reproducing these numbers](#reproducing-these-numbers).

## Measured

Sixteen typed segments. Nothing below is estimated or rounded from a design intent; every figure is
read off a ride profile run over the built geometry.

| | |
|---|---|
| Segments | 16 — 4 straight, 12 raw (9 pitch, of which 3 are the side-stepping loop; 3 the banked turn, authored in the frame the loop leaves) |
| Developed length | **591.86 m** (sum of segment arc lengths, along the heartline) |
| Horizontal extent | 367.43 m |
| Lift crest | **50.07 m** above station datum, at S = 159.0 m |
| Lowest point | 0.00 m — the layout does not go below its own station |
| Closure | ends **+0.0015 m** relative to the station |
| Curvature continuity | ✅ verified to **1e-9** across all 15 joints |
| Top speed | **107.5 km/h** (29.87 m/s) at S = 282.9 m |
| Vertical G | **+0.66 to +4.51 g** (min at S = 173.8, max at S = 331.2) |
| Peak lateral G | **0.37 g** at S = 429.2 m |
| Peak roll rate | 66.7 °/s at S = 415.5 m |
| Loop | R9.0 m, 54 m eased in and out. Apex **27.90 m** at S = 359.5 m (+1.77 g there) |
| Loop, minimum felt G | **+1.73 g** at S = 357.0 m — 2.52 m of arc *before* the apex, at 27.55 m |
| Loop legs | **3.46 m** apart at closest (S = 325.0 / 394.0 m); side-step 5.5 m, exit twist 17.9° — against the 3.0 m corridor |
| Banked turn | R32.0 m at **65.92°**, clothoid in and out |
| Ride time | **63.1 s**, dispatch to standstill at S = 567.7 m in the brake run |
| Heartline | 1.1 m above rail centreline |
| Train | **15 m** long, nine sample points |
| Chain speed | 4.0 m/s |

> **Every dynamic figure above moved on 2026-08-06**, when `DragK` was measured at **0.000100**
> against a 142 km/h coast-down and the previously *derived* 0.00045 turned out to be 4.5× too high.
> Nothing about the geometry changed — length, crest, closure, continuity, the loop's radius and
> apex height are all identical. What moved is everything that depends on how much energy the train
> keeps: the ride is faster, and it pulls harder.
>
> The loop is where it shows most. Apex felt G went **+1.16 → +1.78 g** and the minimum through the
> loop **+1.13 → +1.74**, because the train arrives with more speed left. Peak vertical reached
> **+4.52 g**, and peak lateral rose from 0.28 to 0.37. The previous numbers were not measurement
> error — they were a correct simulation of a train dragged 4.5× harder than a real one.
>
> See `PHASE0_FINDINGS.md` for the recording and the fit.

Sampled at 0.5 m. The G extremes are the sampled ones — at 1.0 m spacing the maximum reads slightly
differently, which is the sampling grid landing differently on a sharp peak, not a difference
in the physics.

The two loop rows are **two different points, not two samples of one**, and finer sampling will not
make them converge. Felt G bottoms out slightly *before* the top because the train is still slowing
as it climbs the back of the loop, so the speed that sets the centripetal term is still falling while
the height is still rising. Quote +1.13 g for ride feel and 27.90 m for height; quoting "apex 27.63 m"
conflates them, which an earlier version of this page did.

## Zones

Zones come from contiguous runs of segments carrying the same zone kind.

| Zone | Arc length | Notes |
|---|---|---|
| Station | 0 → 20.00 m | chain at 4.0 m/s |
| Lift | 20.00 → 160.67 m | chain continues over the crest |
| Course | 160.67 → 414.72 m | drop, pull-out, loop |
| Banked turn | 414.72 → 521.72 m | free running |
| Brake run | 521.72 → 591.72 m | release speed 0.0 m/s |

Station and lift are one zone as far as the physics is concerned — both are `Lift` at 4.0 m/s and
they are contiguous, so `RebuildFromSegments()` emits a single zone spanning 0 → 160.67 m. The split
above is presentational.

## Two things worth knowing about it

**The chain runs *over* the crest, and the window is narrow in both directions.** Release at the top
of the climb and the train strands, because the straight tops out while the track is still rising.
Carry the chain down the far side and it is worse — `MakeLift` holds a fast train back as well as
pulling a slow one, so it sits on the train at 4 m/s down a 34° drop and takes the ride's energy with
it. Only the **first** of the crest's two eased-pitch segments is powered; pitch crosses zero 17.4 m
into a 20.6 m segment, so the chain lets go just past the top. Powering both halves instead costs
10 km/h of top speed and flattens the loop apex.

**The loop side-steps, and its two legs pass 3.46 m apart — since 2026-08-26.** Until then it was
planar, built from pure pitch curvature, and its legs passed **0.189 m** apart against rails 1.215 m
wide: a rider through solid steel, with every segment valid, every joint continuous and every G
reading plausible. It was left in deliberately because every cheap fix measured worse than the
defect — constant torsion anywhere on it either made it circular (+9.49 g) or translated it 9 m
down per metre of side-step (see [`PHASE0_FINDINGS.md`](PHASE0_FINDINGS.md)).

The fix is a new authored quantity rather than a tuning: **`TorsionRatio`**, torsion as a constant
multiple of curvature. By Lancret's theorem that makes the eased loop a *generalised helix* — its
tangent keeps one angle ρ to a fixed horizontal axis — across the curvature ramps constant torsion
could never follow. The loop projects onto the plane perpendicular to that axis as the very teardrop
it always was (curvature scaled by sin²ρ, length by 1/sin ρ) and advances along the axis at cos ρ per
metre, so the tangent returns **exactly** (5e-10 measured), the height is preserved, and the legs
land 5.5 m apart along the axis — 3.46 m at their closest, because they cross about 63 % of the loop
apart. The ratio is `5.5 / 110.5 = 0.0498`, ρ = 87.15°. **Every ride figure above is unchanged to
the second decimal**: the projection *is* the old loop, the length grew 0.14 m, and peak lateral G
stays 0.37 g because the rider's up follows the bend (`RollMode = FollowsTorsion`) — measured
path-relative instead, the loop's whole twist would read as lateral G. The path frame leaves the
loop twisted by **2π·cos ρ = 17.9°**, and the banked turn and brake run are authored *in that frame*
(`InTwistedFrame`: curvature rotated back, roll carrying the offset) — a level turn typed as plain
yaw in the twisted frame pitches by sin(twist) of its turning, measured as **24 m of drop** before
that was done.

## Reproducing these numbers

No Unreal install needed. There is a committed driver that does exactly this —
[`Prototypes/TrackSpline/reference_figures.cpp`](../Prototypes/TrackSpline/reference_figures.cpp).
It mirrors `ReferenceLayout()` segment for segment, runs the ride profile, and prints every figure on
this page:

```sh
clang++ -std=c++17 -O2 -I Prototypes/TrackSpline \
    -o reference_figures.exe Prototypes/TrackSpline/reference_figures.cpp
./reference_figures.exe
```

That is what makes this page checkable rather than trusted, and it is why the driver stayed when the
brand pack left: it was never branding, it is the measurement. (`--data-js` still emits the pack's
data file, for anyone keeping a local copy of it.)

To write your own instead:

```cpp
#include "TrackIO.h"      // Prototypes/TrackSpline
#include "RideProfile.h"  // Prototypes/TrainPhysics

// ...build the same FAuthoredSegment list as ReferenceLayout(), then:
FTrack Track = BuildTrack(Doc);
FTrainConfig Cfg;  Cfg.TrainLength = 15.0;      // the slice rides a train with length
FTrain Train(Track, Cfg);
// ...add the zones from contiguous runs of segment zone kinds, then:
FRideProfile P = RunRideProfile(Train, Track, 0.5);
```

```sh
g++ -std=c++17 -O2 -I Prototypes/TrackSpline -I Prototypes/TrainPhysics -o export export.cpp && ./export
```

Two settings account for most of the ways a reproduction can disagree with this page:

- **`TrainLength`.** The default is `0.0` — a point mass. The vertical slice sets 15 m
  (`ATUCoasterRide::TrainLengthM`). A point mass on this layout reads noticeably lower peak G,
  because it pays the full depth of the loop's bottom where a long train does not.
- **Which crest segment carries the lift zone.** See above. Getting this wrong is a 10 km/h error.

`P.bCompleted` is `false` for this layout and that is correct, not a stall: the brake run's release
speed is 0.0 m/s, so the train stops inside it rather than reaching the end of the track.

## A note on stale figures

Several documents in this repository still publish earlier figures for this layout. They are wrong,
for three separate reasons, and none is a mistake in the reasoning that produced them:

**543.7 m and a 34.9 m lift** predate the closure solve. The layout used to end 8.498 m *below* its
own station and run its whole back half underground, with the rail centreline bottoming out at
−9.60 m. `TrackClose.h` fixed it by freeing the lift climb: 55.0 m → 75.11 m. That is +20.11 m of
25° climb, which is exactly +20.11 m of developed length and +8.50 m of crest — hence 543.73 → 563.84
and 34.85 → 43.35.

**563.8 m, a 43.4 m lift, 101.0 km/h and 0.36 lateral** predate the rolling-resistance correction of
2026-08-02, which is the change that produced the figures now at the top of this page. `RollingResistance`
had been justified against steel-on-steel — a railway figure, where a coaster runs polyurethane on
steel — and correcting 0.006 → 0.024 left the loop cresting at +0.13 g. The drop went 12 m → **24 m**
to restore +1.13 g, and the lift 75.11 m → **90.99 m** purely to close the ending back to station
level. Hence 563.84 → 591.72 developed, 43.35 → 50.07 crest, and the lateral peak easing 0.36 → 0.28.
See [`PHASE0_FINDINGS.md`](PHASE0_FINDINGS.md).

**+0.70..+4.25 g** is stale for a third reason and was already out of date when the changelog
re-quoted it. The changelog's reasoning is sound — the drop geometry had not moved at that point, so
the ride figures really were unchanged across the closure fix, and that reproduced exactly. But the
figures it carried forward dated from an earlier state of the physics and ride-profile code. The
loop-apex value in particular drifted four ways across the docs (+1.05, +1.25, +1.34, +1.52) and none
of them reproduces against current code; it is **+1.13 g**, measured above.

If you are updating those documents, this page is the source to copy from. If you change the layout,
change this page first.
