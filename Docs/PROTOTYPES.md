# The prototypes

Seven standalone prototypes under `Prototypes/`, all plain C++17 with **no engine dependency**, and 36
assert suites between them. They build and run without an Unreal install, which makes them the
lowest-friction way into this codebase — and keeps the maths honest by preventing it from quietly
depending on engine behaviour.

They are not throwaways. `TrackSpline.h`, `TrainPhysics.h` and `BlockSignal.h` are the **canonical
designs**, compiled *into* the engine by `TrackUnlimited.Build.cs` rather than copied — so the
standalone assert suites test exactly the code that ships.

## Building and running

A compiler is the only requirement.

```sh
cd Prototypes/TrackSpline
clang++ -std=c++17 -Wall -Wextra -O2 -o test_trackspline test_trackspline.cpp && ./test_trackspline
```

Same shape for each of the others. **Run from inside the prototype's own directory** — the tests
include their headers by relative path. `TrainPhysics` has two suites, `test_trainphysics.cpp` and
`test_twotrains.cpp`; build them separately.

That is still the right way to iterate on ONE suite. For the other question — *is all of it still
green* — there is a runner:

```sh
cd Prototypes
./run_all_tests.sh          # CXX=g++ ./run_all_tests.sh to say which compiler
```

It builds and runs all thirty-six, prints the failing assertion for anything that breaks, and exits
non-zero — so it is a CI step as it stands. It runs each suite **from its own directory**, because
they include their headers by relative path and a couple read fixture files beside them.

Tests are plain `assert`s with no framework: add to the existing file and call your function from
`main`. `BlockSignal` and `TrackSpline` finish in well under a second; `TrainPhysics` takes about six,
because it simulates tens of thousands of ticks and each one evaluates the track — pass `-O2` if you
are iterating on it.

> Read [`PHASE0_FINDINGS.md`](PHASE0_FINDINGS.md) before changing any of these headers. It records
> what is verified, with numbers, and — more usefully — which limitations are deliberate. Several
> obvious-looking fixes have already been tested and shown to make things worse.
>
> A `ponytail:` comment marks a deliberate simplification whose ceiling and upgrade path are named
> right there in the comment. `grep -rn "ponytail:" Prototypes/` lists them all. Check whether
> something is one of these before filing it as an oversight.

---

## `TrackSpline/` — geometry

The curvature-continuous track model. Segments are curvature profiles over arc length; geometry comes
from integrating a moving orthonormal frame along that profile, so C² continuity is a property of the
data rather than something fitted.

| File | What it holds |
|---|---|
| `TrackSpline.h` | The segment model, the frame integrator, felt G, roll vs world bank |
| `TrackIO.h` | The authored data model and the diffable JSON save format |
| `TrackValidate.h` | Diagnostics, including self-clearance. Reports, never repairs |
| `TrackClose.h` | Circuit closure: measure the gap, then solve it |
| `TrackHistory.h` | Undo/redo as snapshots, with the save format as identity |
| `TrackProfile.h` | A generic cross-section — gauge, rail and spine dimensions, tie spacing |

**Proves:** curvature continuity holds across joints without fitting; heartline-relative banking
produces sensible felt G; the round trip through the save format is exact.

## `TrainPhysics/` — motion

1D train motion along that track. Gravity is an **exact energy exchange** rather than an integrated
force, so a frictionless circuit conserves energy at any timestep — a deliberately bad 1/30 s tick and
a fine 1/300 s one agree on final speed to 1e-6 around a full loop, where a force-integrating model
drifts. Rolling resistance follows the actual normal load; air
drag is lumped; one zone type covers lift, launch, brake and station — and a zone now carries **two
devices**, because a real block brake is a friction pad that can only remove energy and drive tyres that
push and hold. The pad is a **ceiling**, never a setpoint: a train already under its limit is untouched,
which is the guarantee that makes it a brake and the reason a trim cannot start a train. Asserted with a
hostile value — a bare pad whose "limit" is 40 m/s holds a 10 m/s train at exactly 10.000000, where a
setpoint would have hauled it up. Negative means *no pad*, which is every zone built before it existed,
asserted bit-identical over 900 steps.

| File | What it holds |
|---|---|
| `TrainPhysics.h` | The motion model, zones, the train's length, runtime zone retargeting |
| `RideProfile.h` | The whole ride measured at edit time — speed, three G axes, roll rate, height |
| `GEnvelope.h` | Judging that profile against published acceleration envelopes |
| `test_twotrains.cpp` | Two trains on the real preset geometry — the only file crossing all three layers |

**Proves:** energy conservation is exact rather than approximate; friction stopping distance matches
the closed form; drag decays speed exponentially at the predicted rate; a train with length behaves
differently from a point mass in the direction a designer expects — length softens sharp features and
steadies inversions.

`test_twotrains.cpp` is the integration check, and it is where a block brake can finally demonstrate
itself: with one train nothing is ever ahead of it. It transcribes
`ATUCoasterRide::TwoTrainCircuitLayout()` — the actor cannot be included from here, same precedent as
`reference_figures.cpp` — and proves the outer pre-station brake **stops a moving train** (7.64 m/s
down to 0 in 4.7 m), **holds it while the block ahead is occupied** (3.88 s, stopped *and* red, which
is a different claim from merely stopped), and **releases it** once the train ahead clears.

It also proves the layout is a **closed circuit** — position, heading and roll at the seam, plus the
total turning being one lap rather than the 446° it used to be — that a **held train parks inside its
own block** instead of straddling the seam, and that the circuit **carries four trains**: 1 through 4,
seven minutes each, every train lapping, zero violations, never a shared block.

Its last test is a line-for-line stand-in for `ATUCoasterRide::Tick`, because that function cannot be
compiled without Unreal and its dispatch policy has nowhere else to be checked: ten minutes of ride,
both trains circulating through the seam under their own power, no deadlock, no shared block, zero
violations. **Change the actor's tick order and change that test with it.**

It also **prints the circuit's canonical figures** on the way out, the same job
`reference_figures.cpp` does for the reference layout — so a number quoted anywhere about this preset
can be checked by running one thing:

```
30 segments, 1288.02 m, C2 yes
seam 0.000000 m, 0.000084 deg heading, 0.000000 deg roll
top 136.8 km/h   vertical -0.93 .. +3.46   lateral 0.30
crest 48.5 m   clearance 11.68 m   peak roll rate 17.5 deg/s   100 s
8 blocks, 5 of them able to hold a train, so 4 trains
```

### `GEnvelope.h` — turning a G number into a verdict

`RideProfile.h` reports "4.25 g at S = 310 m". This says whether that is *allowed*, against the
duration-dependent acceleration envelopes of ASTM F2291 and EN 13814. **Report, never repair**, as
everywhere else — a track outside an envelope is still buildable and still rideable, it is just told
where and by how much.

**The limit tables are UNVERIFIED.** They came from a research session, not from a copy of either
standard, and both documents are paywalled. The header says so at the top and the test suite prints
it on every run. The code is deliberately indifferent to the numbers: bands are data, and swapping in
a verified table means editing one function.

What makes it a real implementation rather than a lookup:

- **You cannot check a peak.** A limit is a curve against *duration* — 6 g is allowed for under a
  second and 3 g is not allowed for twelve. The bands are stored inverted (*no run above 4 g may last
  longer than 1 s*), which is equivalent to the published form and much easier to scan for; the header
  works a case through both ways, because getting the inversion backwards produces a checker that
  silently passes 5.9 g held for three seconds.
- **It resamples onto a uniform time base first.** `RunRideProfile` samples by *arc length*, which is
  right for a graph against S and useless here: at 1 m spacing a sample is 0.025 s apart at 40 m/s and
  0.5 s apart at 2 m/s, so durations measured on that grid measure the train's speed.
- **It filters at 5 Hz before judging**, per F2137, with a second-order Butterworth run **forward
  then backward** so there is zero phase lag. Not a nicety: the product is "go and look at S = this",
  and a single-pass IIR reports the violation half a second downstream on innocent geometry. Measured
  — a one-sample 12 g integrator artefact reads as 10.8 g unfiltered and vanishes at 5 Hz.
- **An impact is not a short violation.** Events under 0.20 s (roughly neuromuscular reaction time)
  leave the pass/fail entirely, or every curvature joint fails every track. They are reported *only
  when the exemption is what saved them* — a short excursion that would not have violated its band
  even if sustained is noise. Getting that ordering wrong gave the worked layout seven "impacts" that
  were 1.02 g wobbles over the 1 g band, which is the resting value of every ride ever authored.
- **Combined loading is a normalised ellipse**, because you cannot sit at 100% of two axis limits at
  once. Asserted with a case where all three axes pass individually and the rider is still outside.
- **One event is one finding.** The bands nest, so a 5 g pull-out crosses four of them and arrives as
  four findings for one thing an author fixes once.

**Not applied, and the verdict says so rather than a comment** — push-pull (reduced +Gz allowance
after sustained −Gz) has no reduction factor in the source, and inventing one would be worse than
omitting it; EN's measurement point is nearer head level and this model computes felt G at the
heartline.

The suite's last test runs a real 406 m layout — 96 km/h, 40 m of drop, 2.64 g at the pull-out — and
**asserts the train completed.** That assertion is the one that makes the rest mean anything: the
first three runs of this suite reported "within envelope, zero findings" on a train that crawled 46 m
up a hill it could not climb and stopped. A stalled train produces a short, flat, entirely conformant
profile, and judging a ride that did not happen is the one failure mode a conformance tool must not
have.

## `BlockSignal/` — signalling

The `CLEAR → OCCUPIED → BUFFER(x) → CLEAR` state machine and the dispatch permissive logic, with unit
tests. See [`SIGNALLING.md`](SIGNALLING.md) for what the states mean and why the buffer exists.

| File | What it holds |
|---|---|
| `BlockSignal.h` | The per-block state machine and the permissive. Knows about blocks and nothing else |
| `RideSignals.h` | The mapping layer: arc length → block index, each train's nose-and-tail range, one permissive keyed to the destination |
| `TrackSensors.h` | The sensors a PLC actually reads, and a train counter that derives occupancy from their trips alone |
| `TrackDrives.h` | The motors it commands, on the output side: ramp, feedback, torque, and a fault that stays quiet on a healthy ride |
| `StationProcess.h` | What has to happen at a platform before a train may leave it. One position, gated by contacts rather than a clock |
| `DeviceAudit.h` | What a layout's devices will actually DO, judged against the ride it produces — with the numbers in the sentence |
| `PlcUnit.h` | The controller as a MACHINE: key switch, watchdog, program identity, power. The standard PLC, not the safety chain |
| `Cia402.h` | The drive state machine, as specified. Eight states, one of which produces torque |
| `SignalWatch.h` | Change detection over named channels — edge, not level, with a seeding rule |
| `SimDigest.h` | A fingerprint of every scan, for proving two runs are the same run |
| `ShowBus.h` | Tier 3's only connection to the ride. Read-only by shape, not by policy |
| `Scenario.h` | A timeline of faults against the scan clock, so a fault has a WHEN |
| `Evacuation.h` | Can everybody get off? Catwalk spans against where trains actually stopped |

`RideSignals.h` consumes **doubles and a train index, not an `FTrain`** — `RearS`, `FrontS`, `dt`.
That keeps it independent of the physics, lets the assert suite drive it with bare numbers, and makes
the Unreal actor a caller like any other. Neither `BlockSignal.h` nor `TrainPhysics.h` was modified to
support it.

**Proves:** the permissive denies the cases it should, including the wrap-around case where a
lookahead spanning the whole circuit would include the asking train's own block; that one range diff
handles a straddle, a rollback and a lap-end teleport with no special cases; and that a train stopped
short of the station does *not* deny its own dispatch through a block it is standing in — which it
would do permanently, since nothing would ever clear it.

**Proves, for two trains,** the three identity failures that used to be silent: that a train entering
a block another is standing in is *counted* rather than suppressed, that one train moving on does not
release a block another is parked in, and that a permissive asked on B's behalf is not answered with
A's blocks. All three were confirmed to bite by mutation — reverting each identity check kills exactly
one test and no others. The single-argument forms **deny** on a multi-train instance rather than
aliasing to train 0, so a caller that forgot its index fails closed instead of reproducing the whole
class of bug.

**Proves, for a circuit,** that a reversed rear/front pair is read as a **seam straddle** — the train
holding the last block and the first — rather than sorted into a claim on the entire ring; that the
seam is not a hiding place, so a train arriving across it still collides with and is still denied by
one parked in the first block; and that a point-to-point layout's reversed pair is still sorted,
because a strip has no seam. Three more mutations, three more kills.

**The permissive derives its braking distance** when `SetHoldingBlocks` is supplied: it clears from
the destination to the next block that can *hold* a train, because a train let into a block with no
device in it is committed until it reaches somewhere it can stop. A fixed count cannot say that — on
the closed circuit, whose free runs are 696 m and 184 m, the count rule let **four trains collide**
(14 violations at lookahead 1, 18 at lookahead 2) and the derived rule runs all four clean.

**Records two limits rather than fixing them**, both asserted so they stay known quantities: a block
crossed entirely within one `dt` is never entered and never arms its overlap (needs a block under
0.5 m at 60 Hz and 30 m/s), and the enter-before-exit ordering is *not* observable with a single
train — reversing the two loops fails no assertion, and the comment says so rather than implying the
suite covers it.

Two contracts it cannot enforce from the inside, so they are the caller's: `Tick` is **once per
frame, not once per train** (overlaps live on blocks, and there is no clock in here to notice), and
update order within a frame is observable — but it **fails closed**, reporting a following train half
a frame early rather than half a frame late.

### `TrackSensors.h` — what the PLC is actually allowed to know

`FRideSignals` is *handed* the truth: each train's exact rear and front, every frame, for free. That
is the difference between simulating a control system and simulating the answer one would have got. A
real PLC has no idea where a train is — it knows a switch at 872.1 m went high. So:

```
FTrain span  ->  SENSORS (physical)  ->  PLC (logical)
```

`FTrackSensors` is the middle box and the **only** layer entitled to know a position, because a
proximity switch genuinely is a device a physical train physically covers. It reports presence and
counts edges, and deliberately does **not** report which train — a switch says "metal is over me" and
nothing else. Identity is inferred downstream from the order things trip in, which is unambiguous only
because no two trains may share a block: the interlocking and the identity tracking hold each other up.

`FBlockCounter` then derives occupancy from trips alone:

> **trains in block i = (times sensor i was entered) − (times sensor i+1 was fully cleared)**

Rising on the way in, **falling** on the way out — that asymmetry is the whole trick, and it is why
the sensor counts both edges. Use one edge for both ends and a block goes clear with a train still
lying across its boundary. It is a train counter, which railways have used for a century, and it
carries the property that matters: a block reading **two** is a collision, detected without anything
ever having known a position.

**Proves:** that the sensor layer is *sufficient* rather than decorative — the counter's occupancy is
compared against a perfect-knowledge span test on every scan of three full laps and never differs. Also
that a sensor is one device two trains cannot each decide the state of, that the circuit's seam is just
more track to it, and that a counter must be **seeded**. That last one is not incidental: computing
occupancy from lifetime edge totals sends the block *behind* a train's starting position to −1 the
moment its tail leaves the first sensor, because it gets counted out of a block it was never counted
into. Real rides are swept and their counters zeroed for exactly this reason, and `IsInconsistent()`
reports a negative count separately from a collision, because the fix is different.

**Records one limit:** a train that arrives and leaves between two scans is never seen at all — no
rise, no fall, no trace. Real hardware latches the switch so a brief trip survives until the controller
reads it, and that latch is the fix if it ever matters, not a faster scan.

**And one measured boundary: the counter is FORWARD-ONLY, and it fails safe.** A 20 m train crossing a
boundary forwards is right at every step — block 0, straddling both, block 1. Reverse it over the same
boundary and it leaves `block 0 = −1, block 1 = 2` where the truth is 1 and 0, because a rising edge
means *"metal arrived over me"* and reading it as *"a nose entered the block ahead"* is only true while
trains go one way. The saving grace is that −1 is what the counter already calls a **lie** and 2 is the
collision condition — both E-stop conditions the ride acts on — so a reversing train stops the ride
loudly rather than running on an interlocking that quietly believes the wrong thing.

**`FDirectionalCounter` is the answer, and it is a second head rather than a cleverer rule.** Two heads
a surveyed gap apart, and the *order* of their edges says which way the metal went — the same idiom
`FSpeedTraps` already uses, asked of a different question. The rule is symmetric:

| edge order | meaning |
|---|---|
| rise (First, Second) | entered the block **after** the boundary |
| rise (Second, First) | entered the block **before** it |
| fall (First, Second) | fully left the block **before** it |
| fall (Second, First) | fully left the block **after** it |

Neither direction is a special case, so every event has an exact opposite — asserted by oscillating a
train across a boundary five times and requiring the counts to land back exactly where they started.
Without that property a reverse section poisons the occupancy slowly over a day's operation. A **dead
head counts nothing** rather than guessing, because half a pair carries no direction and a counter that
invented one would be worse than one that missed. Both heads crossed in a single scan is **dropped**,
not guessed: there is no order left to read. Latency is one gap, since the block is credited when the
*second* head rises — the first moment direction is known.

Nothing consumes it yet; `FBlockCounter` is untouched and still passes its three-lap agreement test.
See [`DIRECTION_AND_ROUTES.md`](DIRECTION_AND_ROUTES.md).

**The first switch actually wired in is the stop mark.** One per zone, at
`deviceEnd − HoldNoseClearanceM`, and it turns the dispatcher's parking rule from arithmetic into
geometry: *truck forward until the mark trips.* `ServeHolds` used to compute
`zoneEnd − clearance − ½trainLength` and compare it to `GetDistance()` — two quantities a PLC does not
have. Now train length is consumed once, where the mark is placed, which is where an installer with a
tape measure consumes it, and the holding logic reads a boolean. Measured at all five devices on the
two-train circuit: the mark sits 1.00 m short of the block end, the nose parks 0.18 m past the mark,
and **0.81 m** of the metre survives — identical everywhere, because the overshoot is `v²/2a` at the
crawl speed and grip. `test_twotrains.cpp` prints the table and asserts the margin directly.

Inputs are scanned **once at the top of the frame**, before any logic runs — read inputs, execute,
write outputs, as IEC 61131-3 does — so the second train is served against the same snapshot as the
first rather than against a track the first has already moved along.

**Still handed the answer:** `FRideSignals` reads spans, not the counter. The layer underneath it is
built and proven equivalent, so the switch-over has a net to fall into, but it has not happened. See
[`SIGNALLING.md#what-the-system-actually-knows`](SIGNALLING.md#what-the-system-actually-knows) for the
full ledger of what still cheats.

### `DeviceAudit.h` — what the devices will actually do

`TrackValidate.h` reads the *authored* values and says whether they are self-consistent: a negative
radius, a clothoid that does not meet its neighbour. It can do that with the numbers alone. **Nothing
here can be answered that way.** Whether a brake is long enough depends on the train's length; whether
it can stop what arrives depends on the speed at that point, which depends on every metre of track
before it. Those are properties of the *ride* rather than the segment, so they can only be checked once
the profile exists — a separate pass, not more rules in the validator.

**The point is the sentence, not the detection.**

> *block brake at 400 m: a train arrives at 30.4 m/s and needs 154 m to stop at 3.0 m/s², but only
> 129 m is usable. It leaves the block at 12.3 m/s. Lengthen it by 25 m, brake harder, or slow the
> train upstream.*

"Too short" is a rule somebody has to go and look up and then work out whether they care about. That
is the failure — and it happens to say what to change and by how much, without choosing which of the
three answers is right. Report, never repair, with enough detail to be worth reporting.

Four checks: a device shorter than its train (with *where the stop mark lands*), one that cannot stop
what arrives (with *the speed it leaves at*), a trim used as a block boundary, and a launch used as
one. **It reads each device's own deceleration**, not a global figure — a single number for every
brake on a ride is a second source of truth and disagrees with the physics in whichever direction the
guess was wrong. On a section with a friction pad it judges with the harder of the two devices.

**Proves:** each check fires on the right thing *and stays silent otherwise*, which is the half that
matters. A healthy brake reports nothing — the assertion that stops all the others from firing
constantly while every test still passes. A trim that is **not** a block boundary reports nothing,
because trimming is what a trim is for and a checker that complained about every trim on every ride
would be switched off within a day. The exit speed is asserted against a hand-computed value, since a
number this pass *invents* is the one somebody will act on.

**Not here:** capacity and deadlock. "Is there still a holding block between these two trains" is a
property of the block *graph* and the dispatch policy, and it wants the interlocking's own walk rather
than a second one that could disagree with it.

### `TrackDrives.h` — the output side, and what a motor can disagree about

The mirror of `TrackSensors.h`. Sensors are the PLC's inputs; drives are its outputs, and the output
side was a lie in exactly the way the input side had been — `ServeHolds` wrote a speed straight into
`FTrain`, the track was at that speed the same instant, and there was nothing to disagree with.

A VFD is the muscle for **one** motor: it takes a speed command, ramps its output toward it, and turns
a tyre or a chain at whatever that means in m/s. Writing a command is the PLC's entire authority over
the ride — how fast the drive gets there is the drive's business. One drive per zone, indexed the same
way, so a friction-only trim brake keeps its slot with no motor in it.

**Three numbers that can disagree:** `Commanded` ≠ `Output` is ramping, `Output` ≠ `Actual` is slipping,
and `Load` = 1 is full torque. Load costs nothing to produce: `FTrain::Step` already clamps what a zone
asks for to what the zone *has*, and that clamped value is the force on the train.

**Proves:** that a fault needs slip *and* torque *and* time *and* **not gaining**, which is the term
that took a measurement to find. A launch is sustained slip at full torque *by definition* — 0 to
38 m/s at 6 m/s² is 6.3 seconds of it — so the first three alone report a healthy launch as a failure,
and did: the two-train circuit faulted drive 1, its launch, on the first dispatch. Removing the
"gaining" term still fails that assertion, so the guard is confirmed to bite. Also that an unloaded
drive cannot slip (without the feedback sweep, every drive faults seconds after its train leaves), that
a fault does not clear itself, and that a faulted drive keeps driving — what a ride does about a failed
motor is an E-stop policy, not a property of the motor.

**Records one deliberate default:** no ramp. Output follows command instantly, because every number
measured before drives existed was measured that way and a default that ramped would silently move all
of them. A ramp only changes anything if it is slower than the zone's own grip.

One thing it fixed on the way past: zones live on each `FTrain`, so before this every train carried its
own private idea of what every brake on the ride was doing. A drive is one device, and its output goes
to every train's copy each frame.

### `StationProcess.h` — what has to happen before a train may leave

A station is a *process*, not a place: arrive, unload, load, restraints closed and checked, platform
confirmed clear, and only then a dispatch. The block signalling is the last link in that chain rather
than the whole of it, so the permissive became an **AND** — before this a train left the instant the
track ahead was free, which is a ride with nobody in it.

**Every gate is a contact, not a timer.** The stop mark, a restraint lock sensor per car, airgate
switches, the operators' all-clear. `FAutoStationCrew` asserts them on dwell timers because nothing
here simulates a person, and it is a **separate object precisely so it can be deleted**.

**Proves:** that readiness is continuous and the release is latched, which reads like a contradiction
until it deadlocks the ride. A restraint opening in `Ready` takes the permission away the same frame —
but leaving `Departing` out of the permissive killed the real circuit, because a released train rolls
off its stop mark, is no longer "in position", loses the permissive, gets re-braked, and stops with its
tail over its own mark for the rest of the session with nothing reporting anything wrong. Measured: the
launch drive saw no train at all in seven minutes. Also that an unload-only platform must **not** wait
for locked restraints (nobody is aboard to close them), that a load-only platform must not wait for an
unload confirmation that will never come, and that the next train redoes every check — a station that
remembered the last train's would dispatch this one on them.

**One process is one POSITION, not one platform**, so a three-position load platform is three of these
in series. See [`SIGNALLING.md`](SIGNALLING.md#split-platforms-and-what-is-not-built-yet) for the rest
of that shape — multi-position platforms and turnouts — neither of which is built. A backstage buffer
is *not* on that list: it is brake sections with drive tyres, one train each, which the model already
expresses. Only the authoring of several in a row is missing.

## `TrackMesh/` — the track, the structure and the train

Phase 4. Everything you can actually see, generated rather than imported — which is not an
aesthetic preference: an imported model would be the only binary art asset in an MIT repository,
it needs a provenance check generated geometry does not, and it is the one object a contributor
cannot read a diff of.

| File | What it holds |
|---|---|
| `TrackMesh.h` | Rails, spine and ties. `WalkTrack` is the only thing that touches an `FTrack` |
| `TrackSupports.h` | Columns and footings, and where one must NOT go |
| `TrackCatwalk.h` | Evacuation decks and their handrails. The deck is level; only the track banks |
| `TrainMesh.h` | The train: chassis, bogies, three wheel sets, a lofted body, couplers |

**The split that is load-bearing** appears twice, and is the same one both times. `WalkTrack` walks
the track and `BuildTrackMesh` takes the frames it produced and has no `FTrack` at all; `BuildCarMesh`
builds one car in the car’s own local space and `PlaceCars` decides where cars go and produces no
geometry. `EvaluateAt` is O(track length) per call, so a mesher that called it per ring would be
O(n²) — the exact bug the vertical slice’s debug draw had. Splitting it this way makes that
mistake **unreachable rather than discouraged**, because neither half holds the data to make it with.

**Proves:** swept surfaces are watertight and outward-wound, measured by signed volume rather than
eyeballed — a closed mesh encloses positive volume only when every triangle faces outward, so one
reversed cap is a number rather than an opinion. A curve tighter than half the gauge folds the inner
rail through its own axis, and is reported rather than repaired. A column never passes through the
track it is not carrying. The train is built from `CarCount` and `CarLengthM` — **a sample point is
not a car**, and drawing off the nine physics samples is what made a 15 m train look accidentally
plausible and the 6 m small-batch vehicle look like nine playing cards on edge.

**Articulation comes free**, and it is the one thing here worth understanding before changing it:
each car takes its frame from *its own arc length*, so a rigid body chords across an arc while the
wheels stay on it. Asserted against `sqrt(R² + h²) - R`, which is not the chord sagitta — 0.0562 m
against 0.0563 m on a 20 m radius, close enough to look right if the placement were wrong.

> **THE PORT RULE, and it was backwards for months.** `M(x,y,z) = (x,-y,z)` is a reflection, so it
> reverses triangle orientation — and UE’s front-face rule is the opposite handedness and reverses
> it back. **Two flips. Do not swap indices at the port.** It hid because a thin tube inside out has
> the same silhouette; it showed the day something SOLID was drawn. A car body is solid.

## `NL2Csv/` — validation fixtures

Reads and writes NoLimits 2's documented tab-separated spline export — position plus front/left/up
unit vectors, one row per sample, which is an `FTrackFrame` sequence and so reads with a `strtod`
loop. Real NL2 layouts can therefore be driven through this model, and our tracks opened in NL2 to
compare G and speed traces against an independent simulator.

The writer exists so the reader is testable with no NL2 install.

**This is a validation path, not an authoring path** — see
[`AUTHORING.md#importing-from-nolimits-2`](AUTHORING.md#importing-from-nolimits-2). Do not commit NL2
park files or exports of real rides; `Prototypes/NL2Csv/Tracks/` is gitignored for that reason.

## `NL2Telemetry/` — live comparison

Captures NoLimits 2's live telemetry, so the model can be checked against a running independent
simulator rather than only against closed forms. `calibrate.cpp` fits `RollingResistance` and `DragK`
against a recorded speed trace.

**Also proves a negative, which is why it earns its place:** on the recording available, the two
coefficients cannot be separated. The correlation between their predictors — `N·g` and `v²` — is
**0.975** with a condition number around 2000, so no fit can attribute the energy loss to one rather
than the other; the split between them is a free parameter, not a result. Visible as `DragK` flipping
sign across train lengths while the residual barely moves. The cause is the ride, which tops out at
44.5 km/h over 234 m, and drag needs speed to have anything to be measured against. `DragK` therefore
stays at its derived value, and a *fast* reference recording is the actionable next step.

It did move one default, though. Pin `DragK` and rolling resistance alone is well conditioned: three
recordings converge on **0.022–0.026** against a shipped 0.006, with a residual of 0.0005 m/s² — which
says the model's *shape* was right and its coefficient was not. Read alongside the fact that 0.006 had
been justified as steel-on-steel, where a coaster actually runs **polyurethane** on steel at
0.01–0.03, `RollingResistance` was corrected to **0.024**. See
[`PHASE0_FINDINGS.md`](PHASE0_FINDINGS.md).

---

## Conventions

Metres, radians, seconds. A **right-handed** frame where `Tangent × Lateral = Up`, and `+Lateral` is
the rider's **left**. Unreal is centimetres and left-handed. Convert units *and* flip handedness at
the port boundary, never inside the maths — see
[`ARCHITECTURE.md#units-and-handedness`](ARCHITECTURE.md#units-and-handedness).

Use `FTrackFrame::Tangent` for direction. **Never finite-difference `EvaluateAt`**, and treat it as
O(track length) per call.

**Check new type names against all of Unreal, not just against this repo.** These headers are
engine-free but they are compiled *into* the engine, so a name only needs to be unique against the
whole engine. `FFrame` collided with the Blueprint VM's and became `FTrackFrame`; `FField` collided
with `UObject/Field.h`'s and became `FTrackIOField`. Standalone clang compiles both cleanly, so the
collision only appears at the port.
