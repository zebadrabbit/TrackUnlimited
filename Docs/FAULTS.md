# Faults, and what notices them

Injecting failures into the ride and recording **whether the safety design catches them**. Breaking
things is easy; the artifact worth having is the second column.

**Status: the taxonomy holds and most of it is built.** Every hook marked ✅ exists and is
asserted, and `Prototypes/BlockSignal/Scenario.h` schedules them against the scan counter. What is
still hand-rolled is the assertion vocabulary, not the mechanism.

---

## Where a fault belongs

Not a central fault manager. Every physical failure in a control system shows up as one of three
things, and each layer already owns one of them:

| what fails | how it appears | who owns it |
|---|---|---|
| a sensor, its wiring, its face | **an input that lies** | `FTrackSensors` |
| a motor, a contactor, a brake pad | **an output that does not take effect** | `FTrackDrives` |
| a restraint, a gate | **feedback that disagrees with the command** | `FCommandedBank` |

That taxonomy is why the injection hooks sit on the devices. A manager reaching into all three would
have to know each one's internals, and the devices already model the disagreement — `Commanded` vs
`Output` vs `Actual` was built for exactly this before anybody was thinking about faults.

**Determinism is a requirement, not a nicety.** A scenario that reproduces differently every run
cannot prove anything. There is no random source anywhere in these prototypes, and `Chatter` toggles
on a scan counter for that reason.

---

## The matrix

| fault | injectable | what catches it | how |
|---|---|---|---|
| Restraint group will not close | ✅ `FCommandedBank` `StuckGroup` | **Dispatch held, for ever** | `IsClosedAndLocked()` is ANDed over groups; the permissive never completes. Not timed out of the way. |
| Restraint group will not release | ✅ same hook | **Reported per group** | `GroupState() == Stuck`, drawn red on the car. A rider who cannot get out is a different emergency, not a lesser one. |
| Gate section jammed | ✅ `FAutoStationCrew::StuckGate` | **Dispatch held, for ever** | Same shape; asserted separately because the two banks are commanded from adjacent lines. |
| Block sensor dead | ✅ `ESensorFault::Dead` | **E-stop** | The counter and the interlocking disagree. Neither can say which is wrong — which is the property a second detection method buys. |
| Block sensor stuck on | ✅ `ESensorFault::StuckOn` | **E-stop, and it fails SAFE** | Over-reports occupancy, so the ride stops rather than admitting a train. Not a mirror image of Dead — see below. |
| Loose connection / chatter | ✅ `ESensorFault::Chatter` | **E-stop** | Edges from nothing; the counter drifts from the span and the cross-check trips. |
| Drive slipping / stalled motor | ⚠️ detection only | **Fault → E-stop** | Slip **and** torque **and** time **and** not gaining. No injection hook yet; feedback can be reported by hand. |
| **Failed brake** | ✅ `FTrackDrives::SetDeliveredFraction` | **Depends entirely on where the ride was** | Three measured answers, one of which is "nothing". See below — still the most interesting entry here. |
| Broken wire on a safety input | ✅ `FSafetyChannel` | **Reads as a pressed button** | Inputs are normally-closed, so a stop is demanded by the ABSENCE of current. Nothing has to know why it stopped. |
| Welded contactor | ✅ `FContactorPair::Weld` | **Blocks the reset, for ever** | Mirror contacts (NC, mechanically linked) in series into the reset circuit. One weld is latent and the second is power reaching a machine the relay switched off — and now that is expressible. |
| Short to the supply on one channel | ✅ | **The other channel's polarity** | Two channels of OPPOSITE polarity: the fault that makes one read safe makes the other read demanded. Beyond the discrepancy window it latches. |
| A diverse sensor pair loses one | ✅ `FDiversePair` | **Degrades to a stop** | Not to the surviving sensor. One dead and one saying safe is a single point of failure wearing the appearance of a checked one. |
| Restraint power lost | ✅ `FRestraintLock` | **Bars stay DOWN** | The exception that proves the rule: fail-safe is not "de-energise", it is "fail to the state that cannot hurt anybody", and for a restraint that is LOCKED. |

### De-energise to trip, and the wiring under the E-stop

**BUILT 2026-08-06.** `SafetyIo.h`. `FTrackDrives` already held the E-stop and its stop categories;
this is everything BELOW that, which the project had been modelling as plain booleans.

**A safety input is a circuit, not a button.** It carries current while all is well, and a stop is
demanded by the ABSENCE of current — so a broken wire, a pulled connector, a dead supply and a
pressed button are the same signal, and nothing has to know which. The failure modes of the wiring
land on the safe side by construction rather than by being enumerated. It is the same shape as three
things already here — the fail-safe brake, CiA 402's active-low quick stop, the block counter's
falling edge — and this is where the idea finally gets named.

**Two channels, opposite polarity.** One channel fails to danger in one specific way: a short to the
supply makes a normally-closed contact read energised for ever, which reads as safe, which means a
pressed E-stop does nothing. Doubling the channel does not help — one short in a shared loom takes
both. Opposite polarity does: the fault that makes one read safe makes the other read demanded. That
is the difference between **redundancy and diversity**.

**A stop is EITHER channel, not both.** A pair that required agreement would be a system where one
broken wire disables the E-stop.

**The comparison is not instant.** A dual-channel button's two contact blocks never switch on the
same millisecond, so an instant comparison faults on every legitimate press. Hence the discrepancy
window — 0.5 s here, 3 s on some relays. Measured: 50 ms of disagreement is fine, a full second is a
latched fault.

**Instantaneous and delayed outputs, and Cat 1 is impossible without both.** A relay whose contactors
open the instant the button is pressed leaves nothing to ramp with, so "controlled stop with power
retained" would be a Cat 0 wearing a label. The delayed output is where the retention physically
lives, and its deadline is the same 5 s the drives already hold — the same relay described from two
sides.

> **One safety mechanism caught another's bug during this work.** The delay was armed from
> construction rather than from a transition, so a relay that had never been enabled held its
> contactors closed for the first five seconds of its life. It surfaced because the mirror contacts
> then refused the very first reset, correctly reporting that the mains had never opened. That is
> worth more than the assertion that noticed.

**A restraint fails the other way, and that is not an exception to the rule.** Fail-safe was never
"de-energise"; it is *fail to the state that cannot hurt anybody*, and for a brake that is applied
while for a restraint it is shut. `FRestraintLock` needs command AND power to unlock, and can say
which of the two identical-looking kinds of "locked" it is in.

**Not modelled:** OSSD pulse testing, cross-monitoring between separate relays, PL/SIL arithmetic.
Those are how an installation *proves* a category rather than implements one, and they need a
failure-rate model this project has no business inventing.

### Dead and stuck-on are not mirror images

A dead sensor **under**-reports and a stuck one **over**-reports, and only one of them fails safe.

- Stuck on: a block that never goes clear. The ride stops. Annoying, safe.
- Dead: a block that never goes occupied — a train admitted on top of another.

That asymmetry is the whole argument for wiring detection so the **safe** direction is the
de-energised one, and it is why the second detection method exists rather than a better single
sensor. Asserted both ways in `test_tracksensors.cpp`.

### The failed brake, now measured

A brake that does not bite is **expressible** as of 2026-08-06:
`FTrackDrives::SetDeliveredFraction(zone, 0..1)`, pushed to every train's copy of that zone each
frame exactly as `Output` is, and multiplying both tractive authorities where `FTrain::Step` clamps
what a zone asks for to what it has.

Three things stay separate, and keeping them separate is the whole design. The **command** is still
correct, the **drive** still writes it, and the **output** still reaches it — a glazed pad does not
change what the PLC asked for or what the caliper did about it. So a degraded device still reports
`IsReady`, and it must: refusing a dispatch because a pad is worn would be a fault detector wearing a
permissive's clothes, and nothing has measured that the pad is worn. Health lives on the **drive**,
not on `FTrackZone` (a track file must not be able to ship a broken brake) and not on `FTrain` (every
train carries its own copy of every zone, so two trains would disagree about one piece of hardware).

The section this replaces predicted one outcome. There are **three**, and which one you get depends
entirely on where the ride was when the device failed. Measured on the two-train circuit, every zone
killed outright, at two, three and four trains — `test_twotrains.cpp`,
`TestAFailedBrakeAndWhatDoesAndDoesNOTCatchIt`.

**1. A dead device with a train standing on it stops the ride, and nobody arranged that.** The
dispatcher's rule is *truck forward until a switch says far enough*. A train on a device that
delivers nothing never reaches its stop mark, so it is never in position, so the permissive never
grants. Zero laps, zero violations, nothing moves. The ride does not dispatch a train it could not
have held — a fail-safe falling out of having made holding a question a **switch** answers instead of
a number in the program, and the strongest argument yet for that decision.

**2. Loaded, two independent mechanisms catch it.** Three trains, transfer tyres killed while a train
is *approaching* rather than standing on them: it overruns into a block it was not given, the
interlocking raises a violation at 32.3 s, and the block counter — which knows only that switches
tripped — independently counts a block occupied twice. Both trip the E-stop.

**3. Sparse, it is ABSORBED — silently.** Two trains, the outer brake killed. The arriving train is
not held there, reaches the transfer tyres at **16.1 m/s instead of 6.1**, and is stopped by them.
The layout has the length. No violation is raised because no train was ever endangered.

> **This entry said something else for a few hours and it was wrong.** The claim was that the train
> "circulates at 30.5 m/s past a station it should be parked in, undetected" — a runaway nothing
> caught. The 30.5 was the train's speed at the *end* of a four-minute run, which says only that it
> was somewhere fast at t = 240, and it was read as evidence of something it does not show. The speed
> trap below is what corrected it, by measuring what the train was actually doing at every device on
> the way round. The original card text — *"the layout catches it, if the layout has somewhere to
> catch it"* — turns out to have been right.

**So a failed brake on a well-laid-out ride is not a safety event at all.** It is a **capacity and
schedule** event: a block that should have held a train did not, the headway is wrong, and every
downstream device is working harder than it was specified to. Nothing in this model reports any of
that, and the detector actually missing is *"a train did not stop where it was told to"* — not
overspeed.

### The speed trap, and what it turned out to be for

**BUILT 2026-08-06.** `FSpeedTraps` in `TrackSensors.h`. Two switches a **surveyed** distance apart
and a clock: speed is the gap over the time between their rising edges, with no position and no train
identity. That is how a real installation measures it, and it is the shape `FTrackSensors` already
had. The gap is read from the sensor positions **once, at commissioning**, and never again — the same
idiom as the stop mark consuming train length at survey rather than in the dispatcher.

Four things it gets right, each asserted:

- **The quantisation is real and is reported.** A scan is a tick, so at 240 Hz a train doing 30 m/s
  covers 0.125 m per scan and a 1 m trap measures it in 8 ticks — a **12.5% bound** before anything
  else goes wrong. A 10 m trap is 80 ticks and 1.25%. What is asserted is the *bound*, not the
  realised error: quantisation is not monotonic in gap width on any single pass, and the first
  version of that test asserted the wrong one and failed.
- **A backwards pass is counted, never turned into a speed.** Second switch without the first is a
  rollback or a missed trip; both are worth knowing and neither is a measurement.
- **A train that stops between the switches disarms the trap.** Without it the next train through is
  measured against a clock started minutes ago — worse than no reading, because it looks like one.
- **It reports; it does not decide.** The comparison against `v²/2a` for the device ahead is the PLC
  program's, the same rule the drives' fault detection runs on.

**And on the real circuit it never trips, which is the result rather than a disappointment** — a
protective detector that fires when nothing is unsafe is worse than none. What it shows instead is
the **stopping margin**, and that is the useful number: healthy, the tightest moment on the whole ride
is the outer brake at **13.7 m** to spare; with one brake dead the worst case moves to the transfer
tyres and falls to **5.1 m**. Still positive, so still absorbed — and down by nearly a factor of
three. A margin degrades continuously where a trip is a cliff, and it is what an operator should
actually be shown.

---

### A degraded device is silent about itself

Worth knowing before it surprises somebody: the drive layer does **not** fault on its own
degradation. `FTrain::GetZoneLoad` reports the applied acceleration as a fraction of the zone's
**authored** authority, so a device delivering 30% reads 30% torque — and the drive's fault rule
needs *full* torque. That is deliberate on both counts. It makes the degradation directly visible on
the VFD module rather than hidden behind a saturated bar, and a device reporting its own failure is
precisely the thing a second, independent means of detection exists so as not to have to trust.

## The scheduler exists

`Prototypes/BlockSignal/Scenario.h`: a timeline of injections against the **scan count**, not the
wall clock — a step is due on an exact scan, and a scan that was skipped counts the step as missed
rather than firing it late. Stable-sorted, rewindable, and deterministic in the sense that matters,
which is that `FSimDigest` fingerprints every scan so two runs can be compared as runs rather than as
outcomes.

Actions cover the restraint and gate banks, the sensors, the drives (`DegradeDrive`, where `B` is a
percentage so healing is the same action with `B = 100`), the operator's controls, and the
controller's power. What is still hand-rolled is the **assertion vocabulary** — *did the ride stop,
how long did it take, did a train move while anything was unsecured* — of which the per-frame
securing invariant and the failed-brake test in `test_twotrains.cpp` are the worked examples.

## A foundation, not a claim

Somebody could build a training rig on this. **This project does not make that claim and should not
start**, and that is a settled position rather than a caution to revisit: nobody here is an authority
on operator training, certification is not ours to grant, and a simulator that says it trains people
has taken on an authority it has not earned.

What this project owes such a builder is different, and more useful:

- **Stable hooks.** Faults injectable at the devices, the event stream on disk, deterministic replay.
  Those are the things that are painful to retrofit and cheap to design in, and they are what the
  matrix above is really for.
- **Honest limits, written down.** A downstream builder needs to know exactly what they are standing
  on. `PHASE0_FINDINGS.md` is the ledger — a heartline point mass, no rider biomechanics, an
  acceleration envelope table that is **unverified research rather than a copy of either standard**,
  **CiA 402 bit assignments written from memory** (the state machine is asserted; the numeric bit
  positions are not verified against IEC 61800-7-201, and are the first thing to check if this ever
  meets real hardware), and an interlocking still handed a span rather than inferring identity from
  trip order. Every one of those matters to somebody deciding whether this is a fair basis for
  anything.
- **Nothing dressed up.** The reason the matrix records what is *not* caught, and the reason
  `GEnvelope.h` prints its own provenance on every run, is that a foundation is only worth building
  on if it is candid about where it stops.

The MIT licence's no-warranty clause is a legal statement, not an ethical one. Being straight about
the model's limits is the ethical one, and it is the whole of what this project has to do here.
