# Faults, and what notices them

Injecting failures into the ride and recording **whether the safety design catches them**. Breaking
things is easy; the artifact worth having is the second column.

**Status: partly built.** The injection hooks marked ✅ below exist and are asserted. Nothing
schedules them yet — there is no scenario layer, and that is the missing piece rather than a missing
concept.

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
| Broken wire on a safety input | ❌ | **Nothing** | Needs inputs modelled normally-closed — de-energise to trip. On the Tier 1 card, not built. |
| Welded contactor | ❌ | **Nothing** | Needs external device monitoring: NC aux contacts in series into the reset, so a welded contactor blocks it. On the Tier 1 card. |

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

**3. Sparse, NOTHING catches it.** Two trains, the outer brake killed. The arriving train is not
stopped, rolls through into the next block — *which was empty* — and keeps circulating. Four minutes,
two laps, still doing **30.5 m/s** past a station it was supposed to be parked in, and **not one
violation**.

That third case is the honest state of things and the reason this entry is no longer titled after the
interlocking. Block signalling answers *"is the block ahead free"*; it was. It protects trains from
each other and was never a check on whether a device works — a sparse circuit simply has the room to
absorb a failure. The old text's "so it *is* detected, indirectly" was true only for case 2 and was
never measured.

**What is missing is the question no layer here asks: is this train going too fast for what is in
front of it.** The fix is a **speed trap** — two block-boundary switches a surveyed distance apart,
giving speed from the time between trips, with no position and no train identity. That is how a real
installation measures it and it is already the shape `FTrackSensors` has. Compare it against `v²/2a`
for the next holding device, which build time already computes in
`TestEveryHoldingBlockCanActuallyStopWhatArrives`. Not built: it is a new detector rather than a
wiring job, and case 3 above is what justifies building it.

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
