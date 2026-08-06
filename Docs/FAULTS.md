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
| **Failed brake** | ❌ | **Nothing directly** | See below. The most interesting gap. |
| Broken wire on a safety input | ❌ | **Nothing** | Needs inputs modelled normally-closed — de-energise to trip. On the Tier 1 card, not built. |
| Welded contactor | ❌ | **Nothing** | Needs external device monitoring: NC aux contacts in series into the reset, so a welded contactor blocks it. On the Tier 1 card. |

### Dead and stuck-on are not mirror images

A dead sensor **under**-reports and a stuck one **over**-reports, and only one of them fails safe.

- Stuck on: a block that never goes clear. The ride stops. Annoying, safe.
- Dead: a block that never goes occupied — a train admitted on top of another.

That asymmetry is the whole argument for wiring detection so the **safe** direction is the
de-energised one, and it is why the second detection method exists rather than a better single
sensor. Asserted both ways in `test_tracksensors.cpp`.

### The failed brake is the real gap

A brake that does not bite is currently **inexpressible**, and it is the failure with the least
protection:

- The train overruns its stop mark, enters the next block, and the interlocking raises a violation →
  E-stop. So it *is* detected, indirectly.
- But an E-stop cannot stop it either, because the thing that stops trains is the brake that failed.
  What actually arrests it is the **next** block brake, or an anti-rollback catch.

So the honest answer today is "the layout catches it, if the layout has somewhere to catch it" — and
that is a property worth being able to *measure* per track rather than assume. Injecting it needs a
zone whose commanded deceleration is not the deceleration it delivers, which the drive layer's
`Output` vs `Actual` split can already express and the physics does not yet read.

---

## What is missing is the scheduler, not the concepts

Three faults are injectable and asserted, and all three are set by hand in a test. What does not
exist is anything that says *"jam group 2 at the load platform at t = 40 s, and clear it at t = 90"*.

That is the scenario layer, and it wants:

- a timeline of injections against the sim clock, deterministic and replayable
- the transition log (`bLogStateTransitions`) as its recording, which is already built and already
  writes to `Saved/Logs/` under `LogTUEvents`
- an assertion vocabulary — *did the ride stop, how long did it take, did a train move while
  anything was unsecured* — of which the per-frame securing invariant in `test_twotrains.cpp` is the
  first example

Not scheduled. The pieces it needs are being built as they come up for other reasons, which is the
right order.

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
