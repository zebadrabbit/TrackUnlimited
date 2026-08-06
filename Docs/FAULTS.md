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

## One caution about calling this training

There is a real use for a rig that can fault a ride and show a crew what the system does — and it is
a use nothing else in this market offers. Worth being deliberate about two things before the word
"training" appears anywhere user-facing:

- **This is not certified training material and must not read as if it were.** A park that used it to
  train operators would be relying on a model whose limits are recorded in
  `PHASE0_FINDINGS.md` — a heartline point mass, no rider biomechanics, an unverified acceleration
  envelope table, and a signalling model that still hands the interlocking a span rather than
  inferring identity from trip order.
- **The MIT licence's no-warranty clause is a legal statement, not an ethical one.** If the tool
  invites that use, the honest thing is to say plainly what it does and does not model, in the
  product rather than only in the docs.

Neither is a reason not to build it. Both are reasons to decide the wording once, deliberately, the
way `UI_CONVENTIONS.md` settles the design language.
