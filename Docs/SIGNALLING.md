# Signalling and ride control

The pillar with no real precedent in either NoLimits 2 or Planet Coaster: simulating the *operator's*
side of running a coaster.

![CLEAR to OCCUPIED to BUFFER(x) to CLEAR](../Brand/github/signalling-1280x420.png)

**Status:** the state machine, the permissive logic and the layer that maps a train's position onto
them all exist as unit-tested standalone prototypes (`Prototypes/BlockSignal/`). What does *not*
exist yet is the last hop: nothing in the Unreal actor constructs any of it, so a running train still
trips no block. That and the generated control panel are Phase 3 — see [`ROADMAP.md`](ROADMAP.md).

## Contents

- [Why this is worth building](#why-this-is-worth-building)
- [The block state machine](#the-block-state-machine)
- [Dispatch permissives](#dispatch-permissives)
- [Circuit topology, and what it does not cover yet](#circuit-topology-and-what-it-does-not-cover-yet)
- [Automatic and manual dispatch](#automatic-and-manual-dispatch)
- [The generated control panel](#the-generated-control-panel)
- [Making the causal chain visible](#making-the-causal-chain-visible)

---

## Why this is worth building

Neither NoLimits 2 nor Planet Coaster meaningfully simulates block signalling, dispatch permissives,
or the control-room hardware — VFDs, indicator panels — that real ride operators actually work with.
It is a genuine, currently-unclaimed differentiator rather than a nice-to-have.

It is also the *lowest-risk* pillar in the plan, which is not obvious from the outside. It is a
from-scratch C++ rebuild of a block-occupancy-plus-buffer system the developer has already designed,
shipped and validated once in Blueprint. That deserves the confidence that comes with "I have built
this before", not the caution owed to the untested parts of the architecture.

## The block state machine

Each block is a small state machine:

```text
CLEAR ──train enters──▶ OCCUPIED ──train exits──▶ BUFFER(x) ──overlap elapses──▶ CLEAR
```

| State | Meaning |
|---|---|
| `CLEAR` | No train, and no overlap being held. Safe to admit one. |
| `OCCUPIED` | A train is physically within the block. |
| `BUFFER(x)` | The train has left, but the block **withholds CLEAR** until a configurable safety overlap has elapsed. **Time-based today** — `FBlockConfig::BufferSeconds`. Distance-based overlap is deferred until the signalling layer can read train position. |
| `CLEAR` | Back to the top. |

The `BUFFER` state is the whole point, and it is named after the real-railway **overlap** concept: the
margin beyond a stop signal that must also be free before the signal behind it can clear. A block
that goes CLEAR the instant a train's tail crosses the boundary is a toy. Real systems hold a margin,
and holding it is what makes the timing of a busy dispatch cycle feel like a real one.

## Dispatch permissives

A **dispatch permissive** is the logic gate that allows a station or launch to release a train.

It reads *forward* through the block list: a station may dispatch only once the next block reports
CLEAR — and for high-speed sections, further ahead than that. The rule being expressed is the
**braking-distance** one: a launch that can put a train at 100 km/h into a block it cannot stop short
of should not be permitted to fire, regardless of what the block immediately ahead says.

`FBlockController` implements the *count*: `CanDispatch(FromBlock, Lookahead)` requires the next
`Lookahead` blocks to report CLEAR. That was the whole of it for a while, and it is a **guess at a
distance** — which is fine until a layout has free runs of 696 m and 184 m, where no single count is
right for both.

`FRideSignals` now derives it instead. `SetHoldingBlocks` supplies one bool per block — does a device
here exist that can stop a train *and* let it go — and the permissive clears **from the destination to
the next block that can hold the train**, however many that is. The reasoning is one sentence: a train
let into a block with nothing in it is **committed**, because there is nothing in there to stop it, so
everything up to its next chance to stop has to be clear before it may go.

Measured on the closed two-train circuit, which has 8 blocks of which 5 can hold:

| trains | fixed count | derived |
|---|---|---|
| 1–3 | clean | clean |
| 4 | **14 violations** at lookahead 1, 18 at lookahead 2 | clean |

The failure is always the same shape: granted a free block, committed, and the block beyond it
occupied on arrival.

The list is optional — without it the fixed count is used unchanged. And the two rules stack, so
`DispatchLookahead` is now **extra headway rather than the safety requirement**; its default is 1, and
raising it on a tight ring buys separation at the cost of throughput.

This is the piece that turns a set of state machines into a safety system, and it is why the logic
lives in C++ rather than Blueprint.

## Holding a train, and where you are allowed to

**A train is held by gating its zone, never by declining to integrate it.** The first version of the
slice held the train at the station by simply not stepping it, which worked for exactly one train in
exactly one place; commanding the device under it to zero holds a train anywhere there is a device to
hold it, and the station stops being a special case.

That makes "where may a train be held" a real question, and it has a two-part answer.

**The device needs both authorities.** A friction or magnetic brake can stop a train and can never
start one again; a launch can start one and can never stop one. Only a drive-tyre run has both, which
is why every real block brake is *brakes plus tyres* and why the authored vocabulary has a
`BlockBrake` kind distinct from `Brake`. `FTrain::FindHoldZoneAt` refuses the other two outright: a
gated launch is an *aborted* launch, and a train parked on a trim brake stands there for the rest of
the session with no symptom but a ride that quietly stopped.

**The block needs the length.** Stopping the train it actually receives costs `v²/2a`, and no zone
kind can change that. On the two-train preset the mid-course brake receives a train at 28.19 m/s and
would need 66.2 m to stop it inside a 45 m block, so it is authored as a trim and stays one —
authoring it as a block brake would build a device that closes and is then run straight through, which
is worse than a trim because it *looks* like an interlock. The three pre-station devices pass the same
test with room to spare (4.8, 1.5 and 0.9 m needed) and are the ones that hold.

Together these cap how many trains a layout can run: **the number of places that can both hold a train
and release it**. Four on the two-train preset, one on every other, and asking for more is refused
with a log line rather than granted onto open course.

Holding devices rest **closed**. A device that opens because nobody is asking is a device that fails
open, so the resting state is brakes-on and a permissive is what opens one — for the frames it is
granted, and no longer.

And a held train is braked to a **position**, not to zero wherever it happens to be. A zone says
*reach this speed*, and zero is reachable immediately, so a train commanded to hold stops within about
0.3 m of where the zone starts. In the station — whose start is the seam of the circuit — that leaves
the back half of the train in the *last* block, so a dwelling train holds two. On a tight ring that is
enough to deadlock three trains, each denied by the tail of the one in front, with nothing reported.
Commanding `sqrt(2·a·d)` instead eases the train down and parks it mid-device, which needs no new
authored concept because the dispatcher was already setting the target every frame.

## Two trains, and the three reads that carry identity

`FRideSignals` tracks N trains against one `FBlockController`. Widening the storage was never the
work; three reads had no notion of *which* train they were about, and each one failed open silently
until it did:

- **Entry.** "Was I already in this block" silently meant "was the last train to update already in
  it", so a train rolling into an occupied block matched and the only function that can report a
  violation never ran. A collision was *suppressed*, not missed.
- **Exit.** Releasing "the" old range meant one train walked out of another's blocks on its behalf,
  and a block read CLEAR with a train parked in it. A block is now released only when no *other*
  train's range covers it.
- **The permissive.** Skipping "blocks the asking train holds" fired for whichever train updated
  last, so asked on B's behalf it skipped A's blocks as B's own and granted entry into an occupied
  one.

`Tick` is **once per frame, not once per train** — overlaps live on blocks, so N calls a frame expire
a 5 s overlap in 5/N seconds and nothing in the layer can tell. Within a frame the update order is
observable and **fails closed**: a train not yet updated still reads at its previous span, so a
following train is reported half a frame early rather than half a frame late.

## Circuit topology, and what it does not cover yet

The block list is modelled as a closed **circuit**: lookahead wraps past the last block to the first,
and a lookahead spanning the *whole* circuit is denied — because it would include the asking train's
own block, and a train cannot clear itself.

That is correct for a circuit and **wrong for a shuttle or a transfer spur**, which will need an
explicit topology when they exist. This is a known limitation, recorded rather than papered over.

Two more, for the same reason. `FBlockController` takes **enter/exit events keyed by block index** and
holds no train identity, deliberately — the identity lives one layer up, in `FRideSignals`, which is
the only place that knows where each train is. And the **block count is fixed at construction** from
the config vector, so re-blocking a layout means building a new controller rather than mutating one.

One more that is about the *ride* rather than the signalling. The two-train preset is a genuinely
closed circuit — 0.000000 m of position, 0.000084° of heading, 0.000000° of roll — but `FTrain` still
clamps at the end of the track, so a lap is a **teleport** from the last block back to the station
rather than a wrap. The signalling sees that jump and interlocks it with no special case: the old
range exits and arms its overlaps, the new range enters, and the arriving train is held at the end of
the track until the station is clear. Because the seam closes to zero, the teleport is now invisible.

Making it a real wrap is `S -= TotalLength` **plus** one thing that belongs in this document:
`FRideSignals::Update` swaps a rear/front pair that arrives reversed, and on a circuit a reversed pair
is not an error — it means the train **straddles the seam**, holding the last block and the first. The
swap would instead claim every block between them, which is the whole ring. That is a range that has
to wrap, not a range that has to be sorted.

## Automatic and manual dispatch

Two modes, matching how real ride control actually runs:

- **Automatic** — the system dispatches as soon as the permissives are satisfied.
- **Manual** — a human operator decides when to dispatch or hold.

**The safety interlocks apply identically in both modes.** Manual mode changes *who decides the
timing*, never *whether the safety logic can be bypassed*. Any design that lets manual mode override
a permissive is wrong, and would also be a poor simulation of a real system, where it is precisely
what the interlocks exist to prevent.

## The generated control panel

The panel is **generated, not hand-built per coaster**. Walk the same ordered block and segment list
that drives the 3D mesh and the physics, and for each element emit its control-room counterpart:

| Source | Panel element |
|---|---|
| a block | an indicator — clear / occupied / buffer countdown |
| a powered segment (lift chain, tire-drive launch) | a **VFD module** |

A VFD module shows what a real variable-frequency drive shows: target frequency or speed, actual
motor feedback, torque or current draw, and ramp rate.

This makes the panel a second generated *view* over the same canonical track data as the geometry and
the physics — one more thing driven parametrically rather than authored by hand, consistent with how
the track editor itself works. Adding a block to a layout adds an indicator to the panel because
there is nowhere else for it to come from.

Presentation is staged deliberately: a **2D generated HMI panel first**, which keeps the system
strictly data-driven and cheap to build, with an optional 3D-modelled control booth as a later
presentation layer over the same underlying data — not a separate system.

## Making the causal chain visible

Just as important as the data model: when a sensor trips, that should visibly propagate.

```text
sensor trips  →  indicator lights  →  logic evaluates  →  actuator commanded  →  actuator responds
```

A VFD ramping a motor, a brake engaging — the response should be *observable*, not an instantaneous
state change. That visibility is most of the pedagogical and differentiating value of this pillar. It
is what makes the system feel like a real ride control system rather than an invisible if-statement,
and it is the reason the panel is worth building as a first-class feature instead of a prop.
