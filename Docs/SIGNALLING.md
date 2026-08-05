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

## How many trains a circuit carries

There is a formula, and it is short:

> **trains = (blocks that can stop a train and let it go again) − 1**

The minus one is the whole thing. One block has to stay empty, or every train is standing exactly
where the train behind it needs to go and the ring cannot rotate — no violation, no crash, just a ride
that never moves again. Asserted for rings of three to eight blocks in `test_ridesignals.cpp`, driven
on bare numbers because capacity is a property of the signalling and has nothing to do with geometry.

Note what counts. A **launch** cannot stop a train and a **trim brake** cannot release one, so neither
is a block boundary for this purpose however much hardware is bolted to it. Only drive-tyre runs and
block brakes count.

That is also the reason a high-throughput ride is built with **more block sections** rather than
longer trains: sections buy trains, and trains buy capacity. The two-train preset has five holding
blocks and runs four; the other three presets have a station and a trim brake, so they have one, and
one place is one train.

## Three different things that all look like "a brake on the track"

Worth separating, because the count a real ride quotes is usually **not** its block count, and
conflating them makes a layout look like it should carry far more trains than it can.

| | What it is for | Does it bound a block? |
|---|---|---|
| **Block brake** | Interlocking. Stop a train, hold it, release it on a permissive. | **Yes** — this is the only one that does |
| **Evacuation zone** | Getting *riders off*. Needs walkway, access, egress route — not just a way to stop. | No |
| **Safety catch** | Rollback, or a defect. Passive, and fails closed. | No |

A large ride can have on the order of **25 evacuation zones** and a handful of catch brakes while
running far fewer trains than that, because those counts answer *"can everyone get off safely"* and
*"what happens when something fails"*, not *"how many trains fit"*. Only the first row feeds the
capacity formula.

**The third row is now built.** `FTUTrackSegment::bAntiRollback` marks a segment as ratcheted, the
actor derives contiguous runs into arc-length spans exactly as it does zones, and `FTrain` refuses to
let a train move backwards through one. It is deliberately **not** a zone and not a block boundary:

- Not a zone, because a zone is a *control* device — it has a speed, an authority, and something
  commanding it every frame. A catch has none of those and nothing to command. It also **overlaps**
  zones freely, since a lift hill is a powered run and a ratchet at the same time, which one
  enumerator could not say.
- Not a block boundary, because it cannot *release* a train. It is no more a place to park one than a
  trim brake is, for exactly the same reason.

A caught train is **arrested**, not merely stopped from moving: throwing away the backward advance on
its own would leave it pinned in place carrying its backward velocity for ever, which is not a state
any hardware can be in. The dog takes the energy into the structure.

`FRideProfile::bCaughtByAntiRollback` reports it as a **third distinct outcome**. A stall says the
hill is too tall. A rollback says that *and* that the train is loose heading the wrong way. A catch
says the hill is too tall and **the safety device did its job** — nobody is in danger and the layout
is still wrong. Reporting that as a success because nothing bad happened would be the wrong lesson.

**Evacuation zones are still not modelled at all** — they want walkway geometry, which is Phase 4
meshing territory, and a definition of *reachable* that nothing here has.

Asking for more trains than the layout can carry is refused with a log line rather than granted onto
open course.

Holding devices rest **closed**. A device that opens because nobody is asking is a device that fails
open, so the resting state is brakes-on and a permissive is what opens one — for the frames it is
granted, and no longer.

And holding a train happens in **two stages**, because the hardware is two devices sharing a stretch
of track:

1. **The pad stops it.** A sensor sits just before the brake, so it trips as the train *enters* and
   clamps a fin under the car. It stops the train as hard as it is allowed to — and the limit is
   **rider comfort, not distance**, because the alternative is whiplash. The train therefore stops
   wherever that lands. Measured on this circuit: **0.6–0.7 g** fore-aft, which is where real block
   brakes sit.
2. **The tyres convey it.** Only once it is stopped do drive tyres engage and move it forward into an
   acceptable holding position.

The dispatcher says both in one number — a crawl speed — because a zone closes the gap to its target
using its full authority: from 26 m/s the pad bites with everything it has, and from rest the tyres
push. The sequence falls out.

**The conveying stage is not cosmetic.** Brake alone and the train stops about 0.3 m past the zone
start; the station's start is the circuit's seam, so that leaves the back half in the *last* block, a
dwelling train holds two, and three trains deadlock — each denied by the tail of the one in front,
with nothing reported. Real rides reposition for exactly the same reason.

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

One more that is about the *ride* rather than the signalling, and it is now **done** rather than
limited. The two-train preset is a genuinely closed circuit — 0.000000 m of position, 0.000084° of
heading, 0.000000° of roll — and trains **lap** it, driving through the seam into the station under
their own power. `FTrainConfig::bCircuit` wraps arc length instead of clamping, and the actor sets it
by *measuring* the seam on every rebuild rather than by being told: a layout whose ends are hundreds
of metres apart still returns its trains to the station by teleport, which is the honest thing to do
with track that does not meet.

The part of that which belongs in this document is what a **reversed rear/front pair means**. On a
point-to-point layout it is a caller passing its arguments the wrong way round, and gets sorted. On a
circuit it is a train **straddling the seam**, holding the last block and the first — and sorting it
claims every block *between* them, which is the whole ring, from one train, with nothing reported.
`FRideSignals` therefore walks a possibly-wrapped range through a single helper, so the entry test,
the exit release and the occupancy query cannot disagree about what "the range" is.

The seam is also where a **dwelling train** used to cost a second block, because the station's start
*is* the join — see the holding section above for why that deadlocked three trains and what replaced
it.

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
