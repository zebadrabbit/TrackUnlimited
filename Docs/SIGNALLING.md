# Signalling and ride control

The pillar with no real precedent in either NoLimits 2 or Planet Coaster: simulating the *operator's*
side of running a coaster.

![CLEAR to OCCUPIED to BUFFER(x) to CLEAR](../Brand/github/signalling-1280x420.png)

**Status:** the state machine and permissive logic exist as a unit-tested standalone prototype
(`Prototypes/BlockSignal/`). Wiring them to the ride, and generating the control panel, is Phase 3 —
see [`ROADMAP.md`](ROADMAP.md).

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

What the prototype implements is the *count*. `CanDispatch(FromBlock, Lookahead)` requires the next
`Lookahead` blocks to report CLEAR, and `Lookahead > 1` is how a braking-distance requirement is
expressed today. **Deriving that number from an actual braking distance is not implemented** — it
needs train speed and position, which `FBlockController` deliberately cannot see, and it belongs to
wiring signalling to the ride in Phase 3.

This is the piece that turns a set of state machines into a safety system, and it is why the logic
lives in C++ rather than Blueprint.

## Circuit topology, and what it does not cover yet

The block list is modelled as a closed **circuit**: lookahead wraps past the last block to the first,
and a lookahead spanning the *whole* circuit is denied — because it would include the asking train's
own block, and a train cannot clear itself.

That is correct for a circuit and **wrong for a shuttle or a transfer spur**, which will need an
explicit topology when they exist. This is a known limitation, recorded rather than papered over.

Two more, for the same reason. The controller takes **enter/exit events keyed by block index** and
holds no train identity, so it can say a block is occupied but not by which train — fine for
interlocking, not enough to drive a panel that labels trains. And the **block count is fixed at
construction** from the config vector, so re-blocking a layout means building a new controller rather
than mutating one.

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
