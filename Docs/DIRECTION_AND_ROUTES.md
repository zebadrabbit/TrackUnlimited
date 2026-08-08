# Direction and routes

Two capabilities this system does not have, written down before anything commits to a shape that
would make them expensive.

**Status:** design only. No code exists for either. Nothing here is scheduled — it is here so that
work which *is* scheduled does not accidentally rule it out.

## Contents

- [Why these two, and why together](#why-these-two-and-why-together)
- [Part 1 — direction is not one sign](#part-1--direction-is-not-one-sign)
- [Part 2 — a route is not a block](#part-2--a-route-is-not-a-block)
- [Where the two meet](#where-the-two-meet)
- [What already exists](#what-already-exists)
- [Explicitly out of scope](#explicitly-out-of-scope)
- [Sequencing](#sequencing)

## Why these two, and why together

Two rides prompted this, and the developer's framing is the right one: **they are casual concepts
wearing a mask.**

- **Big Thunder Mountain** — two parallel stations, switches in and out, sharing one circuit.
- **Expedition Everest** — the train climbs, the track ahead is "damaged" (a prop), and while you are
  looking at it a switch moves *behind* you. You roll backwards down a different route.

Neither is exotic. The first is a track merge; the second is a reverse section. Between them they
need exactly two general capabilities, and a great many ordinary rides would use them: shuttle
coasters, turntables, transfer tracks, dual-load platforms, backwards-facing trains, and every
layout with more than one path through it.

**Writing them as ride features would be the mistake.** A "Thunder Mountain mode" is a special case
that rots; a block graph with routes is a model. The same is true of reverse: a "reverse section"
flag would need touching by every rule that assumes forward, and would be wrong in each one
independently.

They are in one document because they interact, and the interaction is the part that is easy to
miss — see [Where the two meet](#where-the-two-meet).

## Part 1 — direction is not one sign

### Travel and orientation are independent

The model has one sign today: the arc-length parameter, and the train advances along it. That
conflates two things that are genuinely separate on real rides.

| Facing | Moving | Ride |
|---|---|---|
| forward | forward | everything currently modelled |
| forward | **backward** | Everest's reverse — riders travel backwards, the cars never turn |
| backward | forward | a turntable or a shuttle that spun the train |
| backward | backward | the same ride as row 1, seen from the other end |

**The proposal is two signs on the train, not a reparameterised track.** Arc length stays fixed and
monotone — it is the survey, and a survey does not change because a train is going the other way.
The train carries:

- `TravelSign` — which way it is moving along `S`.
- `OrientationSign` — which way the nose points along `S`.

Every derived rule then asks for `LeadingEdge = OrientationSign * TravelSign` rather than assuming a
nose. That is what makes the rider-frame flip fall out instead of being special-cased, and it is how
real systems carry it: a railway does not renumber its mileposts for a train running in reverse.

### What breaks, deepest first

**The G envelope gives a silently wrong verdict.** This is the sharpest one and it is measured, not
supposed. `GEnvelope.h` has asymmetric fore-aft bands:

```
PosGx = {{6.0, 0.0}, {2.0, 4.0}}    // +Gx eyeballs-in: 6 g with a headrest, 2 g without for 4 s
NegGx = {{2.0, 4.0}}                // -Gx back-to-chest: 2 g, 4 s
```

Three to one, and correctly so — a body tolerates being pressed into a seat far better than being
thrown out of it. A train riding backwards has **both signs swapped in the rider's frame**: its
braking becomes eyeballs-in and its acceleration becomes back-to-chest. Judged against the forward
bands it will flag a safe event and pass an unsafe one — and report "within envelope" while doing
it. That is the same class of failure as the backwards band reading already recorded in
`PHASE0_FINDINGS.md`, and it is worse, because there is nothing odd-looking to notice.

**The stop mark trips on the nose.** A span covers a point the moment its front reaches it — but
rolling backwards, the leading edge is the tail. The rule becomes "leading edge in the direction of
travel". One line, and one very easy silent failure: a train that never trips its mark keeps
crawling.

**The block counter double-counts.** `FBlockCounter` is entries at sensor *i* minus full clearances
at sensor *i+1*, rising in and falling out. That assumes monotone travel. A train that backs off a
sensor and re-trips it counts twice, and the cross-check against the interlocking then trips the
E-stop on a ride doing nothing wrong.

**The real answer is hardware, not software.** A directional axle counter is two heads a few
centimetres apart; the phase between them says which way the axle went. That is an addition to
`FTrackSensors` in the same spirit as the speed trap — two switches, a surveyed gap, and no
identity — rather than a rule bolted onto the counter.

**`FTrackZone::TargetSpeed` is unsigned.** And Everest's reverse is not a push: the brake *releases*
and gravity does the rest. So what is needed is a holding device whose release direction is
backwards, which the zone model cannot currently say. A signed target is the obvious answer and
wants care — a signed speed makes "faster" ambiguous everywhere it is compared.

**Anti-rollback must be defeatable on that section.** This is safety-relevant rather than a
convenience: a catch you can release is a different device from one you cannot, and the release is a
commanded action with a permissive of its own. `bAntiRollback` is currently passive and fails
closed, which is right for every layout that has one today.

**The dispatch permissive looks ahead.** "Ahead" flips. `FRideSignals::CanRelease` clears from the
destination to the next holding block, and both of those are directional.

## Part 2 — a route is not a block

### What a switch actually breaks

Not the geometry. A turnout is two curves and a frog, and the geometry layer could express one
today. What it breaks is that **every block in this system assumes a ring.**

- `FBlockCounter` is a counter over a ring and says so — on an open layout the wrap is false.
- The permissive walks blocks by index, and "the next one" is `b + 1`.
- `ZoneReleaseSpeed` and the stop marks are indexed lists parallel to a linear zone walk.

A switch means a block has **more than one successor**, so index arithmetic stops being a topology.

### What replaces it

**Route interlocking**, which is a subsystem rather than an addition. The pieces, all of them real
railway concepts with real names:

- **A route is a sequence of blocks plus the point positions it requires.** Granting a dispatch
  becomes *setting a route*, not clearing a block. This is the central change: the permissive stops
  being a question about occupancy and becomes a question about whether a route can be set.
- **Points lock when a route is set**, and stay locked until the train has cleared them. A point that
  can move under a train is the classic derailment, and "locked" has to mean the control layer
  cannot move it — not that it politely does not.
- **Conflicting routes are detected from the topology**, not listed by hand. Two stations onto one
  main line means only one of those routes may be set at a time, and a system that required an author
  to enumerate the conflicts would be wrong the first time somebody added a third path.
- **Flank protection** — a route is not safe merely because its own blocks are clear, if a train on a
  converging path could run into its side.

### Facing and trailing are properties of the route

A **trailing** point takes two paths into one in the direction of travel; a **facing** point splits
one into two. They are different hazards: a facing point failing under a train derails it, which is
why real ones carry a separate facing point lock, and a trailing point is far more forgiving.

**Which one a point is depends on the direction the train is travelling, so it cannot be a property
of the point.** It is a property of the route through it. Everest makes this unavoidable and is the
reason the two halves of this document belong together.

## Where the two meet

On Everest, **the same physical point is trailing on the way up and facing on the way back.**

The train climbs over it — two into one, trailing, forgiving. It stops. The point moves behind it,
which is only permissible because the train has *cleared* it, which is exactly the point-locking
release rule. Then the train rolls back over the same point in the other direction, where it is now
facing, one into two, and the hazard class that needs the lock.

One point, both hazard classes, and nothing distinguishes them but direction.

This is why "facing vs trailing" cannot be authored on the turnout, why the route has to carry the
direction, and why building routes without direction first would produce a model that cannot express
the ride that motivated it.

## What already exists

Listed so nobody rebuilds it.

- **The physics is signed.** `Advance < 0.0` is handled, `FTrainConfig::bAllowRollback` exists, and a
  frictionless train sent at an uncrestable climb is asserted to come back past its start at exactly
  its entry speed. With resistance it takes 12 reversals to settle, 0.8 m from the true bottom.
- **Anti-rollback catches**, on the rising edge, and arrests rather than merely stopping.
- **`FRideSignals` takes doubles and a train index**, not an `FTrain` — so the signalling layer can be
  driven by whatever produces positions, including something that produces them in reverse.
- **`FTrackSensors` reports presence without identity**, which is the right foundation for a
  directional counter: the second head is another switch, not a new concept.
- **The block walk is derived, not authored.** Blocks come from the contiguous-zone walk, so a graph
  replacing a list is a change in one place rather than in every preset.

## Explicitly out of scope

**A launch platform that moves in an elevator fashion** — a vertical transfer that carries a train
between levels. Named here so it is a decision rather than an oversight: it is rare, it is a
mechanism rather than a track topology, and modelling it would mean track that moves, which nothing
else in this system needs. If it is ever wanted it is a transfer table with one more axis, and it
should be built on top of routes rather than alongside them.

The general rule this is an instance of: **rare mechanisms wait; general capabilities do not.**
Direction and routes are in scope because a great many ordinary rides use them. A moving platform is
one ride.

## Sequencing

**Direction first, routes second**, and the order is load-bearing rather than a preference.

Routes carry direction — that is what makes a point facing or trailing — so a route model built
against a system with one sign would have to be revisited the moment reverse arrived. The reverse
is not true: direction is useful on its own (shuttles, backwards trains, a rollback that the
signalling can finally see) and costs nothing that routes would want back.

Within direction, the order is what the failures cost:

1. **The two signs on the train**, and every derived rule asking for the leading edge rather than the
   nose. Cheap, and it is the change everything else is expressed in.
2. **The rider frame flip in `FRideProfile` and `GEnvelope`.** Highest severity of anything here —
   it is a wrong conformance verdict that looks right.
3. **The directional counter in `FTrackSensors`.** Two heads and a phase.
4. **A signed zone target**, for a device that releases backwards.
5. **The permissive's sense of "ahead."**

Neither of these is scheduled. When one is, it gets an engine-free prototype and an assert suite
first, the way `BlockSignal` and `PlcExpr` did — not a feature grafted onto the ring.
