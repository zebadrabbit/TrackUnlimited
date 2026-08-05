# Glossary

The maths vocabulary this project uses, and what each term actually *is* on a
real roller coaster. Written for a reader who knows coasters and is meeting the
differential geometry for the first time — which is the normal direction to come
at this from, not a gap.

`CLAUDE.md` has a shorter list aimed at keeping code and comments consistent.
This one is for understanding. Where a term appears in code, the file is named.

---

## Where the track lives

**Arc length** (`S`, metres)
Distance measured *along the track*, the way a wheel would roll it — not
straight-line distance through the air. Every position in this codebase is "how
far along", never "where in space". A 543 m layout has `S` running 0 → 543.7.

**Curvature** (`k`, 1/metres)
How hard the track is turning, defined as **1 / radius**. Straight track is
`k = 0`. An 8 m radius loop is `k = 0.125`. Using 1/R instead of R is not a
quirk — it lets a straight be a real value rather than a special case, and it
makes "ramp from straight into a curve" a straight line on a graph.

Curvature has a **direction** as well as a size, so it is really a vector: how
much the track bends *left* (yaw) and how much it bends *upward* (pitch), at the
same time. A banked helix bends both at once.

**Curvature profile over arc length**
The core representation, and the thing most different from other track editors.
Instead of *"here are some points in space, draw a smooth line through them"*,
each segment says *"for the next 40 metres, curvature goes from 0 to 1/30"*. The
track is defined by **how it bends**, and its position in space is what you get
by integrating that — walking forward and turning as instructed.

The consequence is worth stating: the shape is the authored thing, and the
position is derived. That is backwards from most 3D tools and it is deliberate.
`Prototypes/TrackSpline/TrackSpline.h`.

**Clothoid** (also *transition curve*, *easement*, *spiral*)
A segment where curvature changes **linearly** with distance. Physically: you
turn the steering wheel at a constant rate. It is what goes between a straight
and a curve so the turn arrives gradually instead of all at once.

Not a coaster invention — railways and highway interchanges use the same curve
for the same reason. Without one, a straight meeting a curve steps from 0 g to
full lateral G instantaneously, which is a jolt you feel in your neck. This is
the single most important reason the representation is curvature-based.

**C² continuity**
Three levels of "smooth" at a joint between two segments:

- **C⁰** — no gap. The two pieces meet.
- **C¹** — no kink. They also point the same way.
- **C²** — no jolt. They *also* have the same curvature.

C⁰ and C¹ are easy and every track editor has them. C² is what separates track
that feels engineered from track that feels approximated. Because segments here
store curvature directly, C² is a property of the *data* — if the numbers match
at the joint, the track is smooth. Nothing is fitted or relaxed afterwards.
`FTrack::IsCurvatureContinuous`.

---

## How the rider is carried along

**Frame**
Three perpendicular directions carried along the track: **Tangent** (forward),
**Lateral** (rider's left), **Up**. Think of the axes painted on a car body as
it drives — they turn with the car. Every geometric question in this project is
answered by producing the frame at some arc length. `FTrackFrame`.

**Parallel transport** / **rotation-minimising frame (RMF)**
Carrying that frame forward with the **least possible twist** about the
direction of travel. It has to rotate to stay aligned with the track, but it
never spins about the forward axis unless the track makes it.

Physical picture: slide a ring along a bent piece of wire without deliberately
rotating it. Where the ring ends up pointing is parallel transport.

This is the reference the whole model uses, because it is defined everywhere —
including upside down, where "level" isn't.

**Holonomy**
The surprising part, and the one that caused a real design decision here.

Parallel transport a frame around a closed path in 3D and **it does not come
back the way it started**. It comes back rotated. Nothing spun it; the rotation
is a property of the path.

Measured in this repo: climb 45°, turn left 90°, descend 45°, applying zero roll
throughout — the rider arrives banked **54.736°** off level. Three right angles
gives exactly 90°.

This is not a bug and it is not floating-point drift. It is the same effect that
makes a Foucault pendulum's swing plane rotate over a day. It is why `Roll = 0`
does not mean "level", and it is the entire reason `ERollMode` exists.
`Docs/PHASE0_FINDINGS.md`, the `Roll = 0` entry.

**Roll** vs **bank**
Not synonyms here, and the difference is a stored per-segment mode.

- **Roll** — measured from the rotation-minimising frame. Defined *everywhere*,
  including vertical and inverted track. What the integrator sees.
- **Bank** — measured from the horizon; what a spirit level reads. What a human
  means by "bank this turn 30°". **Undefined pointing straight up**, because
  there is no horizontal to be level with.

Neither works alone, so segments declare which they mean. `ERollMode`.

**Heartline**
The reference line the rider actually rotates around — roughly chest height on a
seated rider, ~1.1 m above the rail centreline. Banking swings the *rails*
around the heartline, not the heartline around the rails.

This is why it matters: if you bank around the rails, the rider's body gets
swung sideways through space and picks up G that a real ride doesn't have. Real
manufacturers design to the heartline for exactly this reason, and NoLimits 2
exposes it as a first-class concept. `FTrack::RailCentreAt`.

---

## The two that sound worst and mean least

**Torsion** (`τ`, 1/metres)
How fast the curve twists **out of its own plane**.

- A circle — any circle, at any tilt — has **zero** torsion. It is flat.
- A spiral staircase has **constant** torsion. That is a helix.

That is the whole idea. It is one number, and it is the difference between a
helix and a tilted flat circle. Authored without it, "constant radius, constant
climb" produces a circle that returns to its own starting height — which is what
this project measured and recorded before adding the field.

Nobody authors torsion. You author radius, climb angle and turns, and
`MakeHelix` converts. `FTrackSegment::Torsion`.

**Darboux vector** (`Ω`)
The single axis-and-rate describing how the whole frame turns as you advance one
metre. It is angular velocity — the same idea as a spinning object's — but
measured **per metre of track** instead of per second, because the track doesn't
know how fast you're going.

In the code it is literally one line, `Omega = U*Yaw - L*Pitch`, and the
integrator just rotates the frame about it. The impressive name is doing no
work. `FTrack::Integrate`.

**Frenet frame**
The "textbook" frame, defined by the direction the curve bends. Different from
the rotation-minimising frame — and the difference is *exactly* the accumulated
torsion.

That fact is what made helices cheap here: rather than a new segment type with
new machinery, a helix is constant curvature plus constant torsion, and the
existing integrator handles it with one extra rotation.

---

## What the rider feels

**Felt G**
Acceleration as the body experiences it, in units of gravity.

- **1.0 vertical** — sitting on level track, normal weight.
- **0.0** — airtime; you float out of the seat.
- **negative** — ejector airtime; the restraint is holding you in.
- **lateral** — sideways. Comfortable banking cancels this to near zero; an
  unbanked turn does not, and that is the "slammed into the door" feeling.

Computed against the **unbanked** axes, because banking rotates the rider, not
the trajectory. Resolving onto the banked axes instead is the classic error that
produces a banked turn which impossibly cancels its own lateral G. `FeltG`.

**Roll rate**, and what felt G cannot see
Felt G models the rider as a **point** at the heartline. A point has no size, so
spinning it costs nothing — meaning **roll rate does not appear in the G numbers
at all**. Not approximately: the term is structurally absent.

But a real head sits ~0.5 m above the heartline, and spinning that costs `ω²r`.
Measured: 0.031 g at 45 °/s, 0.126 g at 90, **0.503 g at 180**. A fast barrel
roll that reads perfectly smooth at the heartline is half a G of head snap.

Hence a separate metric, because no amount of looking at the G trace will ever
show it. `AnalyseRollRate`, `AnalyseResolvedRollRate`.

---

## Ride control

These are railway signalling terms, used here in their railway sense.

**Block** — a length of track only one train may occupy at a time.

**Buffer** / **overlap** — the safety margin a block holds *after* a train has
physically left it, before it reports clear. Real railways call this an overlap.
It exists because "the train has left" and "it is safe to send another" are not
the same instant.

**Dispatch permissive** — the gate that decides whether a station or launch may
release a train, based on what is clear downstream. On high-speed sections it
also has to look far enough ahead that the *braking distance* fits.

**PLC** — programmable logic controller. The **brain**: it reads the sensors,
holds the block states, evaluates the permissives and commands the devices, on a
fixed scan cycle. A ride's safety interlocking is a PLC program, usually on a
safety-rated PLC separate from the one running the ordinary sequencing.

**In this project, `FRideSignals` plus the dispatcher IS the PLC program.** Block
occupancy, overlaps, permissives, and the four lines that command a holding
device every frame — that is exactly the job. `Docs/CONTROL_ARCHITECTURE.md`
describes the tiers it would be split across.

**VFD** — variable-frequency drive. The **muscle for one motor**, not the brain: it
takes a speed command and drives a tyre or chain motor to it, reporting frequency,
current and torque back. A ride has one per powered device and a PLC telling all
of them what to do — so "the VFD decided to hold the train" is a category error;
the PLC decided, the VFD carried it out.

**VFD module** — the generated control-panel element for one of those drives:
target frequency, actual motor feedback, torque draw, ramp rate. The faceplate an
operator would actually be looking at.

**Block brake** — a brake run *with drive tyres*. It can bring a train to a stop,
hold it there, and start it again. That last part is what separates it from a
**trim brake**, which can only ever slow a train down: park one on a trim and it
stays there for good. Only block brakes bound a block, and only block brakes
count toward how many trains a circuit can carry.

**Anti-rollback** — ratchets, chain dogs, a catch car. A train cannot roll
backwards through a stretch fitted with them. A *safety* device rather than a
control one: no speed, no authority, nothing commanding it, and it works by being
passive. Every lift hill ever built has it. Being caught is a distinct outcome
from stalling and from rolling back — it means the hill was too tall **and** the
hardware did its job.

**Evacuation zone** — somewhere riders can actually be got *off*: needs a walkway,
access, an egress route, not merely somewhere a train can stop. A large ride has
far more of these than it has blocks, and they answer a different question. Not
modelled yet.

`Prototypes/BlockSignal/`.
