# Coaster types

What "which kind of coaster" should mean when somebody starts a new project, sorted by what it
actually costs.

**Status:** design only. No code exists for the new-project picker or for any of the degrees of
freedom below. Nothing here is scheduled — it is here so that work which *is* scheduled does not
accidentally rule it out, and so that the taxonomy question is answered once rather than every time
somebody adds a type.

## Contents

- [The line worth drawing](#the-line-worth-drawing)
- [Already expressible](#already-expressible)
- [One parameter away](#one-parameter-away)
- [Genuinely new — a degree of freedom each](#genuinely-new--a-degree-of-freedom-each)
- [The decision: a type is a PRESET, never a branch](#the-decision-a-type-is-a-preset-never-a-branch)
- [What the picker should be](#what-the-picker-should-be)
- [Sequencing](#sequencing)

## The line worth drawing

The industry's taxonomy is a marketing taxonomy. It sorts by what a rider notices, which is the
right list for a park brochure and the wrong list for a simulator — because two entries that read
as equally different can be a field and a rewrite.

So the same lens the routes document used applies here. **Most of the list is a mask.** Sorted by
what it costs instead, the whole taxonomy falls into three piles:

1. **Already expressible** — the authored vocabulary says it today, and the "type" is a set of
   values somebody could type in by hand.
2. **One parameter away** — a field that does not exist yet, but which is a number on a struct
   rather than a change to how anything is integrated.
3. **A degree of freedom** — a new state variable on the vehicle, integrated alongside the existing
   ones, with its own contribution to felt G. These are real work and each is its own card.

**A type that spans piles is a type that should be split.** "Flying coaster" is a rider frame
(pile 2) and nothing else; "bobsled" is a different constraint model (pile 3) and barely the same
simulation.

## Already expressible

### Every launch mechanism

Chain lift, LSM/LIM, hydraulic, pneumatic. `ETUSegmentZone::Lift` and `Launch` already exist, and
what separates the four in practice is the **acceleration curve and the top speed** — both of which
are authored per device today as `ZoneAccel` and `ZoneSpeed`.

**The one honest gap is the curve, not the type.** A hydraulic launch pulls near-constant force; an
LSM launch is stepped as the train passes each stator group; a pneumatic launch falls off as the
receiver empties. Today each is a single constant. That is a curve on a device — the same shape of
change `ZoneAccel` already was — and it is worth having because *the difference is felt*: a rider
can tell an LSM from a hydraulic launch with their eyes shut.

### Most of "by layout/experience"

- **Dive** — a holding brake at the crest, then a vertical drop. Both exist.
- **Wild mouse** — tight radii and short cars. Both authored.
- **Mine train, junior/family** — parameter sets: speed, radii, train length, restraint kind.
- **Water** — a splashdown is added drag over a span. Nearest existing shape is a zone.

None of these need to exist as a concept. Every one is a **starting layout plus a train config**,
which is what a template already is (`ETemplatePreset` names a preset rather than carrying its own
geometry, for exactly this reason).

## One parameter away

### Track structure — steel, wood, hybrid

This is `FTUTrackStyle`, which exists: gauge, rail diameter, spine drop, tie spacing, tie diameter.
Wood is a different cross-section, a different support vocabulary (bents rather than columns), and
**a roughness term the physics does not have**. Hybrid is a wooden structure carrying a steel
profile, which under this model is simply a style whose support geometry and rail profile disagree —
no new concept at all.

Roughness is the only genuinely missing piece, and it belongs to the *track*, not to the type.

### Seating and orientation — mostly an offset, because the heartline is already the rider

This is the payoff for a decision made in Phase 0. The heartline is where felt G is computed and
what banking is built around: **it is the rider, not the rail.** So the seating taxonomy is mostly a
question of where the *structure* sits relative to that line.

| Type | What it actually is |
|---|---|
| Sit-down | Rails below the heartline. `SpineDropM` as shipped. |
| Inverted | Rails **above** the heartline — the same number, negative. |
| Stand-up | The heartline sits higher relative to the rails. |
| Flying | Inverted plus a seat rotated to prone. |
| Wing | Riders offset **laterally** from the heartline. See below. |

`FacingSign` already covers which way a seat faces, and it was built for backwards-facing seats and
face-off trains rather than for this — which is the good kind of accident.

### Wing coasters are the one worth building properly

A wing rider sits well off the centreline, so **what they feel differs from the heartline by
roll-rate × offset**. Through a fast roll the outboard seat gets a vertical snap the centre seat
never sees, which is precisely why wing coasters feel the way they do.

That is a real physics difference, it is measurable with what `GEnvelope` already does, and — the
argument that makes it worth the field — **it applies to the outer seats of every wide train, not
only to wing coasters.** A four-across train has the same effect at smaller magnitude, and today it
is not modelled at all. The lateral seat offset is therefore a *general* improvement that a wing
coaster happens to need most.

It also composes with something already built: the ride profile takes `FacingSign` as a parameter of
the run rather than a field on the train, precisely so a face-off train can be measured twice. A
seat offset belongs in the same place, for the same reason — **a train has several seats and the
honest way to measure them is to run it once per seat.**

## Genuinely new — a degree of freedom each

Each of these adds a state variable to the vehicle, integrated alongside arc length and speed, with
its own contribution to what a rider feels. None of them is a variation on the others.

| Type | The new freedom | Notes |
|---|---|---|
| **Spinning** | Car yaw about its own vertical axis | Free or braked. Changes what the rider feels moment to moment more than any other entry here. |
| **Suspended** | Pendulum swing about a pivot above the car | Damped, and driven by lateral acceleration. The classic Arrow suspended. |
| **4D** | Seat pitch about a lateral axis | Controlled (rails) or free. Has its own G frame entirely. |
| **Bobsled** | No lateral constraint — a trough, not a track | Barely the same simulation: the car's lateral position is a free variable and the trough wall is a contact problem. |

**Spinning is the one to do first if any.** It is the most common, the effect on the rider is the
largest, and its state is a single angle — where suspended needs a damped oscillator and 4D needs a
second orientation frame that everything measuring G has to be taught about.

**Alpine/rider-braked coasters are a control question, not a vehicle one.** They are the only entry
in the whole taxonomy where the *rider* has an input, which nothing in the control model expresses —
a fifth party alongside safety, control, show and the operator. Interesting, and separate.

## The decision: a type is a PRESET, never a branch

**No `ECoasterType` that the physics or the signalling reads.** The moment a type forks behaviour,
every feature after it costs N times, and this project already has the cautionary tale written down:
NL2's fatal choice was that scripted mode disables automatic block mode entirely, so customising one
brake costs the whole ride.

A type selects **values in fields that already exist**. That is the same answer templates already
give, and it means:

- Choosing "inverted" and then dragging the spine drop back positive is allowed, and gives you
  something that is no longer an inverted coaster and does not need a name.
- A type nobody anticipated is a set of values somebody types, not a pull request.
- Nothing downstream — the interlocking, the envelope, the mesher, the audit — ever learns that
  types exist.

The degrees of freedom in pile 3 *are* real code, but they are properties of a **vehicle**, not of a
type: a spinning car is a car with a yaw freedom, and a type is just a preset that switches it on.

## What the picker should be

A new-project chooser that sets four things and then gets out of the way:

1. A **track style** (`FTUTrackStyle`).
2. A **rider frame** — spine drop sign, seat facing, lateral offsets.
3. A **train config** — length, vehicles, restraint groups.
4. A **starting layout** — an existing template.

**It should say what it set.** A picker that silently configures twenty fields is one nobody can
recover from when they want something between two of the options — and "findable by somebody who
wants it, invisible to somebody who does not" is the rule the whole shell already follows.

**And it must not be a gate.** The blank template exists; a type picker that has to be answered
before anything can be built would be the fork this project rejected at the main menu, arriving one
screen later.

## Sequencing

Nothing here is scheduled. In rough order of value per unit of work:

1. **Launch curves** — a curve where there is a constant. Small, and felt.
2. **Lateral seat offset** — the wing coaster's requirement, and an improvement to every wide train.
3. **Track style presets and wood roughness** — mostly Phase 4's territory anyway.
4. **The picker itself** — cheap once 1–3 exist, because by then it is only choosing values.
5. **Spinning cars** — the first real degree of freedom.
6. **Suspended, 4D** — one at a time, each with its own G accounting.
7. **Bobsled** — a different constraint model; treat as its own project if ever.

**Explicitly out of scope:** anything that would make a type into a mode.
