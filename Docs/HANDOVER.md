# Handover — 2026-08-06

What happened overnight, what needs you, and what I deliberately did not do.

Delete this file once it has been read; it is a note, not a document.

---

## Needs you

**1. Ride it.** Nothing here has been in the editor since the drag coefficient changed. `[P]` twice
for the maintenance view — it now shows the controller (mode, program digest, why it will not run),
the CiA 402 statusword per drive, the scan rate and the fingerprint.

**The reference layout now peaks at +4.52 g and its loop apex went +1.16 → +1.78 g.** That is
correct rather than a regression — the old numbers were a faithful simulation of a train dragged
4.5× harder than a real one. But it is a **meaningfully more forceful ride than it was**, and whether
the layout should be re-tuned is a design decision, not a physics one.

**2. Both decisions from this note are now made and written down.**

- **Current phase is Phase 4 — Track Meshing & Supports.** The control system is deeper than
  anything on the market and completely invisible in a screenshot. Phase 3.5's runtime editor is the
  other defensible answer and swapping them is one line in `CLAUDE.md`.
- **Tier 2 gets EXPRESSIONS, not Structured Text.** Recorded in constraint 7 and in
  `CONTROL_ARCHITECTURE.md`. The trigger for revisiting is concrete: the first ride whose policy the
  default cannot express and a condition cannot reach.

**3. One thing I would like verified if you ever get the chance.** The ASTM F2291 / EN 13814
acceleration tables are unverified research, not a copy of either standard. `GEnvelope.h` prints
that on every run and the docs say it, but the tool prints a standard's name and that carries more
authority than the numbers have earned. Both documents are paywalled.

---

## What got built overnight

All engine-free, all assert-tested, editor builds clean, everything pushed to `main`.

| | |
|---|---|
| `PlcUnit.h` | The controller as a **machine** — key switch, watchdog, program identity, power |
| `Cia402.h` | The drive state machine as specified. Eight states, one produces torque |
| `ShowBus.h` | Tier 3's only connection to the ride. Read-only **by shape**, not by policy |
| `Scenario.h` | A timeline of faults against the scan clock, so a fault has a *when* |

Two of those close cards you had open for a while; two are new cards I created and closed.

**Three results worth knowing about:**

**A program built for a different layout now refuses to RUN.** The identity is a digest of the
*derived* blocks and zones, so a geometry tweak that moves no boundary does not invalidate it. This
is the first detector this project has ever had for *"I changed the code and the editor is still
doing the old thing"* — the exact class you have hit before.

**Block state is not trusted across a power cycle.** The Tier 1 card has asked for a course-clear
walkdown and operator reset since it was written, and it was unimplementable because nothing could be
switched off. It can now.

**"Simulate a stuck harness" runs end to end.** Group 2 jams at scan 100 and frees at scan 2000 —
not one scan of dispatch permission while a bar is open, and then it **recovers**. That last half
matters: a fault that holds the ride for ever after being fixed is its own defect, and you can only
see the difference with something that can free a fault as well as inject one.

---

## What I deliberately did not do

**The standard function block library** (`TON`, `CTU`, `R_TRIG`, `SR`). Its only consumer is the
deferred ST VM. Building it now is dead code, and refactoring the working hand-rolled versions onto
it is risk for style. It stays on the board.

**The failed brake.** It needs the physics to read the drive's *actual* rather than its *commanded*
deceleration, and **every measured figure in the project runs through that clamp**. That is a change
to make awake, with the canonical figures re-measured afterwards — not one to make at five in the
morning while you are asleep.

**Walkways on the presets.** You were right that a human places these, so the presets ship with none
and the evacuation check correctly reports every train as unreachable. That is the honest reading of
a layout nobody has surveyed.

**The actor wiring for `FShowBus`.** The bus exists and is tested; nothing publishes to it yet. Left
open on its card rather than half-done.

---

## Where the board stands

Phase 3.75 is now mostly either **done** or **deliberately deferred with a reason on the card**.
What is left that is genuinely open and unblocked:

- **Effect device layer — DMX addresses, fixture profiles.** `FShowBus` is its foundation.
- **Three time models for cues.** Wants the DMX layer first.
- **Failed brake** — see above, wants you awake.
- **Generated default control program** — largely already true; the card wants it written down.

Everything else in 3.75 waits on the Tier 2 language decision, which is item 2 above.
