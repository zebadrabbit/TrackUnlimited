# Control architecture and scripting

How a track's logic is expressed, who is allowed to express it, and which real-world standards each
layer is modelled on. This is the design reference for the scripting work; the block state machine
itself is [`SIGNALLING.md`](SIGNALLING.md), and the phased plan is [`ROADMAP.md`](ROADMAP.md).

**Status: DESIGN ONLY. Nothing described here is built**, and none of it is scheduled — scripting and
show control are not in any current phase. Read it as a position paper on how the control layer
*should* be shaped if and when it is built, not as a description of the system.

**Provenance.** Drafted in a Cowork session on 2026-08-03, in answer to a question about how real
coaster controls, park lighting and effects work, and what scripting language this project might
need. It is research and argument, not measurement — which makes it a different kind of document
from the rest of `Docs/`, where every number comes from running the code. Judge it accordingly.

**Verification.** The document self-certifies its standards against primary sources and flags what it
could not confirm, which is the right hygiene. Two of the highest-risk claims were independently
spot-checked before this was committed, because a public repo is a bad place to misquote somebody:

- The Alcorn McBride disclaimer quoted below is **genuine** — it appears in their V Series Network
  Controllers user documentation.
- IEC 61131-3 Edition 4.0 was **published 2025-05-22**, and Instruction List was deprecated in
  Edition 3 (2013) and removed in Edition 4, exactly as stated.

The rest of the standards table has *not* been re-verified line by line. The items the draft marks
*unverified* at the bottom are the author's own flags and remain open.

## Contents

- [The governing idea](#the-governing-idea)
- [Three tiers](#three-tiers)
- [Tier 1 — Safety: C++, never scriptable](#tier-1--safety-c-never-scriptable)
- [Tier 2 — Control: Structured Text on a scan cycle](#tier-2--control-structured-text-on-a-scan-cycle)
- [Tier 3 — Show: cues, and Luau where cues run out](#tier-3--show-cues-and-luau-where-cues-run-out)
- [The virtual VFD](#the-virtual-vfd)
- [Effect devices and the arm/permissive idiom](#effect-devices-and-the-armpermissive-idiom)
- [Time, and the three ways a show tells it](#time-and-the-three-ways-a-show-tells-it)
- [Determinism](#determinism)
- [The sandbox](#the-sandbox)
- [What this is NOT](#what-this-is-not)
- [Standards reference](#standards-reference)

---

## The governing idea

> **Show control is advisory. The safety chain is authoritative.**

This is not a design invention. It is how real parks are built, and it is stated plainly by the
people who build them. Birket Engineering — one of the handful of firms that actually deliver these
systems for Disney and Universal — lists *Ride Controls* and *Show Controls* as separate
disciplines, with show control coordinating pyrotechnics, flame, water cannons and show action
equipment under a supervisor that handles sequencing, operator interfaces and alarms. Alcorn
McBride, whose V16Pro is one of the standard show controllers in the industry, puts the boundary in
writing in their own manual:

> "Our Show Control equipment is not intended for use in applications where a malfunction can
> reasonably be expected to result in personal injury or damage to equipment."

Nothing that can hurt a guest is fired by the show controller alone. Flame, pyro, CO2,
high-pressure water and moving scenery all sit behind a hard permissive from the safety system.
E-stop, zone intrusion, ride fault or maintenance-mode selection drops that permissive
*independently of the show controller*, and the show controller cannot re-assert it — only the
safety chain can. Effects that cannot injure — light, audio, haze, video — are fired directly.

The reason to adopt this wholesale is that it answers, structurally rather than by policing, the
question every user-scripting system eventually faces: *can a downloaded script make the ride
dangerous?* No. Not because we validate scripts well, but because the layer that could is not
scriptable, in the same way and for the same reason it is not scriptable in a real park.

The separation is also the standard architectural fact of industrial safety, independent of
entertainment. Safety logic runs in a certified, separately-versioned, signature-protected program;
functional logic runs alongside it. Data flows safety → standard freely; standard → safety only
through explicit, non-safety-critical channels. A standard tag may *request* something. It may never
*authorise leaving* a safe state.

## Three tiers

```text
┌──────────────────────────────────────────────────────────────────────────┐
│ TIER 3  SHOW            cue lists · timelines · Luau                     │
│                         lights, effects, audio, atmospherics             │
│                         sandboxed · untrusted · may fail freely          │
└───────────────┬──────────────────────────────────────────▲───────────────┘
                │ requests                        permissive │ (hard, one-way)
┌───────────────▼──────────────────────────────────────────┴───────────────┐
│ TIER 2  CONTROL         IEC 61131-3 Structured Text · scan cycle         │
│                         dispatch, block sequencing, VFD commands         │
│                         deterministic · budgeted · faults, never stalls  │
└───────────────┬──────────────────────────────────────────▲───────────────┘
                │ requests                        permissive │ (hard, one-way)
┌───────────────▼──────────────────────────────────────────┴───────────────┐
│ TIER 1  SAFETY          C++ only. Not scriptable. Not overridable.       │
│                         e-stop · restraint proof · overspeed · block veto│
└──────────────────────────────────────────────────────────────────────────┘
```

Read the arrows carefully: **requests go up the diagram, permissives come down, and there is no
other path.** A tier can ask the tier below for something and be refused. It can never grant itself
anything.

| | Tier 1 Safety | Tier 2 Control | Tier 3 Show |
|---|---|---|---|
| Language | C++ | ST subset | cue data + Luau |
| Authored by | the project | the track author | the track author |
| Trust | trusted | untrusted | untrusted |
| Determinism | required | required | not required |
| On fault | forces safe state | latches, holds ride | disables that cue |
| Real-world analogue | safety PLC | standard PLC | show controller |

## Tier 1 — Safety: C++, never scriptable

Owns exactly what a real safety PLC owns: the e-stop chain, restraint closed-and-locked proof, gate
and airlock interlocks, overspeed, anti-rollback, block-occupancy veto, brake-closed proof, STO
commands to drives, and position/zone limits.

There is no scripting hook into this tier. Not a callback, not a "safety override" flag, not a
debug bypass that ships. A track file cannot contain anything that modifies it.

Behaviours worth implementing because they are what makes the tier read as real:

- **De-energise to trip.** The safe state is the de-energised state. Every safety input is modelled
  normally-closed; a broken wire, a lost supply or a pulled connector produces the same result as
  pressing the button. Brakes are spring-applied and pressure-released; restraints need positive
  power to *unlock*, never to lock.
- **Dual channel with cross-monitoring.** Two channels of opposite polarity, so a short between them
  is detected as a cross fault rather than defeating the circuit. Discrepancy monitoring: both
  channels must change within a window (typically 500 ms – 3 s) or the pair faults.
- **Monitored reset.** Reset must go 0 → 1 → 0. A jammed or taped reset cannot cause an automatic
  restart, and restart after e-stop is always a deliberate operator action.
- **External device monitoring.** The NC auxiliary contacts of the downstream contactors feed the
  reset input in series, so a welded contactor blocks reset. This is what makes redundancy actually
  redundant, and it is a satisfying thing to be able to fault in a sim.
- **Stop categories** (IEC 60204-1): Category 0 is uncontrolled — immediate removal of power, the
  machine coasts. Category 1 is a controlled stop with power retained to *achieve* the stop, then
  removed. A coaster e-stop is normally **Category 1** — you do not want to drop a train mid-launch
  or de-power a lift chain with a train on it. Category 0 is what a fault in the safety circuit
  itself produces. "Stop shall override start" is normative and should be literally true in code.
- **Sensors are paired and often diverse.** A prox plus a photo eye, so metal chips, ice or direct
  sunlight cannot fool both. Loss of a pair is a fault, not a degraded mode.
- **Block state is not trusted across a power cycle.** Real practice after a power loss is a manual
  course-clear walkdown and an operator reset. Retaining occupancy silently would be the wrong
  direction to be wrong in — and "walk the course, then reset" is a genuinely good simulator
  mechanic rather than an annoyance.

The one hard invariant worth asserting in tests: **a coaster must have at least one more block than
it runs trains.** Three trains needs four blocks. That is the functional core of the whole pillar.

## Tier 2 — Control: Structured Text on a scan cycle

The ride's own logic — dispatch sequencing, block advance, the ride-cycle state machine, lift and
launch commands, dwell timers, station choreography. Authored in a **subset of IEC 61131-3
Structured Text**, executed on a **scan cycle**.

Two reasons for ST rather than a general-purpose language, and they matter more than familiarity:

1. **It is the domain's actual notation.** Real ride control is written in the IEC 61131-3 languages
   — ladder for interlocks a technician reads at 2 a.m., ST for state machines and maths, SFC for
   the ride cycle. Someone who programs rides for a living can read and write this immediately, and
   what they learn here transfers back out. Nothing else on the market offers that.
2. **The type system is the sandbox.** A scan-cycle language with no heap, no closures, no dynamic
   allocation, and only BOOL / INT / REAL / TIME is trivially deterministic and trivially bounded.
   Bounded `FOR` and no `WHILE` (or a hard per-scan step cap) makes an infinite loop structurally
   impossible rather than something to detect.

Edition note: IEC 61131-3 Ed. 4.0 (2025-05-22) **removed Instruction List entirely**; it was
deprecated in Ed. 3.0 (2013). The normative languages are LD, FBD, ST and SFC. Implement ST first.
Ladder is best added later as a *view over the same AST*, not a second language.

### The scan cycle

```text
per control tick (fixed, e.g. 100 Hz):
  1. INPUT SCAN     copy all sensor/block/drive state into the input process image (frozen)
  2. LOGIC SOLVE    the ST program runs; reads the input image, writes the output image
  3. OUTPUT UPDATE  output image applied atomically to drives, brakes, indicators
  4. HOUSEKEEPING   watchdog, diagnostics, comms
```

Model the process image explicitly. It is where a large and authentic class of PLC bugs lives: a
sensor pulse shorter than the scan time is *missed entirely* unless it is latched. On a coaster
that is a real failure mode — a train flag passing a prox at speed — and reproducing it honestly is
worth more than smoothing it away.

Everything else follows from the scan model. The program never touches the world directly; it reads
a snapshot and writes a staging buffer. That kills reentrancy bugs, makes the sandbox surface tiny,
and means a script that faults can be cut out between scans with the sim untouched.

**A control program must complete within one scan or fault.** Exactly as a real PLC does. Exceeding
the step budget latches a fault and holds the ride; it never stalls the simulation. (Coroutines are
appropriate for *show* cues — "wait 2 s, fire, wait 0.5 s" is naturally long-running — and get a
separate, more forgiving budget on the show tick. They are not appropriate here.)

### Standard function blocks

Provide the standard library, with the standard names, because they are what people already know:
`TON` / `TOF` / `TP` timers, `CTU` / `CTD` / `CTUD` counters, `R_TRIG` / `F_TRIG` edge detection,
`SR` / `RS` latches. Declaration follows the standard: `VAR` non-retentive, `VAR RETAIN` surviving a
warm start. Cycle counters, maintenance-hour meters and last-fault records are retentive; block
occupancy deliberately is not, per Tier 1.

### The NoLimits 2 lesson

NL2 has scripting — NLVM, a Java subset, with no exceptions, no threading, and a deliberately tiny
standard library. Its fatal design decision is that **entering Scripted Mode disables Automatic
Block Mode entirely**: you then hand-code every brake, block, transport and dispatch. Community
reports put station departure plus the first brake run at 500+ lines, and a transfer-table operation
at 1000–1500. Adoption stayed low outside cosmetic uses, and the API is widely described as
inflexible and near-unreusable between coasters.

So: **the generated, data-driven block system is always the default, and a script overrides one
block, one station or one section.** There is never a global switch that trades "everything works"
for "you now own all of it." If someone wants to write a custom transfer-track sequence, they write
that and inherit the rest. This is the single most important usability decision in this document.

## Tier 3 — Show: cues, and Luau where cues run out

Most show authoring should never touch a script. The default surface is a **cue model** copied from
what the industry actually uses, because a lighting programmer arriving from an Eos or a grandMA3
already knows it:

- **Trigger** — an event-condition-action rule. Track-arc-length marker, block sensor, variable
  condition, or another cue's completion.
- **Cue** — an ordered list of timed actions with offsets, targeting fixtures or groups.
- **Sequence / cue list** — cues chained by follow / hang semantics, or locked to a time source.

Luau exists for the cases the cue model genuinely cannot express: conditional show states, ride-mode
dependent behaviour, procedural patterns, an author's own effect algorithm. It is chosen over the
alternatives on the merits — MIT licensed, designed from the start for running untrusted code at
scale, `io` and `package` removed by default, stdlib tables and `_G` readonly and unforgeable,
`loadstring` refuses bytecode, and a VM-level `interrupt` callback for step budgets. Notably it is
also what Rive and Roblox rely on for exactly this problem.

Rejected, with reasons, so this does not get relitigated:

| Option | Why not |
|---|---|
| **Verse** | Exists only inside UEFN/Fortnite. Not available for a standalone UE5 project, no open implementation, no license you could ship under. Epic frames it as the UE6 direction, with UE6 Early Access targeted end of 2027. Do not architect around it; do keep Tier 2 decoupled enough that a backend swap is possible in 2028+. |
| **UE-AngelScript** (Hazelight) | Production-proven and genuinely good — but it is a *fork of the engine source*, not a plugin, so every contributor needs an Epic GitHub account and a from-source editor build. In packaged builds the precompiled cache means `.as` files are not loaded at all, so it is not a user-content pipeline. No sandbox. |
| **UE Python** | Verified editor-only. Epic's docs: not available "when your Project is running in the Unreal Engine in any mode, including Play In Editor, Standalone Game, cooked executable." Dead end for shipping. |
| **UnrealSharp / Mono** | .NET is not a sandbox; Code Access Security is gone and not returning. GC pauses, ~50–80 MB payload, and Linux support still listed as planned. |
| **UnLua / slua** | Both auto-bind the entire UE reflection surface — the exact opposite of what an untrusted-content sandbox needs — and both are stale relative to UE 5.8. |
| **Wren** | Last release April 2021. Dormant. |
| **LuaJIT** | Non-incremental GC with known long pauses, and JIT'd traces skip debug hooks, weakening the step budget. |

Kept in the back pocket, not needed for v1: **libriscv** (BSD-3) or **wasmtime** (Apache-2.0) as a
tier for advanced authors who want to compile C++ or Rust. Both give a genuinely hard memory-isolated
boundary and native instruction limits; libriscv additionally has first-class pause/resume and
serialisation, which doubles as save-state support. Both cost the author a real toolchain, which is
hostile for someone writing a forty-line cue — so they are an escape hatch, never the default.

## The virtual VFD

Model **CiA 402** (standardised as IEC 61800-7-201/301, carried over EtherCAT as CoE) verbatim. It
is the drive profile behind Beckhoff, KEB, SEW, Elmo and others, so this is not an approximation of
a real drive — it is the real interface.

A drive is a controlword (`6040h`), a statusword (`6041h`), and eight states:

```text
Not ready to switch on ──▶ Switch on disabled ──▶ Ready to switch on
                                                        │
                                                        ▼
   Fault ◀── Fault reaction active          Switched on ──▶ Operation enabled ◀──▶ Quick stop active
                                                                (the only state with torque)
```

The canonical enable sequence a real PLC runs, and the one Tier 2 should have to run too:

```text
0x0006  Shutdown          → wait for statusword: Ready to switch on
0x0007  Switch on         → wait for statusword: Switched on
0x000F  Enable operation  → Operation enabled — only now may a target velocity be written
```

Details that carry the authenticity and cost almost nothing:

- **Quick stop is active-low** in both words. Statusword bit 5 reading 0 means quick stop is
  executing.
- **Fault reset requires a rising edge on controlword bit 7.** Holding `0x0080` forever does
  nothing — a genuinely common real-world mistake, and a good thing to let a user make.
- A fault is not instant: the drive enters *Fault reaction active*, runs its configured reaction
  (`605Eh` — coast, ramp on the slow-down ramp, ramp on the quick-stop ramp, or current limit),
  and only then settles in *Fault*.
- **Statusword bit 9 (Remote)** distinguishes obeying the controlword from local/HMI control, and
  bit 10 (Target reached) and bit 11 (Internal limit active) are what a control panel actually
  displays.
- **Modes of operation** (`6060h`): `pv` profile velocity for a lift chain or conveyor, `pt`/`cst`
  torque for a launch, `csp`/`csv` for LSM trains coordinated on a synchronous bus, `vl` for a plain
  fan or pump.

Beyond the state machine, a drive should expose what real drives expose and fault the way they
fault: overcurrent, DC bus over/undervoltage, motor thermal overload, ground fault, phase loss,
plus ramp parameters (accel, decel, S-curve), torque and current limiting, and the choice between a
braking resistor and a regenerative front end. Control topology matters too, and is visible in
behaviour: V/f scalar holds ±2–3 % speed with no feedback and can run multiple motors; sensorless
vector gets to 1–3 %; closed-loop flux vector with an encoder holds sub-1 % and **full rated torque
at zero speed**, which is exactly why a lift hill uses one.

The drive-integrated safety functions (IEC 61800-5-2) are where Tier 1 reaches the drive: **STO**
(safe torque off) is the Category 0 path, **SS1** (safe stop 1 — ramp, then STO) is the Category 1
path a coaster e-stop normally uses.

## Effect devices and the arm/permissive idiom

Model the protocol layer literally — universes, 512 slots, start codes, addresses, fixture profiles
with channel maps — but internally. No network I/O in this design; see [What this is
NOT](#what-this-is-not).

Facts worth building against:

- **DMX has no fades.** The console computes the fade and streams absolute values at frame rate. A
  full 513-slot packet takes ≈22.7 ms, so ≈44 packets/second is the ceiling; shorter packets go
  faster. Any smoothing in a fixture is the fixture's own slew behaviour, not the protocol's.
- **The arm-plus-level idiom is the characteristic shape of effect gear**, and it exists precisely
  so a stray single channel cannot start an effect. The MDG ATMe hazer — an industry-standard unit —
  is three channels: unit power (≥128 = on), haze output (0–255 proportional), and a separate haze
  *enable* (≥128 = on) that must be held. Its behaviour on loss of DMX is itself selectable:
  automatic shutdown, or hold last values.
- **Effects have duty cycles, and a real programmer works around them.** Cheap foggers stop
  accepting fire commands while the heater recovers. A Le Maitre Salamander flame projector has two
  channels — hot surface igniter (energises only above 99 %) and fire (above 50 %) — with a
  mandatory ~10 s igniter heat-up during which firing is inhibited, a 30 s maximum continuous
  solenoid, and recommended bursts of 0.5–1 s.
- **Flame is not fired by the show controller.** NFPA 160 (*Standard for the Use of Flame Effects
  Before an Audience*, current edition 2026) puts a safety-rated flame effect controller and a
  fail-safe positive manual enable — a human-held enable that must be actively asserted — between
  the request and the gas. The interesting thing to simulate is that chain: key/arm asserted, PME
  held, ignition proven, fuel pressure in range, no e-stop, no zone intrusion, duration timer within
  limits. Not the DMX value.

Which yields one predicate that every hazardous effect must satisfy, and which is most of what makes
a park feel like a park rather than a light show:

```text
fire  ⇔  showRequest ∧ safetyPermissive ∧ armed ∧ ¬faulted ∧ withinDutyCycle
```

## Time, and the three ways a show tells it

Three time models coexist in real installations, and a convincing sim needs all three plus nesting.

**A — Absolute timecode.** A master clock (SMPTE LTC / MTC), every cue carrying an `hh:mm:ss:ff`
address, frame rate part of the contract. Deterministic, restartable anywhere, tolerant of one
subsystem dying. Used for fixed-duration sequences.

**B — Relative offsets.** Pre-wait / duration / post-wait, or follow / hang. The distinction between
*time measured from the previous cue's start* and *from the previous cue's completion* is the single
most common source of real-world show programming bugs, and is worth modelling exactly rather than
collapsing into one concept.

**C — Triggered chains.** Event-condition-action with no time reference: "when block 4 clears and
RideMode is Show, play sequence 12." This is what glues a nondeterministic ride to deterministic
shows.

They compose the way a real attraction does: **the ride is model C at the top** — a block state
machine with no fixed timeline; **each scene is A or B** — a zone trigger launches a sequence that
is internally clock-driven for its 10–40 second life; **idle and ambient states are C again**,
looping continuously and pre-empted by scene sequences on priority.

## Determinism

Tier 1 and Tier 2 must be bit-reproducible. Tier 3 need not be, and pretending otherwise would cost
more than it is worth.

Ranked by how often it actually bites:

1. **Table iteration order.** Vanilla Lua's `pairs()` order depends on pointer values and hash
   seeds, so it varies run to run under ASLR. Factorio patched their Lua specifically for this —
   their `pairs()` is insertion-ordered, and the first 1024 numbered keys iterate 1→1024 regardless
   of insertion order — precisely because the game has to be deterministic. Luau does not fix this
   by default. Either ban `pairs()` from the API or supply an ordered replacement.
2. **RNG.** Replace `math.random` with a seeded per-script generator, snapshotted in the save.
3. **Floating point.** Same-platform reproducibility is achievable; cross-platform bit-identical FP
   is not, without confining script maths to integers or fixed point. For Tier 2 this is easy —
   interlock logic is booleans and integers. For Tier 3 it does not matter.
4. **Time coupling.** Expose tick count and a fixed `dt`. Never wall clock, never frame delta.
5. **Invocation order.** Iterate scripts in a stable content-defined order — sorted by ID — never by
   pointer or hash-map order, on either side of the boundary.

Tier 2's scan model gives most of this for free, which is another argument for it: a language with
no heap and no closures has very little left to be non-deterministic with.

## The sandbox

Threat model in order of likelihood: runaway loop (accidental, common) → memory exhaustion → file or
network access → VM escape via crafted bytecode → resource abuse through host calls the budget does
not count.

- One VM per track file. Never shared between files.
- `luaL_sandbox()` plus a sandboxed thread; verify `_G` and every library table is readonly.
- A custom allocator with a hard byte cap. **Luau imposes no memory limit by default** — the address
  space can be exhausted unless you supply one. OOM faults the script; it must not take the process.
- A step budget via the `interrupt` callback. Exceeding it latches a script fault, disables that
  script for the session, and surfaces a UI error. **It never stalls the sim.**
- **Source only. Reject bytecode, and verify by parsing rather than by checking magic bytes.**
  Factorio was fully compromised through exactly this hole: it allowed `load()` on precompiled
  bytecode, and its verifier had off-by-one errors in jump validation, which chained through TValue
  type confusion to arbitrary read/write and a GOT overwrite. Luau's `loadstring` already refuses
  bytecode. Keep it that way and never add a bytecode path.
- Zero file, network, process or module-loading surface. Assets are referenced by manifest-declared
  handle, never by path.
- **Charge instructions for host calls.** OpenTTD documents this exact hole: opcode counting does
  not count the C++ called *from* the script, so one expensive host call in a loop can freeze the
  game. Every exposed function is either O(1) or charges proportionally.

The counter-example worth naming: Garry's Mod exposes `file`, `http` and `RunString` to addon Lua
with essentially no restriction, and has had an endemic Workshop backdoor problem for over a decade
— to the point that third-party backdoor scanners exist as a community necessity. That is the
outcome of "just embed Lua and expose everything."

## What this is NOT

- **Not real network I/O.** No sACN, Art-Net, MSC or timecode on the wire in this design. The
  protocol *model* is internal. Speaking to a real console is a plausible later differentiator and
  a genuine security surface; it is deliberately out of scope here.
- **Not a safety-rated anything.** This is a simulation of a control system, and no part of it
  should ever be represented as suitable for controlling a real machine. The real systems this
  models are certified to IEC 61508 / ISO 13849 by people whose job that is.
- **Not an all-or-nothing scripting mode.** See the NL2 lesson above. If a change to this
  architecture would ever require an author to hand-write their whole block system to customise one
  brake, that change is wrong.
- **Not a general modding API.** Scripts see a process image and a declared API surface. They do not
  see the engine, the actor graph, or the filesystem.

## Standards reference

Verified 2026-08-03 unless marked otherwise.

**Ride and safety**

| Standard | What it is |
|---|---|
| IEC 61131-3 Ed. 4.0 (2025-05-22) | PLC languages: LD, FBD, ST, SFC. IL removed in Ed. 4; deprecated in Ed. 3 (2013). |
| IEC 61800-7-201/301 (CiA 402) | Drive profile — controlword/statusword state machine. |
| IEC 61800-5-2 | Drive safety functions: STO, SS1, SLS. |
| IEC 60204-1 §9 | Stop categories 0/1/2; "stop shall override start". |
| ISO 13849-1 | Performance Levels a–e; Categories B/1/2/3/4. |
| IEC 61508 | Functional safety, SIL. |
| ASTM F2291 | Design of amusement rides and devices. |
| ASTM F2137 | Measuring dynamic characteristics — the SARC test; 5 Hz Butterworth for patron evaluation, 1 Hz for restraint design. |
| ASTM F770 / F2974 | Operation; auditing. |
| EN 13814-1/-2/-3:2019+A1:2024 | European fairground and amusement machinery. |
| ISO 17842-1:2023 | The parallel ISO standard. **Not** a renumbering of EN 13814 — they are separate, closely aligned standards. There is no "EN 17842". |

Acceleration limits, harmonised between ASTM F2291 and EN 13814, in the patron-body-fixed frame
(z = spine, x = chest–back, y = lateral):

| Axis | Limit |
|---|---|
| +Gz | 6 g ≤1 s · 4 g 1–3 s · 3 g 3–12 s · 2 g 12–40 s · 1 g >40 s |
| −Gz | 1.1 g at 3 s |
| ±Gy | 3.0 g ≤2 s · 2.5 g 2–4 s · 1.25 g longer |
| +Gx | 6 g with headrest and onset <5 g/s; 2.0 g without headrest, max 4 s |
| −Gx | 2.0 g, max 4 s |
| Jerk | 5–10 g/s design, 15 g/s allowable in test, computed over a 100 ms least-squares window |

Events under 0.20 s are *impacts* and sit outside the sustained envelope; 0.20 s is roughly
neuromuscular reaction time. Axes combine on a normalised elliptical formula — you cannot sit at
100 % of two limits at once. After ≥5 s of sustained −Gz the allowable +Gz is reduced (the
"push-pull" effect). ASTM measures at heart-line level, EN 13814 nearer head level, which is a real
cited difference that changes the numbers.

**Show control**

| Standard | What it is |
|---|---|
| ANSI E1.11 – 2024 | DMX512-A. 250 kbit/s, start code, 512 slots, ≈44 packets/s max. |
| ANSI E1.20 | RDM — remote device management over DMX. |
| ANSI E1.31 | sACN — DMX over IP with priority. |
| ANSI E1.17 – 2015 (R2025) | ACN, with DDL device description. |
| GDTF / MVR | Fixture definition and rig exchange. |
| MIDI Show Control | GO / STOP / RESUME / TIMED_GO / SET, by device ID. |
| SMPTE LTC / MTC | Show sync; 24 / 25 / 29.97DF / 30. |
| NFPA 160 (2026) | Flame effects before an audience. Safety controller, positive manual enable, arming monitored until fired. |

*Unverified, flagged deliberately:* the exact normative wording of ASTM F2291 §11 on redundancy
requirements and whether a PL/SIL floor is mandated for block systems (paywalled); NFPA 160's
Group I–VII boundaries in the 2026 edition (sourced from committee response documents, and the
committee itself formed a task group to revise them); whether sACN per-address priority via start
code `0xDD` is in E1.31 itself or a de facto vendor extension; a handful of CiA 402 object numbers
(`605Ah`–`605Eh`) taken from profile knowledge rather than re-verified against the spec.
