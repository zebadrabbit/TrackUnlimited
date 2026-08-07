# Tier 2 overrides

**You never have to read this page.** Every preset runs on the generated default program: place
track, it works, trains dispatch, blocks interlock. Tuning is numbers in a panel. An override exists
for the case where the default will not do what you want — and nothing above that is ever required.

If you *do* read it: what you are learning is not a language this project invented. It is a subset of
**IEC 61131-3 Structured Text**, the language industrial control actually runs on, spelled the way
the standard spells it. That is deliberate. Industrial control is a real, understaffed, well-paid
field that almost nobody discovers as a teenager, and a coaster is a far better first PLC than a
bottling line. If this taught an invented syntax, learning it would be worth nothing outside.

---

## What an override is

**A pure expression that returns `BOOL`, evaluated once per scan against a snapshot of the ride.**

```
block[2].clear AND NOT ride.estop
```

That is a whole override. There are no statements, no assignment, no loops, and **nothing is
remembered between scans**.

### It can only ever say NO

An override is ANDed onto the permissive it attaches to:

```
    permitted  =  (everything the default already required)  AND  (your expression)
```

So an override can **take a permission away and can never hand one out.** There is no syntax for
loosening a permissive, which is what makes an override from a downloaded track safe **by
construction** rather than because somebody reviewed it. `FPlcExpr::Restrict` is the only entry
point, and it is one `&&`.

A broken override — a typo, a name that does not exist, a type error — leaves the permissive
**unchanged** and reports the error. It does not deny. A typo must not be able to stop a ride any
more than it can start one.

### What it deliberately cannot do

**State and sequencing.** *"Run forward 4 seconds, then reverse, then park backwards"* needs
variables that survive a scan, and there are none. Nothing this project has built or planned needs
that — shuttles, turntables and transfer tracks are all unbuilt or parked — and the day something
does is the day to revisit full Structured Text. The ST design cards stay on the board for that.

What no-state buys, for free:

| | |
|---|---|
| **Determinism** | A pure function of a snapshot. Drops into the replay digest unchanged. |
| **No execution budget** | Cost is the size of the expression, counted when it loads. No loop can run away. |
| **No sandbox** | No heap, no closures, no references outside the snapshot. Nothing to escape into. |

---

## The process image

The snapshot has a name for everything the controller knows. **It is generated from your layout** —
a ride with six blocks has `block[0]` through `block[5]` and nothing else, so an override written
for a different track fails to load rather than quietly reading a slot that now means something
different.

| Name | Type | What |
|---|---|---|
| `ride.trains` | REAL | how many are on the ride |
| `ride.estop` | BOOL | latched emergency stop |
| `ride.outputs_enabled` | BOOL | the controller is permitting commands |
| `ride.scan` | REAL | scan counter |
| `block[i].clear` `.occupied` `.buffer` | BOOL | block state; `buffer` is the overlap held after a train physically exits |
| `zone[i].commanded` `.output` `.actual` | REAL | m/s — what the PLC wrote, what the drive ramped to, what the motor is turning at |
| `zone[i].load` | REAL | 0..1 torque |
| `zone[i].ready` `.faulted` `.holding` | BOOL | `holding` means the device has pads *and* tyres, so it can stop a train and start one |
| `platform[i].ready` `.in_position` `.restraints_locked` `.gates_closed` | BOOL | |
| `platform[i].phase` | REAL | the station sequence, as a number |
| `train[i].speed` | REAL | m/s |
| `train[i].moving` | BOOL | |

**There is no train position, deliberately.** The interlocking is still handed a span internally and
`SIGNALLING.md` records that as a cheat; exposing it to overrides would make the cheat permanent.
An override sees what a real controller sees — switch state, drive readings, derived block state.

The list above is **generated**, not maintained: `FProcessImage::Describe()` enumerates every slot
with its name and type at runtime, and that is what editor autocomplete reads. The assert suite
checks that every described slot binds back to itself, so the table above cannot drift without the
build noticing.

---

## The language

Identifiers and keywords are **case-insensitive**, as IEC 61131-3 requires.

### Operators, lowest precedence first

| | |
|---|---|
| `OR` `XOR` | BOOL only |
| `AND` | BOOL only |
| `=` `<>` `<` `<=` `>` `>=` | `=` and `<>` compare two BOOLs or two REALs; the ordered ones are REAL only |
| `+` `-` | REAL |
| `*` `/` `MOD` | REAL |
| `NOT` `-` (unary) | |
| `( )` | |

**`=` is equality, not assignment.** There is no assignment to confuse it with. **`<>` is
inequality** — not `!=`. **`AND` / `OR` / `NOT`** — not `&&`, `||`, `!`. A subset that used the C
spellings would not be a subset, and a later full ST would be a *replacement* rather than an
extension.

### Functions

```
SEL(G, IN0, IN1)      IN0 when G is FALSE, IN1 when G is TRUE
MIN(a, b)  MAX(a, b)  REAL
```

**Note `SEL`'s argument order.** `IN0` is the *false* case and it comes first. That reads backwards
to almost everybody, and it is the standard's order, so it is kept.

### Comments

```
(* an ST block comment *)
// to the end of the line
```

### Types

`BOOL` and `REAL`. A boolean operator applied to a REAL is an error when the override loads, not a
silent coercion — `zone[0].output AND ...` almost certainly means somebody wanted a comparison and
forgot it.

Division by zero yields **zero**, not an error and not infinity. An override is evaluated inside a
scan that has to finish, and a NaN reaching a permissive would make it neither true nor false.

---

## Worked examples

Every one of these is parsed and bound against a real layout in `test_plcexpr.cpp`, so a stale
example fails the build rather than your evening.

```
(* Hold the station until the mid-course brake is clear as well. *)
block[2].clear

(* Do not dispatch while any train is still moving anywhere on the ride. *)
NOT train[0].moving AND NOT train[1].moving

(* Extra headway: two blocks clear ahead instead of one. *)
block[3].clear AND block[4].clear

(* Do not launch into a drive that is not up to speed. *)
zone[1].ready AND zone[1].output >= 30.0

(* Hold if the transfer drive is pulling more than 90% torque —
   something is dragging and it should be looked at. *)
zone[4].load < 0.9

(* Belt and braces: never while the ride is stopped. *)
NOT ride.estop AND ride.outputs_enabled

(* Speed-dependent, and a good place to check you have SEL the right way round.
   G is block[3].clear. IN0 is 6.0 and applies when G is FALSE — so the train is
   limited to 6 m/s when the block ahead is NOT clear, and 26 when it is. *)
train[1].speed <= SEL(block[3].clear, 6.0, 26.0)

(* The platform's own three contacts, restated as an override. *)
platform[0].restraints_locked AND platform[0].gates_closed AND platform[0].in_position
```

---

## Where this sits

Three tiers, and requests never travel down (`CLAUDE.md` constraint 7):

- **Tier 1, safety** — C++, not scriptable, no debug bypass that ships. The E-stop chain, restraint
  proof, gate interlocks, overspeed, anti-rollback, block veto. **An override cannot reach it.**
- **Tier 2, control** — this page. Dispatch policy and sequencing, on top of a generated C++ default
  program. May ask the tier below and be refused.
- **Tier 3, show** — lights, audio, effects, cameras. **Read-only**: it subscribes to the ride's
  state-transition stream and writes to fixtures, which is also how DMX512 works on the wire.

This answers structurally the question every user-scripting system faces — *can a downloaded track
make the ride dangerous?* No, and not because the scripts are validated well: because the layer that
could is not scriptable, for the same reason it isn't in a real park.

## Reference

- `Prototypes/BlockSignal/PlcImage.h` — the process image
- `Prototypes/BlockSignal/PlcExpr.h` — lexer, parser, evaluator, `Restrict`
- `Prototypes/BlockSignal/test_plcexpr.cpp` — 12 assertions, including every example above
- `Docs/CONTROL_ARCHITECTURE.md` — the tier split and the IEC 61131-3 grounding
