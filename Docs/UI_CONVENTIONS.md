# UI conventions

The presentation rules every panel is built against: framework, resolution, colour, layout, units.

**Status: DECIDED 2026-08-05.** The framework choice is the developer's, made after the argument
below. The other four are defaults taken with reasoning, and each says what changing it costs.

This exists because the alternative is deciding it ten times, differently, once per screen. Nothing
here is built yet — the runtime shell is Phase 3.5. Read it before building the first panel, not
after.

---

## 1. Framework — UMG

**UMG for everything, with custom Slate widgets for the things that are *drawn* rather than
composed.**

The framing "UMG or Slate" is misleading: **UMG is Slate**. A `UButton` is a `UObject` that
constructs and owns an `SButton`; every UMG widget resolves to Slate at runtime. The choice is of
authoring layer, not renderer, which is what makes it cheap to revisit per widget.

**Why UMG.** Iteration speed, which for a solo developer decides it. Slate means a C++ compile for
every padding change, colour, and reorder. Over the runtime segment editor — the largest single
piece of UI work in the project — that difference compounds into weeks of nothing but rebuilds.

**The argument for Slate, and why it did not win.** Two real points:

- *Dense data.* The usual case for Slate is a list of hundreds of rows. **This project does not have
  one.** The reference layout is 23 segments for 615 m; the 1288 m circuit is a few dozen. The only
  thing producing thousands of segments is NL2 CSV import, which is a fixture path nobody edits.
  `UListView` is virtualised (entry-widget pooling) and covers what is actually here.
- *Diffability.* Widget Blueprints are binary `.uasset`; Slate is text that diffs and reviews. This
  project values that elsewhere — `TrackIO.h` stores what was typed specifically so saves diff. Solo
  it costs nothing; for an MIT project taking contributions it means the UI layer is the part
  nobody can review. **This is the strongest argument against the decision and it is recorded here
  rather than lost.** Mitigation: behaviour lives in C++ `UUserWidget` subclasses, and the Widget
  Blueprint holds layout only. A reviewer can then read the logic.

**The escape hatch.** A custom `SWidget` hosted by a thin `UWidget` subclass with a
`RebuildWidget()` override. Reach for it when a panel needs `OnPaint` rather than a widget tree:

- the ride-profile graph — plotted curves, not composed elements
- the generated control-panel schematic — a drawn diagram, already `FCanvas` today

Anything else stays UMG until something measures slow.

**Editor-mode UI is Slate, no choice.** UMG is runtime-only. Everything on Phase 3.5 is the
packaged app, so this does not bind, but it is why the hatch stays open.

---

## 2. Resolution and DPI

| | |
|---|---|
| Minimum supported | **1600 × 900** |
| Design target | **1920 × 1080** |
| Must stay legible at | **3840 × 2160** |

**DPI Scaling Rule: `Shortest Side`** (Project Settings → User Interface), the UE default and the
right one here.

**Clamp the curve so a 4K monitor shows *more*, not bigger.** The failure mode for a data-dense
tool is a 2160-tall display scaling to 2.0 and fitting exactly as many segment rows as the 1080p
laptop did — all that resolution spent on larger glyphs. Cap the scale around **1.5** at 2160 and
let the surplus pixels become rows.

Corollary: **lists and tables size to available space, never to a fixed row count.** A panel that
hardcodes twelve visible rows defeats the clamp.

*Cost of changing later:* low. One curve and any layout that assumed a row count.

---

## 3. Colour

**Dark theme first.** It is a night-park-friendly, screenshot-friendly tool, and every reference
console photographed for the control panel is dark.

### The rule that matters more than the palette

**Hue is never the only channel.** Meaning is carried by position, shape or a text legend, and
colour reinforces it. Real operator panels do this — a lamp has a fixed position, an engraved
legend, *and* a colour — and it is why they remain readable to someone who cannot separate red from
green. Any indicator that would become ambiguous in greyscale is wrong regardless of which hues it
uses.

### Two palettes, because they answer to different masters

**The operator panel keeps hardware colours.** Red / green / amber, as photographed on three real
consoles. Changing these for accessibility would make the panel *less* faithful, which is the whole
point of it. The legend rule above is what makes them safe.

**Analysis graphs and traces use Okabe–Ito**, the standard colourblind-safe qualitative palette.
Do not invent a palette; this one already exists and is designed for exactly this.

| Channel | Colour | Hex | RGB |
|---|---|---|---|
| Vertical G | sky blue | `#56B4E9` | 86, 180, 233 |
| Lateral G | orange | `#E69F00` | 230, 159, 0 |
| Speed | bluish green | `#009E73` | 0, 158, 115 |
| Roll rate | reddish purple | `#CC79A7` | 204, 121, 167 |

This **replaces the current trace palette** (green / orange / cyan / magenta in
`ATUCoasterRide::DrawRideProfile`), which the ride-profile card flagged as not accessible — the
green/orange pair is the common deuteranopia collision.

Remaining Okabe–Ito entries, if a fifth channel appears: vermillion `#D55E00`, yellow `#F0E442`.
**Blue `#0072B2` is excluded** — roughly 3:1 against the background below, too weak for a 2 px line.

### Surface ramp

| Role | Hex |
|---|---|
| Background | `#14171A` |
| Panel | `#1E2226` |
| Border / divider | `#2E343A` |
| Text, primary | `#E6E9EC` |
| Text, secondary | `#9AA3AB` |

**Contrast check: WCAG AA** — 4.5:1 for body text, 3:1 for large text and graphical elements. Run it
on any new colour before it ships.

*Cost of changing later:* moderate and rising with screen count, which is why it is settled now.

---

## 4. Layout — fixed

**No docking, no user-arrangeable panels, no saved workspace.** Each mode gets a designed layout.

Dramatically cheaper, and right for 0.1 — a docking system is weeks of work whose payoff is people
rearranging a tool they have not learned yet. Every panel on the board is a *view of one thing*
(segments, validation, profile, operate), not a workspace someone composes.

*Cost of changing later:* high if retrofitted onto ten screens, low if the day comes when panels
are already separate widgets that do not assume their own size. **So: no panel hardcodes its own
dimensions.** That single restraint is what keeps the option open, and it costs nothing today.

---

## 5. Units — always shown, and never converted on the way in

**Authored fields carry the model's own units: metres and degrees.** Never converted for display,
never stored converted. This is the same rule `TrackIO.h` follows — the file stores what was typed —
and it means a number on screen and the number in the save are the same number.

**Derived readouts carry rider units**, because they are read by a person judging a ride, not
editing one:

| Quantity | Unit |
|---|---|
| Length, height, arc position | m |
| Angle, bank, roll | ° |
| Speed | km/h |
| Acceleration / felt G | g |
| Roll rate | °/s |
| Time, dwell, buffer | s |

**The unit is always in the label or as a suffix, never implied by context.** A field reading `30.5`
is a defect; `30.5 m` is a field.

Radians stay internal and never reach a widget. `FTrackSegment` stores radians; the editor shows
degrees and converts at the boundary — the one place conversion is allowed, because it is a display
concern rather than authored data.

**Imperial units are not in 0.1.** The authored/derived split above is what makes adding them later
a display-layer change rather than a data-model one: only the right column moves.

*Cost of changing later:* low, by construction.

---

## What is not decided here

- **Font.** Wants seeing on a 4K panel before being fixed. Whatever it is, it needs tabular figures
  — a column of numbers that shifts as digits change is unreadable.
- **Iconography.** Nothing needs an icon yet.
- **Sound.** Phase 5.
