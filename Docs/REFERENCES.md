# References

Outside work this project relies on, and what each one actually contributed. Kept because a
number with no source is a number nobody can check, and because people whose work shaped a design
decision here should be named.

**If you use something from this list, cite it where you use it** — the code comments carry the
attribution alongside the figure, not only this page.

## Academic

### Self, Ian (2024)

> **Self, Ian.** *Parametric Design and Optimization of Roller Coaster Support Structures
> Considering Sustainability and Maintenance.* Master of Science thesis, Architectural Engineering,
> The Pennsylvania State University. Defended 27 February 2024.
>
> Committee: **Nathan Brown** (thesis advisor), **Thomas E. Boothby**, **John Messner**,
> **Rebecca Napolitano**; **James Freihaut** (program head).
>
> <https://etda.libraries.psu.edu/catalog/27672izs5144>

A parametric structural analysis of steel coaster support structures under dead, live (ride vehicle)
and wind loads, with member sizing and multi-objective topology optimisation (NSGA-II) over a ground
structure in which supports can be switched on and off. Three case studies at different scales: a
small ride, a large ride, and individual ride elements.

**Its headline result**, in the author's terms: early-design optimisation reduced structural mass by
about **25%** against a baseline built from industry rules of thumb, and about **20%** against a
baseline modelled on an existing ride — with embodied carbon, and the inspection burden of fewer
connections, as the motivating objectives rather than cost alone.

**What it contributed here, in `Prototypes/TrackMesh/TrackSupports.h`:**

- **A cited maximum span.** The header's support spacing was "roughly 6–12 m depending on section
  and load", written from general knowledge. The thesis reports a **maximum spacing of forty feet
  (~12.2 m)** as the reference practice, citing Hunt (2018) — so the project's 9 m default is now
  inside a figure with a source rather than inside a recollection.
- **Support density is not uniform in a real design.** The optimisation found that supports
  **cluster where ride G-forces are highest**, arrived at independently by the optimiser rather than
  imposed. This project places on a uniform span today and already computes G at every arc length,
  so this is a concrete and citable future refinement rather than a guess — recorded in the header
  as such.
- **What it does not cover, and therefore neither do we on its authority.** Foundations are
  explicitly out of the thesis's scope ("additional material factors such as the concrete used in
  the construction of the foundations... are not directly addressed"). The footing geometry in this
  project is ordinary structural practice, not something the thesis supports, and is commented that
  way.

**Not used:** the optimisation itself, the member sizing, the mass/embodied-carbon objective. This
project places supports; it does not size them or check them against load. That distinction matters
— nothing here should be read as structural analysis.

### Hunt (2018)

> **Design Analysis of Roller Coasters.** Cited *within* Self (2024); not read directly.

Source of the forty-foot maximum support spacing above. Attributed second-hand deliberately: the
figure reached this project through Self's literature review, and saying so is the difference
between a citation and a claim.

## Standards and specifications

Named in the code and in `CLAUDE.md` where they are relied on. Listed here so the set is visible in
one place.

| Standard | Where it is used |
|---|---|
| **ASTM F2291 / EN 13814** | Acceleration envelopes. `Prototypes/TrainPhysics/GEnvelope.h` — **the limit tables are UNVERIFIED research, not a copy of either standard** (both paywalled), and the header, `PROTOTYPES.md` and the suite banner all say so. |
| **IEC 61131-3** | PLC scan model (read inputs, execute, write outputs) and the ST spelling of Tier 2 overrides. |
| **IEC 60204-1** | Stop categories 0 and 1. |
| **IEC 61800-5-2** | STO and SS1, which the two stop categories map onto. |
| **IEC 61800-7-201/301, CiA 402** | The drive state machine — implemented as specified rather than modelled. |
| **NFPA 160** | Why flame effects are not fired by the show controller. |
| **DMX512** | Universes, slots, start codes, and the 44 Hz packet ceiling. |

## Sound

Freesound files keep their `id__username__title.wav` name **on purpose**: the ID is the permalink
and the username is the author, so a file that gets copied somewhere still carries its own
attribution. Do not rename them.

| File | Author | Freesound ID |
|---|---|---|
| `416080__davidlay1__air-release.wav` | davidlay1 | [416080](https://freesound.org/s/416080/) |
| `131931__mcpable__slips-air-release-v4.wav` | mcpable | [131931](https://freesound.org/s/131931/) |
| `131932__mcpable__slips-air-release-v3.wav` | mcpable | [131932](https://freesound.org/s/131932/) |
| `131933__mcpable__slips-air-release-v2.wav` | mcpable | [131933](https://freesound.org/s/131933/) |
| `131934__mcpable__slips-air-release-v1.wav` | mcpable | [131934](https://freesound.org/s/131934/) |

Used for **brake release** in `Content/Audio/Brakes`, one picked at random per release. A coaster
brake is spring-applied and air-released, so the hiss is the pad **letting go** rather than grabbing.

### ⚠ LICENCES ARE UNVERIFIED AND MUST BE CHECKED BEFORE ANY RELEASE

Freesound hosts material under **CC0**, **CC-BY**, **CC-BY-NC** and **CC Sampling+**, and the licence
is per upload rather than per site. **CC-BY-NC and Sampling+ are not compatible with this
repository**, which is MIT and expects to be redistributed and forked commercially — and neither is
anything requiring attribution that this project fails to give.

Nothing about the filename says which applies. Each ID above links to its own page; the licence is
stated there. Until somebody has checked all five:

- **CC0** — nothing owed, though the credit above is kept anyway.
- **CC-BY** — usable, and the credit above is the obligation. It must survive into a packaged build,
  not only live in this file.
- **CC-BY-NC or Sampling+** — **must be replaced**, however good it sounds.

This is the same class of constraint as the one on paid marketplace assets and manufacturer
trademarks: the licence decides, not the quality.

## Other

- **Okabe–Ito palette** — the colourblind-safe qualitative palette used for analysis traces.
  Chosen rather than invented, per `Docs/UI_CONVENTIONS.md`.
- **NoLimits 2** — used as a validation fixture and as a design foil. Its documented tab-separated
  spline export is read and written by `Prototypes/NL2Csv/`; a recorded run of a dead-flat launch
  calibrated `DragK` and `Crr` (`PHASE0_FINDINGS.md`). **No NL2 park files or exports of real rides
  are committed here** — `Prototypes/NL2Csv/Tracks/` is gitignored for that reason.
- **Three operator consoles**, photographed, which corrected the generated control panel. What they
  changed is recorded in `SIGNALLING.md` § "What three real consoles taught it".
- **Alcorn McBride** — their own manual's statement that show equipment "is not intended for use in
  applications where a malfunction can reasonably be expected to result in personal injury", which
  is the external support for the three-tier split.
