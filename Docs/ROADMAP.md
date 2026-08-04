# Roadmap

Solo-developer paced, and staged so there is a genuinely shippable, demonstrable milestone at the end
of every phase. That matters for more than morale: each one is something you could post a devlog or a
video about, which is how open-source contributors and an eventual community get recruited.

Free/open-source projects of this ambition die from scope creep far more often than from lack of
ambition. The staging below is the defence.

**Current phase: 2 — Physics & Ride Feel.**

| Phase | Goal | Shippable artifact | Status |
|---|---|---|---|
| **0 — Prototype** | Prove the core feel: a hand-authored spline, a train that follows it with real physics, a camera. | A single layout you can ride, with correct-feeling G-forces. | ✅ **Complete** |
| **1 — Track Editor MVP** | Numeric segment-list editor, banking control, undo/redo from day one, live G graph, circuit closure solver. | Build an arbitrary coaster from scratch in-editor. | ✅ **Complete** |
| **2 — Physics & Ride Feel** | Friction and drag tuning, lift hills, launches, block brakes, heartline camera polish. | A ride-through that feels right to a NoLimits 2 veteran. | 🔶 **In progress** |
| **3 — Ride Control System** | Block state machine wired to the ride, dispatch permissives, auto/manual modes, generated 2D control panel with VFD visualisation. | Watch — or run — a coaster dispatch itself safely from a real generated panel. | 🔶 **Half met early** — the coaster dispatches itself safely, with up to four interlocked trains lapping a closed circuit. The generated panel and manual mode are not built, and they are the differentiator. |
| **4 — Track Meshing & Supports** | Procedural rail/tie/support generation, at least one visual track style. | A coaster that looks like a real structure, not a spline in space. | Planned |
| **5 — Scenery & Park Layer** | Terrain sculpting, static scenery placement, basic landscaping. | A small themed area around the coaster. | Planned |
| **6 — Sharing & OSS Launch** | Save/load, export format documentation, public repo, first public build. | A public 0.1 release and an announcement devlog. | Planned |
| **7 — Community Growth** | Contributor onboarding, issue triage, more coaster styles by demand. | A growing contributor base. | Ongoing |
| **Stretch** | VR ride-throughs; a walk-in 3D control booth over the same generated data. | Revisited once the flatscreen core is solid. | Deferred |

Phases 0–3 are the highest-risk, highest-value work and deserve the most patience. Everything after
Phase 3 is comparatively conventional game development — but a coaster sim that does not *feel*
physically right, or does not dispatch safely, is worthless regardless of how good the scenery tools
are.

---

## Phase 0 — complete

The three hardest cores were prototyped standalone, outside the engine, because de-risking there is
cheaper than de-risking inside it. Two more prototypes were added beyond the original plan for the
same reason. Then the vertical slice: a hand-authored layout that builds against UE 5.8, rides, and
reads out speed and G on screen — from the same headers the standalone assert suites test.

The Coaster Forge build-vs-adapt evaluation closed here: **build**. It is a commercial product and
cannot be redistributed under MIT, so "adapt" was never actually available.

Detail, with numbers: [`PHASE0_FINDINGS.md`](PHASE0_FINDINGS.md).

## Phase 1 — complete

The track became data you can edit rather than code you recompile:

- a typed segment list in Unreal's Details panel, with a live viewport preview and ride-profile traces
- a diffable JSON save format that stores what was authored, never what was derived
- snapshot undo/redo with the save format as identity
- a damped Gauss-Newton circuit-closure solver over the parameters you free
- validation that reports and never repairs, including self-clearance
- world-referenced roll as a per-segment mode, alongside path-relative
- a train with length, and a seat you can choose

Two cards remain open in the Phase 1 list, and both are **deliberate deferrals rather than unfinished
work**:

- **the loop side-step** — the authored vocabulary cannot express a rideable one; five fixes were
  measured and rejected
- **ride-profile trace legibility** — an aesthetics pass that wants designing, not accreting

## Phase 2 — in progress

**The bar is "feels right to an NL2 veteran."** This is the credibility milestone; the whole project's
standing with the audience that made NoLimits 2 successful rests on it.

Landed so far: calibration against NoLimits 2 telemetry, and the correction it forced.
`RollingResistance` had been justified as steel-on-steel — a railway figure — where a coaster runs
polyurethane wheels on steel; three recordings converge on 0.022–0.026 against a shipped 0.006, so the
default moved to **0.024**. The residual of 0.0005 m/s² says the model's shape was right all along.
The reference layout was re-tuned around it: a deeper drop to restore the loop apex, and a longer lift
purely to close the ride back to station level.

Also landed, and it took Phase 3's headline with it: **a closed circuit carrying four interlocked
trains that run real laps.** The two-train preset closes to 0.000000 m of position, 0.000084° of
heading and 0.000000° of roll — by *shape* rather than by solver, an oval whose two exactly-180°
turns cancel — and `FTrain` wraps arc length rather than clamping, so a train drives through the seam
into the station under its own power. Block brakes hold and release on a permissive, and the
permissive now **derives its braking distance** from the layout instead of clearing a fixed count of
blocks. Measured: 1 to 4 trains, every train lapping, zero violations, never two in a block.

Two defects on the way there were only findable by running more trains than anyone had: a fixed
lookahead let four trains collide, and a held train parking at the *start* of its device deadlocked
three. Both are in [`PHASE0_FINDINGS.md`](PHASE0_FINDINGS.md), both are now asserted directly rather
than via the symptom.

Still ahead, and worth starting next: the consequences of train length are only partly worked through.
It was *hypothesised* that the same omission explains the fitted `DragK` landing 3.2× above its
physically derived value — that is neither confirmed nor ruled out, because refitting across train
lengths leaves the predictor correlation pinned at 0.975 and this data cannot test it. Separating the
pair needs a **fast** reference recording; the existing one tops out at 44.5 km/h, and drag needs speed
to have anything to be measured against. See
[`PROTOTYPES.md`](PROTOTYPES.md#nl2telemetry--live-comparison).

---

## Execution notes

Three things matter more than usual for a solo, free, open-source project of this scope:

1. **Build in public early.** Devlogs or short clips at the end of each phase are how contributors and
   a community get recruited. A coaster-sim audience demonstrably wants to engage with development
   progress.
2. **Protect Phases 0–2 from scope creep.** It will be tempting to start on scenery, because it is
   more visually rewarding than tuning friction coefficients. The physics core is the entire reason
   this project would be credible.
3. **Ship small and testable.** Favour increments over broad refactors. There is no funding cushion
   and no team to absorb a stalled rewrite.

Live day-to-day task status lives on
[Trello](https://trello.com/b/Uzqm38o7/coaster-sim-nolimits-successor), not here. This page is the
design reference.
