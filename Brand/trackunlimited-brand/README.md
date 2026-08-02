# TrackUnlimited — brand and promo pack

Technical-drawing visual identity for TrackUnlimited. Everything here is one self-contained file:
no build step, no webfonts, no network requests. Open any `.html` in a browser.

## What's here

| File | What it is |
|---|---|
| `brand-system.html` | The system itself — marks, palette with live WCAG contrast checks, type scale, line weights, motion rules, do/don't. **Read this first.** |
| `loading-screen.html` | Animated 16:9 technical-drawing loading screen. |
| `index.html` | The website. Drop it in a repo and turn GitHub Pages on. |
| `social-pack.html` | Ten social cards at exact platform sizes, each with SVG and PNG export. |
| `brand/*.svg` | Wordmark, lockup, mark, badge, avatar, favicon as standalone vector files. |

## Before you publish

1. **`index.html` — fill in the links.** There is a `LINKS` object at the top of the page's script
   with `CHANGEME` placeholders for the repository, plan and issues URLs. That is the only place
   links are defined.
2. **`index.html` — export the OG image.** Open `social-pack.html`, download
   `og-1200x630.png`, and put it next to `index.html`. The `og:image` meta tag already points at
   that filename.
3. **Favicon.** `brand/favicon.svg` — add `<link rel="icon" href="brand/favicon.svg">` to
   `index.html` if you want it.

## Loading screen

Query parameters, for capture and for engine use:

- `?p=0.42` — freeze at 42 % and disable animation. Use this to grab an exact still.
- `?static=1` — render the finished state immediately, no animation, progress at 100 %.
- `?loop=0` — play once and hold at 100 % instead of looping.

`prefers-reduced-motion` is honoured everywhere; the draw-in is skipped and the final state renders
straight away.

### Getting it into Unreal

Two routes, depending on what you want:

- **Static texture.** Load `loading-screen.html?p=0.0` in a browser at 3840×2160 (or use a headless
  screenshot) and save the PNG. That gives a UMG background image at any resolution you like — it is
  vector, so it does not degrade.
- **Live UMG.** The layout is a plain 1920×1080 coordinate space and every element is data-driven
  from `TU.meta`, `TU.zones` and the `S/X/Z` arrays in the page's script. Rebuilding it as a UMG
  widget is mostly a matter of re-emitting the same polylines and text at the same coordinates, with
  the real load progress replacing the simulated one. The block-state lamps and the readout are
  already written as pure functions of one number (arc length), so they port directly.

## Social pack

Each card exports as SVG (editable vector) or PNG (1× / 2× the platform size). Rasterisation happens
in the browser — nothing is uploaded anywhere. The devlog card has editable fields above the preview
so it can be reused for every post rather than redrawn.

Cards: announcement 1600×900 · link preview 1200×630 · three square features 1080×1080 · devlog
1200×675 · portrait stat sheet 1080×1350 · profile banner 1500×500 · YouTube channel art 2560×1440 ·
avatar 512×512.

Banners have a **safe-area toggle** that shows the crop guides. Guides are preview-only and are never
included in an export.

## Where the numbers come from

Every figure and every line of the elevation drawing is generated, not drawn by hand: the project's
own `Prototypes/TrackSpline` and `Prototypes/TrainPhysics` headers were compiled against the current
`ReferenceLayout()` in `Source/TrackUnlimited/TUCoasterRide.cpp`, and `RunRideProfile()` was run over
the result with the slice's own 15 m train. The output lives in the `TU` object inlined into each
page.

Measured, current as of this pack:

```
16 segments · 563.84 m developed · 342.87 m horizontal · crest 43.35 m
101.0 km/h · vertical G +0.66 to +4.20 · peak lateral 0.36 g · 59.2 s
loop R9.0 m with 54 m eases · banked turn R32.0 m at 65.92°
curvature-continuous, verified to 1e-9 across all 15 joints
```

**Heads-up:** several repo documents still publish the pre-closure-solver figures
(543.7 m, 34.9 m lift) and an older set of ride figures. Those are stale — see the note that came
with this pack. The numbers above are what the current code actually produces.

## Regenerating

The pack is built from `src/` by `build.py`, which inlines `src/tokens.css`, `src/data.js` and the
brand geometry from `src/logo.py` into the templates. If you only want to change a colour, edit
`src/tokens.css` and rebuild — or edit the `:root` block at the top of any single HTML file, since
each one carries its own copy.

Licence: same as the project (MIT). No manufacturer trademarks, logos or ride designs are used
anywhere in these assets.
