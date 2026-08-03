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

Measured, regenerated 2026-08-02:

```
16 segments · 591.72 m developed · 367.21 m horizontal · crest 50.07 m
105.2 km/h · vertical G +0.66 to +4.20 · peak lateral 0.28 g · 63.4 s
loop R9.0 m with 54 m eases, apex 27.63 m at +1.13 g
banked turn R32.0 m at 65.92° · closes to +0.0015 m
curvature-continuous, verified to 1e-9 across all 15 joints
```

`Docs/REFERENCE_LAYOUT.md` is the canonical home for these. If this pack and that page ever
disagree, that page is right.

**These are not the figures the pack shipped with.** The pack was built against 563.84 m / 43.35 m /
101.0 km/h, which were correct at the time and were superseded hours later: `RollingResistance` had
been justified as steel-on-steel, where a coaster runs polyurethane on steel, and correcting
0.006 → 0.024 forced a re-tune of the reference layout (deeper drop to restore the loop apex, longer
lift to close back to station level). `src/data.js` has been regenerated against the current code.

## Regenerating

The pack is built from `src/` by `build.py`, which inlines `src/tokens.css`, `src/data.js` and the
brand geometry from `src/logo.py` into the templates. If you only want to change a colour, edit
`src/tokens.css` and rebuild — or edit the `:root` block at the top of any single HTML file, since
each one carries its own copy.

## TODO for Cowork

**1. `build.py` cannot run: `templates/` is not in the repo.** It reads `templates/{loading,site,social,brand}.html`
and substitutes `/*__TOKENS__*/`, `/*__DATA__*/` and `/*__LOGO__*/` into them. Only the *built*
outputs are here, so the pack currently cannot be rebuilt from source at all — which is what blocked
fixing item 2 by hand. Restore `templates/`, or reconstruct them by replacing each inlined block in
the built HTML with its marker.

**2. Two PNGs bake superseded figures into the artwork.** Both were pulled from the repo rather than
committed, because both are captioned as measured output and a wrong "MEASURED, NOT DRAWN" diagram is
worse than none:

| File | Shows | Should show |
|---|---|---|
| `github/layout-1280x560.png` | 563.84 m · 43.35 m · 101.0 km/h · apex 27.90 m | 591.72 m · 50.07 m · 105.2 km/h · apex 27.63 m at +1.13 g |
| `github/social-preview-1280x640.png` | 563.84 m · 43.35 m · 101.0 km/h | as above |

`src/data.js` is already correct — it was regenerated directly from the prototype headers, so a
rebuild picks the new figures up with no hand-editing. The elevation *shape* changes slightly too
(the lift is 15.9 m longer and the drop 12 m deeper), so the traces want re-plotting, not just the
labels retyping.

Also stale, same cause, lower priority: the built `index.html`, `brand-system.html`,
`loading-screen.html` and `social-pack.html` each carry their own inlined copy of the old `TU`
object. A rebuild fixes all four at once.

Once both are done, restore the diagram to the README (`## The reference layout`) and to
`Docs/REFERENCE_LAYOUT.md`, where a note currently explains its absence.

**3. Export the loading screen at a usable size.** There is no PNG of it in the repo at all yet —
only the `.html`. Two different assets are involved and they are easy to confuse:

| Asset | Where it goes | Size | Format |
|---|---|---|---|
| **In-game loading screen** | UMG widget background | **3840×2160** (16:9) | PNG, RGBA |
| Startup splash | `Content/Splash/Splash.bmp` + `EdSplash.bmp` | **720×370** | 24-bit uncompressed BMP |
| Window/taskbar icon | `Content/Splash/IconDefault.bmp` + `EdIconDefault.bmp` | **64×64** | 24-bit BMP |
| Packaged app icon | `Build/Windows/Application.ico` | 16/24/32/48/64/128/256 | multi-res ICO |

Everything except the loading screen already exists at those sizes — do not resize them, Unreal
expects exactly these.

For the loading screen, capture `loading-screen.html?static=1` at 3840×2160. That is 2× the page's
own 1920×1080 coordinate space, so text and hairlines stay crisp, and it downsamples cleanly to 1440p
and 1080p. It is vector, so any integer multiple of 1920×1080 works if 4K is heavier than wanted —
but do not go below 1920×1080, which is where the 1 px technical-drawing rules start to break up.

On import: Texture Group **UI**, compression **UserInterface2D (RGBA)**, and **turn mipmaps off**.
The default settings will DXT-compress it and put visible blocking artifacts along exactly the thin
bright lines the whole design is made of. Non-power-of-two is fine for UI textures.

Longer term the live-UMG route in [Getting it into Unreal](#getting-it-into-unreal) is better than any
export — the page is data-driven from `TU`, so a UMG rebuild tracks the layout automatically instead
of going stale the way the artwork above just did.

**Regenerate after any change to `ReferenceLayout()` or to the physics defaults.** This went stale
within a day the first time. `src/data.js` carries the same warning at the top.

Licence: same as the project (MIT). No manufacturer trademarks, logos or ride designs are used
anywhere in these assets.
