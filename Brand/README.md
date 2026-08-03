# TrackUnlimited — brand and promo pack

Technical-drawing visual identity for TrackUnlimited. Every built page is one self-contained file:
no build step needed to *use* one, no webfonts, no network requests. Open any `.html` in a browser.

## What's here

| File | What it is |
|---|---|
| `brand-system.html` | The system itself — marks, palette with live WCAG contrast checks, type scale, line weights, motion rules, do/don't. **Read this first.** |
| `loading-screen.html` | Animated 16:9 technical-drawing loading screen. |
| `index.html` | The website. Drop it in a repo and turn GitHub Pages on. |
| `social-pack.html` | Fifteen cards at exact platform sizes, each with SVG and PNG export. |
| `brand/*.svg` | Wordmark, lockup, mark, badge, avatar, favicon as standalone vector files. |
| `github/*.png` | The five images the repo README and `Docs/` use, pre-rendered. |
| `export/*.png` | The loading screen at 3840×2160, ready for UMG. |
| `templates/`, `src/`, `build.py` | The sources. `python3 build.py` regenerates all four pages. |

## Rebuilding

```sh
clang++ -std=c++17 -O2 -I Prototypes/TrackSpline -o gen_data.exe Brand/src/gen_data.cpp
./gen_data.exe > Brand/src/data.js     # measure the layout
cd Brand && python3 build.py           # templates/ + src/ -> the four built pages
```

**`src/data.js` is generated, not hand-edited** — `src/gen_data.cpp` compiles the project's own
prototype headers against `ReferenceLayout()` and prints it. Editing `data.js` by hand works until
the next regeneration silently reverts it, which is how the "loop apex" figure below came to be wrong
in two places at once. Change `gen_data.cpp`.

`build.py` inlines `src/tokens.css`, `src/data.js` and the brand geometry from `src/logo.py` into
`templates/{loading,site,social,brand}.html`, replacing the `/*__TOKENS__*/`, `/*__DATA__*/` and
`/*__LOGO__*/` markers. `src/logo.py` needs `fonttools`, and converts the wordmark to outlines from
TeX Gyre Heros Cn so the mark carries no font dependency; if that font is not installed, the built
pages keep whatever outlines `src/logo.json` already holds.

To re-render `github/*.png` after a data change: open `social-pack.html`, scroll to the GitHub group,
and use each card's **PNG @1×** button. For `export/*.png`, see [below](#exporting-the-loading-screen).

> **One re-render outstanding, on one image.** `github/layout-1280x560.png` still labels its loop
> callout `27.63 m` — the height where felt G bottoms out, not the loop's apex of `27.90 m`, which are
> two different points on the track rather than two samples of one. `data.js` and all four built pages
> are corrected, and every other figure on that PNG is current; it just needs the **PNG @1×** button
> pressed once. The other four `github/*.png` are unaffected — `social-preview` carries no apex
> callout, and the hero, authoring and signalling figures carry no measured figures at all.

If you only want to change a colour, edit `src/tokens.css` and rebuild — or edit the `:root` block at
the top of any single built page, since each one carries its own copy.

## Before you publish

1. **`index.html` — fill in the links.** There is a `LINKS` object at the top of the page's script
   with `CHANGEME` placeholders for the repository, plan and issues URLs. That is the only place
   links are defined.
2. **`index.html` — export the OG image.** Open `social-pack.html`, download `og-1200x630.png`, and
   put it next to `index.html`. The `og:image` meta tag already points at that filename.
3. **Favicon.** `brand/favicon.svg` — add `<link rel="icon" href="brand/favicon.svg">` to
   `index.html` if you want it.
4. **GitHub social preview.** `github/social-preview-1280x640.png` → Settings → General → Social
   preview.

## Loading screen

Query parameters, for capture and for engine use:

- `?p=0.42` — freeze at 42 % and disable animation. Use this to grab an exact still.
- `?static=1` — render the finished state immediately, no animation, progress at 100 %.
- `?loop=0` — play once and hold at 100 % instead of looping.

`prefers-reduced-motion` is honoured everywhere; the draw-in is skipped and the final state renders
straight away.

### Exporting the loading screen

Already done, in `export/`:

| File | Frame |
|---|---|
| `loading-screen-3840x2160.png` | `?p=0` — train in the station, 0 %, "READING SEGMENT LIST". The frame a loading screen should open on. |
| `loading-screen-complete-3840x2160.png` | `?static=1` — 100 %, train in the brake run. Useful as a still or a press image. |

Both are RGBA PNG at exactly 2× the page's own 1920×1080 coordinate space, so text and hairlines stay
crisp and they downsample cleanly to 1440p and 1080p. It is vector underneath, so any integer
multiple of 1920×1080 works if 4K is heavier than wanted — but do not go below 1920×1080, which is
where the 1 px technical-drawing rules start to break up.

To re-capture at another size or progress value, load the page at that viewport and screenshot, or:

```sh
python3 -c "
from playwright.sync_api import sync_playwright
with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={'width': 3840, 'height': 2160})
    pg.goto('file:///ABSOLUTE/PATH/loading-screen.html?p=0')
    pg.wait_for_timeout(1500)
    pg.screenshot(path='loading-screen-3840x2160.png')
    b.close()"
```

**On import into Unreal:** Texture Group **UI**, compression **UserInterface2D (RGBA)**, and **turn
mipmaps off**. The defaults will DXT-compress it and put visible blocking artifacts along exactly the
thin bright lines the whole design is made of. Non-power-of-two is fine for UI textures.

These are the *loading screen* only. The splash and icon assets are different sizes and formats and
already exist in the repo — `Content/Splash/Splash.bmp` and `EdSplash.bmp` at 720×370 24-bit BMP,
`IconDefault.bmp` and `EdIconDefault.bmp` at 64×64, `Build/Windows/Application.ico` multi-res. Do not
resize those; Unreal expects exactly those dimensions.

### Getting it into Unreal, properly

Longer term the live-UMG route beats any export. The layout is a plain 1920×1080 coordinate space and
every element is data-driven from `TU.meta`, `TU.zones` and the `S/X/Z` arrays in the page's script.
Rebuilding it as a UMG widget is mostly a matter of re-emitting the same polylines and text at the
same coordinates, with the real load progress replacing the simulated one. The block-state lamps and
the readout are already written as pure functions of one number (arc length), so they port directly —
and a UMG rebuild then tracks the layout automatically instead of going stale the way a PNG does.

## Social pack

Each card exports as SVG (editable vector) or PNG (1× / 2× the platform size). Rasterisation happens
in the browser — nothing is uploaded anywhere. The devlog card has editable fields above the preview
so it can be reused for every post rather than redrawn.

Social: announcement 1600×900 · link preview 1200×630 · three square features 1080×1080 · devlog
1200×675 · portrait stat sheet 1080×1350 · profile banner 1500×500 · YouTube channel art 2560×1440 ·
avatar 512×512.

GitHub: social preview 1280×640 · README hero 1280×400 · reference-layout figure 1280×560 · authoring
figure 1280×420 · signalling figure 1280×420.

Banners have a **safe-area toggle** that shows the crop guides. Guides are preview-only and are never
included in an export.

## Where the numbers come from

Every figure and every line of the elevation drawing is generated, not drawn by hand, and the thing
that generates them is committed: [`src/gen_data.cpp`](src/gen_data.cpp) compiles the project's own
`Prototypes/TrackSpline` and `Prototypes/TrainPhysics` headers against the current `ReferenceLayout()`
in `Source/TrackUnlimited/TUCoasterRide.cpp` and runs `RunRideProfile()` over the result with the
slice's own 15 m train. The output is `src/data.js`, inlined into each page at build time.

That it is committed is the point. The caption on the layout diagram reads "MEASURED, NOT DRAWN",
and until this existed nobody could check that without writing their own driver first.

Measured, regenerated 2026-08-02 after the rolling-resistance correction:

```text
16 segments · 591.72 m developed · 367.21 m horizontal · crest 50.07 m
105.2 km/h · vertical G +0.66 to +4.20 · peak lateral 0.28 g · 63.4 s
loop R9.0 m with 54 m eases, apex 27.90 m · minimum inverted G +1.13 g, 2.26 m earlier
banked turn R32.0 m at 65.92° · closes to +0.0015 m
curvature-continuous, verified to 1e-9 across all 15 joints
```

`Docs/REFERENCE_LAYOUT.md` is the canonical home for these. If this pack and that page ever disagree,
that page is right.

**Regenerate after any change to `ReferenceLayout()` or to the physics defaults.** This went stale
within a day the first time, when `RollingResistance` was corrected from a steel-on-steel figure to
the 0.024 a polyurethane-on-steel coaster actually runs, and the layout was re-tuned around it. Six
array checksums in `src/data.js` were verified byte-identical against an independent recompile of the
prototype headers before this pack was rebuilt — that is the check worth repeating.

Licence: same as the project (MIT). No manufacturer trademarks, logos or ride designs are used
anywhere in these assets.
