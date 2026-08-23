# Third-party content

`LICENSE` (MIT) covers the code and everything original to this project. It does
**not** cover the files listed here — each carries its own licence, and this page
is where those live.

**This file ships**, copied beside the executable by `Tools/Package-Windows.ps1`. Attribution that only exists in a git repository does not
reach somebody who downloads a build, and most content licences require it to.
Anything added below must also appear in the application's credits.

## Why per-asset and not one blanket statement

A licence belongs to a file, not to a folder or a project. Two grass meshes from
the same store page can be under different terms, and "everything in Content/ is
CC-BY" is the kind of claim that is convenient, unverifiable, and wrong the first
time somebody adds an exception.

## Rules for adding anything here

1. **The licence decides, not the quality.** Check the licence before the
   download, on the listing itself rather than on a search filter — "free" and
   "CC-BY" are different things, and stores mix them on the same page.
2. **Redistributable in SOURCE form, or it does not go in this repository.**
   This is the one that catches people. Fab's Standard License, the Unity Asset
   Store's EULA and most marketplace terms permit shipping an asset inside a
   packaged product and forbid publishing the asset itself — which is exactly
   what committing it to a public repository does. CC0, CC-BY, and CC-BY-SA all
   allow it. See also `CLAUDE.md` constraint 3.
3. **No NonCommercial, ever.** CC-BY-NC contradicts the MIT grant this project
   makes: MIT says commercial use is permitted and NC says it is not, so a
   downstream user cannot obey both. Same for any "personal use only" term.
4. **ShareAlike is allowed but must be understood.** CC-BY-SA does not reach the
   code — a program that loads a mesh is not a derivative work of the mesh — but
   a *modified* SA asset must stay SA. It cannot be relicensed MIT.
5. **Record modifications.** CC-BY and CC-BY-SA both require saying that a work
   was changed. Retexturing, decimating and rescaling all count.
6. **Keep original filenames where they carry attribution**, as the Freesound
   `id__username__title` convention does — a file that gets copied elsewhere then
   still names its author.

7. **Models are for SCENERY, never for the ride.** Track, train, supports,
   catwalks, devices and the station are generated (see `CLAUDE.md` § the train
   was designed first, and `Docs/TRAIN_DESIGN.md`): a contributor can read a diff
   of them and nothing about the simulation depends on a binary. A tree, a
   rock or a bush is set dressing the physics never reads, and the audio rows
   above are the precedent for committing it. This rule is what stops the
   precedent being read as permission to import a coaster.

## Audio

| File | Title | Author | Source | Licence |
|---|---|---|---|---|
| `Content/Audio/Brakes/416080__davidlay1__air-release.wav` | Air release | davidlay1 | [Freesound 416080](https://freesound.org/s/416080/) | CC0 |
| `Content/Audio/Brakes/131934__mcpable__slips-air-release-v1.wav` | Slips air release v1 | mcpable | [Freesound 131934](https://freesound.org/s/131934/) | CC0 |
| `Content/Audio/Brakes/131933__mcpable__slips-air-release-v2.wav` | Slips air release v2 | mcpable | [Freesound 131933](https://freesound.org/s/131933/) | CC0 |
| `Content/Audio/Brakes/131932__mcpable__slips-air-release-v3.wav` | Slips air release v3 | mcpable | [Freesound 131932](https://freesound.org/s/131932/) | CC0 |
| `Content/Audio/Brakes/131931__mcpable__slips-air-release-v4.wav` | Slips air release v4 | mcpable | [Freesound 131931](https://freesound.org/s/131931/) | CC0 |

CC0 is a public-domain dedication and asks for nothing. The credit is kept
anyway: it costs five rows, and a project that credits people only when forced
to is not one anybody should want to contribute to.

## Meshes, textures and materials

One row per DOWNLOAD rather than per file where a pack is one archive from one
page under one licence: the path is the folder every file from that archive
was imported into, and the original filenames are kept inside it. ONLY WHAT
IS WIRED IS IMPORTED: a texture in the repository that nothing uses is 15 MB
on every clone for nobody, so a pack goes in when a material reads it. A
further sixteen CC0 packs (Poly Haven, ambientCG, OpenGameArt) were selected
and licence-checked on 2026-08-22 for the landscape pass and are held
outside the repository until then.

**Import notes that are the same for all of them.** Only colour, normal,
roughness/ARM and AO maps are imported; displacement, `.blend`, `.usdc`, `.mtlx`,
`.tres` and preview images are not. Poly Haven normals are OpenGL (flip green
on import), ambientCG ships both and the DX one is used. Poly Haven's `arm`
texture is AO / roughness / metallic in R / G / B. Nobiax packs carry specular
maps rather than roughness; where used, roughness is derived as one minus spec.

| File | Title | Author | Source | Licence | Modified |
|---|---|---|---|---|---|
| `Content/Env/Textures/PlasteredWall04/` | Plastered Wall 04 | Poly Haven | [polyhaven.com/a/plastered_wall_04](https://polyhaven.com/a/plastered_wall_04) | CC0 | no |
| `Content/Env/Textures/PaintedPlasterWall/` | Painted Plaster Wall | Poly Haven | [polyhaven.com/a/painted_plaster_wall](https://polyhaven.com/a/painted_plaster_wall) | CC0 | no |
| `Content/Env/Textures/RubberTiles/` | Rubber Tiles | Poly Haven | [polyhaven.com/a/rubber_tiles](https://polyhaven.com/a/rubber_tiles) | CC0 | no |
| `Content/Env/Textures/WhitePaintedSteel/` | White painted steel | generated for this project by the developer | — | MIT (original) | — |


|---|---|---|---|---|---|
| — | — | — | — | — | — |

## Fonts, icons and UI

*Nothing yet.* The interface draws on the engine's debug canvas and uses no
imported typeface.

## Not in this repository, and deliberately

- **Train and Rail System (Polygon Jelly)** — a paid marketplace asset used as
  local reference while prototyping. Never redistributed, and no code or content
  from it is here. `CLAUDE.md` constraint 3.
- **Unreal Engine source** — `CLAUDE.md` constraint 4. Contributors get the
  engine from Epic themselves.
- **NoLimits 2 park files and exports of real rides** —
  `Prototypes/NL2Csv/Tracks/` is gitignored for this reason. The CSV reader is a
  validation path, not a library of other people's work.
