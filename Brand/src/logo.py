#!/usr/bin/env python3
"""TrackUnlimited brand marks — generated as outlines, not live text.

Wordmark letterforms are converted to SVG paths from the font outlines so the
mark renders identically everywhere, with no webfont dependency. The curvature
mark is drawn from the same clothoid -> arc -> clothoid curvature profile the
track model actually uses.
"""
import json, os
from fontTools.ttLib import TTFont
from fontTools.pens.svgPathPen import SVGPathPen
from fontTools.pens.transformPen import TransformPen

HEROS_R = "/usr/share/texmf/fonts/opentype/public/tex-gyre/texgyreheroscn-regular.otf"
HEROS_B = "/usr/share/texmf/fonts/opentype/public/tex-gyre/texgyreheroscn-bold.otf"

_cache = {}


def _font(path):
    if path not in _cache:
        f = TTFont(path)
        _cache[path] = (f, f.getGlyphSet(), f["head"].unitsPerEm, f.getBestCmap())
    return _cache[path]


def text_path(s, font_path, size, x=0.0, tracking=0.0):
    """Outline `s` as an SVG path `d`, baseline at y=0, y-down. Returns (d, width)."""
    font, gs, upem, cmap = _font(font_path)
    scale = size / upem
    pen_out = []
    cursor = x
    for ch in s:
        name = cmap.get(ord(ch))
        if name is None:
            cursor += size * 0.5 + tracking
            continue
        spen = SVGPathPen(gs, ntos=lambda v: f"{v:.2f}")
        # y-flip into SVG space, scaled to `size`
        tpen = TransformPen(spen, (scale, 0, 0, -scale, cursor, 0))
        gs[name].draw(tpen)
        d = spen.getCommands()
        if d:
            pen_out.append(d)
        cursor += gs[name].width * scale + tracking
    return " ".join(pen_out), (cursor - x - tracking if s else 0.0)


# ---------------------------------------------------------------- curvature mark
def curvature_mark(size=100.0):
    """The project's own signature: kappa(s) for straight -> clothoid -> arc ->
    clothoid -> straight. Flat, linear ramp, plateau, linear ramp, flat — the
    shape a banked turn has in the one representation TrackUnlimited stores.

    Returned in a `size` x `size` box as geometry only (no stroke attrs), so the
    caller controls colour and weight.
    """
    u = size / 100.0
    x0, x1 = 10 * u, 90 * u
    base = 70 * u              # kappa = 0 axis
    top = 30 * u               # kappa = peak (constant-radius arc)
    w = x1 - x0
    # breakpoints as fractions of the span: two clothoids around one arc
    bp = [0.00, 0.12, 0.34, 0.66, 0.88, 1.00]
    ys = [base, base, top, top, base, base]
    pts = [(x0 + b * w, y) for b, y in zip(bp, ys)]
    trace = "M " + " L ".join(f"{x:.2f} {y:.2f}" for x, y in pts)
    axis = f"M {x0:.2f} {base:.2f} L {x1:.2f} {base:.2f}"
    # arc-length ticks below the axis
    ticks = []
    for i in range(9):
        tx = x0 + w * i / 8.0
        ticks.append(f"M {tx:.2f} {base:.2f} L {tx:.2f} {base + 5 * u:.2f}")
    # corner registration marks
    c = 10 * u
    m = 4 * u
    corners = []
    for cx, cy, sx, sy in ((0, 0, 1, 1), (size, 0, -1, 1), (0, size, 1, -1), (size, size, -1, -1)):
        corners.append(f"M {cx + sx * m:.2f} {cy + sy * (m + c):.2f} "
                       f"L {cx + sx * m:.2f} {cy + sy * m:.2f} "
                       f"L {cx + sx * (m + c):.2f} {cy + sy * m:.2f}")
    return {"trace": trace, "axis": axis, "ticks": " ".join(ticks),
            "corners": " ".join(corners), "peakY": top, "baseY": base,
            "x0": x0, "x1": x1}


# ---------------------------------------------------------------- lockups
def wordmark(size=64.0, tracking_ratio=0.055):
    tr = size * tracking_ratio
    d1, w1 = text_path("TRACK", HEROS_R, size, 0.0, tr)
    gap = size * 0.10
    d2, w2 = text_path("UNLIMITED", HEROS_B, size, w1 + tr + gap, tr)
    total = w1 + tr + gap + w2
    return {"light": d1, "bold": d2, "width": total, "cap": size * 0.72}


def svg_wordmark(size=64.0, fg="#E8F2F8", accent="#7FD8FF", rule=True, bg=None):
    wm = wordmark(size)
    cap = wm["cap"]
    pad = size * 0.28
    ruleY = pad + cap + size * 0.30
    h = ruleY + size * 0.34 + pad * 0.4
    w = wm["width"] + pad * 2
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w:.1f} {h:.1f}" '
             f'width="{w:.0f}" height="{h:.0f}" role="img" aria-label="TrackUnlimited">']
    if bg:
        parts.append(f'<rect width="{w:.1f}" height="{h:.1f}" fill="{bg}"/>')
    g = f'<g transform="translate({pad:.1f},{pad + cap:.1f})">'
    parts.append(g)
    parts.append(f'<path d="{wm["light"]}" fill="{fg}"/>')
    parts.append(f'<path d="{wm["bold"]}" fill="{fg}"/>')
    parts.append("</g>")
    if rule:
        # a drafting dimension line under the wordmark
        y = ruleY
        ex = size * 0.16
        parts.append(
            f'<g stroke="{accent}" stroke-width="{max(1.0, size*0.028):.2f}" fill="none" '
            f'stroke-linecap="square">'
            f'<path d="M {pad:.1f} {y:.1f} L {pad + wm["width"]:.1f} {y:.1f}"/>'
            f'<path d="M {pad:.1f} {y - ex:.1f} L {pad:.1f} {y + ex:.1f}"/>'
            f'<path d="M {pad + wm["width"]:.1f} {y - ex:.1f} L {pad + wm["width"]:.1f} {y + ex:.1f}"/>'
            f'</g>')
    parts.append("</svg>")
    return "\n".join(parts)


def svg_mark(size=128.0, fg="#7FD8FF", axis="#3E7E9B", corner="#FFB020", bg=None,
             radius=0.0, simple=False, scale=1.0):
    """`simple=True` drops the ticks and corner marks and fattens the trace —
    the form that survives a 16 px favicon. `scale` grows the geometry about the
    box centre, for badge and favicon lockups that want the mark to fill."""
    m = curvature_mark(size)
    p = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {size:.0f} {size:.0f}" '
         f'width="{size:.0f}" height="{size:.0f}" role="img" aria-label="TrackUnlimited mark">']
    if bg:
        p.append(f'<rect width="{size:.0f}" height="{size:.0f}" rx="{radius:.0f}" fill="{bg}"/>')
    t = ""
    if scale != 1.0:
        o = size * (1 - scale) / 2.0
        t = f' transform="translate({o:.2f},{o:.2f}) scale({scale:.3f})"'
    p.append(f'<g fill="none" stroke-linecap="square"{t}>')
    if simple:
        p.append(f'<path d="{m["axis"]}" stroke="{axis}" stroke-width="{size*0.05:.2f}"/>')
        p.append(f'<path d="{m["trace"]}" stroke="{fg}" stroke-width="{size*0.10:.2f}" '
                 f'stroke-linejoin="round"/>')
    else:
        p.append(f'<path d="{m["axis"]}" stroke="{axis}" stroke-width="{size*0.016:.2f}"/>')
        p.append(f'<path d="{m["ticks"]}" stroke="{axis}" stroke-width="{size*0.014:.2f}"/>')
        p.append(f'<path d="{m["corners"]}" stroke="{corner}" stroke-width="{size*0.018:.2f}"/>')
        p.append(f'<path d="{m["trace"]}" stroke="{fg}" stroke-width="{size*0.055:.2f}" '
                 f'stroke-linejoin="round"/>')
    p.append("</g></svg>")
    return "\n".join(p)


def svg_lockup(size=56.0, fg="#E8F2F8", accent="#7FD8FF", bg=None):
    """Mark + wordmark on one line — the primary horizontal lockup."""
    ms = size * 1.5
    wm = wordmark(size)
    pad = size * 0.32
    gap = size * 0.42
    w = pad * 2 + ms + gap + wm["width"]
    h = pad * 2 + ms
    p = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w:.1f} {h:.1f}" '
         f'width="{w:.0f}" height="{h:.0f}" role="img" aria-label="TrackUnlimited">']
    if bg:
        p.append(f'<rect width="{w:.1f}" height="{h:.1f}" fill="{bg}"/>')
    m = curvature_mark(ms)
    p.append(f'<g transform="translate({pad:.1f},{pad:.1f})" fill="none" stroke-linecap="square">')
    p.append(f'<path d="{m["axis"]}" stroke="#3E7E9B" stroke-width="{ms*0.018:.2f}"/>')
    p.append(f'<path d="{m["ticks"]}" stroke="#3E7E9B" stroke-width="{ms*0.016:.2f}"/>')
    p.append(f'<path d="{m["corners"]}" stroke="#FFB020" stroke-width="{ms*0.022:.2f}"/>')
    p.append(f'<path d="{m["trace"]}" stroke="{accent}" stroke-width="{ms*0.055:.2f}" stroke-linejoin="round"/>')
    p.append("</g>")
    tx = pad + ms + gap
    ty = pad + ms * 0.5 + wm["cap"] * 0.5
    p.append(f'<g transform="translate({tx:.1f},{ty:.1f})">')
    p.append(f'<path d="{wm["light"]}" fill="{fg}"/><path d="{wm["bold"]}" fill="{fg}"/>')
    p.append("</g></svg>")
    return "\n".join(p)


if __name__ == "__main__":
    # Ship layout puts brand/ next to src/; dev layout keeps a dist/. Match build.py.
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(root, "dist", "brand") if os.path.isdir(os.path.join(root, "dist")) \
        else os.path.join(root, "brand")
    os.makedirs(out, exist_ok=True)
    files = {
        "wordmark.svg": svg_wordmark(64),
        "wordmark-mono-light.svg": svg_wordmark(64, fg="#E8F2F8", accent="#E8F2F8"),
        "wordmark-mono-dark.svg": svg_wordmark(64, fg="#0B0E11", accent="#0B0E11"),
        "mark.svg": svg_mark(128),
        "mark-simple.svg": svg_mark(128, simple=True),
        "mark-badge.svg": svg_mark(512, bg="#0B0E11", radius=96, scale=1.06),
        "avatar-512.svg": svg_mark(512, bg="#0B0E11", radius=0, scale=1.06),
        "lockup.svg": svg_lockup(56),
        "favicon.svg": svg_mark(64, bg="#0B0E11", radius=10, simple=True, scale=1.45),
    }
    for name, body in files.items():
        with open(os.path.join(out, name), "w") as f:
            f.write(body + "\n")
    # path data for inlining into the HTML deliverables
    print(json.dumps({
        "wordmark": wordmark(64),
        "mark100": curvature_mark(100.0),
    }))
