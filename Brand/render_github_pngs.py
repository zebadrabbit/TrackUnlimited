#!/usr/bin/env python3
"""Render github/*.png from social-pack.html with a headless browser.

The five README/Docs figures are the one part of the brand chain that was not
reproducible: gen_data.cpp measures the layout, build.py inlines it into the
pages, and then somebody had to open social-pack.html and press PNG @1x by hand.
That is how a diagram captioned "MEASURED, NOT DRAWN" ended up published with a
loop-apex label that no longer matched the data driving the same page.

    python3 render_github_pngs.py                      # all five
    python3 render_github_pngs.py gh-layout-1280x560   # just one

Requires Edge or Chrome. It drives neither via CDP nor Playwright: it writes a
copy of the page with a small script appended that throws away everything except
the one card's SVG, then screenshots the viewport at that card's exact size.

Rendering is browser- and machine-dependent. Glyph rasterisation differs a little
between engines and font sets, so re-render the WHOLE set on one machine rather
than a single card, or that card will not quite match its siblings.
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(ROOT, "social-pack.html")
OUT = os.path.join(ROOT, "github")

BROWSERS = [
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
    "/usr/bin/microsoft-edge",
    "/usr/bin/google-chrome",
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
]

# Appended to a copy of the page. CARDS is a global in card order and the
# sections are emitted in that order, so the index lines them up — the sections
# themselves carry no id to select on.
INJECT = """
<style>html,body{margin:0!important;padding:0!important;overflow:hidden!important}</style>
<script>
addEventListener('load', function () {
  var i = CARDS.findIndex(function (c) { return c.id === %(id)r; });
  if (i < 0) { document.title = 'NOCARD'; return; }
  var svg = document.querySelectorAll('section.item')[i].querySelector('svg');
  if (!svg) { document.title = 'NOSVG'; return; }
  document.body.replaceChildren(svg);
  svg.style.display = 'block';
  document.title = 'ready';
});
</script>
"""


def find_browser():
    for p in BROWSERS:
        if os.path.isfile(p):
            return p
    for n in ("msedge", "google-chrome", "chromium"):
        p = shutil.which(n)
        if p:
            return p
    sys.exit("no Edge or Chrome found — install one, or export the PNGs by hand "
             "from social-pack.html (see Brand/README.md)")


def card_ids():
    """The gh-* cards and their sizes, read from the page rather than hardcoded."""
    src = open(PAGE, encoding="utf-8").read()
    found = re.findall(r"card\('(gh-[\w-]+)'.*?(\d{3,4}), *(\d{3,4}),", src, re.S)
    seen, out = set(), []
    for cid, w, h in found:
        if cid not in seen:
            seen.add(cid)
            out.append((cid, int(w), int(h)))
    return out


def png_size(path):
    import struct
    with open(path, "rb") as f:
        return struct.unpack(">II", f.read(24)[16:24])


def render(browser, cid, w, h):
    src = open(PAGE, encoding="utf-8").read()
    body = src.rfind("</body>")
    patched = src[:body] + (INJECT % {"id": cid}) + src[body:] if body > 0 else src + (INJECT % {"id": cid})

    # Written beside the page, not in a temp dir: the page is self-contained, but
    # keeping it here means a relative asset would still resolve if one is ever added.
    fd, tmp = tempfile.mkstemp(suffix=".html", dir=ROOT, prefix=".render-")
    os.close(fd)
    dest = os.path.join(OUT, cid[len("gh-"):] + ".png")
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(patched)
        if os.path.exists(dest):
            os.remove(dest)
        subprocess.run([
            browser, "--headless=new", "--disable-gpu", "--hide-scrollbars",
            "--force-device-scale-factor=1", "--font-render-hinting=none",
            "--virtual-time-budget=5000", "--window-size=%d,%d" % (w, h),
            "--screenshot=" + dest, "file:///" + tmp.replace("\\", "/"),
        ], check=True, capture_output=True)
    finally:
        os.remove(tmp)

    if not os.path.exists(dest):
        sys.exit("%s: browser wrote no file" % cid)
    got = png_size(dest)
    if got != (w, h):
        # A silently mis-sized figure is worse than none: it would be committed
        # and only noticed as a blurry README image.
        sys.exit("%s: expected %dx%d, got %dx%d" % (cid, w, h, got[0], got[1]))
    print("  %-34s %4dx%-4d  %d KB" % (os.path.basename(dest), w, h,
                                       os.path.getsize(dest) // 1024))


def main():
    browser = find_browser()
    cards = card_ids()
    wanted = sys.argv[1:]
    if wanted:
        known = {c[0] for c in cards}
        bad = [w for w in wanted if w not in known]
        if bad:
            sys.exit("unknown card(s): %s\nknown: %s" % (", ".join(bad),
                                                         ", ".join(sorted(known))))
        cards = [c for c in cards if c[0] in wanted]
        print("rendering %d of %d cards — the rest keep whatever renderer made "
              "them, see the note in this script" % (len(cards), len(known)))
    print("using %s" % browser)
    os.makedirs(OUT, exist_ok=True)
    for cid, w, h in cards:
        render(browser, cid, w, h)


if __name__ == "__main__":
    main()
