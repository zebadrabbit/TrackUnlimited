#!/usr/bin/env python3
"""Inline the shared sources into each self-contained deliverable.

Every output is one file with no external requests: tokens, track data and
brand geometry are substituted into the templates at build time.
"""
import json, os, re, shutil, subprocess, sys

ROOT = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(ROOT, "src")
TPL = os.path.join(ROOT, "templates")
# Ship layout: templates/, src/ and the built pages all sit in Brand/, so write
# next to this file. Dev layout keeps a dist/ — use it when it exists.
DIST = os.environ.get("TU_DIST") or (
    os.path.join(ROOT, "dist") if os.path.isdir(os.path.join(ROOT, "dist")) else ROOT)

LOGO_JSON = os.path.join(SRC, "logo.json")
# logo.py converts the wordmark to outlines and needs fonttools plus the TeX Gyre
# Heros Cn font files. If either is missing, fall back to the last generated
# logo.json rather than failing the whole build — the geometry does not change
# unless the mark itself does.
try:
    out = subprocess.run([sys.executable, os.path.join(SRC, "logo.py")],
                         check=True, capture_output=True, text=True).stdout
    with open(LOGO_JSON, "w") as f:
        f.write(out)
except Exception as e:
    if not os.path.exists(LOGO_JSON):
        raise SystemExit("logo.py failed and there is no src/logo.json to fall back on: %s" % e)
    print("logo.py failed (%s) — reusing the existing src/logo.json" % type(e).__name__)

tokens = open(os.path.join(SRC, "tokens.css")).read()
data = open(os.path.join(SRC, "data.js")).read()
data = re.sub(r"^if \(typeof module.*$", "", data, flags=re.M)
logo = open(os.path.join(SRC, "logo.json")).read().strip()

OUT = {
    "loading.html": "loading-screen.html",
    "site.html": "index.html",
    "social.html": "social-pack.html",
    "brand.html": "brand-system.html",
}

os.makedirs(DIST, exist_ok=True)
for tpl, out in OUT.items():
    p = os.path.join(TPL, tpl)
    if not os.path.exists(p):
        print("skip (no template):", tpl)
        continue
    s = open(p).read()
    s = s.replace("/*__TOKENS__*/", tokens)
    s = s.replace("/*__DATA__*/", data)
    s = s.replace("/*__LOGO__*/", "const LOGO = " + logo + ";")
    with open(os.path.join(DIST, out), "w") as f:
        f.write(s)
    print("wrote %s  (%.1f KB)" % (os.path.relpath(os.path.join(DIST, out), ROOT),
                                   len(s) / 1024))
