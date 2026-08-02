#!/usr/bin/env python3
"""Inline the shared sources into each self-contained deliverable.

Every output is one file with no external requests: tokens, track data and
brand geometry are substituted into the templates at build time.
"""
import json, os, re, shutil, subprocess, sys

ROOT = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(ROOT, "src")
TPL = os.path.join(ROOT, "templates")
DIST = os.path.join(ROOT, "dist")

subprocess.run([sys.executable, os.path.join(SRC, "logo.py")], check=True,
               stdout=open(os.path.join(SRC, "logo.json"), "w"))

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
    print("wrote dist/%s  (%.1f KB)" % (out, len(s) / 1024))
