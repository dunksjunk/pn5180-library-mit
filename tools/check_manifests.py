# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Rich Dunkel
"""Check that library.properties (Arduino) and library.json (PlatformIO) agree.

Compared: name, version, repository URL, license, main header, and that every
example folder is listed in library.json with its own base and existing
files. Exit code 0 = consistent, 1 = not.
Lives in tools/; run from anywhere: python tools/check_manifests.py
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read_properties(path):
    props = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        props[key.strip()] = value.strip()
    return props


def norm_url(url):
    url = url.strip().rstrip("/")
    return url[:-4] if url.endswith(".git") else url


def main():
    props = read_properties(ROOT / "library.properties")
    manifest = json.loads((ROOT / "library.json").read_text(encoding="utf-8"))
    errors = []

    def same(what, a, b):
        if a != b:
            errors.append(f"{what}: library.properties={a!r} library.json={b!r}")

    same("name", props.get("name"), manifest.get("name"))
    same("version", props.get("version"), manifest.get("version"))
    repo = manifest.get("repository", {})
    same("url", norm_url(props.get("url", "")),
         norm_url(repo.get("url", "") if isinstance(repo, dict) else ""))
    same("license", props.get("license"), manifest.get("license"))
    same("header", props.get("includes"), manifest.get("headers"))

    on_disk = sorted(p.name for p in (ROOT / "examples").iterdir() if p.is_dir())
    listed = sorted(e.get("name") for e in manifest.get("examples", []))
    same("examples", on_disk, listed)
    # Each listed example must point at its own folder and existing sketch files.
    for e in manifest.get("examples", []):
        name, base, files = e.get("name"), e.get("base"), e.get("files") or []
        if base != f"examples/{name}":
            errors.append(f"example {name}: base {base!r} != 'examples/{name}'")
        if not files:
            errors.append(f"example {name}: no files listed")
        for f in files:
            if not (ROOT / str(base) / f).is_file():
                errors.append(f"example {name}: missing file {base}/{f}")

    if errors:
        for e in errors:
            print("check_manifests: " + e)
        return 1
    print(f"check_manifests: OK ({props.get('name')} {props.get('version')})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
