#!/usr/bin/env python3
"""
Run wrapper for fusionml C++ unit tests.

Builds and runs each test binary. Exit 0 if all pass, 1 otherwise.
Intended for macOS/Linux. Requires clang++ or g++ and zlib development headers.

Usage:
  From repo root:    python fusionml/tests/run_fusion_tests.py
  From fusionml:     python tests/run_fusion_tests.py
  From fusionml/tests: python run_fusion_tests.py

Options:
  --build-only   Build all tests but do not run them.
  --list         List test names and exit.
  -v, --verbose  Print compiler and runner commands.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

# -----------------------------------------------------------------------------
# Paths
# -----------------------------------------------------------------------------

def _find_repo_root() -> Path:
    """Locate repo root (directory containing package.json)."""
    d = Path(__file__).resolve().parent
    for _ in range(6):
        if (d / "package.json").exists():
            return d
        parent = d.parent
        if parent == d:
            break
        d = parent
    raise RuntimeError("Cannot find repo root (package.json)")

REPO_ROOT = _find_repo_root()
FUSIONML = REPO_ROOT / "fusionml"
SRC = FUSIONML / "src"
TESTS_DIR = FUSIONML / "tests"
BUILD_DIR = TESTS_DIR / ".build"

# -----------------------------------------------------------------------------
# Test definitions: (name, test_cpp, source_cpp_files[], extra_flags[])
# -----------------------------------------------------------------------------

TESTS = [
    (
        "altitude_calculator",
        "test_altitude_calculator.cpp",
        ["AltitudeCalculator.cpp"],
        [],
    ),
    (
        "json_recorder",
        "test_json_recorder.cpp",
        ["JsonRecorder.cpp"],
        ["-lz"],
    ),
    (
        "flight_phase_detector",
        "test_flight_phase_detector.cpp",
        ["FlightPhaseDetector.cpp"],
        [],
    ),
    (
        "xyzgeomag",
        "test_xyzgeomag.cpp",
        [],  # header-only
        [],
    ),
    (
        "unavins",
        "test_unavins.cpp",
        ["uNavINS.cpp", "AltitudeCalculator.cpp"],
        [],
    ),
]

# -----------------------------------------------------------------------------
# Compiler
# -----------------------------------------------------------------------------

def _get_compiler() -> str:
    for name in ("clang++", "g++"):
        path = shutil.which(name)
        if path:
            return path
    raise RuntimeError("No C++ compiler found (tried clang++, g++)")

# -----------------------------------------------------------------------------
# Build & run
# -----------------------------------------------------------------------------

def _build_one(compiler: str, name: str, test_cpp: str, sources: list[str], extra: list[str], verbose: bool) -> Path | None:
    """Build a single test. Return path to binary or None on failure."""
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    binary = BUILD_DIR / name

    test_path = TESTS_DIR / test_cpp
    if not test_path.exists():
        print(f"  skip {name}: missing {test_path}", file=sys.stderr)
        return None

    src_paths = [SRC / f for f in sources]
    for p in src_paths:
        if not p.exists():
            print(f"  skip {name}: missing {p}", file=sys.stderr)
            return None

    inc = f"-I{SRC}"
    cmd = [compiler, "-std=c++14", inc, str(test_path)]
    cmd.extend(str(p) for p in src_paths)
    cmd.extend(extra)
    cmd.extend(["-o", str(binary)])

    if verbose:
        print("  " + " ".join(cmd))

    result = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO_ROOT)
    if result.returncode != 0:
        print(f"  build failed: {name}", file=sys.stderr)
        if result.stderr:
            print(result.stderr, file=sys.stderr)
        return None
    return binary

def _run_one(binary: Path, verbose: bool) -> tuple[bool, str]:
    """Run a test binary. Return (success, combined stdout+stderr)."""
    result = subprocess.run(
        [str(binary)],
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    out = (result.stdout or "") + (result.stderr or "")
    if verbose:
        print(f"  run {binary.name}")
    return result.returncode == 0, out

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Build and run fusionml C++ unit tests.",
        epilog="Run from repo root or from fusionml/tests.",
    )
    ap.add_argument("--build-only", action="store_true", help="Build only, do not run")
    ap.add_argument("--list", action="store_true", help="List test names and exit")
    ap.add_argument("-v", "--verbose", action="store_true", help="Verbose output")
    args = ap.parse_args()

    if args.list:
        for name, *_ in TESTS:
            print(name)
        return 0

    try:
        compiler = _get_compiler()
    except RuntimeError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    if args.verbose:
        print(f"compiler: {compiler}")
        print(f"repo root: {REPO_ROOT}")
        print(f"fusionml: {FUSIONML}")

    built: list[tuple[str, Path]] = []
    for name, test_cpp, sources, extra in TESTS:
        b = _build_one(compiler, name, test_cpp, sources, extra, args.verbose)
        if b is not None:
            built.append((name, b))

    if not built:
        print("No tests built.", file=sys.stderr)
        return 1

    if args.build_only:
        print(f"Built {len(built)} test(s) under {BUILD_DIR}")
        return 0

    failed: list[str] = []
    for name, binary in built:
        ok, out = _run_one(binary, args.verbose)
        if out.strip():
            print(out.strip())
        if not ok:
            failed.append(name)

    if failed:
        print(f"\nfusionml tests: {len(failed)} failed — {', '.join(failed)}")
        return 1
    print(f"\nfusionml tests: all {len(built)} passed")
    return 0

if __name__ == "__main__":
    sys.exit(main())
