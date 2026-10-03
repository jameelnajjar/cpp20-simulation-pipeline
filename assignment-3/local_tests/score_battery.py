#!/usr/bin/env python3
"""Build, run varied HW3 scenarios, print per-run scores and averages."""
from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILD = Path(os.environ.get("BUILD_DIR", ROOT / "build-wsl")).resolve()
IDS = "213309941_213727837"
SIM = BUILD / "Simulator" / f"simulator_{IDS}"
ALGO = BUILD / "Algorithm" / f"Algorithm_{IDS}.so"
MC = BUILD / "MissionControl" / f"MissionControl_{IDS}.so"
WORK = Path(__file__).resolve().parent / "work" / "score_battery"
INPUTS = Path(__file__).resolve().parent / "inputs"

SCORES: list[tuple[str, float, int]] = []  # label, score, steps
ERRORS: list[str] = []


def run(cmd: list[str], timeout: int = 600) -> subprocess.CompletedProcess[str]:
    print("  $", " ".join(str(c) for c in cmd), flush=True)
    return subprocess.run(cmd, text=True, capture_output=True, timeout=timeout)


def newest_dir(parent: Path, prefix: str) -> Path | None:
    dirs = [p for p in parent.glob(prefix + "*") if p.is_dir()]
    return max(dirs, key=lambda p: p.stat().st_mtime) if dirs else None


def parse_a2(path: Path, label: str) -> None:
    text = path.read_text(encoding="utf-8", errors="replace")
    blocks = re.split(r"\n\s*-\s+simulation_map:", text)
    # first chunk is header
    rest = text.split("runs:", 1)[-1]
    scores = [float(x) for x in re.findall(r"mission_score:\s*([-0-9.]+)", rest)]
    steps = [int(x) for x in re.findall(r"steps:\s*(\d+)", rest)]
    for i, score in enumerate(scores):
        st = steps[i] if i < len(steps) else 0
        SCORES.append((f"{label}[{i}]", score, st))
        print(f"    run {i}: score={score:.4f} steps={st}")


def parse_competitive(path: Path) -> None:
    text = path.read_text(encoding="utf-8", errors="replace")
    print("    competitive_report:")
    print("   ", " | ".join(line.strip() for line in text.splitlines() if line.strip())[:400])


def stage_one(name: str, src: Path) -> Path:
    folder = WORK / name
    if folder.exists():
        shutil.rmtree(folder)
    folder.mkdir(parents=True)
    shutil.copy2(src, folder / src.name)
    return folder


def competition(label: str, compose: Path, algos: dict[str, Path], timeout: int = 600) -> None:
    folder = WORK / f"cmp_{label}"
    if folder.exists():
        shutil.rmtree(folder)
    folder.mkdir(parents=True)
    for name, src in algos.items():
        shutil.copy2(src, folder / name)
    proc = run([
        str(SIM), "-competition",
        f"simulation={compose}",
        f"mission_control={MC}",
        f"algorithms_folder={folder}",
        "num_threads=2",
    ], timeout=timeout)
    out = proc.stdout + proc.stderr
    if proc.returncode != 0:
        ERRORS.append(f"{label} exit {proc.returncode}: {out[-400:]}")
        print("    FAIL", out[-400:])
        return
    results = newest_dir(folder, "competition_")
    if results is None:
        ERRORS.append(f"{label} no results folder")
        return
    report = results / "competitive_report.yaml"
    if report.exists():
        parse_competitive(report)
    for yml in sorted(results.glob("simulation_output_*.yaml")):
        parse_a2(yml, f"{label}/{yml.stem}")


def comparative(label: str, compose: Path, mcs: dict[str, Path], algo: Path,
                timeout: int = 300) -> None:
    folder = WORK / f"mc_{label}"
    if folder.exists():
        shutil.rmtree(folder)
    folder.mkdir(parents=True)
    for name, src in mcs.items():
        shutil.copy2(src, folder / name)
    proc = run([
        str(SIM), "-comparative",
        f"simulation={compose}",
        f"mission_control_folder={folder}",
        f"algorithm={algo}",
        "num_threads=2",
    ], timeout=timeout)
    if proc.returncode != 0:
        ERRORS.append(f"{label} exit {proc.returncode}")
        print("    FAIL", (proc.stdout + proc.stderr)[-400:])
        return
    results = newest_dir(folder, "comparative_results_")
    if results is None:
        ERRORS.append(f"{label} no results")
        return
    for yml in sorted(results.glob("simulation_output_*.yaml")):
        parse_a2(yml, f"{label}/{yml.stem}")


def find_so(name: str) -> Path:
    matches = [p for p in BUILD.rglob(name) if "CMakeFiles" not in p.parts]
    if not matches:
        raise FileNotFoundError(name)
    return matches[0]


def main() -> int:
    start = time.time()
    WORK.mkdir(parents=True, exist_ok=True)

    print("== build ==")
    conf = run(["cmake", "-S", str(ROOT), "-B", str(BUILD),
                "-DCMAKE_BUILD_TYPE=Release", "-DHW3_LOCAL_TESTS=ON"], timeout=300)
    if conf.returncode != 0:
        print(conf.stdout, conf.stderr)
        return 1
    build = run(["cmake", "--build", str(BUILD), "-j"], timeout=600)
    if build.returncode != 0:
        print(build.stdout, build.stderr)
        return 1

    maps = run([sys.executable, str(INPUTS / "map" / "generate_maps.py")])
    if maps.returncode != 0:
        print(maps.stderr)
        return 1

    official = {f"Algorithm_{IDS}.so": ALGO}
    extras = {
        "FinishNowAlgo.so": find_so("FinishNowAlgo.so"),
        "HoverScanAlgo.so": find_so("HoverScanAlgo.so"),
        "HoverMoveAlgo.so": find_so("HoverMoveAlgo.so"),
        "NeverFinishAlgo.so": find_so("NeverFinishAlgo.so"),
        "RotateOnlyAlgo.so": find_so("RotateOnlyAlgo.so"),
        "ElevateFarAlgo.so": find_so("ElevateFarAlgo.so"),
    }
    twin = find_so("TwinMissionControl.so")

    print("\n== tiny open (all extra algos + official) ==")
    competition("tiny_all", INPUTS / "compose_tiny.yaml", {**official, **extras}, timeout=240)

    print("\n== tiny walled (official + wall) ==")
    competition("tiny_wall", INPUTS / "compose_walled.yaml", {
        **official, "WallAlgo.so": find_so("WallAlgo.so"),
    }, timeout=180)

    print("\n== tiny cartesian FinishNow ==")
    competition("tiny_cart", INPUTS / "compose_cartesian.yaml", {
        "FinishNowAlgo.so": extras["FinishNowAlgo.so"],
    }, timeout=120)

    print("\n== tiny comparative official MC + Twin ==")
    comparative("tiny_twins", INPUTS / "compose_tiny.yaml", {
        f"MissionControl_{IDS}.so": MC,
        "TwinMissionControl.so": twin,
    }, extras["FinishNowAlgo.so"], timeout=120)

    print("\n== SMALL room (official, 2 lidars) ==")
    competition("small_room", INPUTS / "compose_size_small.yaml", official, timeout=180)

    print("\n== SMALL outdoor (official) ==")
    competition("small_out", INPUTS / "compose_size_small_out.yaml", official, timeout=300)

    print("\n== MEDIUM large-room (official) ==")
    competition("medium_room", INPUTS / "compose_size_medium.yaml", official, timeout=300)

    print("\n== LARGE house-lower (official) ==")
    competition("large_house", INPUTS / "compose_size_large.yaml", official, timeout=600)

    print("\n== SMALL room extra algos (FinishNow + HoverScan) ==")
    competition("small_extras", INPUTS / "compose_size_small.yaml", {
        "FinishNowAlgo.so": extras["FinishNowAlgo.so"],
        "HoverScanAlgo.so": extras["HoverScanAlgo.so"],
    }, timeout=180)

    valid = [s for s in SCORES if s[1] >= 0]
    err_scores = [s for s in SCORES if s[1] < 0]
    print("\n========== SCORE SUMMARY ==========")
    for label, score, steps in SCORES:
        tag = "OK " if score >= 0 else "ERR"
        print(f"  {tag}  {score:8.4f}  steps={steps:5d}  {label}")

    def avg(rows: list[tuple[str, float, int]]) -> float:
        return sum(r[1] for r in rows) / len(rows) if rows else float("nan")

    print()
    print(f"Runs with a numeric score:     {len(SCORES)}")
    print(f"Successful runs (score >= 0):  {len(valid)}")
    print(f"Error runs (score < 0):        {len(err_scores)}")
    if valid:
        print(f"Average score (success only):  {avg(valid):.4f}")
        print(f"Min / max success:             {min(r[1] for r in valid):.4f} / {max(r[1] for r in valid):.4f}")
        print(f"Average steps (success only):  {sum(r[2] for r in valid)/len(valid):.1f}")
    if SCORES:
        print(f"Average score (all runs, errors as -1): {avg(SCORES):.4f}")
    official_rows = [s for s in valid if "Algorithm_" in s[0]]
    if official_rows:
        print(f"Official algorithm success runs: {len(official_rows)}")
        print(f"Official algorithm average:      {avg(official_rows):.4f}")
    print(f"Elapsed: {time.time()-start:.1f}s")
    if ERRORS:
        print("Runner errors:")
        for e in ERRORS:
            print(" -", e)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
