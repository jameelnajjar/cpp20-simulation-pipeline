#!/usr/bin/env python3
"""Local HW3 scenario battery. Not staff tests — exercises our Simulator/MC/Algorithm
against extra plugins and tiny maps until official .so files arrive.

Run from WSL:
  python3 local_tests/run_battery.py
  BUILD_DIR=build-wsl python3 local_tests/run_battery.py
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILD = Path(os.environ.get("BUILD_DIR", ROOT / "build-wsl")).resolve()
IDS = "213309941_213727837"
SIM_NAME = f"simulator_{IDS}"
ALGO_NAME = f"Algorithm_{IDS}.so"
MC_NAME = f"MissionControl_{IDS}.so"
WORK = Path(__file__).resolve().parent / "work"
INPUTS = Path(__file__).resolve().parent / "inputs"

PASS = 0
FAIL = 0
FAILURES: list[str] = []


def log(msg: str) -> None:
    print(msg, flush=True)


def record(ok: bool, name: str, detail: str = "") -> None:
    global PASS, FAIL
    if ok:
        PASS += 1
        log(f"  PASS  {name}")
    else:
        FAIL += 1
        extra = f" — {detail}" if detail else ""
        FAILURES.append(f"{name}{extra}")
        log(f"  FAIL  {name}{extra}")


def run(cmd: list[str], cwd: Path | None = None, timeout: int = 180) -> subprocess.CompletedProcess[str]:
    log("    $ " + " ".join(cmd))
    return subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        text=True,
        capture_output=True,
        timeout=timeout,
    )


def usage_ok(proc: subprocess.CompletedProcess[str]) -> bool:
    text = (proc.stdout or "") + (proc.stderr or "")
    return proc.returncode == 1 and "Usage:" in text


def newest_dir(parent: Path, prefix: str) -> Path | None:
    dirs = [p for p in parent.glob(prefix + "*") if p.is_dir()]
    if not dirs:
        return None
    return max(dirs, key=lambda p: p.stat().st_mtime)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""


def stage(dst: Path, files: dict[str, Path]) -> Path:
    if dst.exists():
        shutil.rmtree(dst)
    dst.mkdir(parents=True)
    for name, src in files.items():
        if not src.exists():
            raise FileNotFoundError(src)
        shutil.copy2(src, dst / name)
    return dst


def find_so(name: str) -> Path:
    matches = list(BUILD.rglob(name))
    # Prefer the local_tests / Algorithm / MissionControl outputs, skip CMakeFiles.
    matches = [p for p in matches if "CMakeFiles" not in p.parts]
    if not matches:
        raise FileNotFoundError(f"{name} under {BUILD}")
    return matches[0]


def find_sim() -> Path:
    matches = [p for p in BUILD.rglob(SIM_NAME) if p.is_file() and os.access(p, os.X_OK)]
    matches = [p for p in matches if "CMakeFiles" not in p.parts]
    if not matches:
        raise FileNotFoundError(SIM_NAME)
    return matches[0]


def cmake_build() -> None:
    log("== configure + build ==")
    conf = run(
        [
            "cmake",
            "-S",
            str(ROOT),
            "-B",
            str(BUILD),
            "-DCMAKE_BUILD_TYPE=Release",
            "-DHW3_LOCAL_TESTS=ON",
        ],
        timeout=300,
    )
    if conf.returncode != 0:
        log(conf.stdout)
        log(conf.stderr)
        raise SystemExit("cmake configure failed")
    build = run(["cmake", "--build", str(BUILD), "-j"], timeout=600)
    if build.returncode != 0:
        log(build.stdout)
        log(build.stderr)
        raise SystemExit("cmake build failed")
    record(True, "cmake build")


def generate_maps() -> None:
    script = INPUTS / "map" / "generate_maps.py"
    proc = run([sys.executable, str(script)])
    record(proc.returncode == 0 and (INPUTS / "map" / "open.npy").exists(), "generate maps",
           proc.stderr)


def test_cli(sim: Path) -> None:
    log("== CLI usage ==")
    empty = WORK / "empty_dir"
    empty.mkdir(parents=True, exist_ok=True)
    compose = INPUTS / "compose_tiny.yaml"
    algo = find_so(ALGO_NAME)
    mc = find_so(MC_NAME)

    cases = [
        ([str(sim)], "no args"),
        ([str(sim), "-comparative", "-competition", f"simulation={compose}"], "both modes"),
        ([str(sim), "-comparative", f"simulation={compose}",
          f"mission_control_folder={empty}", f"algorithm={algo}"], "empty mc folder"),
        ([str(sim), "-competition", f"simulation={compose}",
          f"mission_control={mc}", f"algorithms_folder={empty}"], "empty algo folder"),
        ([str(sim), "-comparative", "simulation=/no/such.yaml",
          f"mission_control_folder={empty}", f"algorithm={algo}"], "missing composition"),
        ([str(sim), "-comparative", f"simulation={compose}",
          f"mission_control_folder={empty}", f"algorithm={algo}", "foo=bar"], "unsupported key"),
        ([str(sim), "-bogus"], "unsupported flag"),
    ]
    for cmd, name in cases:
        proc = run(cmd)
        record(usage_ok(proc), f"CLI {name}",
               f"exit={proc.returncode} stderr={proc.stderr[-300:]}")


def test_arg_order(sim: Path, algo: Path, mc_dir: Path, compose: Path) -> None:
    log("== argument order ==")
    proc = run([
        str(sim),
        f"algorithm={algo}",
        f"num_threads=1",
        "-comparative",
        f"mission_control_folder={mc_dir}",
        f"simulation={compose}",
    ])
    record(proc.returncode == 0 and "Simulation finished" in (proc.stdout + proc.stderr),
           "CLI any-order comparative", proc.stderr[-400:])


def test_comparative_twins(sim: Path, algo_finish: Path, official_mc: Path, twin_mc: Path,
                           abort_mc: Path, silent: Path, throwing: Path, compose: Path) -> None:
    log("== comparative (twins + failing MCs) ==")
    folder = stage(WORK / "mc_cmp", {
        MC_NAME: official_mc,
        "TwinMissionControl.so": twin_mc,
        "AbortMissionControl.so": abort_mc,
        "SilentPlugin.so": silent,
        "ThrowingMissionControl.so": throwing,
    })
    proc = run([
        str(sim), "-comparative",
        f"simulation={compose}",
        f"mission_control_folder={folder}",
        f"algorithm={algo_finish}",
        "num_threads=4",
        "-verbose",
    ], timeout=180)
    out = proc.stdout + proc.stderr
    record(proc.returncode == 0, "comparative twins exit 0", out[-500:])
    results = newest_dir(folder, "comparative_results_")
    record(results is not None, "comparative_results folder")
    if results is None:
        return
    report = read(results / "comparative_report.yaml")
    record("comparative_report" in report, "comparative_report.yaml exists")
    record("MissionControl_" in report and "TwinMissionControl.so" in report,
           "twins appear in comparative report")
    record("same_results" in report, "same_results grouping present")
    record("SilentPlugin.so" in report and "AbortMissionControl.so" in report
           and "ThrowingMissionControl.so" in report,
           "failing MCs listed", report[-800:])
    verbose_files = list(results.glob("verbose_*.txt"))
    record(len(verbose_files) >= 1, "verbose log created", f"count={len(verbose_files)}")
    maps = list(results.glob("output_map_*.npy"))
    record(len(maps) >= 1, "output maps saved", f"count={len(maps)}")


def test_competitive_mix(sim: Path, official_mc: Path, plugins: dict[str, Path],
                         compose: Path, walled: Path) -> None:
    log("== competitive mix ==")
    folder = stage(WORK / "algo_cmp", plugins)
    proc = run([
        str(sim), "-competition",
        f"simulation={compose}",
        f"mission_control={official_mc}",
        f"algorithms_folder={folder}",
        "num_threads=4",
    ], timeout=240)
    out = proc.stdout + proc.stderr
    record(proc.returncode == 0, "competitive mix exit 0", out[-500:])
    results = newest_dir(folder, "competition_")
    record(results is not None, "competition folder")
    if results is None:
        return
    report = read(results / "competitive_report.yaml")
    record("competitive_report" in report, "competitive_report.yaml exists")
    for good in ("FinishNowAlgo.so", "HoverScanAlgo.so", "HoverMoveAlgo.so",
                 "NeverFinishAlgo.so", "RotateOnlyAlgo.so"):
        record(good in report, f"competitive lists {good}")
    for bad in ("NoopAlgo.so", "InvalidAlgo.so", "SilentPlugin.so"):
        record(bad in report, f"competitive mentions {bad}")
    # errors: section should include the crashing/invalid plugins
    errors_block = report.split("errors:")[-1] if "errors:" in report else ""
    record("NoopAlgo.so" in errors_block, "noop listed under errors")
    record("InvalidAlgo.so" in errors_block, "invalid listed under errors")
    record("SilentPlugin.so" in errors_block, "silent listed under errors")
    record("FinishNowAlgo.so" not in errors_block, "finish-now is not an error")

    log("== competitive wall map ==")
    wall_folder = stage(WORK / "algo_wall", {
        "WallAlgo.so": plugins["WallAlgo.so"],
        ALGO_NAME: plugins[ALGO_NAME],
    })
    proc = run([
        str(sim), "-competition",
        f"simulation={walled}",
        f"mission_control={official_mc}",
        f"algorithms_folder={wall_folder}",
        "num_threads=2",
    ], timeout=180)
    record(proc.returncode == 0, "walled competitive exit 0", (proc.stdout + proc.stderr)[-400:])
    wres = newest_dir(wall_folder, "competition_")
    if wres is None:
        record(False, "walled competition folder")
        return
    wreport = read(wres / "competitive_report.yaml")
    wall_logs = "\n".join(read(p) for p in wres.glob("error_log*.txt"))
    wall_logs += "\n".join(read(p) for p in wres.rglob("*.txt"))
    record("WALL_COLLISION" in wall_logs or "WallAlgo.so" in wreport.split("errors:")[-1],
           "wall algo reported as error / WALL_COLLISION")
    official_logs = "\n".join(
        read(p) for p in wres.glob("*") if p.is_file() and ALGO_NAME.replace(".so", "") in p.name
    )
    # Official algorithm must not command a wall hit.
    all_txt = "\n".join(read(p) for p in wres.rglob("*.txt"))
    official_hit = "WALL_COLLISION" in all_txt and ALGO_NAME in all_txt
    # A WALL_COLLISION string from WallAlgo's mission is expected; check official's own log.
    official_error_files = [p for p in wres.rglob("error_log*.txt")]
    official_wall = False
    for p in official_error_files:
        text = read(p)
        if "WALL_COLLISION" in text and "Algorithm_" in p.name:
            official_wall = True
    record(not official_wall, "official algorithm did not hit a wall")
    record(len(list(wres.glob("output_map_*.npy"))) >= 1, "maps saved after wall collision")


def test_cartesian(sim: Path, official_mc: Path, finish: Path, compose: Path) -> None:
    log("== cartesian product ==")
    folder = stage(WORK / "algo_cart", {"FinishNowAlgo.so": finish})
    mc_dir = stage(WORK / "mc_cart", {MC_NAME: official_mc})
    proc = run([
        str(sim), "-comparative",
        f"simulation={compose}",
        f"mission_control_folder={mc_dir}",
        f"algorithm={finish}",
        "num_threads=1",
    ], timeout=120)
    record(proc.returncode == 0, "cartesian comparative exit 0", (proc.stdout + proc.stderr)[-400:])
    results = newest_dir(mc_dir, "comparative_results_")
    if results is None:
        record(False, "cartesian results folder")
        return
    yamls = list(results.glob("simulation_output_*.yaml"))
    record(len(yamls) >= 1, "assignment-2 per-plugin yaml")
    body = "\n".join(read(p) for p in yamls)
    # 1 sim × 2 missions × 1 drone × 2 lidars = 4 runs
    run_count = body.count("mission_score:")
    record(run_count == 4, "cartesian 4 runs", f"got {run_count}")


def test_oversize_oob(sim: Path, official_mc: Path, oversize: Path, oob: Path,
                      elevate: Path, compose: Path) -> None:
    log("== oversize / OOB / elevate (must not crash) ==")
    folder = stage(WORK / "algo_bounds", {
        "OversizeAlgo.so": oversize,
        "OobAlgo.so": oob,
        "ElevateFarAlgo.so": elevate,
    })
    proc = run([
        str(sim), "-competition",
        f"simulation={compose}",
        f"mission_control={official_mc}",
        f"algorithms_folder={folder}",
        "num_threads=3",
    ], timeout=180)
    record(proc.returncode == 0, "bounds competitive exit 0", (proc.stdout + proc.stderr)[-400:])
    results = newest_dir(folder, "competition_")
    record(results is not None and (results / "competitive_report.yaml").exists(),
           "bounds report written")


def main() -> int:
    start = time.time()
    WORK.mkdir(parents=True, exist_ok=True)
    generate_maps()
    cmake_build()

    sim = find_sim()
    official_algo = find_so(ALGO_NAME)
    official_mc = find_so(MC_NAME)
    finish = find_so("FinishNowAlgo.so")
    twin = find_so("TwinMissionControl.so")
    abort = find_so("AbortMissionControl.so")
    silent = find_so("SilentPlugin.so")
    throwing = find_so("ThrowingMissionControl.so")

    test_cli(sim)

    mc_one = stage(WORK / "mc_one", {MC_NAME: official_mc})
    test_arg_order(sim, finish, mc_one, INPUTS / "compose_tiny.yaml")

    test_comparative_twins(sim, finish, official_mc, twin, abort, silent, throwing,
                           INPUTS / "compose_tiny.yaml")

    mix = {
        ALGO_NAME: official_algo,
        "FinishNowAlgo.so": finish,
        "NoopAlgo.so": find_so("NoopAlgo.so"),
        "InvalidAlgo.so": find_so("InvalidAlgo.so"),
        "HoverScanAlgo.so": find_so("HoverScanAlgo.so"),
        "HoverMoveAlgo.so": find_so("HoverMoveAlgo.so"),
        "NeverFinishAlgo.so": find_so("NeverFinishAlgo.so"),
        "RotateOnlyAlgo.so": find_so("RotateOnlyAlgo.so"),
        "SilentPlugin.so": silent,
        "WallAlgo.so": find_so("WallAlgo.so"),
    }
    test_competitive_mix(sim, official_mc, mix, INPUTS / "compose_tiny.yaml",
                         INPUTS / "compose_walled.yaml")
    test_cartesian(sim, official_mc, finish, INPUTS / "compose_cartesian.yaml")
    test_oversize_oob(sim, official_mc, find_so("OversizeAlgo.so"), find_so("OobAlgo.so"),
                      find_so("ElevateFarAlgo.so"), INPUTS / "compose_tiny.yaml")

    log("")
    log(f"== summary: {PASS} passed, {FAIL} failed in {time.time() - start:.1f}s ==")
    for item in FAILURES:
        log("  - " + item)
    return 1 if FAIL else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except subprocess.TimeoutExpired as e:
        log(f"TIMEOUT: {e}")
        sys.exit(1)
    except Exception as e:
        log(f"RUNNER ERROR: {e}")
        sys.exit(1)
