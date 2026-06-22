from __future__ import annotations

import os
import subprocess
from pathlib import Path
from typing import Callable, Sequence


MID360_CONFIG_FILENAMES = ("MID360s_config.json", "MID360_config.json")


def default_config_paths(home: Path | None = None) -> list[Path]:
    config_dir = (home or Path.home()) / "sunray_map/src/livox_ros_driver2/config"
    return [config_dir / name for name in MID360_CONFIG_FILENAMES]


def default_config_path(home: Path | None = None) -> Path:
    return default_config_paths(home=home)[0]


def resolve_config_paths(
    raw_paths: Sequence[str] | None,
    verbose: bool = False,
    progress: Callable[[str], None] | None = None,
    verbose_print: Callable[[bool, str], None] | None = None,
) -> list[Path]:
    if raw_paths:
        return [Path(raw_path).expanduser() for raw_path in raw_paths]
    for default_path in default_config_paths():
        if default_path.is_file():
            return [default_path]
    discovered = discover_mid360_config_paths(
        verbose=verbose,
        verbose_print=verbose_print,
    )
    if discovered:
        if progress:
            progress(f"using discovered MID360 config: {discovered[0]}")
        if len(discovered) > 1 and verbose_print:
            ignored = [str(path) for path in discovered[1:]]
            verbose_print(verbose, f"other MID360 configs ignored: {ignored}")
        return [discovered[0]]
    return [default_config_path()]


def discover_mid360_config_paths(
    verbose: bool = False,
    roots: Sequence[Path] | None = None,
    verbose_print: Callable[[bool, str], None] | None = None,
) -> list[Path]:
    found: list[Path] = []
    seen: set[Path] = set()
    for root in roots or (Path.cwd(), Path.home()):
        root = root.expanduser()
        if not root.is_dir():
            continue
        output = run_text(find_config_command(root), timeout=8)
        for line in output.splitlines():
            path = Path(line.strip()).expanduser()
            if path.is_file() and path not in seen:
                seen.add(path)
                found.append(path)
    found.sort(key=config_priority)
    if verbose_print:
        verbose_print(verbose, "discovered MID360 configs: " + discovered_summary(found))
    return found


def find_config_command(root: Path) -> list[str]:
    return [
        "find",
        str(root),
        "-maxdepth",
        "8",
        "-type",
        "f",
        "(",
        "-name",
        "MID360s_config.json",
        "-o",
        "-name",
        "MID360_config.json",
        ")",
    ]


def discovered_summary(paths: Sequence[Path]) -> str:
    if not paths:
        return "N/A"
    return str(
        [
            {
                "path": str(path),
                "mtime": int(path.stat().st_mtime),
            }
            for path in paths
        ]
    )


def config_priority(path: Path) -> tuple[int, int, float, int, str]:
    lowered = str(path).lower()
    low_priority_tokens = ("backup", ".cache", "/build/", "/devel/", "/log/", "/logs/")
    low_priority = 1 if any(token in lowered for token in low_priority_tokens) else 0
    try:
        newest_first = -path.stat().st_mtime
    except OSError:
        newest_first = 0.0
    preferred_tokens = ("sunray_map", "drone3plot", "/sunray/", "ws_loc")
    preferred_rank = next(
        (idx for idx, token in enumerate(preferred_tokens) if token in lowered),
        len(preferred_tokens),
    )
    try:
        filename_rank = MID360_CONFIG_FILENAMES.index(path.name)
    except ValueError:
        filename_rank = len(MID360_CONFIG_FILENAMES)
    return low_priority, filename_rank, newest_first, preferred_rank, str(path)


def run_text(command: list[str], timeout: float | None = None) -> str:
    try:
        proc = subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode(errors="replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode(errors="replace")
        return stdout + stderr
    return proc.stdout + proc.stderr
