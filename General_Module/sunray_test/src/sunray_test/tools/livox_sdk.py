from __future__ import annotations

import json
import os
import re
import tempfile
import subprocess
from pathlib import Path
from typing import Callable, Sequence


SDK_QUERY_CACHE = Path.home() / ".cache/sunray_test/livox_sdk_sn_query"


def query_sn_by_sdk(
    config_paths: Sequence[Path],
    lidar_ip: str,
    iface_ip: str | None,
    timeout_sec: float,
    verbose: bool = False,
    progress: Callable[[str], None] | None = None,
    verbose_print: Callable[[bool, str], None] | None = None,
) -> str | None:
    sdk_roots = sdk2_candidates()
    log_verbose = verbose_print or noop_verbose
    if not sdk_roots:
        log_verbose(verbose, "SDK SN query skipped: Livox-SDK2 not found")
        return None
    base_config = next((path for path in config_paths if path.is_file()), None)
    if base_config is None:
        log_verbose(verbose, "SDK SN query: MID360 config not found; using temporary minimal config")
    for sdk_root in sdk_roots:
        binary = build_sdk_query_tool(sdk_root, verbose=verbose, verbose_print=log_verbose)
        if binary is None:
            continue
        query_config = make_sdk_query_config(base_config, lidar_ip, iface_ip)
        try:
            if progress:
                progress(f"querying SN with Livox SDK2 ({sdk_root})")
            output = run_text(
                [str(binary), str(query_config), str(max(1.0, timeout_sec))],
                timeout=timeout_sec + 8,
            )
            log_verbose(verbose, f"SDK SN query output from {sdk_root}: {output.strip() or 'N/A'}")
            match = re.search(r"^SDK_SN\s+([A-Z0-9]{10,16})\s*$", output, flags=re.MULTILINE)
            if match:
                return match.group(1)
        finally:
            try:
                query_config.unlink()
            except OSError:
                pass
    return None


def sdk2_candidates() -> list[Path]:
    candidates: list[Path] = []

    def add_candidate(path: Path) -> None:
        resolved = Path(os.path.expandvars(str(path))).expanduser()
        if is_sdk_root(resolved) and resolved not in candidates:
            candidates.append(resolved)

    for env_name in ("LIVOX_SDK2_ROOT", "LIVOX_SDK_ROOT", "LIVOX_SDK_PATH"):
        env_value = os.environ.get(env_name)
        if env_value:
            add_candidate(Path(env_value))

    for path in quick_sdk_paths():
        add_candidate(path)

    if candidates:
        return candidates

    for root in unique_existing_roots((Path.cwd(), Path.home())):
        output = run_text(
            [
                "find",
                str(root),
                "-maxdepth",
                "7",
                "-type",
                "f",
                "-path",
                "*/include/livox_lidar_api.h",
            ],
            timeout=8,
        )
        for line in output.splitlines():
            api_header = Path(line.strip())
            if api_header.name == "livox_lidar_api.h":
                add_candidate(api_header.parent.parent)
        if candidates:
            break
    return candidates


def is_sdk_root(path: Path) -> bool:
    return (
        (path / "include/livox_lidar_api.h").is_file()
        and (path / "include/livox_lidar_def.h").is_file()
    )


def quick_sdk_paths() -> list[Path]:
    return [
        Path.home() / "sunray_map/app/Livox-SDK2",
        Path.home() / "sunray_map/app/Livox_SDK2",
        Path.home() / "sunray_map/src/Livox-SDK2",
        Path.home() / "sunray_map/src/Livox_SDK2",
        Path.home() / "Sunray_V2/drivers/Livox_SDK2",
        Path.home() / "Sunray_V2_Internal_Test/drivers/Livox_SDK2",
        Path.home() / "Documents/Sunray_v2/drivers/Livox_SDK2",
        Path.home() / "sunray_livox_driver/driver/Livox-SDK2",
        Path.home() / "sunray_livox_driver/driver/Livox_SDK2",
        Path.home() / "livox_ws/src/Livox-SDK2",
        Path.home() / "livox_ws/src/Livox_SDK2",
    ]


def unique_existing_roots(roots: Sequence[Path]) -> list[Path]:
    unique: list[Path] = []
    for root in roots:
        resolved = root.expanduser()
        if resolved.is_dir() and resolved not in unique:
            unique.append(resolved)
    return unique


def build_sdk_query_tool(
    sdk_root: Path,
    verbose: bool = False,
    verbose_print: Callable[[bool, str], None] | None = None,
) -> Path | None:
    build_dir = SDK_QUERY_CACHE / re.sub(r"[^A-Za-z0-9_.-]+", "_", str(sdk_root))
    binary = build_dir / "livox_sdk_sn_query"
    if binary.is_file():
        return binary
    (verbose_print or noop_verbose)(verbose, f"SDK SN query skipped: no cached helper at {binary}")
    return None


def default_sdk_query_config(lidar_ip: str, iface_ip: str | None) -> dict:
    host_ip = iface_ip or "0.0.0.0"
    return {
        "lidar_summary_info": {
            "lidar_type": 8,
        },
        "MID360": {
            "lidar_net_info": {
                "cmd_data_port": 56100,
                "push_msg_port": 56200,
                "point_data_port": 56300,
                "imu_data_port": 56400,
                "log_data_port": 56500,
            },
            "host_net_info": {
                "cmd_data_ip": host_ip,
                "cmd_data_port": 56101,
                "push_msg_ip": host_ip,
                "push_msg_port": 56201,
                "point_data_ip": host_ip,
                "point_data_port": 56301,
                "imu_data_ip": host_ip,
                "imu_data_port": 56401,
                "log_data_ip": "",
                "log_data_port": 56501,
            },
        },
        "lidar_configs": [
            {
                "ip": lidar_ip,
                "pcl_data_type": 1,
                "pattern_mode": 0,
                "extrinsic_parameter": {
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                    "x": 0,
                    "y": 0,
                    "z": 0,
                },
            }
        ],
    }


def make_sdk_query_config(base_config: Path | None, lidar_ip: str, iface_ip: str | None) -> Path:
    if base_config and base_config.is_file():
        data = json.loads(base_config.read_text(encoding="utf-8"))
    else:
        data = default_sdk_query_config(lidar_ip, iface_ip)
    lidar_configs = data.setdefault("lidar_configs", [{}])
    if not lidar_configs:
        lidar_configs.append({})
    lidar_configs[0]["ip"] = lidar_ip
    if iface_ip:
        update_host_ip(data, iface_ip)
    tmp = tempfile.NamedTemporaryFile(
        "w",
        prefix="sunray_mid360_sdk_query_",
        suffix=".json",
        encoding="utf-8",
        delete=False,
    )
    with tmp:
        json.dump(data, tmp, indent=2, ensure_ascii=False)
        tmp.write("\n")
    return Path(tmp.name)


def update_host_ip(data: dict, iface_ip: str) -> None:
    mid360 = data.get("MID360")
    if not isinstance(mid360, dict):
        return
    host_net_info = mid360.get("host_net_info")
    if not isinstance(host_net_info, dict):
        return
    for key in (
        "cmd_data_ip",
        "push_msg_ip",
        "point_data_ip",
        "imu_data_ip",
        "log_data_ip",
    ):
        if key in host_net_info:
            host_net_info[key] = iface_ip


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


def noop_verbose(_: bool, __: str) -> None:
    return None
