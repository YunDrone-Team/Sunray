#!/usr/bin/env python3
"""Discover a Livox MID360-like lidar on an Ethernet port and update config.

Default mode is read-only: it actively scans for the lidar IP first, then tries
cheap optional sources for broadcast code/SN.
Use --apply with --config to modify MID360_config.json.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import ipaddress
import json
import os
import re
import select
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path


DISCOVERY_PORT = 56000
LIVOX_MAC_PREFIXES = ("8c:58:23",)
MIN_ACTIVE_SCAN_LIDAR_SCORE = 100
LIDAR_SEARCH_NETWORK = ipaddress.ip_network("192.168.1.0/24")
RAW_ARP_PROBE_SOURCE_IPS = ("192.168.1.250", "192.168.1.251")
SDK_QUERY_CACHE = Path.home() / ".cache/sunray_test/livox_sdk_sn_query"


SDK_QUERY_MAIN_CPP = r'''
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>

namespace {
std::mutex g_mutex;
std::condition_variable g_cv;
std::atomic<bool> g_found(false);
std::string g_sn;
std::string g_ip;
uint8_t g_dev_type = 0;

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void*) {
  (void)handle;
  if (info == nullptr) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_sn = info->sn;
    g_ip = info->lidar_ip;
    g_dev_type = info->dev_type;
    g_found = true;
  }
  g_cv.notify_one();
}
}  // namespace

int main(int argc, const char* argv[]) {
  if (argc < 2 || argc > 3) {
    std::fprintf(stderr, "usage: %s <MID360_config.json> [timeout_sec]\n", argv[0]);
    return 2;
  }
  const char* config_path = argv[1];
  const double timeout_sec = argc == 3 ? std::atof(argv[2]) : 4.0;

  DisableLivoxSdkConsoleLogger();
  if (!LivoxLidarSdkInit(config_path)) {
    std::fprintf(stderr, "SDK_ERROR init_failed config=%s\n", config_path);
    LivoxLidarSdkUninit();
    return 2;
  }
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  std::unique_lock<std::mutex> lock(g_mutex);
  g_cv.wait_for(lock, std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000.0)), [] {
    return g_found.load();
  });

  if (!g_found.load()) {
    std::printf("SDK_SN N/A\n");
    LivoxLidarSdkUninit();
    return 1;
  }

  std::printf("SDK_SN %s\n", g_sn.c_str());
  std::printf("SDK_IP %s\n", g_ip.c_str());
  std::printf("SDK_DEV_TYPE %u\n", static_cast<unsigned>(g_dev_type));
  LivoxLidarSdkUninit();
  return 0;
}
'''


def default_config_path() -> Path:
    return Path.home() / "sunray_map/src/livox_ros_driver2/config/MID360_config.json"


def _config_priority(path: Path) -> tuple[int, float, int, str]:
    lowered = str(path).lower()
    low_priority_tokens = ("backup", ".cache", "/build/", "/devel/", "/log/", "/logs/")
    low_priority = 1 if any(token in lowered for token in low_priority_tokens) else 0
    try:
        newest_first = -path.stat().st_mtime
    except OSError:
        newest_first = 0.0
    preferred_tokens = ("sunray_map", "drone3plot", "/sunray/", "ws_loc")
    preferred_rank = next((idx for idx, token in enumerate(preferred_tokens) if token in lowered), len(preferred_tokens))
    return low_priority, newest_first, preferred_rank, str(path)


def discover_mid360_config_paths(verbose: bool = False) -> list[Path]:
    found: list[Path] = []
    seen: set[Path] = set()
    for root in (Path.cwd(), Path.home()):
        root = root.expanduser()
        if not root.is_dir():
            continue
        output = run_text(
            [
                "find",
                str(root),
                "-maxdepth",
                "8",
                "-type",
                "f",
                "-path",
                "*/livox_ros_driver2/config/MID360_config.json",
            ],
            timeout=8,
        )
        for line in output.splitlines():
            path = Path(line.strip()).expanduser()
            if path.is_file() and path not in seen:
                seen.add(path)
                found.append(path)
    found.sort(key=_config_priority)
    verbose_print(
        verbose,
        "discovered MID360 configs: "
        + (
            str(
                [
                    {
                        "path": str(path),
                        "mtime": int(path.stat().st_mtime),
                    }
                    for path in found
                ]
            )
            if found
            else "N/A"
        ),
    )
    return found


def resolve_config_paths(raw_paths: list[str] | None, verbose: bool = False) -> list[Path]:
    if raw_paths:
        return [Path(raw_path).expanduser() for raw_path in raw_paths]
    default_path = default_config_path()
    if default_path.is_file():
        return [default_path]
    discovered = discover_mid360_config_paths(verbose=verbose)
    if discovered:
        progress(f"using discovered MID360 config: {discovered[0]}")
        if len(discovered) > 1:
            verbose_print(verbose, f"other MID360 configs ignored: {[str(path) for path in discovered[1:]]}")
        return [discovered[0]]
    return [default_path]


@dataclass
class Discovery:
    lidar_ip: str | None = None
    broadcast_code: str | None = None
    requested_host_ip: str | None = None
    iface_ip: str | None = None
    raw_packets: int = 0
    method: str = ""


def append_method(method: str, suffix: str) -> str:
    return f"{method}+{suffix}" if method else suffix


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


def iface_ipv4(iface: str) -> str | None:
    output = run_text(["ip", "-4", "-o", "addr", "show", "dev", iface])
    match = re.search(r"\binet\s+(\d+\.\d+\.\d+\.\d+)/", output)
    return match.group(1) if match else None


def list_ipv4_interfaces() -> list[tuple[str, str, int]]:
    output = run_text(["ip", "-4", "-o", "addr", "show"])
    interfaces: list[tuple[str, str, int]] = []
    for line in output.splitlines():
        match = re.search(r"^\d+:\s+([^:\s]+)\s+.*\binet\s+(\d+\.\d+\.\d+\.\d+)/(\d+)", line)
        if not match:
            continue
        iface, ip, prefix = match.groups()
        if iface == "lo" or iface.startswith("docker"):
            continue
        interfaces.append((iface, ip, int(prefix)))
    return interfaces


def list_ethernet_interfaces() -> list[str]:
    output = run_text(["ip", "-br", "link"])
    interfaces: list[str] = []
    for line in output.splitlines():
        parts = line.split()
        if len(parts) < 3:
            continue
        iface, state = parts[0], parts[1]
        flags = " ".join(parts[3:])
        if "@" in iface:
            iface = iface.split("@", 1)[0]
        if iface == "lo" or iface.startswith(("docker", "veth", "br-", "virbr")):
            continue
        if not iface.startswith(("eth", "en", "eno", "ens", "enp", "enx")):
            continue
        if state != "UP" and "LOWER_UP" not in flags:
            continue
        interfaces.append(iface)
    return interfaces


def candidate_ifaces(requested_iface: str, config_paths: list[Path]) -> list[str]:
    if requested_iface.strip().lower() != "auto":
        return [requested_iface]

    configured_ips = [read_config_lidar_ip(path) for path in config_paths]
    configured_ips = [ip for ip in configured_ips if ip]
    interfaces = list_ipv4_interfaces()
    ethernet_interfaces = list_ethernet_interfaces()
    ipv4_iface_names = {iface for iface, _, _ in interfaces}
    candidates: list[str] = []

    for configured_ip in configured_ips:
        try:
            lidar_addr = ipaddress.ip_address(configured_ip)
        except ValueError:
            continue
        for iface, host_ip, prefix in interfaces:
            network = ipaddress.ip_network(f"{host_ip}/{prefix}", strict=False)
            if lidar_addr in network and iface not in candidates:
                candidates.append(iface)

    for iface, host_ip, _ in interfaces:
        if iface in ethernet_interfaces and ip_in_lidar_search_network(host_ip) and iface not in candidates:
            candidates.append(iface)
    for iface in [iface for iface in ethernet_interfaces if iface not in ipv4_iface_names]:
        if iface not in candidates:
            candidates.append(iface)
    for iface in ethernet_interfaces:
        if iface not in candidates:
            candidates.append(iface)
    for iface, _, _ in interfaces:
        if iface not in candidates:
            candidates.append(iface)
    return candidates or ["eth0"]


def verbose_print(enabled: bool, message: str) -> None:
    if enabled:
        print(f"[debug] {message}", file=sys.stderr)


def progress(message: str) -> None:
    print(f"[scan] {message}", flush=True)


def ip_in_lidar_search_network(ip: str) -> bool:
    try:
        return ipaddress.ip_address(ip) in LIDAR_SEARCH_NETWORK
    except ValueError:
        return False


def parse_ascii_from_hex_dump(lines: list[str], verbose: bool = False) -> str | None:
    blob = bytearray()
    for line in lines:
        match = re.search(r"0x[0-9a-fA-F]+:\s+(.*)$", line)
        if not match:
            continue
        for token in match.group(1).split():
            if not re.fullmatch(r"[0-9a-fA-F]{4}", token):
                break
            blob.extend(bytes.fromhex(token))
    if not blob:
        verbose_print(verbose, "hex dump: no payload bytes found")
        return None
    text = "".join(chr(b) if 32 <= b < 127 else " " for b in blob)
    candidates = re.findall(r"[A-Z0-9]{10,16}", text)
    verbose_print(verbose, f"hex dump: decoded {len(blob)} bytes")
    verbose_print(verbose, f"hex dump ascii: {text.strip() or 'N/A'}")
    verbose_print(verbose, f"broadcast candidates: {candidates or 'N/A'}")
    if not candidates:
        return None
    candidates.sort(key=lambda item: (("LIVOX" in item) or ("ARM" in item), len(item)), reverse=True)
    verbose_print(verbose, f"selected broadcast_code candidate: {candidates[0]}")
    return candidates[0]


def _looks_like_livox_sn(value: str) -> bool:
    return bool(re.fullmatch(r"[A-Z0-9]{10,16}", value)) and any(ch.isalpha() for ch in value)


def parse_sn_from_text(text: str) -> str | None:
    patterns = [
        r"(?i)\bSN\s*[:=]\s*([A-Z0-9]{10,16})\b",
        r"(?i)\bsn\s*[:=]\s*([A-Z0-9]{10,16})\b",
        r"(?i)\bbroadcast[_ -]?code\s*[:=]\s*([A-Z0-9]{10,16})\b",
    ]
    for pattern in patterns:
        for match in re.finditer(pattern, text):
            candidate = match.group(1).upper()
            if _looks_like_livox_sn(candidate):
                return candidate
    return None


def recent_log_sn(lidar_ip: str | None, max_age_days: int = 14, verbose: bool = False) -> str | None:
    log_root = Path.home() / ".ros/log"
    if not log_root.is_dir():
        verbose_print(verbose, "recent log SN skipped: ~/.ros/log not found")
        return None
    output = run_text(
        [
            "find",
            str(log_root),
            "-type",
            "f",
            "-mtime",
            f"-{max_age_days}",
        ],
        timeout=5,
    )
    files = [Path(line.strip()) for line in output.splitlines() if line.strip()]
    files = [path for path in files if path.is_file()]
    files.sort(key=lambda path: path.stat().st_mtime, reverse=True)
    verbose_print(verbose, f"recent log SN scan files={len(files)}")

    fallback_sn: str | None = None
    for path in files[:80]:
        try:
            with path.open("rb") as fh:
                fh.seek(0, os.SEEK_END)
                size = fh.tell()
                fh.seek(max(0, size - 256 * 1024), os.SEEK_SET)
                text = fh.read().decode(errors="replace")
        except OSError:
            continue
        sn = parse_sn_from_text(text)
        if not sn:
            continue
        verbose_print(verbose, f"recent log SN candidate: sn={sn}, file={path}")
        if lidar_ip and lidar_ip in text:
            return sn
        if fallback_sn is None:
            fallback_sn = sn
    return fallback_sn


def parse_tcpdump(output: str, iface: str, verbose: bool = False) -> Discovery:
    result = Discovery(iface_ip=iface_ipv4(iface))
    hex_lines: list[str] = []
    lines = output.splitlines()
    verbose_print(verbose, f"parse iface={iface}, iface_ip={result.iface_ip or 'N/A'}")
    verbose_print(verbose, f"tcpdump output lines={len(lines)}")
    for line in output.splitlines():
        if f".{DISCOVERY_PORT} >" in line and "IP " in line:
            match = re.search(r"\bIP\s+(\d+\.\d+\.\d+\.\d+)\.%d\s+>" % DISCOVERY_PORT, line)
            if match:
                packet_source_ip = match.group(1)
                if packet_source_ip == result.iface_ip:
                    verbose_print(verbose, f"ignore host discovery packet: source_ip={packet_source_ip}, line={line}")
                    continue
                if not ip_in_lidar_search_network(packet_source_ip):
                    verbose_print(verbose, f"ignore discovery packet outside {LIDAR_SEARCH_NETWORK}: {line}")
                    continue
                result.lidar_ip = packet_source_ip
                result.method = "livox_discovery"
                result.raw_packets += 1
                verbose_print(verbose, f"UDP discovery packet: lidar_ip={result.lidar_ip}, line={line}")
        if "ARP," in line and "who-has" in line and " tell " in line:
            match = re.search(
                r"who-has\s+(\d+\.\d+\.\d+\.\d+)\s+tell\s+(\d+\.\d+\.\d+\.\d+)",
                line,
            )
            if match:
                host_ip, lidar_ip = match.groups()
                verbose_print(verbose, f"ARP request: lidar_ip={lidar_ip}, requested_host_ip={host_ip}")
                if not ip_in_lidar_search_network(lidar_ip):
                    verbose_print(verbose, f"ignore ARP sender outside {LIDAR_SEARCH_NETWORK}: {line}")
                    continue
                if result.lidar_ip is None or result.lidar_ip == lidar_ip:
                    result.lidar_ip = lidar_ip
                    result.requested_host_ip = host_ip
                    result.method = "arp_observed"
        if re.search(r"0x[0-9a-fA-F]+:", line):
            hex_lines.append(line)
    verbose_print(verbose, f"hex dump lines={len(hex_lines)}")
    result.broadcast_code = parse_ascii_from_hex_dump(hex_lines, verbose=verbose)
    verbose_print(
        verbose,
        "parse result: "
        f"lidar_ip={result.lidar_ip or 'N/A'}, "
        f"broadcast_code={result.broadcast_code or 'N/A'}, "
        f"requested_host_ip={result.requested_host_ip or 'N/A'}, "
        f"raw_packets={result.raw_packets}",
    )
    return result


def sniff(iface: str, timeout_sec: float, sudo: bool, verbose: bool = False) -> Discovery:
    tcpdump = shutil.which("tcpdump")
    if not tcpdump:
        raise RuntimeError("tcpdump not found. Install it first, e.g. sudo apt install tcpdump")
    command = [
        tcpdump,
        "-ni",
        iface,
        f"(udp and port {DISCOVERY_PORT}) or arp",
        "-X",
    ]
    if sudo and hasattr(os, "geteuid") and os.geteuid() != 0:
        command.insert(0, "sudo")
    verbose_print(verbose, f"sniff command: {' '.join(command)}")
    verbose_print(verbose, f"sniff timeout: {timeout_sec}s")
    progress(f"listening on {iface} for Livox discovery packets ({timeout_sec:.1f}s)")
    output = run_text(["timeout", "-k", "2", str(timeout_sec), *command], timeout=timeout_sec + 5)
    if verbose:
        preview = "\n".join(output.splitlines()[:40])
        verbose_print(verbose, "tcpdump preview begin")
        if preview:
            print(preview, file=sys.stderr)
        else:
            print("(empty tcpdump output)", file=sys.stderr)
        verbose_print(verbose, "tcpdump preview end")
    return parse_tcpdump(output, iface, verbose=verbose)


def _ping_once(iface: str, ip: str) -> None:
    run_text(["timeout", "0.8", "ping", "-I", iface, "-c", "1", "-W", "1", ip], timeout=1.2)


def iface_mac(iface: str) -> str | None:
    path = Path("/sys/class/net") / iface / "address"
    try:
        mac = path.read_text(encoding="utf-8").strip().lower()
    except OSError:
        return None
    return mac if re.fullmatch(r"[0-9a-f]{2}(:[0-9a-f]{2}){5}", mac) else None


def _mac_bytes(mac: str) -> bytes:
    return bytes(int(part, 16) for part in mac.split(":"))


def _raw_arp_request(src_mac: bytes, source_ip: str, target_ip: str) -> bytes:
    return b"".join(
        [
            b"\xff\xff\xff\xff\xff\xff",
            src_mac,
            b"\x08\x06",
            b"\x00\x01",
            b"\x08\x00",
            b"\x06",
            b"\x04",
            b"\x00\x01",
            src_mac,
            socket.inet_aton(source_ip),
            b"\x00\x00\x00\x00\x00\x00",
            socket.inet_aton(target_ip),
        ]
    )


def raw_arp_scan_via_sudo(iface: str, verbose: bool = False) -> list[dict[str, object]]:
    command = ["sudo"]
    if not sys.stdin.isatty():
        command.append("-n")
    command.extend(
        [
            sys.executable,
            str(Path(__file__).resolve()),
            "--raw-arp-helper",
            iface,
        ]
    )
    if verbose:
        command.append("--verbose")
    output = run_text(command, timeout=8.0)
    verbose_print(verbose, f"sudo raw ARP helper output: {output.strip() or 'N/A'}")
    result_prefix = "RAW_ARP_RESULTS "
    for line in output.splitlines():
        if not line.startswith(result_prefix):
            continue
        try:
            entries = json.loads(line[len(result_prefix) :])
        except json.JSONDecodeError:
            continue
        if isinstance(entries, list):
            return [entry for entry in entries if isinstance(entry, dict)]
    progress("raw ARP scan skipped: run this script with sudo, or grant CAP_NET_RAW, to scan without changing interface IP")
    return []


def raw_arp_scan(iface: str, sudo: bool, verbose: bool = False) -> list[dict[str, object]]:
    mac = iface_mac(iface)
    if not mac:
        verbose_print(verbose, f"raw ARP scan skipped: no MAC for iface={iface}")
        return []
    src_mac = _mac_bytes(mac)
    targets = [str(ip) for ip in LIDAR_SEARCH_NETWORK.hosts()]
    responses: dict[str, dict[str, object]] = {}

    try:
        sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0806))
    except PermissionError:
        if sudo and hasattr(os, "geteuid") and os.geteuid() != 0:
            return raw_arp_scan_via_sudo(iface, verbose=verbose)
        progress("raw ARP scan skipped: root or CAP_NET_RAW is required to scan without changing interface IP")
        return []
    except OSError as exc:
        verbose_print(verbose, f"raw ARP scan skipped: open socket failed: {exc}")
        return []

    try:
        sock.bind((iface, 0))
        sock.setblocking(False)
        progress(f"raw ARP scan on {iface} network={LIDAR_SEARCH_NETWORK} hosts={len(targets)}")
        verbose_print(verbose, f"raw ARP scan iface={iface}, source_mac={mac}, targets={len(targets)}")
        for idx, target_ip in enumerate(targets):
            source_ip = RAW_ARP_PROBE_SOURCE_IPS[idx % len(RAW_ARP_PROBE_SOURCE_IPS)]
            sock.send(_raw_arp_request(src_mac, source_ip, target_ip))
            if idx % 32 == 0:
                _collect_raw_arp_replies(sock, responses, deadline=time.monotonic() + 0.01)
        _collect_raw_arp_replies(sock, responses, deadline=time.monotonic() + 1.2)
    except OSError as exc:
        verbose_print(verbose, f"raw ARP scan skipped: socket operation failed: {exc}")
        return []
    finally:
        sock.close()

    entries = list(responses.values())
    verbose_print(verbose, f"raw ARP scan responses={entries}")
    return entries


def _collect_raw_arp_replies(sock: socket.socket, responses: dict[str, dict[str, object]], deadline: float) -> None:
    while time.monotonic() < deadline:
        timeout = max(0.0, min(0.05, deadline - time.monotonic()))
        readable, _, _ = select.select([sock], [], [], timeout)
        if not readable:
            continue
        try:
            packet = sock.recv(65535)
        except BlockingIOError:
            continue
        if len(packet) < 42 or packet[12:14] != b"\x08\x06":
            continue
        arp = packet[14:42]
        if arp[6:8] != b"\x00\x02":
            continue
        sender_mac = ":".join(f"{byte:02x}" for byte in arp[8:14])
        sender_ip = socket.inet_ntoa(arp[14:18])
        if ip_in_lidar_search_network(sender_ip):
            responses[sender_ip] = {"ip": sender_ip, "mac": sender_mac, "state": "RAW_ARP", "ttl": None}


def _neighbor_entries(iface: str) -> list[dict[str, str]]:
    output = run_text(["ip", "neigh", "show", "dev", iface])
    entries: list[dict[str, str]] = []
    for line in output.splitlines():
        parts = line.split()
        if not parts:
            continue
        ip = parts[0]
        if "lladdr" not in parts:
            continue
        state = parts[-1]
        if state in {"FAILED", "INCOMPLETE"}:
            continue
        mac = parts[parts.index("lladdr") + 1]
        entries.append({"ip": ip, "mac": mac.lower(), "state": state})
    return entries


def _ping_ttl(iface: str, ip: str) -> int | None:
    output = run_text(["timeout", "1.5", "ping", "-I", iface, "-c", "1", "-W", "1", ip], timeout=2.0)
    match = re.search(r"\bttl=(\d+)", output, flags=re.IGNORECASE)
    return int(match.group(1)) if match else None


def _gateway_ips(iface: str) -> set[str]:
    output = run_text(["ip", "route", "show", "dev", iface])
    gateways: set[str] = set()
    for line in output.splitlines():
        match = re.search(r"\bvia\s+(\d+\.\d+\.\d+\.\d+)", line)
        if match:
            gateways.add(match.group(1))
    return gateways


def _score_candidates(entries: list[dict[str, object]], host_ip: str | None, gateways: set[str]) -> list[dict[str, object]]:
    candidates: list[dict[str, object]] = []
    for entry in entries:
        ip = entry["ip"]
        if not ip_in_lidar_search_network(ip):
            continue
        if ip == host_ip or ip in gateways:
            continue
        ttl = entry.get("ttl")
        mac = entry["mac"]
        score = 0
        if mac.startswith(LIVOX_MAC_PREFIXES):
            score += 100
        if isinstance(ttl, int) and ttl >= 200:
            score += 40
        score += 5
        candidates.append({**entry, "ttl": ttl, "score": score})
    candidates.sort(key=lambda item: (item["score"], item["state"] == "REACHABLE"), reverse=True)
    return candidates


def active_scan(iface: str, sudo: bool, verbose: bool = False) -> Discovery:
    host_ip = iface_ipv4(iface)
    result = Discovery(iface_ip=host_ip, method="active_scan")
    gateways = _gateway_ips(iface)

    if host_ip and ip_in_lidar_search_network(host_ip):
        hosts = [str(ip) for ip in LIDAR_SEARCH_NETWORK.hosts() if str(ip) != host_ip]
        progress(f"active scan on {iface} network={LIDAR_SEARCH_NETWORK} hosts={len(hosts)}")
        verbose_print(verbose, f"active scan iface={iface}, network={LIDAR_SEARCH_NETWORK}, hosts={len(hosts)}")
        with concurrent.futures.ThreadPoolExecutor(max_workers=64) as executor:
            futures = [executor.submit(_ping_once, iface, ip) for ip in hosts]
            completed = 0
            report_step = max(16, len(futures) // 8) if futures else 1
            for future in concurrent.futures.as_completed(futures):
                future.result()
                completed += 1
                if completed == len(futures) or completed % report_step == 0:
                    print(f"\r[scan] active scan progress {completed}/{len(futures)}", end="", flush=True)
            if futures:
                print()

        entries = []
        for entry in _neighbor_entries(iface):
            ttl = _ping_ttl(iface, entry["ip"]) if ip_in_lidar_search_network(entry["ip"]) else None
            entries.append({**entry, "ttl": ttl})
    else:
        if host_ip:
            progress(
                f"interface {iface} IPv4 {host_ip} is outside {LIDAR_SEARCH_NETWORK}; "
                "trying raw Ethernet ARP scan without changing the address"
            )
        else:
            progress(f"interface {iface} has no IPv4; trying raw Ethernet ARP scan without configuring an address")
        result.method = "raw_arp_scan"
        entries = raw_arp_scan(iface, sudo=sudo, verbose=verbose)

    candidates = _score_candidates(entries, host_ip, gateways)
    verbose_print(verbose, f"active scan candidates={candidates}")
    if candidates:
        preview = ", ".join(
            f"{item['ip']} ttl={item['ttl'] or 'N/A'} mac={item['mac']} score={item['score']}"
            for item in candidates[:5]
        )
        progress(f"active scan candidates: {preview}")
    else:
        progress(f"active scan found no candidates on {iface}")
    if candidates and candidates[0]["score"] >= MIN_ACTIVE_SCAN_LIDAR_SCORE:
        result.lidar_ip = candidates[0]["ip"]
    elif candidates:
        progress(
            "active scan found online devices, but none match known MID360/Livox signatures; "
            "not treating them as lidar"
        )
    return result


def _sdk2_candidates() -> list[Path]:
    def is_sdk_root(path: Path) -> bool:
        return (
            (path / "include/livox_lidar_api.h").is_file()
            and (path / "include/livox_lidar_def.h").is_file()
        )

    def add_candidate(path: Path) -> None:
        resolved = Path(os.path.expandvars(str(path))).expanduser()
        if is_sdk_root(resolved) and resolved not in candidates:
            candidates.append(resolved)

    candidates: list[Path] = []
    for env_name in ("LIVOX_SDK2_ROOT", "LIVOX_SDK_ROOT", "LIVOX_SDK_PATH"):
        env_value = os.environ.get(env_name)
        if env_value:
            add_candidate(Path(env_value))

    quick_paths = [
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
    for path in quick_paths:
        add_candidate(path)

    if candidates:
        return candidates

    search_roots = [Path.cwd(), Path.home()]
    seen_search_roots: set[Path] = set()
    for root in search_roots:
        root = root.expanduser()
        if not root.is_dir() or root in seen_search_roots:
            continue
        seen_search_roots.add(root)
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
            if api_header.name != "livox_lidar_api.h":
                continue
            add_candidate(api_header.parent.parent)
        if candidates:
            break
    return candidates


def _sdk_source_files(sdk_root: Path) -> list[str]:
    sdk_core = sdk_root / "sdk_core"
    platform = "unix"
    rel_paths = [
        "device_manager.cpp",
        "livox_lidar_sdk.cpp",
        "params_check.cpp",
        "parse_cfg_file.cpp",
        "base/io_loop.cpp",
        "base/thread_base.cpp",
        "base/io_thread.cpp",
        "base/logging.cpp",
        f"base/network/{platform}/network_util.cpp",
        "base/multiple_io/multiple_io_base.cpp",
        "base/multiple_io/multiple_io_epoll.cpp",
        "base/multiple_io/multiple_io_poll.cpp",
        "base/multiple_io/multiple_io_select.cpp",
        "base/multiple_io/multiple_io_kqueue.cpp",
        f"base/wake_up/{platform}/wake_up_pipe.cpp",
        "comm/comm_port.cpp",
        "comm/sdk_protocol.cpp",
        "comm/generate_seq.cpp",
        "upgrade_manager.cpp",
        "upgrade/firmware.cpp",
        "upgrade/livox_lidar_upgrader.cpp",
        "logger_handler/logger_manager.cpp",
        "logger_handler/logger_handler.cpp",
        "logger_handler/file_manager.cpp",
        "data_handler/data_handler.cpp",
        "command_handler/command_impl.cpp",
        "command_handler/general_command_handler.cpp",
        "command_handler/hap_command_handler.cpp",
        "command_handler/mid360_command_handler.cpp",
        "command_handler/build_request.cpp",
        "command_handler/parse_lidar_state_info.cpp",
        "debug_point_cloud_handler/debug_point_cloud_manager.cpp",
        "debug_point_cloud_handler/debug_point_cloud_handler.cpp",
    ]
    files = [str(sdk_root / "3rdparty/FastCRC/FastCRCsw.cpp")]
    files.extend(str(sdk_core / rel_path) for rel_path in rel_paths)
    return files


def _build_sdk_query_tool(sdk_root: Path, verbose: bool = False) -> Path | None:
    build_dir = SDK_QUERY_CACHE / re.sub(r"[^A-Za-z0-9_.-]+", "_", str(sdk_root))
    binary = build_dir / "livox_sdk_sn_query"
    if binary.is_file():
        return binary
    verbose_print(verbose, f"SDK SN query skipped: no cached helper at {binary}")
    return None


def _default_sdk_query_config(lidar_ip: str, iface_ip: str | None) -> dict:
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


def _make_sdk_query_config(base_config: Path | None, lidar_ip: str, iface_ip: str | None) -> Path | None:
    if base_config and base_config.is_file():
        data = json.loads(base_config.read_text(encoding="utf-8"))
    else:
        data = _default_sdk_query_config(lidar_ip, iface_ip)
    lidar_configs = data.setdefault("lidar_configs", [{}])
    if not lidar_configs:
        lidar_configs.append({})
    lidar_configs[0]["ip"] = lidar_ip
    if iface_ip:
        mid360 = data.get("MID360")
        if isinstance(mid360, dict):
            host_net_info = mid360.get("host_net_info")
            if isinstance(host_net_info, dict):
                for key in (
                    "cmd_data_ip",
                    "push_msg_ip",
                    "point_data_ip",
                    "imu_data_ip",
                    "log_data_ip",
                ):
                    if key in host_net_info:
                        host_net_info[key] = iface_ip
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


def query_sn_by_sdk(config_paths: list[Path], lidar_ip: str, iface_ip: str | None, timeout_sec: float, verbose: bool = False) -> str | None:
    sdk_roots = _sdk2_candidates()
    if not sdk_roots:
        verbose_print(verbose, "SDK SN query skipped: Livox-SDK2 not found")
        return None
    base_config = next((path for path in config_paths if path.is_file()), None)
    if base_config is None:
        verbose_print(verbose, "SDK SN query: MID360_config.json not found; using temporary minimal config")
    for sdk_root in sdk_roots:
        binary = _build_sdk_query_tool(sdk_root, verbose=verbose)
        if binary is None:
            continue
        query_config = _make_sdk_query_config(base_config, lidar_ip, iface_ip)
        if query_config is None:
            continue
        try:
            progress(f"querying SN with Livox SDK2 ({sdk_root})")
            output = run_text([str(binary), str(query_config), str(max(1.0, timeout_sec))], timeout=timeout_sec + 8)
            verbose_print(verbose, f"SDK SN query output from {sdk_root}: {output.strip() or 'N/A'}")
            match = re.search(r"^SDK_SN\s+([A-Z0-9]{10,16})\s*$", output, flags=re.MULTILINE)
            if match:
                return match.group(1)
        finally:
            try:
                query_config.unlink()
            except OSError:
                pass
    return None


def discover(ifaces: list[str], timeout_sec: float, sudo: bool, verbose: bool = False) -> tuple[str, Discovery]:
    if not ifaces:
        ifaces = ["eth0"]
    per_iface_timeout = timeout_sec if len(ifaces) == 1 else max(1.5, min(timeout_sec, 2.0))
    last_result = Discovery()
    progress(f"candidate interfaces: {', '.join(ifaces)}")
    for iface in ifaces:
        progress(f"checking interface {iface} by active scan")
        verbose_print(verbose, f"active scan iface={iface}")
        result = active_scan(iface, sudo=sudo, verbose=verbose)
        if result.lidar_ip:
            progress(f"found lidar by active scan on {iface}: {result.lidar_ip}")
            return iface, result
        last_result = result
    for iface in ifaces:
        progress(f"checking interface {iface} by passive discovery fallback")
        verbose_print(verbose, f"try iface={iface}")
        result = sniff(iface, per_iface_timeout, sudo=sudo, verbose=verbose)
        if result.lidar_ip:
            progress(f"found lidar by passive discovery on {iface}: {result.lidar_ip}")
            return iface, result
        last_result = result
    return ifaces[-1], last_result


def update_config(path: Path, lidar_ip: str) -> None:
    data = json.loads(path.read_text(encoding="utf-8"))
    lidar_configs = data.setdefault("lidar_configs", [{}])
    if not lidar_configs:
        lidar_configs.append({})
    lidar_configs[0]["ip"] = lidar_ip

    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"updated: {path}")


def confirm_update(path: Path, configured_ip: str | None, lidar_ip: str, assume_yes: bool) -> bool:
    if assume_yes:
        return True
    if not sys.stdin.isatty():
        return False
    current = configured_ip or "N/A"
    answer = input(f"Update {path} lidar IP from {current} to {lidar_ip}? [y/N]: ").strip().lower()
    return answer in {"y", "yes"}


def read_config_lidar_ip(path: Path) -> str | None:
    if not path.is_file():
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    lidar_configs = data.get("lidar_configs")
    if not isinstance(lidar_configs, list) or not lidar_configs:
        return None
    first_config = lidar_configs[0]
    if not isinstance(first_config, dict):
        return None
    ip = first_config.get("ip")
    return str(ip) if ip else None


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Auto-discover Livox lidar IP/SN from eth discovery packets.",
    )
    parser.add_argument("--raw-arp-helper", metavar="IFACE", help=argparse.SUPPRESS)
    parser.add_argument(
        "-i",
        "--iface",
        default="auto",
        help="Ethernet interface to sniff, or 'auto' to choose by MID360_config.json lidar IP",
    )
    parser.add_argument(
        "-t",
        "--timeout",
        type=float,
        default=8.0,
        help="passive discovery fallback timeout seconds; auto mode caps each interface to 2s",
    )
    parser.add_argument(
        "--sn-timeout",
        type=float,
        default=2.0,
        help="seconds to spend on optional post-scan packet sniffing for SN",
    )
    parser.add_argument("--no-log-sn", action="store_true", help="do not try to fill SN from recent ROS logs")
    parser.add_argument("--no-sniff-sn", action="store_true", help="do not try optional post-scan packet sniffing for SN")
    parser.add_argument("-v", "--verbose", action="store_true", help="print detailed sniff/parse diagnostics")
    parser.add_argument("--no-sudo", action="store_true", help="do not prefix tcpdump with sudo")
    parser.add_argument(
        "--config",
        action="append",
        default=None,
        help=(
            "MID360_config.json path to check/update; can be specified multiple times. "
            f"Default: {default_config_path()}"
        ),
    )
    parser.add_argument("--apply", action="store_true", help="actually update config files")
    parser.add_argument("--yes", action="store_true", help="update config without interactive confirmation when used with --apply")
    parser.add_argument(
        "--sdk-sn",
        action="store_true",
        help="optionally query SN with an existing cached Livox SDK helper; no helper is built automatically",
    )
    parser.add_argument(
        "--no-sdk-sn",
        action="store_true",
        help="deprecated compatibility option; SDK SN query is disabled by default",
    )
    parser.add_argument(
        "--sdk-timeout",
        type=float,
        default=4.0,
        help="seconds to wait for Livox SDK2 SN callback when SDK fallback is enabled",
    )
    parser.add_argument(
        "--require-match",
        action="store_true",
        help="return non-zero when detected lidar IP differs from configured lidar IP",
    )
    args = parser.parse_args()
    if args.raw_arp_helper:
        entries = raw_arp_scan(args.raw_arp_helper, sudo=False, verbose=args.verbose)
        print("RAW_ARP_RESULTS " + json.dumps(entries, separators=(",", ":")))
        return 0

    config_paths = resolve_config_paths(args.config, verbose=args.verbose)
    ifaces = candidate_ifaces(args.iface, config_paths)
    progress("starting MID360 discovery")
    if args.verbose:
        verbose_print(True, f"candidate ifaces: {ifaces}")

    iface, result = discover(ifaces, args.timeout, sudo=not args.no_sudo, verbose=args.verbose)
    if result.lidar_ip and not result.broadcast_code and not args.no_log_sn:
        log_sn = recent_log_sn(result.lidar_ip, verbose=args.verbose)
        if log_sn:
            result.broadcast_code = log_sn
            result.method = append_method(result.method, "log_sn")

    if result.lidar_ip and not result.broadcast_code and not args.no_sniff_sn:
        try:
            sniff_result = sniff(iface, args.sn_timeout, sudo=not args.no_sudo, verbose=args.verbose)
            same_lidar = not sniff_result.lidar_ip or sniff_result.lidar_ip == result.lidar_ip
            if sniff_result.broadcast_code and same_lidar:
                result.broadcast_code = sniff_result.broadcast_code
                result.method = append_method(result.method, "sniff_sn")
            if sniff_result.requested_host_ip and not result.requested_host_ip:
                result.requested_host_ip = sniff_result.requested_host_ip
            result.raw_packets += sniff_result.raw_packets
        except RuntimeError as exc:
            verbose_print(args.verbose, f"optional SN sniff skipped: {exc}")

    if result.lidar_ip and not result.broadcast_code and args.sdk_sn and not args.no_sdk_sn:
        sdk_sn = query_sn_by_sdk(
            config_paths,
            result.lidar_ip,
            result.iface_ip,
            timeout_sec=args.sdk_timeout,
            verbose=args.verbose,
        )
        if sdk_sn:
            result.broadcast_code = sdk_sn
            if result.method:
                result.method = f"{result.method}+sdk_sn"
            else:
                result.method = "sdk_sn"
    print(f"iface:           {iface}")
    print(f"iface_ip:        {result.iface_ip or 'N/A'}")
    print(f"lidar_ip:        {result.lidar_ip or 'N/A'}")
    print(f"broadcast_code:  {result.broadcast_code or 'N/A'}")
    print(f"arp_host_ip:     {result.requested_host_ip or 'N/A'}")
    print(f"discovery_pkts:  {result.raw_packets}")
    print(f"detect_method:   {result.method or 'N/A'}")

    if not result.lidar_ip:
        print("ERROR: no MID360 lidar IP found by passive discovery or active scan", file=sys.stderr)
        return 2

    needs_update = False
    unavailable = False
    config_states = []
    for path in config_paths:
        configured_ip = read_config_lidar_ip(path)
        print(f"config:          {path}")
        print(f"config_lidar_ip: {configured_ip or 'N/A'}")
        if configured_ip and configured_ip != result.lidar_ip:
            needs_update = True
            print(f"config_status:   mismatch ({configured_ip} != {result.lidar_ip})")
        elif configured_ip == result.lidar_ip:
            print("config_status:   match")
        else:
            unavailable = True
            print("config_status:   unavailable")
        config_states.append((path, configured_ip))

    updated_any = False
    if args.apply or needs_update:
        for path, configured_ip in config_states:
            if not path.is_file():
                if args.apply:
                    print(f"ERROR: config file not found: {path}", file=sys.stderr)
                    return 2
                continue
            if configured_ip == result.lidar_ip:
                continue
            should_update = args.yes or (args.apply and not sys.stdin.isatty())
            if not should_update:
                should_update = confirm_update(path, configured_ip, result.lidar_ip, False)
            if should_update:
                update_config(path, result.lidar_ip)
                updated_any = True
            else:
                print(f"skipped: {path}")

    if args.require_match and (needs_update or unavailable) and not updated_any:
        print(
            "ERROR: detected lidar IP cannot be verified against MID360_config.json; rerun with --apply to update it",
            file=sys.stderr,
        )
        return 3
    if not updated_any:
        print("dry-run: add --apply to update config")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
