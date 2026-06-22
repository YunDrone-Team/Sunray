#!/usr/bin/env python3
import importlib.util
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCRIPT_PATH = os.path.join(PACKAGE_ROOT, "scripts", "livox_mid360_autoconfig.py")

spec = importlib.util.spec_from_file_location("livox_mid360_autoconfig", SCRIPT_PATH)
livox_mid360 = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = livox_mid360
spec.loader.exec_module(livox_mid360)


class LivoxMid360AutoconfigTest(unittest.TestCase):
    def test_single_high_ttl_candidate_is_selected_without_livox_mac(self):
        candidates = livox_mid360._score_candidates(
            [
                {
                    "ip": "192.168.1.148",
                    "mac": "4c:43:f6:ea:e6:e6",
                    "state": "REACHABLE",
                    "ttl": 255,
                }
            ],
            host_ip="192.168.1.250",
            gateways=set(),
        )

        selected, reason = livox_mid360._select_active_scan_lidar(candidates)

        self.assertEqual(selected["ip"], "192.168.1.148")
        self.assertEqual(reason, "unique_high_ttl")

    def test_multiple_unknown_high_ttl_candidates_are_ambiguous(self):
        candidates = livox_mid360._score_candidates(
            [
                {
                    "ip": "192.168.1.148",
                    "mac": "4c:43:f6:ea:e6:e6",
                    "state": "REACHABLE",
                    "ttl": 255,
                },
                {
                    "ip": "192.168.1.149",
                    "mac": "02:00:00:00:00:01",
                    "state": "REACHABLE",
                    "ttl": 255,
                },
            ],
            host_ip="192.168.1.250",
            gateways=set(),
        )

        selected, reason = livox_mid360._select_active_scan_lidar(candidates)

        self.assertIsNone(selected)
        self.assertEqual(reason, "ambiguous")

    def test_configured_ip_candidate_is_selected(self):
        candidates = livox_mid360._score_candidates(
            [
                {
                    "ip": "192.168.1.148",
                    "mac": "4c:43:f6:ea:e6:e6",
                    "state": "STALE",
                    "ttl": None,
                }
            ],
            host_ip="192.168.1.250",
            gateways=set(),
            configured_ips={"192.168.1.148"},
        )

        selected, reason = livox_mid360._select_active_scan_lidar(candidates)

        self.assertEqual(selected["ip"], "192.168.1.148")
        self.assertEqual(reason, "config_ip")

    def test_default_config_prefers_mid360s_config(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            config_dir = Path(tmpdir) / "sunray_map/src/livox_ros_driver2/config"
            config_dir.mkdir(parents=True)
            (config_dir / "MID360_config.json").write_text("{}", encoding="utf-8")
            mid360s = config_dir / "MID360s_config.json"
            mid360s.write_text("{}", encoding="utf-8")

            with patch.object(livox_mid360.Path, "home", return_value=Path(tmpdir)):
                self.assertEqual(livox_mid360.resolve_config_paths(None), [mid360s])

    def test_discovery_finds_mid360s_config(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            config_dir = root / "ws/src/livox_ros_driver2/config"
            config_dir.mkdir(parents=True)
            mid360s = config_dir / "MID360s_config.json"
            mid360s.write_text("{}", encoding="utf-8")

            with patch.object(livox_mid360.Path, "cwd", return_value=root):
                with patch.object(livox_mid360.Path, "home", return_value=root):
                    self.assertEqual(livox_mid360.discover_mid360_config_paths(), [mid360s])

    def test_temporary_lidar_subnet_address_adds_and_removes_secondary_ip(self):
        commands = []

        def fake_run_command(command, timeout=None):
            commands.append(command)
            return 0, ""

        with patch.object(livox_mid360, "iface_ipv4", return_value="192.168.20.64"):
            with patch.object(livox_mid360, "iface_has_ipv4", return_value=False):
                with patch.object(livox_mid360, "run_command", side_effect=fake_run_command):
                    with patch.object(livox_mid360.os, "geteuid", return_value=1000):
                        with livox_mid360.temporary_lidar_subnet_address("wlan0", sudo=True) as state:
                            self.assertEqual(state, ("192.168.1.250", True))

        self.assertEqual(commands[0][:5], ["sudo", "-n", "ip", "addr", "add"])
        self.assertEqual(commands[0][5:8], ["192.168.1.250/24", "dev", "wlan0"])
        self.assertEqual(commands[1][:5], ["sudo", "-n", "ip", "addr", "del"])
        self.assertEqual(commands[1][5:8], ["192.168.1.250/24", "dev", "wlan0"])

    def test_temporary_lidar_subnet_address_skips_without_sudo(self):
        with patch.object(livox_mid360, "iface_ipv4", return_value="192.168.20.64"):
            with livox_mid360.temporary_lidar_subnet_address("wlan0", sudo=False) as state:
                self.assertEqual(state, (None, False))

    def test_candidate_ifaces_includes_non_ipv4_scan_interfaces(self):
        with patch.object(livox_mid360, "read_config_lidar_ip", return_value=None):
            with patch.object(
                livox_mid360,
                "list_ipv4_interfaces",
                return_value=[("wlan0", "192.168.20.64", 24)],
            ):
                with patch.object(livox_mid360, "list_scan_interfaces", return_value=["eth0", "wlan0"]):
                    with patch.object(livox_mid360, "list_ethernet_interfaces", return_value=["eth0"]):
                        self.assertEqual(
                            livox_mid360.candidate_ifaces("auto", [Path("/tmp/missing.json")]),
                            ["eth0", "wlan0"],
                        )

    def test_scan_interface_priority_prefers_wired_interfaces(self):
        interfaces = ["wlan0", "usb0", "enp89s0"]

        self.assertEqual(sorted(interfaces, key=livox_mid360.scan_interface_priority), ["enp89s0", "wlan0", "usb0"])


if __name__ == "__main__":
    unittest.main()
