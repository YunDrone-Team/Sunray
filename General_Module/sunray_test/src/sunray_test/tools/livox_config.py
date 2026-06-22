import json
from pathlib import Path
from typing import Any, Optional


def read_lidar_ip(path: Path) -> Optional[str]:
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


def update_lidar_ip(path: Path, lidar_ip: str) -> None:
    data = json.loads(path.read_text(encoding="utf-8"))
    lidar_configs = data.setdefault("lidar_configs", [{}])
    if not lidar_configs:
        lidar_configs.append({})
    first_config: Any = lidar_configs[0]
    if not isinstance(first_config, dict):
        first_config = {}
        lidar_configs[0] = first_config
    first_config["ip"] = lidar_ip
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
