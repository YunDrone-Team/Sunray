import os
import re
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
SUNRAY_CONTROL_LAUNCH_DIR = os.path.abspath(
    os.path.join(PACKAGE_ROOT, "..", "sunray_uav_control", "launch")
)
DEFAULT_LAUNCH_FILES = [
    os.path.join(SUNRAY_CONTROL_LAUNCH_DIR, "external_fusion.launch"),
    os.path.join(SUNRAY_CONTROL_LAUNCH_DIR, "sunray_vrpn.launch"),
]


@dataclass(frozen=True)
class VrpnTarget:
    key: str
    label: str
    launch_files: Sequence[str]


TARGETS = (
    VrpnTarget("external", "external_fusion.launch", (DEFAULT_LAUNCH_FILES[0],)),
    VrpnTarget("vrpn", "sunray_vrpn.launch", (DEFAULT_LAUNCH_FILES[1],)),
    VrpnTarget("all", "all", tuple(DEFAULT_LAUNCH_FILES)),
)
TARGET_BY_KEY = {target.key: target for target in TARGETS}


def read_launch_file(path: str) -> str:
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def write_launch_file(path: str, content: str) -> None:
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(content)


def extract_server(content: str) -> str:
    match = re.search(r'<arg\s+name="server"\s+default="([^"]+)"\s*/>', content)
    if not match:
        raise RuntimeError('cannot find <arg name="server" .../> in launch file')
    return match.group(1)


def replace_server(content: str, new_server: str) -> str:
    pattern = r'(<arg\s+name="server"\s+default=")([^"]+)("\s*/>)'
    replacement = rf"\g<1>{new_server}\g<3>"
    new_content, count = re.subn(pattern, replacement, content, count=1)
    if count != 1:
        raise RuntimeError('failed to update <arg name="server" .../> in launch file')
    return new_content


def load_launch_contents(
    launch_files: Sequence[str] = None,
) -> Dict[str, Tuple[str, str]]:
    contents = {}
    for launch_file in launch_files or DEFAULT_LAUNCH_FILES:
        if not os.path.isfile(launch_file):
            raise RuntimeError(f"launch file not found: {launch_file}")
        content = read_launch_file(launch_file)
        contents[launch_file] = (content, extract_server(content))
    return contents


def selected_contents(
    all_contents: Dict[str, Tuple[str, str]],
    target_key: str,
) -> Dict[str, Tuple[str, str]]:
    target = TARGET_BY_KEY[target_key]
    return {
        launch_file: all_contents[launch_file]
        for launch_file in target.launch_files
    }


def write_updates(
    contents: Dict[str, Tuple[str, str]],
    new_server: str,
) -> List[str]:
    updated_paths = []
    for launch_file, (content, _) in contents.items():
        updated = replace_server(content, new_server)
        write_launch_file(launch_file, updated)
        updated_paths.append(launch_file)
    return updated_paths


def target_text(contents: Dict[str, Tuple[str, str]]) -> str:
    return "、".join(os.path.basename(path) for path in contents)
