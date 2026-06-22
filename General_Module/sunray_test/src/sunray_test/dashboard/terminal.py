import os
import shlex
import shutil
import subprocess
import tempfile
from typing import Any, Dict, List, Sequence


TERMINAL_MISSING_MESSAGE = (
    "gnome-terminal not found; rerun with --no-bringup and start dependencies manually"
)


def compose_bash_command(command: str, delay_s: float = 0.0, hold_open: bool = True) -> str:
    parts: List[str] = []
    if delay_s > 0:
        parts.append(f"sleep {delay_s}")
    if hold_open:
        parts.append(
            "{ "
            + command
            + '; status=$?; echo ""; echo "[sunray_test] tab exited with status ${status}. '
            + 'Press Ctrl+D to close."; exec bash; }'
        )
    else:
        parts.append(command)
    return "; ".join(parts)


def terminal_available() -> bool:
    return bool(shutil.which("gnome-terminal"))


def ensure_terminal_available() -> None:
    if not terminal_available():
        raise SystemExit(TERMINAL_MISSING_MESSAGE)


def key_file_escape(value: str) -> str:
    replacements = {
        "\\": "\\\\",
        "\n": "\\n",
        "\r": "\\r",
        "\t": "\\t",
        "\b": "\\b",
        "\f": "\\f",
    }
    return "".join(replacements.get(char, char) for char in value)


def c_string_escape(value: str) -> str:
    replacements = {
        "\\": "\\\\",
        "\n": "\\n",
        "\r": "\\r",
        "\t": "\\t",
        "\b": "\\b",
        "\f": "\\f",
        '"': '\\"',
    }
    return "".join(replacements.get(char, char) for char in value)


def compressed_key_file_value(value: str) -> str:
    return key_file_escape(c_string_escape(value))


def write_gnome_terminal_config(title: str, tabs: Sequence[Dict[str, Any]]) -> str:
    tab_names = [f"Terminal{index}" for index, _ in enumerate(tabs)]
    lines = [
        "[GNOME Terminal Configuration]",
        "Version=1",
        "CompatVersion=1",
        "Windows=Window0;",
        "",
        "[Window0]",
        f"Terminals={';'.join(tab_names)};",
        "ActiveTerminal=Terminal0",
        "",
    ]
    for index, tab in enumerate(tabs):
        lines.extend(build_terminal_config_block(index, title, tab))

    handle = tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        prefix="sunray_test_terminal_",
        suffix=".conf",
        delete=False,
    )
    with handle:
        handle.write("\n".join(lines))
    return handle.name


def build_terminal_config_block(index: int, title: str, tab: Dict[str, Any]) -> List[str]:
    tab_title = str(tab.get("title") or title)
    shell_command = compose_bash_command(
        str(tab["command"]),
        float(tab.get("delay_s", 0.0)),
        bool(tab.get("hold_open", True)),
    )
    command = "bash -lc " + shlex.quote(shell_command)
    return [
        f"[Terminal{index}]",
        f"Title={key_file_escape(tab_title)}",
        f"WorkingDirectory={compressed_key_file_value(os.getcwd())}",
        f"Command={compressed_key_file_value(command)}",
        "",
    ]


def launch_terminal_window(title: str, tabs: Sequence[Dict[str, Any]]) -> None:
    if not tabs:
        return
    if not shutil.which("gnome-terminal"):
        raise SystemExit(TERMINAL_MISSING_MESSAGE)

    config_path = write_gnome_terminal_config(title, tabs)
    subprocess.Popen(["gnome-terminal", f"--load-config={config_path}"])
