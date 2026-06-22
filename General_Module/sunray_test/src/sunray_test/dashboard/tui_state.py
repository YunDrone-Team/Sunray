from dataclasses import dataclass, field
from time import monotonic
from typing import Any, Dict, List, Sequence

from sunray_test.dashboard.model import DashboardModel, dedupe
from sunray_test.dashboard.session import DashboardRequest
from sunray_test.dashboard.tools import DASHBOARD_TOOLS, DashboardTool
from sunray_test.dashboard.types import DashboardPlan, TestItem


PANE_HARDWARE = "hardware"
PANE_FUNCTION = "function"
PANE_PREVIEW = "preview"
PANE_PARAMS = "params"
PANE_TOOLS = "tools"
PANES = (PANE_FUNCTION, PANE_HARDWARE, PANE_TOOLS)
EXTERNAL_SOURCE_REPEAT_WINDOW_S = 0.14
EXTERNAL_SOURCE_REPEAT_COUNT_TO_OPEN = 3
EXTERNAL_SOURCE_QUICK_VALUES = (0, 3)
PROFILE_OVERRIDE_CYCLE = ("", "sunray150_basic", "sunray150_lidar")

ACTION_START = "start"
ACTION_CANCEL = "cancel"
ACTION_TOOL = "tool"
ACTION_NONE = ""


@dataclass
class TuiState:
    model: DashboardModel
    environment: str
    uav_id: int
    external_source_override: int = None
    profile_override: str = ""
    record_rosbag: bool = True
    continue_on_failure: bool = False
    no_bringup: bool = False
    active_pane: str = PANE_FUNCTION
    hardware_cursor: int = 0
    function_cursor: int = 0
    tool_cursor: int = 0
    preview_scroll: int = 0
    selected_hardware: List[str] = field(default_factory=list)
    selected_functions: List[str] = field(default_factory=list)
    param_overrides: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    editing_params_for: str = ""
    param_cursor: int = 0
    status: str = ""
    active_tool_id: str = ""
    external_source_selector: bool = False
    external_source_cursor: int = 0
    last_external_source_key_at: float = 0.0
    external_source_repeat_count: int = 0
    needs_full_redraw: bool = False

    def __post_init__(self) -> None:
        if not self.selected_hardware:
            self.selected_hardware = [
                item_id
                for item_id in self.model.default_items
                if item_id in self.model.item_by_id
                and self.model.item_by_id[item_id].group == PANE_HARDWARE
            ]

    @property
    def hardware_items(self) -> List[TestItem]:
        return self.model.items_by_group(PANE_HARDWARE)

    @property
    def function_items(self) -> List[TestItem]:
        return self.model.items_by_group(PANE_FUNCTION)

    @property
    def tools(self) -> Sequence[DashboardTool]:
        return DASHBOARD_TOOLS

    @property
    def requested_item_ids(self) -> List[str]:
        return dedupe([*self.selected_hardware, *self.selected_functions])

    def request(self) -> DashboardRequest:
        return DashboardRequest(
            item_ids=self.requested_item_ids,
            source="tui",
            param_overrides=self.param_overrides,
            external_source_override=self.external_source_override,
            profile_override=self.profile_override,
        )

    def plan_cache_key(self) -> tuple:
        overrides = tuple(
            (
                item_id,
                tuple(sorted(values.items())),
            )
            for item_id, values in sorted(self.param_overrides.items())
        )
        return (
            tuple(self.requested_item_ids),
            self.external_source_override,
            self.profile_override,
            self.record_rosbag,
            self.continue_on_failure,
            overrides,
        )

    def profile_override_label(self) -> str:
        if self.profile_override == "sunray150_basic":
            return "基础款"
        if self.profile_override == "sunray150_lidar":
            return "雷达款"
        return "自动"

    def current_external_source(self) -> int:
        return self.model.default_external_source(self.environment, self.external_source_override)

    def current_external_source_option(self) -> Dict[str, Any]:
        return self.model.external_source_option(self.environment, self.current_external_source())

    def external_source_options(self) -> List[Dict[str, Any]]:
        return self.model.external_source_options(self.environment)

    def set_external_source(self, value: int) -> None:
        self.external_source_override = int(value)
        self.status = ""
        self.preview_scroll = 0

    def quick_toggle_external_source(self) -> None:
        options = self.external_source_options()
        available = {int(option["value"]) for option in options}
        quick_values = [value for value in EXTERNAL_SOURCE_QUICK_VALUES if value in available]
        if len(quick_values) < 2:
            self.status = "当前环境缺少 ODOM/MOCAP external_source"
            return
        current = self.current_external_source()
        next_value = quick_values[1] if current == quick_values[0] else quick_values[0]
        self.set_external_source(next_value)

    def open_external_source_selector(self) -> None:
        options = self.external_source_options()
        current = self.current_external_source()
        values = [int(option["value"]) for option in options]
        self.external_source_cursor = values.index(current) if current in values else 0
        self.external_source_selector = True
        self.status = ""

    def close_external_source_selector(self) -> None:
        self.external_source_selector = False
        self.needs_full_redraw = True
        self.status = ""

    def confirm_external_source_selector(self) -> None:
        options = self.external_source_options()
        if not options:
            return
        self.external_source_cursor = wrap_index(self.external_source_cursor, len(options))
        self.external_source_selector = False
        self.needs_full_redraw = True
        self.set_external_source(int(options[self.external_source_cursor]["value"]))

    def move_external_source_cursor(self, delta: int) -> None:
        options = self.external_source_options()
        if options:
            self.external_source_cursor = wrap_index(self.external_source_cursor + delta, len(options))

    def handle_external_source_key(self) -> None:
        now = monotonic()
        repeat_like = (
            self.last_external_source_key_at
            and now - self.last_external_source_key_at <= EXTERNAL_SOURCE_REPEAT_WINDOW_S
        )

        if repeat_like:
            self.last_external_source_key_at = now
            self.external_source_repeat_count += 1
            if self.external_source_repeat_count >= EXTERNAL_SOURCE_REPEAT_COUNT_TO_OPEN:
                self.open_external_source_selector()
                self.last_external_source_key_at = 0.0
                self.external_source_repeat_count = 0
            return

        self.last_external_source_key_at = now
        self.external_source_repeat_count = 1

    def flush_pending_external_source_key(self, force: bool = False) -> None:
        if not self.last_external_source_key_at:
            return
        if not force and monotonic() - self.last_external_source_key_at <= EXTERNAL_SOURCE_REPEAT_WINDOW_S:
            return
        if self.external_source_repeat_count == 1:
            self.quick_toggle_external_source()
        self.reset_external_source_repeat_state()

    def reset_external_source_repeat_state(self) -> None:
        self.last_external_source_key_at = 0.0
        self.external_source_repeat_count = 0

    def maybe_reset_external_source_repeat_state(self, key_name: str) -> None:
        if key_name not in {"e", "E"}:
            self.flush_pending_external_source_key(force=True)
            self.reset_external_source_repeat_state()

    def cycle_external_source(self, direction: int = 1) -> None:
        options = self.model.external_source_options(self.environment)
        if len(options) <= 1:
            self.status = "当前环境只有一个 external_source"
            return
        current = self.current_external_source()
        values = [int(option["value"]) for option in options]
        index = values.index(current) if current in values else 0
        next_option = options[(index + direction) % len(options)]
        self.external_source_override = int(next_option["value"])
        self.status = f"external_source: {next_option['label']}"
        self.preview_scroll = 0

    def cycle_profile_override(self) -> None:
        current = self.profile_override if self.profile_override in PROFILE_OVERRIDE_CYCLE else ""
        index = PROFILE_OVERRIDE_CYCLE.index(current)
        self.profile_override = PROFILE_OVERRIDE_CYCLE[(index + 1) % len(PROFILE_OVERRIDE_CYCLE)]
        self.status = ""
        self.preview_scroll = 0

    def handle_external_source_selector_key(self, key_name: str) -> str:
        if key_name in {"esc", "tab", "q", "Q"}:
            self.close_external_source_selector()
            self.reset_external_source_repeat_state()
            return ACTION_NONE
        if key_name in {"up"}:
            self.move_external_source_cursor(-1)
            return ACTION_NONE
        if key_name in {"down"}:
            self.move_external_source_cursor(1)
            return ACTION_NONE
        if key_name in {"enter", "\n", "\r", " ", "space"}:
            self.confirm_external_source_selector()
            self.reset_external_source_repeat_state()
            return ACTION_NONE
        return ACTION_NONE

    def move_pane(self, direction: int) -> None:
        index = PANES.index(self.active_pane)
        self.active_pane = PANES[(index + direction) % len(PANES)]

    def move_cursor(self, delta: int) -> None:
        if self.editing_params:
            specs = self.current_param_specs()
            if specs:
                self.param_cursor = wrap_index(self.param_cursor + delta, len(specs))
            return
        if self.active_pane == PANE_PREVIEW:
            self.preview_scroll = max(0, self.preview_scroll + delta)
            return
        if self.active_pane == PANE_TOOLS:
            if self.tools:
                self.tool_cursor = wrap_index(self.tool_cursor + delta, len(self.tools))
            return

        items = self.current_items()
        if not items:
            return
        cursor = wrap_index(self.current_item_cursor() + delta, len(items))
        self.set_current_item_cursor(cursor)

    def current_items(self) -> List[TestItem]:
        if self.active_pane == PANE_HARDWARE:
            return self.hardware_items
        if self.active_pane == PANE_FUNCTION:
            return self.function_items
        return []

    def current_item(self) -> TestItem:
        items = self.current_items()
        if not items:
            return None
        return items[self.current_item_cursor()]

    def current_item_cursor(self) -> int:
        if self.active_pane == PANE_HARDWARE:
            return self.hardware_cursor
        if self.active_pane == PANE_FUNCTION:
            return self.function_cursor
        return 0

    def set_current_item_cursor(self, value: int) -> None:
        if self.active_pane == PANE_HARDWARE:
            self.hardware_cursor = value
        elif self.active_pane == PANE_FUNCTION:
            self.function_cursor = value

    def current_tool(self) -> DashboardTool:
        if not self.tools:
            return None
        self.tool_cursor = clamp(self.tool_cursor, 0, len(self.tools) - 1)
        return self.tools[self.tool_cursor]

    @property
    def tool_active(self) -> bool:
        return bool(self.active_tool_id)

    def enter_tool(self) -> None:
        tool = self.current_tool()
        if tool is None:
            return
        self.active_tool_id = tool.tool_id
        self.status = ""

    def exit_tool(self) -> None:
        self.active_tool_id = ""
        self.status = ""

    @property
    def editing_params(self) -> bool:
        return self.active_pane == PANE_PARAMS and bool(self.editing_params_for)

    def current_param_specs(self) -> List[Dict[str, Any]]:
        if not self.editing_params_for:
            return []
        return self.model.item_param_specs(self.editing_params_for)

    def current_param_spec(self) -> Dict[str, Any]:
        specs = self.current_param_specs()
        if not specs:
            return None
        self.param_cursor = clamp(self.param_cursor, 0, len(specs) - 1)
        return specs[self.param_cursor]

    def enter_param_editor(self) -> None:
        item = self.current_item()
        if item is None:
            return
        if not self.model.item_param_specs(item.item_id):
            self.status = f"{item.name} 没有可编辑参数"
            return
        self.editing_params_for = item.item_id
        self.param_cursor = 0
        self.active_pane = PANE_PARAMS
        self.status = ""

    def exit_param_editor(self) -> None:
        if self.editing_params_for in self.model.item_by_id:
            self.active_pane = self.model.item_by_id[self.editing_params_for].group
        else:
            self.active_pane = PANE_FUNCTION
        self.editing_params_for = ""
        self.param_cursor = 0

    def toggle_current(self) -> None:
        item = self.current_item()
        if item is None:
            return
        if item.group == PANE_HARDWARE:
            self.selected_hardware = toggle_item(self.selected_hardware, item.item_id)
        elif item.group == PANE_FUNCTION:
            self.selected_functions = toggle_item(self.selected_functions, item.item_id)
        self.preview_scroll = 0

    def clear_current_group(self) -> None:
        if self.active_pane == PANE_HARDWARE:
            self.selected_hardware = []
        elif self.active_pane == PANE_FUNCTION:
            self.selected_functions = []
        self.preview_scroll = 0

    def select_all_current_group(self) -> None:
        if self.active_pane == PANE_HARDWARE:
            self.selected_hardware = [item.item_id for item in self.hardware_items]
        elif self.active_pane == PANE_FUNCTION:
            self.selected_functions = [item.item_id for item in self.function_items]
        self.preview_scroll = 0

    def get_param_value(self, item_id: str, spec: Dict[str, Any]) -> Any:
        path = str(spec.get("path", ""))
        if item_id in self.param_overrides and path in self.param_overrides[item_id]:
            return self.param_overrides[item_id][path]
        return spec.get("default")

    def set_param_value(self, item_id: str, spec: Dict[str, Any], value: Any) -> None:
        path = str(spec.get("path", ""))
        normalized = self.model.normalize_param_value(spec, value)
        self.param_overrides.setdefault(item_id, {})[path] = normalized
        if normalized == spec.get("default"):
            self.param_overrides[item_id].pop(path, None)
            if not self.param_overrides[item_id]:
                self.param_overrides.pop(item_id, None)
        self.preview_scroll = 0

    def reset_current_param(self) -> None:
        spec = self.current_param_spec()
        if not spec:
            return
        item_id = self.editing_params_for
        path = str(spec.get("path", ""))
        if item_id in self.param_overrides:
            self.param_overrides[item_id].pop(path, None)
            if not self.param_overrides[item_id]:
                self.param_overrides.pop(item_id, None)
        self.preview_scroll = 0

    def adjust_current_param(self, direction: int) -> None:
        spec = self.current_param_spec()
        if not spec:
            return
        item_id = self.editing_params_for
        current = self.get_param_value(item_id, spec)
        param_type = str(spec.get("type", "string"))
        if param_type == "bool":
            self.set_param_value(item_id, spec, not bool(current))
            return
        if param_type == "enum":
            options = [str(option) for option in spec.get("options", [])]
            if not options:
                return
            current_text = str(current)
            index = options.index(current_text) if current_text in options else 0
            self.set_param_value(item_id, spec, options[(index + direction) % len(options)])
            return
        if param_type in {"float", "int"}:
            step = spec.get("step", 1 if param_type == "int" else 0.1)
            value = current if current is not None else spec.get("default", 0)
            self.set_param_value(item_id, spec, value + (step * direction))

    def build_plan(self) -> DashboardPlan:
        return self.model.build_plan(
            requested_item_ids=self.requested_item_ids,
            environment=self.environment,
            uav_id=self.uav_id,
            external_source_override=self.external_source_override,
            record_rosbag=self.record_rosbag,
            continue_on_failure=self.continue_on_failure,
            param_overrides=self.param_overrides,
            profile_override=self.profile_override,
        )

    def handle_key_name(self, key_name: str) -> str:
        if self.external_source_selector:
            return self.handle_external_source_selector_key(key_name)

        self.maybe_reset_external_source_repeat_state(key_name)

        if self.tool_active:
            return self.handle_tool_key_name(key_name)

        if key_name in {"q", "Q"}:
            if self.editing_params:
                self.exit_param_editor()
                return ACTION_NONE
            return ACTION_CANCEL
        if key_name in {"esc"}:
            if self.editing_params:
                self.exit_param_editor()
                return ACTION_NONE
            return ACTION_CANCEL
        if key_name in {"enter", "\n", "\r"}:
            if self.editing_params:
                return ACTION_NONE
            if self.active_pane == PANE_TOOLS:
                return ACTION_TOOL
            return ACTION_START
        if key_name in {"tab"}:
            if self.editing_params:
                self.exit_param_editor()
            elif self.active_pane in {PANE_FUNCTION, PANE_HARDWARE}:
                self.enter_param_editor()
            return ACTION_NONE
        if key_name in {"e", "E"}:
            if self.editing_params:
                return ACTION_NONE
            self.handle_external_source_key()
            return ACTION_NONE
        if key_name in {"m", "M"}:
            if self.editing_params:
                return ACTION_NONE
            self.cycle_profile_override()
            return ACTION_NONE
        if key_name in {"right"}:
            if self.editing_params:
                self.adjust_current_param(1)
                return ACTION_NONE
            self.move_pane(1)
            return ACTION_NONE
        if key_name in {"left"}:
            if self.editing_params:
                self.adjust_current_param(-1)
                return ACTION_NONE
            self.move_pane(-1)
            return ACTION_NONE
        if key_name in {"up"}:
            self.move_cursor(-1)
            return ACTION_NONE
        if key_name in {"down"}:
            self.move_cursor(1)
            return ACTION_NONE
        if key_name in {" ", "space"}:
            if self.editing_params:
                self.adjust_current_param(1)
                return ACTION_NONE
            self.toggle_current()
            return ACTION_NONE
        if key_name in {"a", "A"}:
            if self.editing_params:
                return ACTION_NONE
            self.select_all_current_group()
            return ACTION_NONE
        if key_name in {"c", "C"}:
            if self.editing_params:
                return ACTION_NONE
            self.clear_current_group()
            return ACTION_NONE
        if key_name in {"backspace"}:
            if self.editing_params:
                self.reset_current_param()
            return ACTION_NONE
        return ACTION_NONE

    def handle_tool_key_name(self, key_name: str) -> str:
        if key_name in {"q", "Q", "tab"}:
            self.exit_tool()
            return ACTION_NONE
        return ACTION_TOOL


def toggle_item(values: Sequence[str], item_id: str) -> List[str]:
    if item_id in values:
        return [value for value in values if value != item_id]
    return [*values, item_id]


def clamp(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, value))


def wrap_index(value: int, length: int) -> int:
    if length <= 0:
        return 0
    return value % length
