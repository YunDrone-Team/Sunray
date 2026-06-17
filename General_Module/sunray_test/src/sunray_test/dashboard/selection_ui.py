import re
import sys
from typing import Dict, List, Sequence

from sunray_test.dashboard.model import DashboardModel
from sunray_test.dashboard.session import DashboardRequest, DashboardSession
from sunray_test.dashboard.types import TestItem


def format_hardware_refs(item: TestItem, item_by_id: Dict[str, TestItem] = None) -> str:
    refs: List[str] = []
    for hardware_id in item.required_hardware:
        hardware_item = item_by_id.get(hardware_id) if item_by_id else None
        if hardware_item:
            refs.append(f"{hardware_id}({hardware_item.name})")
        else:
            refs.append(hardware_id)
    return ",".join(refs)


def item_hints(item: TestItem, item_by_id: Dict[str, TestItem] = None) -> List[str]:
    hints: List[str] = []
    if item.requires_airborne:
        hints.append("飞行")
    if item.required_hardware:
        hints.append("依赖硬件: " + format_hardware_refs(item, item_by_id))
    if item.sim_only:
        hints.append("仅仿真")
    if item.exp_only:
        hints.append("仅实机")
    hints.extend(item.tags)
    return hints


def format_item_line(
    index: int,
    item: TestItem,
    selected: bool = False,
    auto_added: bool = False,
    item_by_id: Dict[str, TestItem] = None,
) -> str:
    hints = item_hints(item, item_by_id)
    if auto_added:
        hints.append("自动补齐")
    suffix = f" [{' / '.join(hints)}]" if hints else ""
    marker = "*" if selected else " "
    return f"{index:>2}. {marker} {item.item_id:<16s} {item.name}{suffix}"


def format_item_names(model: DashboardModel, item_ids: Sequence[str]) -> str:
    if not item_ids:
        return "-"
    return ", ".join(f"{item_id}({model.item_by_id[item_id].name})" for item_id in item_ids)


def print_items(model: DashboardModel) -> None:
    print("硬件测试项目：", flush=True)
    for index, item in enumerate(model.items_by_group("hardware"), 1):
        print(format_item_line(index, item, item_by_id=model.item_by_id), flush=True)
    print("\n功能测试项目：", flush=True)
    for index, item in enumerate(model.items_by_group("function"), 1):
        print(format_item_line(index, item, item_by_id=model.item_by_id), flush=True)


def print_dashboard_header(session: DashboardSession, no_bringup: bool = False) -> None:
    print("\n=== Sunray Test Dashboard ===", flush=True)
    print(f"环境: {session.environment}", flush=True)
    print(f"输出目录: {session.output_dir}", flush=True)
    if no_bringup:
        print("启动链路: 已关闭 (--no-bringup)", flush=True)


def tokenize_selection(raw: str) -> List[str]:
    return [token for token in re.split(r"[\s,，]+", raw.strip()) if token]


def parse_selection_range(
    token: str,
    candidates: Sequence[TestItem],
    start_index: int = 1,
) -> List[str]:
    match = re.fullmatch(r"(\d+)-(\d+)", token)
    if not match:
        return []
    start = int(match.group(1))
    end = int(match.group(2))
    min_index = start_index
    max_index = start_index + len(candidates) - 1
    if start < min_index or end < min_index or start > max_index or end > max_index:
        raise SystemExit(f"无效测试项目范围: {token}")
    step = 1 if start <= end else -1
    return [candidates[index - start_index].item_id for index in range(start, end + step, step)]


def parse_selection_tokens(
    raw: str,
    candidates: Sequence[TestItem],
    default_item_ids: Sequence[str],
    start_index: int = 1,
) -> List[str]:
    lowered = raw.strip().lower()
    if not lowered:
        return list(default_item_ids)
    if lowered in {"none", "no", "n", "0"}:
        return []
    if lowered in {"all", "*"}:
        return [item.item_id for item in candidates]
    if lowered in {"default", "d"}:
        return list(default_item_ids)

    selected: List[str] = []
    by_id = {item.item_id: item for item in candidates}
    by_case = {
        str(item.step.get("case", "")).strip(): item
        for item in candidates
        if str(item.step.get("case", "")).strip()
    }
    min_index = start_index
    max_index = start_index + len(candidates) - 1
    for token in tokenize_selection(raw):
        range_item_ids = parse_selection_range(token, candidates, start_index=start_index)
        if range_item_ids:
            selected.extend(range_item_ids)
            continue
        if token.isdigit():
            index = int(token)
            if min_index <= index <= max_index:
                selected.append(candidates[index - start_index].item_id)
                continue
        if token in by_id:
            selected.append(token)
            continue
        if token in by_case:
            selected.append(by_case[token].item_id)
            continue
        raise SystemExit(f"无效测试项目: {token}")
    return dedupe(selected)


def dedupe(values: Sequence[str]) -> List[str]:
    result: List[str] = []
    for value in values:
        if value not in result:
            result.append(value)
    return result


def print_selection_group(
    model: DashboardModel,
    title: str,
    items: Sequence[TestItem],
    selected_item_ids: Sequence[str] = (),
    start_index: int = 1,
) -> None:
    selected = set(selected_item_ids)
    print(f"\n{title}", flush=True)
    for index, item in enumerate(items, start_index):
        print(
            format_item_line(
                index,
                item,
                selected=item.item_id in selected,
                item_by_id=model.item_by_id,
            ),
            flush=True,
        )


def print_selection_help() -> None:
    print(
        "输入编号、编号范围或 ID，多个用逗号分隔；all=全选，none=不选，回车=默认，?=重打列表，* 表示默认选择。",
        flush=True,
    )


def prompt_item_selection(
    model: DashboardModel,
    title: str,
    items: Sequence[TestItem],
    default_item_ids: Sequence[str],
    start_index: int = 1,
) -> List[str]:
    print_selection_group(model, title, items, default_item_ids, start_index=start_index)
    default_text = ",".join(default_item_ids) if default_item_ids else "none"
    print_selection_help()
    while True:
        raw = input(f"选择 [{default_text}]: ")
        if raw.strip().lower() in {"?", "h", "help", "list", "l"}:
            print_selection_group(model, title, items, default_item_ids, start_index=start_index)
            print_selection_help()
            continue
        try:
            return parse_selection_tokens(raw, items, default_item_ids, start_index=start_index)
        except SystemExit as exc:
            print(str(exc), flush=True)
            print("请重新输入。", flush=True)


def describe_selected_items(model: DashboardModel, label: str, item_ids: Sequence[str]) -> None:
    if not item_ids:
        print(f"{label}: none", flush=True)
        return
    names = [f"{item_id}({model.item_by_id[item_id].name})" for item_id in item_ids]
    print(f"{label}: {', '.join(names)}", flush=True)


def print_dependency_preview(model: DashboardModel, requested_item_ids: Sequence[str]) -> None:
    if not requested_item_ids:
        return
    selection = model.resolve_item_dependencies(requested_item_ids)
    dependency_descriptions = model.dependency_descriptions(selection.requested_item_ids)
    if not dependency_descriptions and not selection.auto_added_item_ids:
        return

    print("\n依赖预览", flush=True)
    for description in dependency_descriptions:
        print(f"  - {description}", flush=True)
    if selection.auto_added_item_ids:
        print(f"将自动补齐: {format_item_names(model, selection.auto_added_item_ids)}", flush=True)


def select_dashboard_interactively(
    session: DashboardSession,
    no_bringup: bool = False,
) -> DashboardRequest:
    if not sys.stdin.isatty():
        raise SystemExit("当前不是交互终端，请使用 --items 指定测试项目")
    model = session.model
    print_dashboard_header(session, no_bringup=no_bringup)

    hardware_items = model.items_by_group("hardware")
    function_items = model.items_by_group("function")
    default_hardware = [
        item_id
        for item_id in model.default_items
        if item_id in model.item_by_id and model.item_by_id[item_id].group == "hardware"
    ]

    selected_hardware = prompt_item_selection(
        model=model,
        title="硬件测试",
        items=hardware_items,
        default_item_ids=default_hardware,
        start_index=1,
    )
    selected_functions = prompt_item_selection(
        model=model,
        title="功能测试",
        items=function_items,
        default_item_ids=[],
        start_index=len(hardware_items) + 1,
    )
    requested_item_ids = dedupe([*selected_hardware, *selected_functions])
    print_dependency_preview(model, requested_item_ids)
    selection = model.resolve_item_dependencies(requested_item_ids) if requested_item_ids else None

    print("\n选择结果", flush=True)
    describe_selected_items(model, "硬件测试", selected_hardware)
    describe_selected_items(model, "功能测试", selected_functions)
    if selection and selection.auto_added_item_ids:
        describe_selected_items(model, "将自动补齐", selection.auto_added_item_ids)
    return DashboardRequest(item_ids=requested_item_ids, source="manual")
