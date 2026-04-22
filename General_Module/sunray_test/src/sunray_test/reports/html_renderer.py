from typing import Any, Dict

from sunray_test.reports.renderers.cases import render_case_rows
from sunray_test.reports.renderers.common import escape, format_duration
from sunray_test.reports.renderers.flight import render_artifacts, render_config_snapshot
from sunray_test.reports.renderers.styles import REPORT_STYLES
from sunray_test.reports.renderers.summary import (
    render_report_meta,
    render_score_cards,
    render_stage_timeline,
    render_summary_cards,
    render_watermark_layer,
)


def render_html(payload: Dict[str, Any]) -> str:
    run_info = payload["run_info"]
    summary = payload["summary"]
    artifacts = payload.get("artifacts", {})
    config = payload.get("config", {})
    cases = payload.get("cases", [])

    total = max(int(summary.get("total", 0)), 0)
    passed = max(int(summary.get("pass", 0)), 0)
    pass_rate = (passed / total * 100.0) if total else 0.0
    duration = format_duration(run_info.get("started_at"), run_info.get("finished_at"))

    grade_thresholds = payload.get("flight_metrics", {}).get("scores", {}).get("grade_thresholds", [])
    flight_sections = payload.get("flight_metrics", {}).get("sections", [])
    case_rows_html = render_case_rows(cases, flight_sections, grade_thresholds)
    timeline_html = render_stage_timeline(payload)
    meta_html = render_report_meta(run_info, duration)
    summary_card_html = render_summary_cards(summary, pass_rate)
    score_cards_html = render_score_cards(payload)
    flight_errors = payload.get("flight_metrics", {}).get("errors", [])
    flight_errors_html = ""
    if flight_errors:
        error_items = "".join(f"<li>{escape(item)}</li>" for item in flight_errors)
        flight_errors_html = (
            '<section class="section">'
            '<div class="section-header"><h2 class="section-title">分析提示</h2></div>'
            '<div class="section-body">'
            f'<ul class="plain-list">{error_items}</ul>'
            "</div></section>"
        )
    filtered_config = {key: config.get(key) for key in ("defaults", "topics", "missions") if key in config}
    config_snapshot_html = render_config_snapshot(filtered_config)
    artifacts_html = render_artifacts(artifacts)
    watermark_html = render_watermark_layer("云纵科技")

    return f"""<html class="primary-set">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>{escape(run_info['report_title'])}</title>
  <style>
{REPORT_STYLES}
  </style>
</head>
<body>
  {watermark_html}
  <div class="container">
    <section class="hero">
      <div class="eyebrow">Sunray Test Report</div>
      <h1>{escape(run_info['report_title'])}</h1>
      <div class="hero-subtitle">自动化测试执行结果总览，包含基础元数据、执行汇总、阶段轨迹、用例明细、产物路径与配置快照。</div>
    </section>

    <section class="section">
      <div class="section-header">
        <h2 class="section-title">执行概览</h2>
      </div>
      <div class="section-body">
        <div class="summary-grid">
          {summary_card_html}
        </div>
      </div>
    </section>

    <section class="section">
      <div class="section-header">
        <h2 class="section-title">基础信息</h2>
      </div>
      <div class="section-body">
        <div class="meta-grid">
          {meta_html}
        </div>
      </div>
    </section>

    <section class="section">
      <div class="section-header">
        <h2 class="section-title">阶段轨迹</h2>
      </div>
      <div class="section-body">
        {timeline_html}
      </div>
    </section>

    {score_cards_html}

    <section class="section">
      <div class="section-header">
        <h2 class="section-title">用例明细</h2>
      </div>
      <div class="section-body table-wrap">
        <table>
          <thead>
            <tr>
              <th>#</th>
              <th>Case</th>
              <th>Category</th>
              <th>Result</th>
              <th>Score</th>
              <th>Detail</th>
              <th>Duration</th>
            </tr>
          </thead>
          <tbody>
            {case_rows_html or '<tr><td colspan="7" class="empty-block">没有用例结果数据</td></tr>'}
          </tbody>
        </table>
      </div>
    </section>

    {flight_errors_html}

    <section class="section">
      <div class="section-header">
        <h2 class="section-title">配置快照</h2>
      </div>
      <div class="section-body">
        {config_snapshot_html}
      </div>
    </section>

    <section class="section">
      <div class="section-header">
        <h2 class="section-title">产物信息</h2>
      </div>
      <div class="section-body">
        {artifacts_html}
      </div>
    </section>
  </div>
  <script>
  document.querySelectorAll('.case-expand-details').forEach(function(d){{
    var row=document.getElementById(d.id+'-row');
    if(!row)return;
    d.addEventListener('toggle',function(){{
      row.style.display=d.open?'':'none';
    }});
  }});
  </script>
</body>
</html>"""
