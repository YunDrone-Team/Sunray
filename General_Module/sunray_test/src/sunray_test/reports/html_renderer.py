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
    filtered_config = {key: config.get(key) for key in ("defaults", "analysis", "topics", "missions") if key in config}
    config_snapshot_html = render_config_snapshot(filtered_config, str(run_info.get("platform", "")).strip())
    artifacts_html = render_artifacts(artifacts)
    watermark_html = render_watermark_layer("云纵科技")

    return f"""<html class="primary-set">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>{escape(run_info['report_title'])}</title>
  <link rel="icon" type="image/png" href="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAGQAAABkCAYAAABw4pVUAAAACXBIWXMAAA7EAAAOxAGVKw4bAAAEdGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSfvu78nIGlkPSdXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQnPz4KPHg6eG1wbWV0YSB4bWxuczp4PSdhZG9iZTpuczptZXRhLyc+CjxyZGY6UkRGIHhtbG5zOnJkZj0naHR0cDovL3d3dy53My5vcmcvMTk5OS8wMi8yMi1yZGYtc3ludGF4LW5zIyc+CgogPHJkZjpEZXNjcmlwdGlvbiByZGY6YWJvdXQ9JycKICB4bWxuczpBdHRyaWI9J2h0dHA6Ly9ucy5hdHRyaWJ1dGlvbi5jb20vYWRzLzEuMC8nPgogIDxBdHRyaWI6QWRzPgogICA8cmRmOlNlcT4KICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0nUmVzb3VyY2UnPgogICAgIDxBdHRyaWI6Q3JlYXRlZD4yMDI1LTAxLTE1PC9BdHRyaWI6Q3JlYXRlZD4KICAgICA8QXR0cmliOkV4dElkPjIwYjFmYjRiLTBkZDgtNDkwOC1iNmVkLWM3NTc4ZGM5ODc4MzwvQXR0cmliOkV4dElkPgogICAgIDxBdHRyaWI6VG91Y2hUeXBlPjI8L0F0dHJpYjpUb3VjaFR5cGU+CiAgICA8L3JkZjpsaT4KICAgPC9yZGY6U2VxPgogIDwvQXR0cmliOkFkcz4KIDwvcmRmOkRlc2NyaXB0aW9uPgoKIDxyZGY6RGVzY3JpcHRpb24gcmRmOmFib3V0PScnCiAgeG1sbnM6ZGM9J2h0dHA6Ly9wdXJsLm9yZy9kYy9lbGVtZW50cy8xLjEvJz4KICA8ZGM6dGl0bGU+CiAgIDxyZGY6QWx0PgogICAgPHJkZjpsaSB4bWw6bGFuZz0neC1kZWZhdWx0Jz7lr7zoiKrmoI9MT0dP77yIMTAweDEwMO+8iSAtIDE8L3JkZjpsaT4KICAgPC9yZGY6QWx0PgogIDwvZGM6dGl0bGU+CiA8L3JkZjpEZXNjcmlwdGlvbj4KCiA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0nJwogIHhtbG5zOnBkZj0naHR0cDovL25z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" />
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
        <div class="meta-grid report-meta-grid">
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
          <colgroup>
            <col class="case-col-index" />
            <col class="case-col-name" />
            <col class="case-col-category" />
            <col class="case-col-result" />
            <col class="case-col-score" />
            <col class="case-col-detail" />
            <col class="case-col-duration" />
          </colgroup>
          <thead>
            <tr>
              <th>#</th>
              <th>Case</th>
              <th colspan="3">Status</th>
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
      if(d.open){{
        var group=d.getAttribute('data-case-detail-group');
        if(group){{
          document.querySelectorAll('.case-expand-details[data-case-detail-group="'+group+'"]').forEach(function(other){{
            if(other!==d)other.open=false;
          }});
        }}
      }}
      row.style.display=d.open?'':'none';
    }});
  }});
  document.querySelectorAll('.metric-info').forEach(function(info){{
    var tooltip=info.querySelector('.metric-tooltip');
    if(!tooltip)return;
    function positionTooltip(){{
      tooltip.classList.add('tooltip-fixed');
      tooltip.classList.remove('tooltip-below');
      tooltip.style.left='0px';
      tooltip.style.top='0px';
      var infoRect=info.getBoundingClientRect();
      var tipRect=tooltip.getBoundingClientRect();
      var margin=8;
      var left=infoRect.left + infoRect.width / 2 - tipRect.width / 2;
      left=Math.max(margin, Math.min(left, window.innerWidth - tipRect.width - margin));
      var top=infoRect.top - tipRect.height - margin;
      if(top < margin){{
        top=infoRect.bottom + margin;
        tooltip.classList.add('tooltip-below');
      }}
      tooltip.style.left=left+'px';
      tooltip.style.top=top+'px';
    }}
    function resetTooltip(){{
      tooltip.classList.remove('tooltip-fixed');
      tooltip.classList.remove('tooltip-below');
      tooltip.style.left='';
      tooltip.style.top='';
    }}
    info.addEventListener('mouseenter', positionTooltip);
    info.addEventListener('mouseleave', resetTooltip);
    info.addEventListener('focus', positionTooltip);
    info.addEventListener('blur', resetTooltip);
  }});
  </script>
</body>
</html>"""
