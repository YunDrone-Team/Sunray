REPORT_STYLES = """
:root {
  --primary: #183a6b;
  --bg: #f4f7fb;
  --panel: #ffffff;
  --panel-soft: #f8fbff;
  --text: #172033;
  --muted: #69758a;
  --line: #d9e2ef;
  --shadow: 0 16px 44px rgba(24, 40, 72, 0.10);
  --pass: #17a05d;
  --fail: #d14343;
  --error: #d97a00;
  --unsupported: #617086;
  --accent: #1166cc;
  --mono: "JetBrains Mono", "Menlo", "Consolas", "Liberation Mono", monospace;
  --sans: system-ui, -apple-system, "Segoe UI", Roboto, "Helvetica Neue", "PingFang SC", "Microsoft YaHei", sans-serif;
}
* { box-sizing: border-box; }
body {
  margin: 0;
  font-family: var(--sans);
  background:
    radial-gradient(circle at top left, rgba(17, 102, 204, 0.10), transparent 32%),
    linear-gradient(180deg, #eef4fb 0%, var(--bg) 45%, #edf2f9 100%);
  color: var(--text);
  position: relative;
  min-height: 100vh;
}
.watermark-layer {
  position: fixed;
  inset: 0;
  pointer-events: none;
  z-index: 50;
  overflow: hidden;
}
.watermark-item {
  position: absolute;
  transform: translate(-50%, -50%) rotate(-24deg);
  transform-origin: center;
  font-size: clamp(14px, 1.8vw, 24px);
  font-weight: 800;
  letter-spacing: 0.1em;
  color: rgba(24, 58, 107, 0.075);
  mix-blend-mode: multiply;
  white-space: nowrap;
  user-select: none;
}
.container {
  max-width: 1440px;
  margin: 0 auto;
  padding: 32px 24px 48px;
  position: relative;
  z-index: 1;
}
.hero {
  background: linear-gradient(135deg, #183a6b 0%, #0f5aa6 58%, #5aa7ff 100%);
  color: #fff;
  border-radius: 24px;
  padding: 28px 32px;
  box-shadow: var(--shadow);
  position: relative;
  overflow: hidden;
}
.hero::after {
  content: "";
  position: absolute;
  inset: auto -60px -80px auto;
  width: 260px;
  height: 260px;
  background: rgba(255, 255, 255, 0.12);
  border-radius: 50%;
}
.eyebrow {
  font-size: 12px;
  letter-spacing: 0.16em;
  text-transform: uppercase;
  opacity: 0.75;
  margin-bottom: 10px;
}
h1 {
  margin: 0;
  font-size: 34px;
  line-height: 1.1;
  font-weight: 800;
  letter-spacing: -0.02em;
}
.hero-subtitle {
  margin-top: 12px;
  max-width: 860px;
  font-size: 15px;
  line-height: 1.7;
  color: rgba(255, 255, 255, 0.88);
}
.section {
  margin-top: 24px;
  background: var(--panel);
  border: 1px solid rgba(217, 226, 239, 0.85);
  border-radius: 22px;
  box-shadow: var(--shadow);
  overflow: visible;
}
.section-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
  padding: 20px 24px 0;
}
.section-title {
  margin: 0;
  font-size: 20px;
  font-weight: 700;
}
.section-body {
  padding: 20px 24px 24px;
}
.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
  gap: 14px;
}
.summary-card {
  min-height: 110px;
  padding: 18px;
  border-radius: 18px;
  background: var(--panel-soft);
  border: 1px solid var(--line);
}
.summary-wide {
  grid-column: span 2;
}
.summary-label {
  font-size: 13px;
  color: var(--muted);
  font-weight: 600;
}
.summary-value {
  margin-top: 12px;
  font-size: 28px;
  line-height: 1.2;
  font-weight: 700;
  word-break: break-word;
  font-family: var(--mono);
}
.summary-pass .summary-value { color: var(--pass); }
.summary-fail .summary-value { color: var(--fail); }
.summary-error .summary-value { color: var(--error); }
.summary-unsupported .summary-value { color: var(--unsupported); }
.summary-accent .summary-value { color: var(--accent); }
.meta-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
  gap: 12px;
  overflow: visible;
  position: relative;
}
.report-meta-grid {
  grid-template-columns: repeat(4, minmax(0, 1fr));
}
.kv-item {
  position: relative;
  padding: 14px 16px;
  background: var(--panel-soft);
  border: 1px solid var(--line);
  border-radius: 16px;
}
.kv-item:hover {
  z-index: 20;
}
.kv-key {
  display: block;
  color: var(--muted);
  font-size: 12px;
  margin-bottom: 8px;
  font-weight: 600;
  line-height: 1.45;
  white-space: normal;
  word-break: break-word;
  overflow-wrap: anywhere;
}
.kv-value {
  display: block;
  font-size: 14px;
  white-space: pre-wrap;
  word-break: break-word;
  font-family: var(--mono);
  line-height: 1.6;
}
.kv-chip-row {
  display: flex;
  flex-wrap: wrap;
  gap: 6px;
  margin-top: 8px;
  min-width: 0;
}
.kv-chip-stack {
  margin-top: 8px;
}
.kv-chip-stack .kv-chip-row {
  margin-top: 0;
}
.case-metric-summary,
.case-metric-detail {
  min-width: 0;
}
.case-detail-summary {
  min-width: 0;
}
.case-detail-summary .kv-chip-row {
  flex-wrap: nowrap;
  margin-top: 0;
  overflow: hidden;
  width: 100%;
}
.case-detail-summary .kv-chip {
  flex: 0 1 auto;
  min-width: 0;
}
.case-detail-summary .kv-chip .kv-key,
.case-detail-summary .kv-chip .kv-value {
  min-width: 0;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}
.case-actions {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
  margin-top: 10px;
  min-width: 0;
}
.case-param-details {
  display: inline-block;
  margin-top: 0;
}
.case-param-actions {
  display: block;
  margin-top: 10px;
  min-width: 0;
  width: 100%;
}
.case-param-details summary {
  display: inline-flex;
  align-items: center;
  gap: 6px;
  padding: 5px 10px;
  background: var(--panel-soft);
  border: 1px solid var(--line);
  border-radius: 8px;
  font-size: 11px;
  font-weight: 600;
  color: var(--muted);
  cursor: pointer;
  list-style: none;
  font-family: var(--sans);
  line-height: 1.4;
  white-space: nowrap;
}
.case-param-details summary::-webkit-details-marker {
  display: none;
}
.case-param-details summary::before {
  content: "";
  display: inline-block;
  width: 0;
  height: 0;
  border-left: 4px solid var(--muted);
  border-top: 3px solid transparent;
  border-bottom: 3px solid transparent;
  transition: transform 0.15s;
}
.case-param-details[open] summary::before {
  transform: rotate(90deg);
}
.case-param-detail-body {
  display: block;
  margin-top: 0;
  padding: 14px 16px;
  border: 1px solid var(--line);
  border-radius: 12px;
  background: #f8fbff;
  width: 100%;
  max-width: 100%;
  min-width: 0;
  overflow: hidden;
}
.case-param-detail-row td {
  padding: 0 24px 18px 38px;
}
.case-param-row-body {
  width: 100%;
  max-width: 100%;
}
.case-param-detail-body .kv-chip-row {
  margin-top: 0;
  width: 100%;
}
.case-param-detail-body .kv-chip {
  margin-top: 2px;
  min-width: 0;
}
.case-param-card-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 8px;
  width: 100%;
}
.case-param-card {
  min-width: 0;
  padding: 10px 12px;
  border: 1px solid var(--line);
  border-radius: 8px;
  background: var(--panel-soft);
}
.case-param-card--wide {
  grid-column: 1 / -1;
}
.case-param-card-key {
  display: block;
  color: var(--muted);
  font-size: 12px;
  font-weight: 700;
  line-height: 1.35;
  margin-bottom: 5px;
}
.case-param-card-value {
  color: var(--text);
  font-family: var(--mono);
  font-size: 12px;
  font-weight: 600;
  line-height: 1.55;
  white-space: pre-wrap;
  word-break: break-word;
  overflow-wrap: anywhere;
}
.kv-chip {
  display: inline-flex;
  align-items: baseline;
  gap: 5px;
  max-width: 100%;
  padding: 4px 9px;
  background: var(--panel-soft);
  border: 1px solid var(--line);
  border-radius: 8px;
  font-size: 12px;
  line-height: 1.4;
}
.kv-chip .kv-key {
  display: inline;
  color: var(--muted);
  font-size: 12px;
  margin-bottom: 0;
  font-weight: 600;
}
.kv-chip .kv-value {
  display: inline;
  font-size: 12px;
  white-space: normal;
  word-break: break-word;
  overflow-wrap: anywhere;
  font-family: var(--mono);
  line-height: 1.4;
}
.kv-chip--block-value {
  display: flex;
  align-items: flex-start;
  flex-direction: column;
  gap: 4px;
  width: 100%;
  max-width: 100%;
}
.kv-chip--block-value .kv-value {
  display: block;
  white-space: pre-wrap;
  line-height: 1.55;
}
.metric-info {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 14px;
  height: 14px;
  border-radius: 50%;
  background: var(--line);
  color: var(--muted);
  font-size: 9px;
  font-weight: 700;
  font-style: normal;
  cursor: help;
  margin-left: 4px;
  position: relative;
  vertical-align: middle;
  flex-shrink: 0;
}
.metric-info:hover {
  z-index: 30;
}
.metric-tooltip {
  display: none;
  position: absolute;
  bottom: calc(100% + 8px);
  left: 50%;
  transform: translateX(-50%);
  background: var(--text);
  color: #fff;
  padding: 8px 12px;
  border-radius: 8px;
  font-size: 12px;
  font-weight: 400;
  font-family: var(--sans);
  white-space: normal;
  width: max-content;
  max-width: 280px;
  line-height: 1.6;
  z-index: 100;
  pointer-events: none;
  box-shadow: 0 4px 12px rgba(0,0,0,0.15);
}
.metric-tooltip.tooltip-fixed {
  position: fixed;
  top: 0;
  left: 0;
  bottom: auto;
  transform: none;
  display: block;
  z-index: 9999;
}
.metric-tooltip::after {
  content: "";
  position: absolute;
  top: 100%;
  left: 50%;
  transform: translateX(-50%);
  border: 5px solid transparent;
  border-top-color: var(--text);
}
.metric-tooltip.tooltip-below::after {
  top: auto;
  bottom: 100%;
  border-top-color: transparent;
  border-bottom-color: var(--text);
}
.metric-info:hover .metric-tooltip {
  display: block;
}
.table-wrap {
  overflow-x: auto;
  overflow-y: visible;
}
table {
  width: 100%;
  table-layout: fixed;
  border-collapse: separate;
  border-spacing: 0;
  min-width: 1120px;
}
.case-col-index { width: 4%; }
.case-col-name { width: 13%; }
.case-col-category { width: 10%; }
.case-col-result { width: 11%; }
.case-col-score { width: 10%; }
.case-col-detail { width: 43%; }
.case-col-duration { width: 9%; }
th, td {
  text-align: left;
  vertical-align: top;
  padding: 14px 16px;
  border-bottom: 1px solid var(--line);
  font-size: 14px;
}
th {
  position: sticky;
  top: 0;
  z-index: 1;
  background: #eff5fc;
  color: #344054;
  font-size: 13px;
  white-space: nowrap;
  font-weight: 700;
}
td {
  font-family: var(--mono);
}
tr:last-child td {
  border-bottom: none;
}
.case-row.result-pass td:first-child { border-left: 4px solid var(--pass); }
.case-row.result-fail td:first-child { border-left: 4px solid var(--fail); }
.case-row.result-error td:first-child { border-left: 4px solid var(--error); }
.case-row.result-unsupported td:first-child { border-left: 4px solid var(--unsupported); }
.case-title {
  font-weight: 700;
  color: var(--text);
  font-family: var(--sans);
}
.case-subtitle, .case-time {
  margin-top: 6px;
  color: var(--muted);
  font-size: 12px;
  line-height: 1.5;
  white-space: pre-wrap;
  word-break: break-word;
  font-family: var(--mono);
}
.case-status-meta {
  min-width: 0;
}
.case-status-meta-row {
  display: grid;
  grid-template-columns: minmax(62px, 0.8fr) minmax(92px, 1fr) minmax(52px, 0.7fr);
  align-items: start;
  gap: 12px;
}
.case-status-meta-item {
  min-width: 0;
}
.case-status-meta-category {
  color: var(--text);
}
.case-status-meta-score {
  text-align: center;
}
.case-status-meta .status-badge {
  width: 100%;
  min-width: 0;
}
.case-status-meta .case-time {
  margin-top: 12px;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}
.status-badge {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  min-width: 92px;
  padding: 8px 12px;
  border-radius: 999px;
  font-size: 12px;
  font-weight: 700;
  letter-spacing: 0.04em;
  color: #fff;
  font-family: var(--sans);
}
.status-pass { background: var(--pass); }
.status-fail { background: var(--fail); }
.status-error { background: var(--error); }
.status-unsupported { background: var(--unsupported); }
.status-unknown { background: #44526a; }
.timeline {
  display: grid;
  grid-template-columns: repeat(var(--timeline-count, 1), minmax(0, 1fr));
  gap: 18px;
  align-items: start;
  padding: 6px 0 10px;
}
.timeline-step {
  position: relative;
  border: 1px solid var(--line);
  background: linear-gradient(180deg, #ffffff 0%, #f8fbff 100%);
  border-radius: 14px;
  padding: 12px 10px 10px;
  min-height: 108px;
  box-shadow: 0 8px 22px rgba(24, 40, 72, 0.06);
  min-width: 0;
}
.timeline-step + .timeline-step {
  margin-left: 0;
}
.timeline-step + .timeline-step::before {
  content: "";
  position: absolute;
  left: -19px;
  top: 24px;
  width: 18px;
  height: 2px;
  background: var(--line);
}
.timeline-step + .timeline-step::after {
  content: "";
  position: absolute;
  left: -5px;
  top: 20px;
  border-left: 6px solid var(--line);
  border-top: 4px solid transparent;
  border-bottom: 4px solid transparent;
}
.timeline-dot {
  width: 12px;
  height: 12px;
  border-radius: 999px;
  background: var(--accent);
  margin-bottom: 8px;
  box-shadow: 0 0 0 4px rgba(17, 102, 204, 0.12);
}
.timeline-completed .timeline-dot { background: var(--pass); box-shadow: 0 0 0 4px rgba(23, 160, 93, 0.12); }
.timeline-failed .timeline-dot { background: var(--fail); box-shadow: 0 0 0 4px rgba(209, 67, 67, 0.12); }
.timeline-phase {
  font-size: 14px;
  font-weight: 700;
  line-height: 1.3;
}
.timeline-state {
  margin-top: 5px;
  color: var(--muted);
  font-size: 11px;
  line-height: 1.35;
}
.timeline-time {
  margin-top: 6px;
  font-size: 10px;
  color: var(--muted);
  line-height: 1.35;
  white-space: pre-wrap;
  word-break: break-word;
  font-family: var(--mono);
}
.flight-section {
  border: 1px solid var(--line);
  background: linear-gradient(180deg, #ffffff 0%, #f8fbff 100%);
  border-radius: 18px;
  padding: 18px;
  overflow: visible;
}
.flight-section + .flight-section {
  margin-top: 16px;
}
.flight-title {
  margin: 0 0 14px;
  font-size: 18px;
  font-weight: 700;
}
.flight-subtitle {
  margin: 18px 0 10px;
  font-size: 14px;
  font-weight: 700;
  color: #334155;
}
.flight-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 14px;
}
.flight-card {
  border: 1px solid var(--line);
  background: var(--panel-soft);
  border-radius: 16px;
  padding: 16px;
  overflow: visible;
  position: relative;
}
.flight-card:hover {
  z-index: 20;
}
.flight-card-title {
  font-size: 15px;
  font-weight: 700;
}
.flight-card-subtitle {
  margin-top: 6px;
  color: var(--muted);
  font-size: 12px;
  white-space: pre-wrap;
  font-family: var(--mono);
  line-height: 1.6;
}
.plain-list {
  margin: 0;
  padding-left: 18px;
  color: var(--text);
}
.plain-list li + li {
  margin-top: 6px;
}
.empty-block {
  color: var(--muted);
  font-size: 13px;
}
.case-expand-details {
  margin-top: 0;
}
.case-expand-details summary {
  display: inline-flex;
  align-items: center;
  gap: 6px;
  padding: 5px 10px;
  background: var(--panel-soft);
  border: 1px solid var(--line);
  border-radius: 8px;
  font-size: 11px;
  font-weight: 600;
  color: var(--muted);
  cursor: pointer;
  list-style: none;
  font-family: var(--sans);
  line-height: 1.4;
  white-space: nowrap;
}
.case-expand-details summary::-webkit-details-marker {
  display: none;
}
.case-expand-details summary::before {
  content: "";
  display: inline-block;
  width: 0;
  height: 0;
  border-left: 4px solid var(--muted);
  border-top: 3px solid transparent;
  border-bottom: 3px solid transparent;
  transition: transform 0.15s;
}
.case-expand-details[open] summary::before {
  transform: rotate(90deg);
}
.case-detail-row td {
  padding: 0 16px 14px;
  border-bottom: 1px solid var(--line);
}
.case-flight-body {
  border: 1px solid var(--line);
  border-radius: 14px;
  background: linear-gradient(180deg, #ffffff 0%, #f8fbff 100%);
  padding: 14px 18px 18px;
  overflow: visible;
}
.score-overview {
  display: flex;
  gap: 24px;
  align-items: stretch;
}
.score-main-card {
  min-width: 180px;
  padding: 24px;
  border-radius: 18px;
  background: var(--panel-soft);
  border: 1px solid var(--line);
  text-align: center;
  display: flex;
  flex-direction: column;
  justify-content: center;
}
.score-main-label {
  font-size: 13px;
  color: var(--muted);
}
.score-main-value {
  font-size: 48px;
  font-weight: 800;
  line-height: 1.2;
  margin: 8px 0;
}
.score-main-grade {
  display: inline-block;
  margin: 0 auto;
  padding: 6px 20px;
  border-radius: 999px;
  color: #fff;
  font-size: 14px;
  font-weight: 700;
}
.score-sub-cards {
  flex: 1;
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
  gap: 14px;
}
.score-sub-card {
  padding: 18px;
  border-radius: 18px;
  background: var(--panel-soft);
  border: 1px solid var(--line);
  text-align: center;
}
.score-sub-label {
  font-size: 13px;
  color: var(--muted);
}
.score-sub-value {
  font-size: 32px;
  font-weight: 800;
  line-height: 1.2;
  margin: 8px 0 4px;
}
.score-sub-grade {
  font-size: 13px;
  font-weight: 700;
}
@media (max-width: 600px) {
  .score-overview {
    flex-direction: column;
  }
}
.waypoint-card {
  border: 1px solid var(--line);
  background: var(--panel-soft);
  border-radius: 16px;
  padding: 16px;
}
.waypoint-id {
  display: inline-block;
  width: 24px;
  height: 24px;
  line-height: 24px;
  text-align: center;
  background: var(--primary);
  color: white;
  border-radius: 50%;
  font-size: 12px;
  font-weight: 800;
  margin-right: 8px;
}
.waypoint-metric {
  font-size: 12px;
  margin-top: 10px;
}
.waypoint-metric-label {
  color: var(--muted);
  font-weight: 600;
  margin-bottom: 2px;
}
.waypoint-metric-value {
  font-family: var(--mono);
  color: var(--text);
}
.config-snapshot-layout {
  display: flex;
  flex-direction: column;
  gap: 14px;
}
.config-snapshot-section {
  border: 1px solid var(--line);
  background: var(--panel);
  border-radius: 16px;
  padding: 14px 16px;
  box-shadow: 0 4px 12px rgba(24, 40, 72, 0.06);
}
.config-snapshot-summary {
  cursor: pointer;
  list-style: none;
  display: flex;
  align-items: center;
  min-height: 30px;
  position: relative;
  padding-left: 24px;
  user-select: none;
}
.config-snapshot-summary::-webkit-details-marker {
  display: none;
}
.config-snapshot-summary::before {
  content: "";
  position: absolute;
  left: 0;
  top: 50%;
  transform: translateY(-50%);
  width: 0;
  height: 0;
  border-left: 7px solid var(--primary);
  border-top: 5px solid transparent;
  border-bottom: 5px solid transparent;
  transition: transform 0.2s ease;
}
.config-snapshot-section[open] .config-snapshot-summary::before {
  transform: translateY(-50%) rotate(90deg);
}
.config-snapshot-title {
  color: var(--primary);
  font-size: 14px;
  font-weight: 800;
  line-height: 1.35;
}
.config-snapshot-section-body {
  margin-top: 14px;
}
.config-snapshot-group + .config-snapshot-group {
  margin-top: 20px;
}
.config-snapshot-group .flight-subtitle {
  margin: 0 0 10px;
  padding-top: 2px;
}
.config-snapshot-group .meta-grid {
  grid-template-columns: repeat(auto-fit, minmax(210px, 1fr));
  gap: 12px;
}
.config-snapshot-group .kv-item {
  min-height: 82px;
}
.mission-point-section {
  margin-top: 12px;
}
.mission-point-title {
  font-size: 12px;
  font-weight: 700;
  color: var(--muted);
  margin-bottom: 8px;
}
.mission-point-grid {
  display: grid;
  grid-template-columns: repeat(4, minmax(0, 1fr));
  gap: 10px;
}
.mission-point-item {
  display: grid;
  grid-template-columns: 24px minmax(0, 1fr);
  align-items: center;
  gap: 10px;
  min-width: 0;
  background: #f8fafc;
  border: 1px solid #e3ebf5;
  border-radius: 4px;
  padding: 10px 12px;
}
.mission-point-index {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 24px;
  height: 24px;
  border-radius: 999px;
  background: #ffffff;
  border: 1px solid #cbd8e8;
  color: var(--text);
  font-family: var(--mono);
  font-size: 12px;
  font-weight: 800;
  flex-shrink: 0;
}
.mission-point-value {
  min-width: 0;
  font-family: var(--mono);
  font-size: 12px;
}
.mission-point-coords {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 8px;
  width: 100%;
  min-width: 0;
  overflow: hidden;
}
.mission-point-chip {
  position: relative;
  display: flex;
  flex-direction: column;
  justify-content: center;
  gap: 2px;
  min-width: 0;
  min-height: 38px;
  padding: 2px 0 2px 9px;
  white-space: nowrap;
  background: transparent;
}
.mission-point-chip::before {
  content: "";
  position: absolute;
  left: 0;
  top: 6px;
  bottom: 6px;
  width: 2px;
  border-radius: 999px;
  background: #94a3b8;
}
.mission-point-chip--x::before {
  background: #10b981;
}
.mission-point-chip--y::before {
  background: #f59e0b;
}
.mission-point-chip--z::before {
  background: #0ea5e9;
}
.mission-point-axis {
  display: flex;
  align-items: center;
  color: var(--muted);
  font-family: var(--sans);
  font-size: 10px;
  font-weight: 800;
  line-height: 1.2;
  letter-spacing: 0.04em;
  text-transform: uppercase;
}
.mission-point-number {
  display: block;
  color: var(--text);
  font-family: var(--mono);
  font-size: 14px;
  font-weight: 700;
  line-height: 1.35;
}
@media (max-width: 900px) {
  .container {
    padding: 18px 14px 30px;
  }
  .hero {
    padding: 22px 18px;
    border-radius: 18px;
  }
  h1 {
    font-size: 28px;
  }
  .section {
    border-radius: 18px;
  }
  .section-header {
    padding: 18px 18px 0;
  }
  .section-body {
    padding: 18px;
  }
  .summary-wide {
    grid-column: auto;
  }
  .report-meta-grid {
    grid-template-columns: repeat(2, minmax(0, 1fr));
  }
  .timeline {
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 12px;
  }
  .timeline-step::before,
  .timeline-step::after {
    display: none;
  }
  .mission-point-grid {
    grid-template-columns: repeat(2, minmax(0, 1fr));
  }
  .case-param-card-grid {
    grid-template-columns: 1fr;
  }
}
@media (max-width: 560px) {
  .report-meta-grid {
    grid-template-columns: 1fr;
  }
  .mission-point-grid {
    grid-template-columns: 1fr;
  }
}
"""
