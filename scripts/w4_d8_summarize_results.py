#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Summarize regression CSV into a Markdown report."
    )
    parser.add_argument("--csv", required=True, help="Input CSV path")
    parser.add_argument("--out", required=True, help="Output Markdown path")
    return parser.parse_args()


def safe_float(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def normalize_result(value: str) -> str:
    v = (value or "").strip().upper()
    if v in {"S", "SUCCESS"}:
        return "SUCCESS"
    if v in {"T", "TIMEOUT"}:
        return "TIMEOUT"
    if v:
        return "OTHER"
    return "EMPTY"


def load_rows(csv_path: Path):
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    return rows


def build_report(csv_path: Path, rows):
    total_runs = len(rows)

    success_count = 0
    timeout_count = 0
    other_count = 0

    time_values = []
    recovery_yes_count = 0
    recovery_hits_sum = 0

    for row in rows:
        result = normalize_result(row.get("result", ""))
        if result == "SUCCESS":
            success_count += 1
        elif result == "TIMEOUT":
            timeout_count += 1
        else:
            other_count += 1

        time_sec = safe_float(row.get("time_sec", "0"))
        time_values.append(time_sec)

        recovery = (row.get("recovery", "") or "").strip().upper()
        if recovery == "Y":
            recovery_yes_count += 1

        recovery_hits_raw = (row.get("recovery_hits", "") or "").strip()
        if recovery_hits_raw.isdigit():
            recovery_hits_sum += int(recovery_hits_raw)

    success_rate = (success_count / total_runs * 100.0) if total_runs else 0.0
    avg_time = (sum(time_values) / total_runs) if total_runs else 0.0
    min_time = min(time_values) if time_values else 0.0
    max_time = max(time_values) if time_values else 0.0

    lines = []
    lines.append("# w4_d8 Regression Latest Report")
    lines.append("")
    lines.append("## 1. Input")
    lines.append(f"- Source CSV: `{csv_path}`")
    lines.append(f"- Total rows: {total_runs}")
    lines.append("")
    lines.append("## 2. Summary")
    lines.append("")
    lines.append("| Metric | Value |")
    lines.append("|---|---:|")
    lines.append(f"| Total runs | {total_runs} |")
    lines.append(f"| Success count | {success_count} |")
    lines.append(f"| Timeout count | {timeout_count} |")
    lines.append(f"| Other count | {other_count} |")
    lines.append(f"| Success rate | {success_rate:.2f}% |")
    lines.append(f"| Average time (s) | {avg_time:.3f} |")
    lines.append(f"| Min time (s) | {min_time:.3f} |")
    lines.append(f"| Max time (s) | {max_time:.3f} |")
    lines.append("")
    lines.append("## 3. Recovery Summary")
    lines.append("")
    lines.append("| Metric | Value |")
    lines.append("|---|---:|")
    lines.append(f"| Recovery=Y count | {recovery_yes_count} |")
    lines.append(f"| Recovery hits sum | {recovery_hits_sum} |")
    lines.append("")
    lines.append("## 4. Interpretation")
    lines.append("")
    lines.append("- This report is generated automatically from a regression CSV.")
    lines.append("- It is intended to provide a stable, repeatable summary for interview/demo use.")
    lines.append("- If the CSV schema changes, the script should be updated deliberately rather than edited ad hoc.")

    return "\n".join(lines) + "\n"


def main():
    args = parse_args()
    csv_path = Path(args.csv)
    out_path = Path(args.out)

    rows = load_rows(csv_path)
    report = build_report(csv_path, rows)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(report, encoding="utf-8")

    print(f"[OK] csv={csv_path}")
    print(f"[OK] out={out_path}")
    print(f"[OK] rows={len(rows)}")


if __name__ == "__main__":
    main()
