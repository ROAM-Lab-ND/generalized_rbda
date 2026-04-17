#!/usr/bin/env python3
"""Rebuild Benchmarking Fig4_Performance_Breakdown from measured phase timings.

Data source:
- Benchmarking/data/id_derivatives_profile_measured_YYYY-MM-DD.txt

This script parses warm-phase measured timings from benchmark output:
- total std time (us)
- measured forward_us, backward_us, casadi_us
and computes init_us as the residual:
    init_us = std_us - (forward_us + backward_us + casadi_us)
"""

from __future__ import annotations

from pathlib import Path
import csv
import re
from datetime import date
import os

import matplotlib.pyplot as plt
import numpy as np


def parse_measured_profile(report_path: Path) -> tuple[dict[str, float], dict[str, tuple[float, float, float, float]], dict[str, int]]:
    totals: dict[str, float] = {}
    phases: dict[str, tuple[float, float, float, float]] = {}
    dofs: dict[str, int] = {}

    profile_re = re.compile(
        r"\[IDDerivProfileSummary\] (.+?) warm forward_us=([0-9.]+) backward_us=([0-9.]+) casadi_us=([0-9.]+)(?: getsq_us=([0-9.]+))?(?: getsq_internal_us=([0-9.]+))?"
    )
    table_re = re.compile(
        r"^(.*?)\s+(\d+)\s+(\d+)\s+([0-9.]+)\s+([0-9.]+)\s+([0-9.]+)$"
    )

    for raw_line in report_path.read_text().splitlines():
        line = raw_line.strip("\n")

        p = profile_re.search(line)
        if p:
            name = p.group(1).strip()
            getsq_us = float(p.group(5)) if p.group(5) is not None else 0.0
            getsq_internal_us = float(p.group(6)) if p.group(6) is not None else getsq_us
            phases[name] = (float(p.group(2)), float(p.group(3)), float(p.group(4)), getsq_internal_us)
            continue

        t = table_re.match(line.strip())
        if t:
            name = t.group(1).strip()
            dofs[name] = int(t.group(2))
            totals[name] = float(t.group(4))

    return totals, phases, dofs


def write_measured_csv(path: Path, rows: list[dict[str, str]]) -> None:
    fieldnames = [
        "label",
        "source_model",
        "dof",
        "total_us",
        "init_us",
        "forward_us",
        "backward_us",
        "casadi_us",
        "getsq_us",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    stamp = os.getenv("GRBDA_FIG4_STAMP", date.today().isoformat())
    measured_report = repo_root / "generalized_rbda" / "Benchmarking" / "data" / f"id_derivatives_profile_measured_{stamp}.txt"
    out_dir = repo_root / "generalized_rbda" / "Benchmarking" / "figures"
    data_out_dir = repo_root / "generalized_rbda" / "Benchmarking" / "data"
    out_dir.mkdir(parents=True, exist_ok=True)
    data_out_dir.mkdir(parents=True, exist_ok=True)

    if not measured_report.exists():
        raise FileNotFoundError(f"Missing measured report: {measured_report}")

    totals, phases, dof_map = parse_measured_profile(measured_report)

    # Display label and source model name from benchmark output.
    robots = [
        ("Mini Cheetah (+R)", "MiniCheetah (rotors)"),
        ("Mini Cheetah (-R)", "MiniCheetah (no rotors)"),
        ("MIT Humanoid (+R)", "MIT_Humanoid (rotors)"),
        ("MIT Humanoid (-R)", "MIT_Humanoid (no rotors)"),
        ("Tello (+R/+M)", "Tello (+R/+M)"),
        ("Tello (+R/-M)", "Tello (+R/-M)"),
        ("Tello with Arms (+R/+M)", "Tello with Arms (+R/+M)"),
        ("KUKA LWR 4+", "Kuka LWR 4+ (URDF)"),
    ]

    labels: list[str] = []
    dofs: list[int] = []
    total_vals: list[float] = []
    init_vals: list[float] = []
    forward_vals: list[float] = []
    forward_exclusive_vals: list[float] = []
    backward_vals: list[float] = []
    casadi_vals: list[float] = []
    getsq_vals: list[float] = []

    measured_rows: list[dict[str, str]] = []

    for label, source_name in robots:
        if source_name not in totals:
            raise KeyError(f"Missing total timing for model: {source_name}")
        if source_name not in phases:
            raise KeyError(f"Missing warm phase summary for model: {source_name}")

        total = totals[source_name]
        forward, backward, casadi, getsq = phases[source_name]
        init = max(total - (forward + backward + casadi), 0.0)
        dof = dof_map.get(source_name, 0)

        labels.append(label)
        dofs.append(dof)
        total_vals.append(total)
        init_vals.append(init)
        forward_vals.append(forward)
        forward_exclusive_vals.append(max(forward - getsq, 0.0))
        backward_vals.append(backward)
        casadi_vals.append(casadi)
        getsq_vals.append(getsq)

        measured_rows.append(
            {
                "label": label,
                "source_model": source_name,
                "dof": str(dof),
                "total_us": f"{total:.4f}",
                "init_us": f"{init:.4f}",
                "forward_us": f"{forward:.4f}",
                "backward_us": f"{backward:.4f}",
                "casadi_us": f"{casadi:.4f}",
                "getsq_us": f"{getsq:.4f}",
            }
        )

    x = np.arange(len(labels))
    width = 0.72

    fig, ax = plt.subplots(figsize=(12, 6), dpi=300)

    c_init = "#4C78A8"
    c_forward = "#59A14F"
    c_backward = "#F28E2B"
    c_casadi = "#E15759"
    c_getsq = "#76B7B2"

    b0 = np.zeros(len(labels))
    ax.bar(x, init_vals, width, color=c_init, label="Init")
    b1 = np.array(init_vals)
    ax.bar(x, forward_exclusive_vals, width, bottom=b1, color=c_forward, label="Forward (excl. getSq)")
    b2 = b1 + np.array(forward_exclusive_vals)
    ax.bar(x, backward_vals, width, bottom=b2, color=c_backward, label="Backward")
    b3 = b2 + np.array(backward_vals)
    ax.bar(x, casadi_vals, width, bottom=b3, color=c_casadi, label="CasADi")
    b4 = b3 + np.array(casadi_vals)
    ax.bar(x, getsq_vals, width, bottom=b4, color=c_getsq, label="getSq")

    for xi, total, dof in zip(x, total_vals, dofs):
        ax.text(xi, total + max(total_vals) * 0.02, f"{dof} DOF", ha="center", va="bottom", fontsize=8)

    ax.set_title("Fig. 4: First-Order ID Derivatives Performance Breakdown (Measured)", fontsize=13, weight="bold")
    ax.set_ylabel("Time per Iteration (microseconds)")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=22, ha="right")
    ax.grid(axis="y", alpha=0.25)
    ax.set_axisbelow(True)
    ax.legend(ncol=5, loc="upper center", bbox_to_anchor=(0.5, 1.12), frameon=False)

    fig.tight_layout()

    measured_csv = data_out_dir / f"fig4_performance_breakdown_measured_{stamp}.csv"
    write_measured_csv(measured_csv, measured_rows)

    png_path = out_dir / "Fig4_Performance_Breakdown.png"
    pdf_path = out_dir / "Fig4_Performance_Breakdown.pdf"
    png_updated_path = out_dir / f"Fig4_Performance_Breakdown_updated_{stamp}.png"
    pdf_updated_path = out_dir / f"Fig4_Performance_Breakdown_updated_{stamp}.pdf"

    try:
        fig.savefig(png_path, dpi=300)
        fig.savefig(pdf_path)
        fig.savefig(png_updated_path, dpi=300)
        fig.savefig(pdf_updated_path)
        print(f"Wrote: {png_path}")
        print(f"Wrote: {pdf_path}")
        print(f"Wrote: {png_updated_path}")
        print(f"Wrote: {pdf_updated_path}")
    except PermissionError:
        png_fallback = png_updated_path
        pdf_fallback = pdf_updated_path
        fig.savefig(png_fallback, dpi=300)
        fig.savefig(pdf_fallback)
        print("Could not overwrite canonical Fig4 files (permission denied).")
        print(f"Wrote fallback: {png_fallback}")
        print(f"Wrote fallback: {pdf_fallback}")

    print(f"Wrote measured data: {measured_csv}")


if __name__ == "__main__":
    main()
