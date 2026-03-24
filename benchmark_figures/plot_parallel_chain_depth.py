#!/usr/bin/env python3
"""
Plot the parallel chain depth benchmark results.
Shows how computation time varies with cross-link depth position.
"""

import matplotlib.pyplot as plt
import numpy as np

# Data from benchmark output
depths = list(range(0, 41))  # 0 = baseline, 1-40 = cross-link depths
labels = ["Baseline"] + [f"Depth {i}" for i in range(1, 41)]

# Median times (more robust to outliers)
median_times = [
    195.85,  # Baseline
    194.83,  # Depth 1
    195.74,  # Depth 2
    196.57,  # Depth 3
    196.49,  # Depth 4
    195.98,  # Depth 5
    195.96,  # Depth 6
    196.35,  # Depth 7
    197.27,  # Depth 8
    196.55,  # Depth 9
    196.67,  # Depth 10
    197.69,  # Depth 11
    196.89,  # Depth 12
    196.92,  # Depth 13
    197.37,  # Depth 14
    207.67,  # Depth 15
    207.28,  # Depth 16
    207.68,  # Depth 17
    207.60,  # Depth 18
    208.07,  # Depth 19
    208.02,  # Depth 20
    197.57,  # Depth 21
    198.09,  # Depth 22
    197.94,  # Depth 23
    198.08,  # Depth 24
    198.32,  # Depth 25
    197.82,  # Depth 26
    198.13,  # Depth 27
    209.20,  # Depth 28
    198.34,  # Depth 29
    198.25,  # Depth 30
    198.58,  # Depth 31
    199.14,  # Depth 32
    208.67,  # Depth 33
    208.94,  # Depth 34
    209.02,  # Depth 35
    198.97,  # Depth 36
    209.18,  # Depth 37
    209.66,  # Depth 38
    209.69,  # Depth 39
    199.55,  # Depth 40
]

# Standard deviations
std_times = [
    26.23,  # Baseline
    0.98,   # Depth 1
    3.62,   # Depth 2
    0.50,   # Depth 3
    5.56,   # Depth 4
    0.41,   # Depth 5
    5.01,   # Depth 6
    0.31,   # Depth 7
    3.39,   # Depth 8
    0.49,   # Depth 9
    24.87,  # Depth 10
    0.49,   # Depth 11
    4.86,   # Depth 12
    0.75,   # Depth 13
    3.59,   # Depth 14
    2.13,   # Depth 15
    5.50,   # Depth 16
    1.10,   # Depth 17
    4.89,   # Depth 18
    1.31,   # Depth 19
    3.67,   # Depth 20
    0.56,   # Depth 21
    4.90,   # Depth 22
    0.50,   # Depth 23
    6.76,   # Depth 24
    1.21,   # Depth 25
    3.68,   # Depth 26
    1.09,   # Depth 27
    5.59,   # Depth 28
    0.90,   # Depth 29
    8.65,   # Depth 30
    3.53,   # Depth 31
    0.59,   # Depth 32
    1.96,   # Depth 33
    5.85,   # Depth 34
    2.51,   # Depth 35
    5.94,   # Depth 36
    4.44,   # Depth 37
    2.62,   # Depth 38
    6.74,   # Depth 39
    1.02,   # Depth 40
]

# Create figure with two subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

# Left plot: Main results
ax1.errorbar(depths[1:], median_times[1:], yerr=std_times[1:],
             fmt='o-', capsize=3, markersize=4, label='With cross-link')
ax1.axhline(y=median_times[0], color='r', linestyle='--', linewidth=2,
            label=f'Baseline (no cross-link): {median_times[0]:.1f} µs')
ax1.fill_between([0, 41],
                  median_times[0] - std_times[0],
                  median_times[0] + std_times[0],
                  color='red', alpha=0.1)

ax1.set_xlabel('Cross-Link Depth', fontsize=12)
ax1.set_ylabel('Median Time (µs)', fontsize=12)
ax1.set_title('ID Derivative Computation Time vs Cross-Link Depth\n(40-link parallel chains)', fontsize=12)
ax1.legend(loc='upper left')
ax1.grid(True, alpha=0.3)
ax1.set_xlim(0, 41)
ax1.set_ylim(190, 220)

# Highlight the "slow" depths
slow_depths = [15, 16, 17, 18, 19, 20, 28, 33, 34, 35, 37, 38, 39]
for d in slow_depths:
    ax1.axvspan(d-0.3, d+0.3, alpha=0.2, color='orange')

# Right plot: Coefficient of variation (noise analysis)
cv = [s/m * 100 for m, s in zip(median_times, std_times)]
colors = ['red' if c > 5 else 'green' for c in cv[1:]]

ax2.bar(depths[1:], cv[1:], color=colors, alpha=0.7, edgecolor='black', linewidth=0.5)
ax2.axhline(y=5, color='orange', linestyle='--', linewidth=2, label='5% CV threshold')
ax2.axhline(y=cv[0], color='red', linestyle=':', linewidth=2,
            label=f'Baseline CV: {cv[0]:.1f}%')

ax2.set_xlabel('Cross-Link Depth', fontsize=12)
ax2.set_ylabel('Coefficient of Variation (%)', fontsize=12)
ax2.set_title('Measurement Noise Analysis\n(CV < 5% indicates stable measurement)', fontsize=12)
ax2.legend(loc='upper right')
ax2.grid(True, alpha=0.3, axis='y')
ax2.set_xlim(0, 41)
ax2.set_ylim(0, 15)

plt.tight_layout()
plt.savefig('/home/dvolpi/Source/alt-GRBDA/generalized_rbda/benchmark_figures/parallel_chain_depth.png',
            dpi=150, bbox_inches='tight')
plt.savefig('/home/dvolpi/Source/alt-GRBDA/generalized_rbda/benchmark_figures/parallel_chain_depth.pdf',
            bbox_inches='tight')
print("Saved: parallel_chain_depth.png and parallel_chain_depth.pdf")

# Also create a simplified summary plot
fig2, ax = plt.subplots(figsize=(10, 5))

# Group depths by timing behavior
baseline_time = median_times[0]
normal_depths = []
slow_depths_vals = []

for i, t in enumerate(median_times[1:], 1):
    if t > 205:
        slow_depths_vals.append((i, t))
    else:
        normal_depths.append((i, t))

normal_x, normal_y = zip(*normal_depths) if normal_depths else ([], [])
slow_x, slow_y = zip(*slow_depths_vals) if slow_depths_vals else ([], [])

ax.scatter(normal_x, normal_y, c='blue', s=50, label='Normal (~197 µs)', alpha=0.7)
ax.scatter(slow_x, slow_y, c='red', s=50, label='Elevated (~208 µs)', alpha=0.7)
ax.axhline(y=baseline_time, color='green', linestyle='--', linewidth=2,
           label=f'Baseline: {baseline_time:.1f} µs')

ax.set_xlabel('Cross-Link Depth', fontsize=12)
ax.set_ylabel('Median Time (µs)', fontsize=12)
ax.set_title('Cross-Link Depth vs Computation Time\n(Some depths show ~5% slower execution)', fontsize=12)
ax.legend()
ax.grid(True, alpha=0.3)
ax.set_xlim(0, 41)
ax.set_ylim(190, 215)

# Annotate the slow regions
ax.annotate('Depths 15-20', xy=(17.5, 208), fontsize=9, ha='center')
ax.annotate('Depth 28', xy=(28, 210), fontsize=9, ha='center')
ax.annotate('Depths 33-35, 37-39', xy=(36, 210), fontsize=9, ha='center')

plt.tight_layout()
plt.savefig('/home/dvolpi/Source/alt-GRBDA/generalized_rbda/benchmark_figures/parallel_chain_depth_summary.png',
            dpi=150, bbox_inches='tight')
print("Saved: parallel_chain_depth_summary.png")

plt.show()
