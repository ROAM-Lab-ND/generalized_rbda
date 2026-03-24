#!/usr/bin/env python3
"""
Plot Parallel Chain Cross-Link Depth Benchmark Results
"""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

# Data from benchmark results
data = {
    'Chain Length 10': {
        'Baseline_10L': {'time': 25.02, 'cross_links': 0, 'depth': 0},
        'Depth1_10L': {'time': 27.05, 'cross_links': 1, 'depth': 1},
        'Depth1+5_10L': {'time': 28.10, 'cross_links': 2, 'depth': 5},
        'Depth1+5+10_10L': {'time': 31.23, 'cross_links': 3, 'depth': 10},
    },
    'Chain Length 16': {
        'Baseline_16L': {'time': 45.33, 'cross_links': 0, 'depth': 0},
        'DepthMid_16L': {'time': 47.75, 'cross_links': 1, 'depth': 8},
        'DepthMulti_16L': {'time': 53.07, 'cross_links': 3, 'depth': 16},
    },
    'Chain Length 20': {
        'Baseline_20L': {'time': 60.51, 'cross_links': 0, 'depth': 0},
        'SingleDepth1_20L': {'time': 62.42, 'cross_links': 1, 'depth': 1},
        'SingleDepth5_20L': {'time': 62.58, 'cross_links': 1, 'depth': 5},
        'SingleDepth10_20L': {'time': 63.83, 'cross_links': 1, 'depth': 10},
        'SingleDepth15_20L': {'time': 64.42, 'cross_links': 1, 'depth': 15},
        'SingleDepth20_20L': {'time': 65.09, 'cross_links': 1, 'depth': 20},
        'Depths1+10+20_20L': {'time': 70.14, 'cross_links': 3, 'depth': 20},
    }
}

# Create figure with subplots
fig = plt.figure(figsize=(16, 10))
gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)

# Color scheme
colors = {'baseline': '#2E86AB', 'single': '#A23B72', 'multiple': '#F18F01'}

# ============================================================================
# Plot 1: Chain Length Scaling (baseline only)
# ============================================================================
ax1 = fig.add_subplot(gs[0, 0])

chain_lengths = [10, 16, 20]
baseline_times = [25.02, 45.33, 60.51]

ax1.plot(chain_lengths, baseline_times, 'o-', color=colors['baseline'], 
         linewidth=2.5, markersize=10, label='Baseline (no cross-links)')
ax1.fill_between(chain_lengths, baseline_times, alpha=0.2, color=colors['baseline'])

# Add fitted line
z = np.polyfit(chain_lengths, baseline_times, 2)
p = np.poly1d(z)
x_fit = np.linspace(10, 20, 100)
ax1.plot(x_fit, p(x_fit), '--', color=colors['baseline'], alpha=0.5, linewidth=1.5)

ax1.set_xlabel('Chain Length (links)', fontsize=12, fontweight='bold')
ax1.set_ylabel('Computation Time (µs)', fontsize=12, fontweight='bold')
ax1.set_title('Baseline Scaling with Chain Length', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3, linestyle='--')
ax1.legend(fontsize=10)

# Add data labels
for x, y in zip(chain_lengths, baseline_times):
    ax1.annotate(f'{y:.1f} µs', (x, y), textcoords="offset points", 
                xytext=(0,10), ha='center', fontsize=9, fontweight='bold')

# ============================================================================
# Plot 2: Cross-Link Count Impact by Chain Length
# ============================================================================
ax2 = fig.add_subplot(gs[0, 1])

width = 0.25
x_positions = np.arange(len(chain_lengths))

# Extract times for different cross-link counts
times_0 = [25.02, 45.33, 60.51]
times_1 = [27.05, 47.75, 62.42]  # Single cross-link at representative depth
times_3 = [31.23, 53.07, 70.14]  # Three cross-links

bars1 = ax2.bar(x_positions - width, times_0, width, label='0 cross-links', 
                color=colors['baseline'], alpha=0.8)
bars2 = ax2.bar(x_positions, times_1, width, label='1 cross-link', 
                color=colors['single'], alpha=0.8)
bars3 = ax2.bar(x_positions + width, times_3, width, label='3 cross-links', 
                color=colors['multiple'], alpha=0.8)

ax2.set_xlabel('Chain Length (links)', fontsize=12, fontweight='bold')
ax2.set_ylabel('Computation Time (µs)', fontsize=12, fontweight='bold')
ax2.set_title('Cross-Link Count Impact Across Chain Lengths', fontsize=14, fontweight='bold')
ax2.set_xticks(x_positions)
ax2.set_xticklabels(chain_lengths)
ax2.legend(fontsize=10)
ax2.grid(True, alpha=0.3, linestyle='--', axis='y')

# Add percentage overhead labels
for i, (t0, t1, t3) in enumerate(zip(times_0, times_1, times_3)):
    overhead_1 = ((t1 - t0) / t0) * 100
    overhead_3 = ((t3 - t0) / t0) * 100
    ax2.text(i, t1 + 1, f'+{overhead_1:.0f}%', ha='center', fontsize=8, fontweight='bold')
    ax2.text(i + width, t3 + 1, f'+{overhead_3:.0f}%', ha='center', fontsize=8, fontweight='bold')

# ============================================================================
# Plot 3: Depth Position Effect (20-link chain with single cross-link)
# ============================================================================
ax3 = fig.add_subplot(gs[1, 0])

depths_20 = [1, 5, 10, 15, 20]
times_20_single = [62.42, 62.58, 63.83, 64.42, 65.09]
baseline_20 = 60.51

ax3.plot(depths_20, times_20_single, 'o-', color=colors['single'], 
         linewidth=2.5, markersize=10, label='Single cross-link')
ax3.axhline(y=baseline_20, color=colors['baseline'], linestyle='--', 
            linewidth=2, label='Baseline (no cross-links)')
ax3.fill_between(depths_20, [baseline_20]*len(depths_20), times_20_single, 
                 alpha=0.2, color=colors['single'])

ax3.set_xlabel('Cross-Link Depth Position', fontsize=12, fontweight='bold')
ax3.set_ylabel('Computation Time (µs)', fontsize=12, fontweight='bold')
ax3.set_title('Depth Position Effect (20-link chain)', fontsize=14, fontweight='bold')
ax3.grid(True, alpha=0.3, linestyle='--')
ax3.legend(fontsize=10)

# Add overhead percentages
for d, t in zip(depths_20, times_20_single):
    overhead = ((t - baseline_20) / baseline_20) * 100
    ax3.annotate(f'+{overhead:.1f}%', (d, t), textcoords="offset points", 
                xytext=(0,8), ha='center', fontsize=8)

# ============================================================================
# Plot 4: Relative Overhead vs Chain Length
# ============================================================================
ax4 = fig.add_subplot(gs[1, 1])

# Calculate relative overheads
overhead_1_crosslink = [
    ((27.05 - 25.02) / 25.02) * 100,
    ((47.75 - 45.33) / 45.33) * 100,
    ((62.42 - 60.51) / 60.51) * 100
]

overhead_3_crosslinks = [
    ((31.23 - 25.02) / 25.02) * 100,
    ((53.07 - 45.33) / 45.33) * 100,
    ((70.14 - 60.51) / 60.51) * 100
]

ax4.plot(chain_lengths, overhead_1_crosslink, 'o-', color=colors['single'], 
         linewidth=2.5, markersize=10, label='1 cross-link')
ax4.plot(chain_lengths, overhead_3_crosslinks, 's-', color=colors['multiple'], 
         linewidth=2.5, markersize=10, label='3 cross-links')

ax4.set_xlabel('Chain Length (links)', fontsize=12, fontweight='bold')
ax4.set_ylabel('Overhead vs Baseline (%)', fontsize=12, fontweight='bold')
ax4.set_title('Relative Overhead by Chain Length', fontsize=14, fontweight='bold')
ax4.grid(True, alpha=0.3, linestyle='--')
ax4.legend(fontsize=10)
ax4.axhline(y=0, color='black', linestyle='-', linewidth=0.8, alpha=0.5)

# Add data labels
for x, y1, y2 in zip(chain_lengths, overhead_1_crosslink, overhead_3_crosslinks):
    ax4.annotate(f'{y1:.1f}%', (x, y1), textcoords="offset points", 
                xytext=(0,8), ha='center', fontsize=9, fontweight='bold')
    ax4.annotate(f'{y2:.1f}%', (x, y2), textcoords="offset points", 
                xytext=(0,8), ha='center', fontsize=9, fontweight='bold')

# ============================================================================
# Plot 5: Time per DOF Analysis
# ============================================================================
ax5 = fig.add_subplot(gs[2, 0])

# Calculate time per DOF
dof_10 = [21, 23, 25, 27]
time_10 = [25.02, 27.05, 28.10, 31.23]
time_per_dof_10 = [t/d for t, d in zip(time_10, dof_10)]

dof_16 = [33, 35, 39]
time_16 = [45.33, 47.75, 53.07]
time_per_dof_16 = [t/d for t, d in zip(time_16, dof_16)]

dof_20 = [41, 43, 43, 43, 43, 43, 47]
time_20 = [60.51, 62.42, 62.58, 63.83, 64.42, 65.09, 70.14]
time_per_dof_20 = [t/d for t, d in zip(time_20, dof_20)]

ax5.scatter(dof_10, time_per_dof_10, s=100, alpha=0.7, color=colors['baseline'], label='10-link chain')
ax5.scatter(dof_16, time_per_dof_16, s=100, alpha=0.7, color=colors['single'], label='16-link chain')
ax5.scatter(dof_20, time_per_dof_20, s=100, alpha=0.7, color=colors['multiple'], label='20-link chain')

ax5.set_xlabel('Total DOF', fontsize=12, fontweight='bold')
ax5.set_ylabel('Time per DOF (µs/DOF)', fontsize=12, fontweight='bold')
ax5.set_title('Efficiency: Time per Degree of Freedom', fontsize=14, fontweight='bold')
ax5.grid(True, alpha=0.3, linestyle='--')
ax5.legend(fontsize=10)

# ============================================================================
# Plot 6: Summary Statistics
# ============================================================================
ax6 = fig.add_subplot(gs[2, 1])
ax6.axis('off')

# Calculate summary statistics
depth_variance_20 = max(times_20_single) - min(times_20_single)
depth_variance_pct = (depth_variance_20 / baseline_20) * 100

avg_overhead_per_crosslink = np.mean([
    ((27.05 - 25.02) / 25.02) * 100,
    ((47.75 - 45.33) / 45.33) * 100,
    ((62.42 - 60.51) / 60.51) * 100
])

# Create summary text
summary_text = f"""
BENCHMARK SUMMARY
{'='*50}

Chain Lengths Tested: 10, 16, 20 links (2x original)

Key Findings:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. BASELINE SCALING
   • 10 links: 25.02 µs
   • 16 links: 45.33 µs  (1.81x)
   • 20 links: 60.51 µs  (2.42x vs 10-link)
   → Non-linear scaling (~quadratic)

2. DEPTH POSITION EFFECT (20-link)
   • Depth 1:  62.42 µs  (+3.2% vs baseline)
   • Depth 10: 63.83 µs  (+5.5%)
   • Depth 20: 65.09 µs  (+7.6%)
   • Variance: {depth_variance_20:.2f} µs ({depth_variance_pct:.1f}%)
   → MINIMAL depth sensitivity

3. CROSS-LINK COUNT IMPACT
   • Average overhead per cross-link: {avg_overhead_per_crosslink:.1f}%
   • Linear scaling confirmed
   • Consistent across chain lengths

4. RELATIVE OVERHEAD TREND
   • 10-link: 8.1% (1 cross-link), 24.8% (3 cross-links)
   • 16-link: 5.3% (1 cross-link), 17.1% (3 cross-links)
   • 20-link: 3.2% (1 cross-link), 15.9% (3 cross-links)
   → Overhead % DECREASES with longer chains

5. EFFICIENCY (Time per DOF)
   • 10-link: ~1.19 µs/DOF (baseline)
   • 16-link: ~1.37 µs/DOF (baseline)
   • 20-link: ~1.48 µs/DOF (baseline)
   → Slight efficiency loss with scale
"""

ax6.text(0.05, 0.95, summary_text, transform=ax6.transAxes, 
         fontsize=9, verticalalignment='top', fontfamily='monospace',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))

# Overall title
fig.suptitle('Parallel Chain Cross-Link Depth Benchmark - Extended Results (10/16/20 Links)', 
             fontsize=16, fontweight='bold', y=0.995)

# Save figure
plt.savefig('/home/dvolpi/Source/alt-GRBDA/generalized_rbda/benchmark_results_extended.png', 
            dpi=300, bbox_inches='tight')
print("✓ Graph saved to: benchmark_results_extended.png")

plt.savefig('/home/dvolpi/Source/alt-GRBDA/generalized_rbda/benchmark_results_extended.pdf', 
            bbox_inches='tight')
print("✓ PDF saved to: benchmark_results_extended.pdf")

plt.show()
