#!/usr/bin/env python3
"""
Plot OSQP optimizer convergence from debug logs.

Usage:
    python3 plot_osqp_convergence.py <log_file> [output_dir]
    
Example:
    python3 plot_osqp_convergence.py /tmp/osqp_debug.log ./plots
"""

import re
import json
import sys
from pathlib import Path
from collections import defaultdict
import numpy as np

def extract_osqp_iterations(log_file):
    """Parse OSQP/TrajOpt optimization data from log file - ALL cost components."""
    iterations = defaultdict(lambda: defaultdict(list))
    
    with open(log_file, 'r') as f:
        lines = f.readlines()
    
    chunk_id = None
    iteration_count = {}
    
    for line in lines:
        # Detect chunk boundaries
        if 'Chunk' in line and '/' in line:
            match = re.search(r'Chunk\s+(\d+)/(\d+)', line)
            if match:
                chunk_id = int(match.group(1))
                iteration_count[chunk_id] = 0
        
        # Parse cost table rows - format: | ---------- | value | value | ... | cost_name
        # Example: | ---------- |  1.829e+02 |  1.642e+02 |  1.642e+02 |  1.868e+01 |  1.868e+01 |  1.000e+00 | joint_vel_cost
        cost_line_pattern = r'\|\s*-+\s*\|(.*?)\|\s*(\w+(?:_\w+)*)\s*$'
        
        match = re.search(cost_line_pattern, line)
        if match and chunk_id is not None:
            values_str = match.group(1)
            cost_name = match.group(2).strip()
            
            # Only process actual cost metrics (skip header lines)
            if cost_name in ['joint_vel_cost', 'joint_accel_cost', 'joint_jerk_cost', 'SUM', 'TOTAL']:
                # Extract numerical values from the values string
                # Format: |  1.829e+02 |  1.642e+02 |  1.642e+02 |  1.868e+01 |
                values = re.findall(r'([\d\.\+\-e]+)', values_str)
                
                if values:
                    try:
                        # Usually the second value is the relevant one (current iteration)
                        cost_val = float(values[1]) if len(values) > 1 else float(values[0])
                        
                        # Ensure we have entry for this cost type
                        if cost_name not in iterations[chunk_id]:
                            iterations[chunk_id][cost_name] = []
                        
                        iterations[chunk_id][cost_name].append(cost_val)
                        
                        # Keep track of iteration numbers
                        if 'iter' not in iterations[chunk_id]:
                            iterations[chunk_id]['iter'] = []
                        if len(iterations[chunk_id][cost_name]) > len(iterations[chunk_id]['iter']):
                            iterations[chunk_id]['iter'].append(len(iterations[chunk_id][cost_name]) - 1)
                    except (ValueError, IndexError):
                        pass
    
    return dict(iterations)

def extract_cost_data(log_file):
    """Extract cost/objective values from log."""
    costs = defaultdict(list)
    
    with open(log_file, 'r') as f:
        lines = f.readlines()
    
    # Pattern for objective/cost values
    cost_pattern = r'cost[:\s]*([\d\.\+\-e]+)|objective[:\s]*([\d\.\+\-e]+)'
    
    chunk_id = None
    for line in lines:
        if 'Chunk' in line and '/' in line:
            match = re.search(r'Chunk\s+(\d+)/(\d+)', line)
            if match:
                chunk_id = int(match.group(1))
        
        match = re.search(cost_pattern, line, re.IGNORECASE)
        if match and chunk_id is not None:
            try:
                cost_val = float(match.group(1) or match.group(2))
                costs[chunk_id].append(cost_val)
            except (ValueError, TypeError):
                pass
    
    return dict(costs)

def save_as_json(iterations, output_file):
    """Save extracted data as JSON."""
    # Convert to serializable format
    data = {
        'chunks': {}
    }
    
    for chunk_id, chunk_data in iterations.items():
        data['chunks'][str(chunk_id)] = {
            k: v for k, v in chunk_data.items()
        }
    
    with open(output_file, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"✓ Data saved to {output_file}")

def plot_convergence(iterations, output_dir):
    """Generate convergence plots for all cost components."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("WARNING: matplotlib not installed. Skipping plots.")
        return False
    
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    if not iterations:
        print("No iteration data to plot")
        return False
    
    # Check if we have cost data
    has_any_cost = False
    all_cost_types = set()
    for chunk_data in iterations.values():
        for key in chunk_data.keys():
            if key not in ['iter', 'cost']:
                has_any_cost = True
                all_cost_types.add(key)
    
    if not has_any_cost:
        print("No cost component data found to plot")
        return False
    
    # Remove 'SUM' from detail plot (we'll plot separately)
    detail_costs = sorted([c for c in all_cost_types if c not in ['SUM', 'TOTAL']])
    has_sum = 'SUM' in all_cost_types or 'TOTAL' in all_cost_types
    sum_key = 'SUM' if 'SUM' in all_cost_types else 'TOTAL'
    
    # Create multi-panel plot
    num_plots = 2 if has_sum else 1
    fig, axes = plt.subplots(num_plots, 1, figsize=(14, 5 * num_plots))
    if num_plots == 1:
        axes = [axes]
    fig.suptitle('TrajOpt Optimization Cost Components', fontsize=16, fontweight='bold')
    
    # Plot 1: Individual cost components
    for chunk_id, chunk_data in sorted(iterations.items()):
        if 'iter' not in chunk_data or not chunk_data['iter']:
            continue
        
        iters = np.array(chunk_data['iter'])
        
        for cost_type in detail_costs:
            if cost_type in chunk_data and chunk_data[cost_type]:
                costs = np.array(chunk_data[cost_type])
                if len(costs) == len(iters):
                    axes[0].plot(iters, costs, 'o-', 
                               label=f'C{chunk_id} {cost_type}', 
                               alpha=0.7, linewidth=2)
    
    axes[0].set_xlabel('Iteration/Attempt', fontsize=11)
    axes[0].set_ylabel('Cost', fontsize=11)
    axes[0].set_title('Individual Cost Components', fontsize=12)
    axes[0].legend(fontsize=9, loc='best', ncol=2)
    axes[0].grid(True, alpha=0.3)
    
    # Plot 2: Total cost and individual components overlaid
    if has_sum:
        for chunk_id, chunk_data in sorted(iterations.items()):
            if 'iter' not in chunk_data or not chunk_data['iter']:
                continue
            
            iters = np.array(chunk_data['iter'])
            
            # Plot total (thick line)
            if sum_key in chunk_data and chunk_data[sum_key]:
                sum_costs = np.array(chunk_data[sum_key])
                if len(sum_costs) == len(iters):
                    axes[1].plot(iters, sum_costs, 'o-', 
                               label=f'C{chunk_id} {sum_key} (Total)', 
                               alpha=0.9, linewidth=3, markersize=8)
            
            # Plot components (thin lines)
            for cost_type in detail_costs:
                if cost_type in chunk_data and chunk_data[cost_type]:
                    costs = np.array(chunk_data[cost_type])
                    if len(costs) == len(iters):
                        axes[1].plot(iters, costs, '--', 
                                   label=f'C{chunk_id} {cost_type}', 
                                   alpha=0.5, linewidth=1.5)
        
        axes[1].set_xlabel('Iteration/Attempt', fontsize=11)
        axes[1].set_ylabel('Cost', fontsize=11)
        axes[1].set_title('Total & Component Costs Overlay', fontsize=12)
        axes[1].legend(fontsize=8, loc='best', ncol=2)
        axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plot_file = output_path / 'osqp_convergence.png'
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f"✓ Main plot saved to {plot_file}")
    
    # Create additional log-scale plot for better visibility
    fig2, ax = plt.subplots(figsize=(12, 6))
    fig2.suptitle('TrajOpt Optimization - Log Scale', fontsize=14, fontweight='bold')
    
    for chunk_id, chunk_data in sorted(iterations.items()):
        if 'iter' not in chunk_data or not chunk_data['iter']:
            continue
        
        iters = np.array(chunk_data['iter'])
        
        # Plot all costs including sum
        for cost_type in sorted(all_cost_types):
            if cost_type in chunk_data and chunk_data[cost_type]:
                costs = np.array(chunk_data[cost_type])
                if len(costs) == len(iters) and np.all(costs > 0):
                    is_sum = cost_type in ['SUM', 'TOTAL']
                    ax.semilogy(iters, costs, 'o-' if is_sum else '--', 
                              label=f'C{chunk_id} {cost_type}',
                              alpha=0.8 if is_sum else 0.5,
                              linewidth=2.5 if is_sum else 1.5,
                              markersize=6 if is_sum else 4)
    
    ax.set_xlabel('Iteration/Attempt', fontsize=11)
    ax.set_ylabel('Cost (log scale)', fontsize=11)
    ax.set_title('All Cost Components (Log Scale)', fontsize=12)
    ax.legend(fontsize=9, loc='best', ncol=2)
    ax.grid(True, alpha=0.3, which='both')
    
    plt.tight_layout()
    log_plot_file = output_path / 'osqp_convergence_log.png'
    plt.savefig(log_plot_file, dpi=150, bbox_inches='tight')
    print(f"✓ Log-scale plot saved to {log_plot_file}")
    
    # Print detailed statistics
    print("\n" + "="*70)
    print("Detailed Cost Convergence Analysis")
    print("="*70)
    
    chunks = sorted(iterations.keys())
    for chunk_id in chunks:
        chunk_data = iterations[chunk_id]
        if chunk_data.get('iter'):
            print(f"\n📊 Chunk {chunk_id}:")
            print(f"   Iterations: {len(chunk_data['iter'])}")
            
            for cost_type in sorted(all_cost_types):
                if cost_type in chunk_data and chunk_data[cost_type]:
                    costs = chunk_data[cost_type]
                    start = costs[0]
                    end = costs[-1]
                    improvement = start - end
                    pct = 100 * improvement / start if start != 0 else 0
                    
                    marker = "📉" if improvement > 0 else "📈"
                    print(f"   {marker} {cost_type:20s}: {start:9.2f} → {end:9.2f}  " 
                          f"({improvement:+7.2f}, {pct:+6.1f}%)")
    
    return True

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_osqp_convergence.py <log_file> [output_dir]")
        sys.exit(1)
    
    log_file = Path(sys.argv[1])
    if not log_file.exists():
        print(f"ERROR: Log file not found: {log_file}")
        sys.exit(1)
    
    output_dir = sys.argv[2] if len(sys.argv) > 2 else log_file.parent / "osqp_plots"
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Parsing OSQP log: {log_file}")
    
    # Extract data
    iterations = extract_osqp_iterations(str(log_file))
    costs = extract_cost_data(str(log_file))
    
    if not iterations:
        print("WARNING: No OSQP iteration data found in log file.")
        print("Ensure optimizer debug logging is enabled.")
    
    # Save as JSON
    json_file = output_dir / "osqp_iterations.json"
    save_as_json(iterations, json_file)
    
    # Generate plots
    plot_convergence(iterations, output_dir)
    
    print(f"\n✓ All outputs saved to: {output_dir}")

if __name__ == "__main__":
    main()
