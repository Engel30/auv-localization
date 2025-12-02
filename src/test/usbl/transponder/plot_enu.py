#!/usr/bin/env python3
"""
Parse USBL transponder log file and plot ENU positions in 3D.
"""

import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def parse_log(filepath):
    """Extract ENU positions from the log file."""
    positions = []
    
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    east = north = up = None
    position_num = None
    
    for line in lines:
        # Match position number
        pos_match = re.search(r'POSITION RECEIVED #(\d+)', line)
        if pos_match:
            position_num = int(pos_match.group(1))
        
        # Match East value
        east_match = re.search(r'East:\s+([-\d.]+)\s*m', line)
        if east_match:
            east = float(east_match.group(1))
        
        # Match North value
        north_match = re.search(r'North:\s+([-\d.]+)\s*m', line)
        if north_match:
            north = float(north_match.group(1))
        
        # Match Up value
        up_match = re.search(r'Up:\s+([-\d.]+)\s*m', line)
        if up_match:
            up = float(up_match.group(1))
            # Once we have Up, we have all three values
            if east is not None and north is not None:
                positions.append({
                    'num': position_num,
                    'east': east,
                    'north': north,
                    'up': up
                })
                east = north = up = None
    
    return positions

def plot_3d(positions):
    """Create a 3D plot of ENU positions."""
    east = [p['east'] for p in positions]
    north = [p['north'] for p in positions]
    up = [p['up'] for p in positions]
    nums = [p['num'] for p in positions]
    
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # Create color gradient based on time/sequence
    colors = plt.cm.viridis(np.linspace(0, 1, len(positions)))
    
    # Plot the trajectory line
    ax.plot(east, north, up, 'b-', alpha=0.5, linewidth=1, label='Trajectory')
    
    # Plot points with color gradient (earlier = darker, later = lighter)
    scatter = ax.scatter(east, north, up, c=nums, cmap='viridis', 
                         s=80, edgecolors='black', linewidth=0.5)
    
    # Mark start and end points
    ax.scatter([east[0]], [north[0]], [up[0]], c='green', s=150, 
               marker='^', edgecolors='black', linewidth=1.5, label='Start', zorder=5)
    ax.scatter([east[-1]], [north[-1]], [up[-1]], c='red', s=150, 
               marker='s', edgecolors='black', linewidth=1.5, label='End', zorder=5)
    
    # Add colorbar
    cbar = plt.colorbar(scatter, ax=ax, shrink=0.6, pad=0.1)
    cbar.set_label('Position #', fontsize=11)
    
    # Labels and title
    ax.set_xlabel('East (m)', fontsize=12, labelpad=10)
    ax.set_ylabel('North (m)', fontsize=12, labelpad=10)
    ax.set_zlabel('Up (m)', fontsize=12, labelpad=10)
    ax.set_title('USBL Transponder ENU Positions', fontsize=14, fontweight='bold')
    
    # Add legend
    ax.legend(loc='upper left')
    
    # Set equal aspect ratio for better visualization
    max_range = max(
        max(east) - min(east),
        max(north) - min(north),
        max(up) - min(up)
    ) / 2.0
    
    mid_e = (max(east) + min(east)) / 2.0
    mid_n = (max(north) + min(north)) / 2.0
    mid_u = (max(up) + min(up)) / 2.0
    
    ax.set_xlim(mid_e - max_range, mid_e + max_range)
    ax.set_ylim(mid_n - max_range, mid_n + max_range)
    ax.set_zlim(mid_u - max_range, mid_u + max_range)
    
    plt.tight_layout()
    return fig

def main():
    log_file = '/Users/matteocolletta/Documents/GitHub/2025_Jetson_HL_Nav-Guidance/logs/log_vicino.log'
    
    print("Parsing log file...")
    positions = parse_log(log_file)
    
    print(f"Found {len(positions)} position readings:")
    print("-" * 50)
    for p in positions:
        print(f"  #{p['num']:2d}: E={p['east']:+.2f}m, N={p['north']:+.2f}m, U={p['up']:+.2f}m")
    print("-" * 50)
    
    print("\nCreating 3D plot...")
    fig = plot_3d(positions)
    
    output_path = '/Users/matteocolletta/Documents/GitHub/2025_Jetson_HL_Nav-Guidance/logs/plots/enu_positions_3d.png'
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_path}")
    
    plt.show()

if __name__ == '__main__':
    main()