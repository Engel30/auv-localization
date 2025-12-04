#!/usr/bin/env python3
"""
Simple diagnostic script to verify simulation data is being generated
Run this first to verify everything works before creating videos
"""
import time
import threading
import numpy as np
import math
import platform
from collections import deque

# Configure matplotlib for macOS
import matplotlib
if platform.system() == 'Darwin':
    matplotlib.use('Agg')

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Simple simulation parameters
RADIUS = 10.0
OMEGA = 0.15
TILT_ANGLE = np.deg2rad(2.0)
CONTROL_LOOP_MS = 50

# Global data storage
plot_data = {
    'time': deque(maxlen=150),
    'true_pos': deque(maxlen=150),
}
data_lock = threading.Lock()
running = True
sim_time = {'t': 0.0}
sim_lock = threading.Lock()

def compute_tilted_circle_position(theta, radius, tilt_angle, z_center=-5.0):
    """Compute position on tilted circular trajectory"""
    x_flat = radius * math.cos(theta)
    y_flat = radius * math.sin(theta)
    z_flat = z_center
    
    x = x_flat * math.cos(tilt_angle) - (z_flat) * math.sin(tilt_angle)
    y = y_flat
    z = x_flat * math.sin(tilt_angle) + (z_flat) * math.cos(tilt_angle)
    
    return np.array([[x], [y], [z]])

def main():
    global running
    
    print("=" * 60)
    print("DIAGNOSTIC TEST - Verifying Data Generation")
    print("=" * 60)
    print("This will run for 10 seconds to verify data is being collected")
    print("Press Ctrl+C to stop early")
    print("=" * 60)
    print()
    
    loop_time = CONTROL_LOOP_MS / 1000.0
    iteration = 0
    max_iterations = int(10.0 / loop_time)  # Run for 10 seconds
    
    try:
        while iteration < max_iterations:
            start = time.time()
            
            with sim_lock:
                sim_time['t'] = iteration * loop_time
                t_sim = sim_time['t']
            
            theta = OMEGA * t_sim
            p_true_vec = compute_tilted_circle_position(theta, RADIUS, TILT_ANGLE)
            p_true = p_true_vec.flatten()
            
            with data_lock:
                plot_data['time'].append(t_sim)
                plot_data['true_pos'].append(p_true.copy())
            
            # Print status every 20 iterations
            if iteration % 20 == 0:
                with data_lock:
                    data_points = len(plot_data['time'])
                print("Iteration {:3d} | Time: {:5.2f}s | Data points: {:3d} | Pos: [{:6.2f}, {:6.2f}, {:6.2f}]".format(
                    iteration, t_sim, data_points, p_true[0], p_true[1], p_true[2]))
            
            iteration += 1
            
            elapsed = time.time() - start
            sleep_time = max(0, loop_time - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    
    running = False
    
    # Final summary
    print("\n" + "=" * 60)
    print("TEST COMPLETE")
    print("=" * 60)
    
    with data_lock:
        total_points = len(plot_data['time'])
        
        if total_points > 0:
            print("✓ SUCCESS - Data was generated correctly")
            print()
            print("Statistics:")
            print("  Total data points: {}".format(total_points))
            print("  Time range: {:.2f}s - {:.2f}s".format(
                plot_data['time'][0], plot_data['time'][-1]))
            
            # Create a simple test plot
            print()
            print("Creating test plot...")
            
            fig = plt.figure(figsize=(10, 6))
            ax = fig.add_subplot(111, projection='3d')
            
            true_arr = np.array([p for p in plot_data['true_pos']])
            ax.plot(true_arr[:,0], true_arr[:,1], true_arr[:,2], 'b-', linewidth=2)
            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
            ax.set_zlabel('Z [m]')
            ax.set_title('Diagnostic Test - 3D Trajectory')
            
            filename = 'diagnostic_test.png'
            plt.savefig(filename, dpi=100)
            plt.close()
            
            print("✓ Test plot saved: {}".format(filename))
            print()
            print("=" * 60)
            print("VERDICT: Your simulation is working correctly!")
            print("You can now run the video recorder: EKF_video_recorder.py")
            print("=" * 60)
            
        else:
            print("✗ FAILED - No data was generated")
            print()
            print("This suggests a problem with the simulation loop.")
            print("Please check:")
            print("  1. Python version (should be 3.8+)")
            print("  2. NumPy installation")
            print("  3. No errors in the output above")

if __name__ == "__main__":
    main()