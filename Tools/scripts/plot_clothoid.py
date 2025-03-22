#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil
import argparse
import math

def load_log(filename):
    """Load the log file and extract clothoid data"""
    mlog = mavutil.mavlink_connection(filename)
    
    # Initialize data structures
    path_points = {
        'entry': {'x': [], 'y': [], 'hdg': []},
        'turn': {'x': [], 'y': [], 'hdg': []},
        'exit': {'x': [], 'y': [], 'hdg': []}
    }
    
    vehicle_path = {'x': [], 'y': [], 'hdg': [], 'time': []}
    states = []
    curvatures = []
    
    while True:
        msg = mlog.recv_match(type=['CLTP', 'CLTS', 'CLTH'])
        if msg is None:
            break
            
        if msg.get_type() == 'CLTP':
            # Path points
            if msg.Type == 0:  # Entry spiral
                path_points['entry']['x'].append(msg.PosX)
                path_points['entry']['y'].append(msg.PosY)
                path_points['entry']['hdg'].append(math.radians(msg.Hdg))
            elif msg.Type == 1:  # Constant turn
                path_points['turn']['x'].append(msg.PosX)
                path_points['turn']['y'].append(msg.PosY)
                path_points['turn']['hdg'].append(math.radians(msg.Hdg))
            elif msg.Type == 2:  # Exit spiral
                path_points['exit']['x'].append(msg.PosX)
                path_points['exit']['y'].append(msg.PosY)
                path_points['exit']['hdg'].append(math.radians(msg.Hdg))
                
        elif msg.get_type() == 'CLTS':
            # Vehicle state
            vehicle_path['x'].append(msg.PosX)
            vehicle_path['y'].append(msg.PosY)
            vehicle_path['hdg'].append(math.radians(msg.Hdg))
            vehicle_path['time'].append(msg.TimeUS * 1e-6)
            
        elif msg.get_type() == 'CLTH':
            # Navigation state and parameters
            states.append(msg.State)
            curvatures.append(msg.TargCrv)
    
    return path_points, vehicle_path, states, curvatures

def plot_heading_vector(x, y, hdg, length=1.0, color='k', alpha=0.5):
    """Plot a heading vector at the specified position"""
    dx = length * math.cos(hdg)
    dy = length * math.sin(hdg)
    plt.arrow(x, y, dx, dy, head_width=0.2, head_length=0.3, fc=color, ec=color, alpha=alpha)

def plot_clothoid_navigation(filename):
    """Plot the clothoid navigation visualization"""
    path_points, vehicle_path, states, curvatures = load_log(filename)
    
    # Create figure
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 16))
    
    # Plot path and vehicle trajectory
    ax1.set_title('Clothoid Navigation Path')
    
    # Plot path points
    ax1.plot(path_points['entry']['x'], path_points['entry']['y'], 'b-', label='Entry Spiral')
    ax1.plot(path_points['turn']['x'], path_points['turn']['y'], 'g-', label='Constant Turn')
    ax1.plot(path_points['exit']['x'], path_points['exit']['y'], 'r-', label='Exit Spiral')
    
    # Plot vehicle path
    ax1.plot(vehicle_path['x'], vehicle_path['y'], 'k:', label='Vehicle Path')
    
    # Plot heading vectors at key points
    for segment in path_points.values():
        for i in range(len(segment['x'])):
            plot_heading_vector(segment['x'][i], segment['y'][i], segment['hdg'][i])
    
    # Plot vehicle heading vectors at intervals
    step = max(1, len(vehicle_path['x']) // 20)  # Plot ~20 vectors along path
    for i in range(0, len(vehicle_path['x']), step):
        plot_heading_vector(vehicle_path['x'][i], vehicle_path['y'][i], 
                          vehicle_path['hdg'][i], color='gray')
    
    ax1.axis('equal')
    ax1.grid(True)
    ax1.legend()
    
    # Plot curvature over time
    ax2.set_title('Target Curvature vs Time')
    times = np.array(vehicle_path['time']) - vehicle_path['time'][0]
    ax2.plot(times, curvatures, 'b-', label='Target Curvature')
    
    # Add state transitions
    state_names = ['Straight', 'Entry', 'Turn', 'Exit']
    for i, state in enumerate(states):
        if i > 0 and states[i] != states[i-1]:
            ax2.axvline(times[i], color='r', linestyle='--', alpha=0.5)
            ax2.text(times[i], ax2.get_ylim()[1], state_names[state], 
                    rotation=90, verticalalignment='top')
    
    ax2.grid(True)
    ax2.legend()
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Curvature (1/m)')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot clothoid navigation data')
    parser.add_argument('logfile', help='Path to APM log file')
    args = parser.parse_args()
    
    plot_clothoid_navigation(args.logfile) 