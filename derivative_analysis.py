#!/usr/bin/env python3
"""
Constraint Derivative Analysis Script
Demonstrates the implementation of derivative functions for h1, h2, h3, h4
Based on the formulations in README.md
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from simulation import RobotSimulation
import os

# Create plots directory if it doesn't exist
os.makedirs('plots', exist_ok=True)

def analyze_derivatives_along_trajectory():
    """
    Analyze constraint derivatives along a complete robot trajectory
    """
    print("="*80)
    print("CONSTRAINT DERIVATIVES ANALYSIS ALONG TRAJECTORY")
    print("="*80)
    
    # Create simulation instance
    sim = RobotSimulation()
    sim.generate_random_trajectory()
    
    # Time points for analysis
    time_points = np.linspace(0, sim.total_time, 50)
    
    # Storage for analysis data
    analysis_data = {
        'time': [],
        'h_values': {'h_d': [], 'h_y': [], 'h_s': [], 'h_a': []},
        'h_derivatives': {'h_d': [], 'h_y': [], 'h_s': [], 'h_a': []},
        'spatial_derivatives': {
            'h_d': {'x': [], 'y': [], 'theta': []},
            'h_y': {'x': [], 'y': [], 'theta': []},
            'h_s': {'x': [], 'y': [], 'theta': []},
            'h_a': {'x': [], 'y': [], 'theta': []}
        },
        'cbf_conditions': {'h_d': [], 'h_y': [], 'h_s': [], 'h_a': []},
        'robot_pos': [],
        'robot_vel': [],
        'distances': []
    }
    
    print(f"Analyzing {len(time_points)} time points along trajectory...")
    
    for t in time_points:
        # Get robot state
        robot_pos, robot_vel = sim.get_robot_state_at_time(t)
        robot_acc = np.array([0.0, 0.0])  # Constant velocity motion
        
        # Calculate constraint values
        h_d, h_y, h_s, h_a, h_min, min_distance = sim.calculate_constraints(robot_pos, robot_vel)
        
        # Calculate derivatives
        h_d_dot, h_y_dot, h_s_dot, h_a_dot = sim.calculate_constraint_derivatives(
            robot_pos, robot_vel, robot_acc
        )
        
        # Calculate spatial derivatives
        spatial_gradients = sim.calculate_spatial_derivatives(robot_pos, robot_vel)
        
        # Store data
        analysis_data['time'].append(t)
        analysis_data['h_values']['h_d'].append(h_d)
        analysis_data['h_values']['h_y'].append(h_y if h_y is not None else np.nan)
        analysis_data['h_values']['h_s'].append(h_s)
        analysis_data['h_values']['h_a'].append(h_a)
        
        analysis_data['h_derivatives']['h_d'].append(h_d_dot)
        analysis_data['h_derivatives']['h_y'].append(h_y_dot if h_y_dot is not None else np.nan)
        analysis_data['h_derivatives']['h_s'].append(h_s_dot)
        analysis_data['h_derivatives']['h_a'].append(h_a_dot)
        
        # Store spatial derivatives
        for constraint in ['h_d', 'h_y', 'h_s', 'h_a']:
            analysis_data['spatial_derivatives'][constraint]['x'].append(spatial_gradients[constraint]['x'])
            analysis_data['spatial_derivatives'][constraint]['y'].append(spatial_gradients[constraint]['y'])
            analysis_data['spatial_derivatives'][constraint]['theta'].append(spatial_gradients[constraint]['theta'])
        
        # Calculate CBF conditions: ḣ + α·h
        alpha = 1.0
        analysis_data['cbf_conditions']['h_d'].append(h_d_dot + alpha * h_d)
        analysis_data['cbf_conditions']['h_y'].append(
            (h_y_dot + alpha * h_y) if (h_y_dot is not None and h_y is not None) else np.nan
        )
        analysis_data['cbf_conditions']['h_s'].append(h_s_dot + alpha * h_s)
        analysis_data['cbf_conditions']['h_a'].append(h_a_dot + alpha * h_a)
        
        analysis_data['robot_pos'].append(robot_pos.copy())
        analysis_data['robot_vel'].append(robot_vel.copy())
        analysis_data['distances'].append(min_distance)
    
    return sim, analysis_data

def plot_derivative_analysis(sim, analysis_data):
    """
    Create comprehensive plots of constraint derivatives analysis
    """
    fig, axes = plt.subplots(3, 2, figsize=(16, 18))
    fig.suptitle('Constraint Derivatives Analysis', fontsize=16, fontweight='bold')
    
    time = analysis_data['time']
    constraint_names = ['h_d', 'h_y', 'h_s', 'h_a']
    constraint_labels = ['Distance', 'Yielding', 'Speed', 'Acceleration']
    colors = ['blue', 'green', 'orange', 'purple']
    
    # Plot 1: Constraint Values
    ax1 = axes[0, 0]
    for i, (name, label, color) in enumerate(zip(constraint_names, constraint_labels, colors)):
        values = analysis_data['h_values'][name]
        ax1.plot(time, values, label=f'h_{{{label.lower()[0]}}} ({label})', 
                color=color, linewidth=2)
    
    ax1.axhline(y=0, color='red', linestyle='--', alpha=0.7, label='Safety Boundary')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Constraint Value')
    ax1.set_title('Constraint Functions h_i(t)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Constraint Derivatives
    ax2 = axes[0, 1]
    for i, (name, label, color) in enumerate(zip(constraint_names, constraint_labels, colors)):
        derivatives = analysis_data['h_derivatives'][name]
        ax2.plot(time, derivatives, label=f'ḣ_{{{label.lower()[0]}}} ({label})', 
                color=color, linewidth=2, linestyle='--')
    
    ax2.axhline(y=0, color='red', linestyle='--', alpha=0.7)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Constraint Derivative')
    ax2.set_title('Constraint Derivatives ḣ_i(t)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: CBF Conditions
    ax3 = axes[1, 0]
    for i, (name, label, color) in enumerate(zip(constraint_names, constraint_labels, colors)):
        cbf_values = analysis_data['cbf_conditions'][name]
        ax3.plot(time, cbf_values, label=f'CBF_{{{label.lower()[0]}}} ({label})', 
                color=color, linewidth=2)
    
    ax3.axhline(y=0, color='red', linestyle='--', alpha=0.7, label='CBF Boundary')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('CBF Condition Value')
    ax3.set_title('Control Barrier Function Conditions: ḣ_i + α·h_i ≥ 0')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Robot Trajectory
    ax4 = axes[1, 1]
    positions = np.array(analysis_data['robot_pos'])
    ax4.plot(positions[:, 0], positions[:, 1], 'k-', linewidth=2, label='Robot Path')
    ax4.plot(sim.start_point[0], sim.start_point[1], 'go', markersize=10, label='Start')
    ax4.plot(sim.goal_point[0], sim.goal_point[1], 'b^', markersize=10, label='Goal')
    
    # Plot humans and safety circles
    for i, human_pos in enumerate(sim.human_pos):
        ax4.plot(human_pos[0], human_pos[1], 'ro', markersize=12, label=f'Human {i+1}' if i == 0 else '')
        circle_front = Circle((human_pos[0], human_pos[1]), 3.0, fill=False, color='red', linestyle='--', alpha=0.5)
        circle_side = Circle((human_pos[0], human_pos[1]), 1.2, fill=False, color='orange', linestyle='--', alpha=0.5)
        ax4.add_patch(circle_front)
        ax4.add_patch(circle_side)
    
    ax4.set_xlabel('X (m)')
    ax4.set_ylabel('Y (m)')
    ax4.set_title('Robot Trajectory')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.set_aspect('equal')
    
    # Plot 5: Distance to Humans
    ax5 = axes[2, 0]
    ax5.plot(time, analysis_data['distances'], 'black', linewidth=2, label='Min Distance to Human')
    ax5.axhline(y=3.0, color='red', linestyle='--', alpha=0.7, label='Front Region (ρ₀=3.0m)')
    ax5.axhline(y=1.2, color='orange', linestyle='--', alpha=0.7, label='Side Region (ρ₁=1.2m)')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Distance (m)')
    ax5.set_title('Distance to Closest Human')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # Plot 6: Speed Profile
    ax6 = axes[2, 1]
    velocities = np.array(analysis_data['robot_vel'])
    speeds = np.linalg.norm(velocities, axis=1)
    ax6.plot(time, speeds, 'purple', linewidth=2, label='Robot Speed')
    ax6.axhline(y=1.0, color='red', linestyle='--', alpha=0.7, label='Max Speed (V_M=1.0 m/s)')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Speed (m/s)')
    ax6.set_title('Robot Speed Profile')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save the plot
    filename = 'plots/derivative_analysis.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"\n📁 Derivative analysis plot saved to: {filename}")
    
    plt.show()
    
    return fig

def plot_spatial_derivatives(sim, analysis_data):
    """
    Create plots specifically for spatial derivatives analysis
    """
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Spatial Derivatives Analysis (∇h_i)', fontsize=16, fontweight='bold')
    
    time = analysis_data['time']
    constraint_names = ['h_d', 'h_y', 'h_s', 'h_a']
    constraint_labels = ['Distance', 'Yielding', 'Speed', 'Acceleration']
    colors = ['blue', 'green', 'orange', 'purple']
    
    # Plot 1: X-direction spatial derivatives
    ax1 = axes[0, 0]
    for i, (name, label, color) in enumerate(zip(constraint_names, constraint_labels, colors)):
        x_derivatives = analysis_data['spatial_derivatives'][name]['x']
        ax1.plot(time, x_derivatives, label=f'∂h_{{{label.lower()[0]}}}/∂x ({label})', 
                color=color, linewidth=2)
    
    ax1.axhline(y=0, color='red', linestyle='--', alpha=0.7)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('∂h/∂x')
    ax1.set_title('X-Direction Spatial Derivatives')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Y-direction spatial derivatives
    ax2 = axes[0, 1]
    for i, (name, label, color) in enumerate(zip(constraint_names, constraint_labels, colors)):
        y_derivatives = analysis_data['spatial_derivatives'][name]['y']
        ax2.plot(time, y_derivatives, label=f'∂h_{{{label.lower()[0]}}}/∂y ({label})', 
                color=color, linewidth=2)
    
    ax2.axhline(y=0, color='red', linestyle='--', alpha=0.7)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('∂h/∂y')
    ax2.set_title('Y-Direction Spatial Derivatives')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Gradient magnitude
    ax3 = axes[1, 0]
    for i, (name, label, color) in enumerate(zip(constraint_names, constraint_labels, colors)):
        x_derivatives = np.array(analysis_data['spatial_derivatives'][name]['x'])
        y_derivatives = np.array(analysis_data['spatial_derivatives'][name]['y'])
        gradient_magnitude = np.sqrt(x_derivatives**2 + y_derivatives**2)
        ax3.plot(time, gradient_magnitude, label=f'||∇h_{{{label.lower()[0]}}}|| ({label})', 
                color=color, linewidth=2)
    
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('||∇h||')
    ax3.set_title('Gradient Magnitude')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Gradient direction (angle)
    ax4 = axes[1, 1]
    for i, (name, label, color) in enumerate(zip(constraint_names, constraint_labels, colors)):
        x_derivatives = np.array(analysis_data['spatial_derivatives'][name]['x'])
        y_derivatives = np.array(analysis_data['spatial_derivatives'][name]['y'])
        gradient_angle = np.arctan2(y_derivatives, x_derivatives) * 180 / np.pi
        ax4.plot(time, gradient_angle, label=f'∠∇h_{{{label.lower()[0]}}} ({label})', 
                color=color, linewidth=2)
    
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Gradient Angle (degrees)')
    ax4.set_title('Gradient Direction')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save the plot
    filename = 'plots/spatial_derivatives_analysis.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"\n📁 Spatial derivatives analysis plot saved to: {filename}")
    
    plt.show()
    
    return fig

def plot_comprehensive_derivatives(sim, analysis_data):
    """
    Create a comprehensive plot showing both time and spatial derivatives
    """
    fig, axes = plt.subplots(4, 2, figsize=(18, 20))
    fig.suptitle('Comprehensive Derivatives Analysis', fontsize=16, fontweight='bold')
    
    time = analysis_data['time']
    constraint_names = ['h_d', 'h_y', 'h_s', 'h_a']
    constraint_labels = ['Distance', 'Yielding', 'Speed', 'Acceleration']
    colors = ['blue', 'green', 'orange', 'purple']
    
    for i, (name, label, color) in enumerate(zip(constraint_names, constraint_labels, colors)):
        # Left column: Time derivatives
        ax_time = axes[i, 0]
        time_derivatives = analysis_data['h_derivatives'][name]
        ax_time.plot(time, time_derivatives, color=color, linewidth=2, label=f'ḣ_{{{label.lower()[0]}}}')
        ax_time.axhline(y=0, color='red', linestyle='--', alpha=0.7)
        ax_time.set_xlabel('Time (s)')
        ax_time.set_ylabel(f'ḣ_{{{label.lower()[0]}}}')
        ax_time.set_title(f'{label} - Time Derivative')
        ax_time.legend()
        ax_time.grid(True, alpha=0.3)
        
        # Right column: Spatial derivatives (X and Y components)
        ax_spatial = axes[i, 1]
        x_derivatives = analysis_data['spatial_derivatives'][name]['x']
        y_derivatives = analysis_data['spatial_derivatives'][name]['y']
        
        ax_spatial.plot(time, x_derivatives, color=color, linewidth=2, label=f'∂h_{{{label.lower()[0]}}}/∂x')
        ax_spatial.plot(time, y_derivatives, color=color, linewidth=2, linestyle='--', label=f'∂h_{{{label.lower()[0]}}}/∂y')
        ax_spatial.axhline(y=0, color='red', linestyle='--', alpha=0.7)
        ax_spatial.set_xlabel('Time (s)')
        ax_spatial.set_ylabel(f'∇h_{{{label.lower()[0]}}}')
        ax_spatial.set_title(f'{label} - Spatial Derivatives')
        ax_spatial.legend()
        ax_spatial.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save the plot
    filename = 'plots/comprehensive_derivatives_analysis.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"\n📁 Comprehensive derivatives analysis plot saved to: {filename}")
    
    plt.show()
    
    return fig

def print_derivative_summary(analysis_data):
    """
    Print a summary of the derivative analysis
    """
    print("\n" + "="*80)
    print("DERIVATIVE ANALYSIS SUMMARY")
    print("="*80)
    
    time = np.array(analysis_data['time'])
    
    print("Constraint Values Statistics:")
    print("-" * 40)
    for name, label in [('h_d', 'Distance'), ('h_y', 'Yielding'), ('h_s', 'Speed'), ('h_a', 'Acceleration')]:
        values = np.array(analysis_data['h_values'][name])
        valid_values = values[~np.isnan(values)]
        
        if len(valid_values) > 0:
            print(f"{label:12} (h_{name[2]}): min={np.min(valid_values):6.3f}, "
                  f"max={np.max(valid_values):6.3f}, mean={np.mean(valid_values):6.3f}")
            
            # Check for violations
            violations = np.sum(valid_values < 0)
            if violations > 0:
                print(f"             ⚠️  {violations}/{len(valid_values)} time points violated!")
        else:
            print(f"{label:12} (h_{name[2]}): N/A (not active)")
    
    print("\nConstraint Derivatives Statistics:")
    print("-" * 40)
    for name, label in [('h_d', 'Distance'), ('h_y', 'Yielding'), ('h_s', 'Speed'), ('h_a', 'Acceleration')]:
        derivatives = np.array(analysis_data['h_derivatives'][name])
        valid_derivatives = derivatives[~np.isnan(derivatives)]
        
        if len(valid_derivatives) > 0:
            print(f"ḣ_{name[2]} ({label:9}): min={np.min(valid_derivatives):6.3f}, "
                  f"max={np.max(valid_derivatives):6.3f}, mean={np.mean(valid_derivatives):6.3f}")
        else:
            print(f"ḣ_{name[2]} ({label:9}): N/A (not active)")
    
    print("\nSpatial Derivatives Statistics:")
    print("-" * 40)
    for name, label in [('h_d', 'Distance'), ('h_y', 'Yielding'), ('h_s', 'Speed'), ('h_a', 'Acceleration')]:
        x_derivatives = np.array(analysis_data['spatial_derivatives'][name]['x'])
        y_derivatives = np.array(analysis_data['spatial_derivatives'][name]['y'])
        gradient_magnitude = np.sqrt(x_derivatives**2 + y_derivatives**2)
        
        print(f"∇h_{name[2]} ({label:9}): ||∇h|| min={np.min(gradient_magnitude):6.3f}, "
              f"max={np.max(gradient_magnitude):6.3f}, mean={np.mean(gradient_magnitude):6.3f}")
        print(f"            ∂/∂x: min={np.min(x_derivatives):6.3f}, max={np.max(x_derivatives):6.3f}")
        print(f"            ∂/∂y: min={np.min(y_derivatives):6.3f}, max={np.max(y_derivatives):6.3f}")
    
    print("\nCBF Conditions Statistics:")
    print("-" * 40)
    for name, label in [('h_d', 'Distance'), ('h_y', 'Yielding'), ('h_s', 'Speed'), ('h_a', 'Acceleration')]:
        cbf_values = np.array(analysis_data['cbf_conditions'][name])
        valid_cbf = cbf_values[~np.isnan(cbf_values)]
        
        if len(valid_cbf) > 0:
            violations = np.sum(valid_cbf < 0)
            print(f"CBF_{name[2]} ({label:9}): min={np.min(valid_cbf):6.3f}, "
                  f"violations={violations}/{len(valid_cbf)}")
            
            if violations > 0:
                print(f"             ❌ CBF condition violated {violations} times!")
            else:
                print(f"             ✅ CBF condition always satisfied")
        else:
            print(f"CBF_{name[2]} ({label:9}): N/A (not active)")
    
    # Overall safety assessment
    print(f"\nOverall Safety Assessment:")
    print("-" * 40)
    min_distance = np.min(analysis_data['distances'])
    print(f"Minimum distance to human: {min_distance:.3f} m")
    
    if min_distance < 1.2:
        print("⚠️  WARNING: Robot entered critical side region (< 1.2m)")
    elif min_distance < 3.0:
        print("⚠️  CAUTION: Robot entered front safety region (< 3.0m)")
    else:
        print("✅ Robot maintained safe distance throughout trajectory")

def main():
    """
    Main function to run the derivative analysis
    """
    print("Starting constraint derivative analysis...")
    
    # Run the analysis
    sim, analysis_data = analyze_derivatives_along_trajectory()
    
    # Print summary
    print_derivative_summary(analysis_data)
    
    # Create plots
    fig = plot_derivative_analysis(sim, analysis_data)
    
    # Create spatial derivatives plots
    fig_spatial = plot_spatial_derivatives(sim, analysis_data)
    
    # Create comprehensive derivatives plots
    fig_comprehensive = plot_comprehensive_derivatives(sim, analysis_data)
    
    print("\n" + "="*80)
    print("ANALYSIS COMPLETE")
    print("="*80)
    print("The derivative functions have been successfully implemented and analyzed.")
    print("Generated plots:")
    print("  📊 derivative_analysis.png - Basic constraint and time derivatives")
    print("  📊 spatial_derivatives_analysis.png - Spatial derivatives (∇h)")
    print("  📊 comprehensive_derivatives_analysis.png - Combined view")
    print("\nKey features implemented:")
    print("  ✅ Time derivatives for all 4 constraint functions (h₁, h₂, h₃, h₄)")
    print("  ✅ Spatial derivatives (gradients) for all constraint functions")
    print("  ✅ CBF condition evaluation (ḣᵢ + α·hᵢ ≥ 0)")
    print("  ✅ Gradient magnitude and direction analysis")
    print("  ✅ Comprehensive trajectory analysis")
    print("  ✅ Multi-plot visualization system")
    print("\nFormulations based on README.md equations (7), (8), (9), (10)")

if __name__ == "__main__":
    main()
