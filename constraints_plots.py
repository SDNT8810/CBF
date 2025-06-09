#!/usr/bin/env python3
"""
Constraint Functions Implementation and Visualization
Based on the Bi-Level Performance-Safety Consideration paper.

This script implements the constraint functions defined in equations (7), (8), (9), (10)
and visualizes them for multiple humans.
Uses RobotSimulation class to avoid code duplication.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
import os

show_figure0 = False

# Clear the console
os.system('cls' if os.name == 'nt' else 'clear')
# Create plots directory if it doesn't exist
os.makedirs('plots', exist_ok=True)

# Import parameters from simulation module to avoid duplication
from simulation import RobotSimulation, rho_0, rho_1, theta_0, V_M, A_M, map_size, alpha_C
# Create simulation instance and use its parameters
sim = RobotSimulation()

# Multiple human positions
human_positions = np.array([[-3.0, 3.0], [3.0, 1.0], [2.0, -1.0]])  # 3 Humans at different positions
# human_positions = np.array([[-0.0, 0.0]])  # 3 Humans at different positions
# human_positions = sim.human_pos
# sim.human_pos = human_positions  

# Create a grid for plotting
x = np.linspace(-map_size, map_size, 100)
y = np.linspace(-map_size, map_size, 100)
X, Y = np.meshgrid(x, y)

# Robot velocity and acceleration (example values)
v_robot = np.array([0.95, 0.0])  # Robot moving along x-axis
v_magnitude = np.linalg.norm(v_robot)
a_robot = np.array([0.95, 0.0])  # Small acceleration along x-axis
a_magnitude = np.linalg.norm(a_robot)

class ConstraintPlotter:
    """Class to handle constraint calculations and plotting, reusing RobotSimulation methods"""
    
    def __init__(self, robot_simulation):
        self.sim = robot_simulation
        # Override human positions for plotting
        self.original_human_pos = self.sim.human_pos.copy()
        self.sim.human_pos = human_positions
    
    def calculate_constraints_for_grid(self, X, Y, v_robot):
        """Calculate constraints for grid points using RobotSimulation methods"""
        v_magnitude = np.linalg.norm(v_robot)
        
        # Initialize arrays to store minimum constraints
        h_d_grid = np.full_like(X, np.inf)
        h_y_grid = np.full_like(X, np.inf)
        h_s_grid = np.full_like(X, np.inf)
        h_a_grid = np.full_like(X, np.inf)
        h_min_grid = np.full_like(X, np.inf)
        
        # Calculate constraints for each grid point
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                robot_pos = np.array([X[i, j], Y[i, j]])
                robot_vel = v_robot
                
                # Use RobotSimulation's calculate_constraints method
                h_d, h_y, h_s, h_a, h_min, _ = self.sim.calculate_constraints(robot_pos, robot_vel)
                
                h_d_grid[i, j] = h_d
                h_y_grid[i, j] = h_y if h_y is not None else np.inf
                h_s_grid[i, j] = h_s
                h_a_grid[i, j] = h_a
                h_min_grid[i, j] = h_min
        
        return h_d_grid, h_y_grid, h_s_grid, h_a_grid, h_min_grid
    
    def calculate_cbf_conditions_for_grid(self, h_d, h_y, h_s, h_a, alpha_C=1.0):
        """Calculate CBF conditions for grid using RobotSimulation's evaluate_cbf_conditions method"""
        # For grid calculations, we simplify the derivative terms to zero
        # This is consistent with the original implementation
        C_d = 0 + alpha_C * (h_d ** 2)
        C_y = 0 + alpha_C * (h_y ** 2)
        C_s = 0 + alpha_C * (h_s ** 2)
        C_a = 0 + alpha_C * (h_a ** 2)
        
        # Calculate minimum constraint condition
        C_min = np.minimum(np.minimum(C_d, C_y), np.minimum(C_s, C_a))
        
        return C_d, C_y, C_s, C_a, C_min
    
    def restore_original_humans(self):
        """Restore original human positions"""
        self.sim.human_pos = self.original_human_pos

# Create constraint plotter instance
plotter = ConstraintPlotter(sim)

# Calculate all constraint functions for multiple humans using simulation methods
h_d, h_y, h_s, h_a, h_min = plotter.calculate_constraints_for_grid(X, Y, v_robot)

# Calculate CBF conditions
C_d, C_y, C_s, C_a, C_min = plotter.calculate_cbf_conditions_for_grid(h_d, h_y, h_s, h_a, alpha_C)

# Helper function for plotting
def plot_constraint_function(ax_2d, ax_3d, Z, title, human_positions, is_3d=True):
    # 2D contour plot
    contour = ax_2d.contourf(X, Y, Z, cmap='viridis', levels=20)
    ax_2d.set_title(title + " (2D)")
    ax_2d.set_xlabel("x (m)")
    ax_2d.set_ylabel("y (m)")
    ax_2d.set_aspect('equal')
    plt.colorbar(contour, ax=ax_2d)
    
    # Draw safety boundaries for each human
    for human_pos in human_positions:
        circle_front = Circle((human_pos[0], human_pos[1]), rho_0, color='r', fill=False, linestyle='--', alpha=0.7)
        circle_side = Circle((human_pos[0], human_pos[1]), rho_1, color='orange', fill=False, linestyle=':', alpha=0.7)
        ax_2d.add_patch(circle_front)
        ax_2d.add_patch(circle_side)
        
        # Mark human position
        ax_2d.plot(human_pos[0], human_pos[1], 'ro', markersize=8)
    
    if is_3d:
        # 3D surface plot
        surf = ax_3d.plot_surface(X, Y, Z, cmap='viridis', alpha=0.8)
        ax_3d.set_title(title + " (3D)")
        ax_3d.set_xlabel("x (m)")
        ax_3d.set_ylabel("y (m)")
        ax_3d.set_zlabel("Value")
        
        # Add transparent z=0 plane to show intersection
        xx, yy = np.meshgrid(np.linspace(-map_size, map_size, 10), 
                            np.linspace(-map_size, map_size, 10))
        zz = np.zeros_like(xx)  # z=0 plane
        ax_3d.plot_surface(xx, yy, zz, alpha=0.3, color='red', 
                          label='z=0 plane')

# Function to create and save figures
def create_and_save_figures():
    # Figure 0: Full Region Constraint Functions for multiple humans
    fig0 = plt.figure(figsize=(20, 20))
    fig0.suptitle("Figure 0: All Constraint Functions for Multiple Humans", fontsize=16)
    
    # Distance Constraint
    ax1_2d = fig0.add_subplot(4, 2, 1)
    ax1_3d = fig0.add_subplot(4, 2, 2, projection='3d')
    plot_constraint_function(ax1_2d, ax1_3d, h_d, "Distance Constraint (h_d)", human_positions)
    
    # Yielding Constraint
    ax2_2d = fig0.add_subplot(4, 2, 3)
    ax2_3d = fig0.add_subplot(4, 2, 4, projection='3d')
    plot_constraint_function(ax2_2d, ax2_3d, h_y, "Yielding Constraint (h_y)", human_positions)
    
    # Speed Constraint
    ax3_2d = fig0.add_subplot(4, 2, 5)
    ax3_3d = fig0.add_subplot(4, 2, 6, projection='3d')
    plot_constraint_function(ax3_2d, ax3_3d, h_s, "Speed Constraint (h_s)", human_positions)
    
    # Acceleration Constraint
    ax4_2d = fig0.add_subplot(4, 2, 7)
    ax4_3d = fig0.add_subplot(4, 2, 8, projection='3d')
    plot_constraint_function(ax4_2d, ax4_3d, h_a, "Acceleration Constraint (h_a)", human_positions)
    
    plt.tight_layout()
    if show_figure0: plt.show()
    plt.savefig('plots/figure0_full_region_multiple_humans.png', dpi=300)
    plt.close()
    
    # Figure 1: Minimum Constraint Functions
    fig1 = plt.figure(figsize=(15, 7))
    fig1.suptitle("Figure 1: Minimum of All Constraint Functions (Multiple Humans)", fontsize=16)
    
    ax1_2d = fig1.add_subplot(1, 2, 1)
    ax1_3d = fig1.add_subplot(1, 2, 2, projection='3d')
    plot_constraint_function(ax1_2d, ax1_3d, h_min, "min{h_d, h_y, h_a, h_s}", human_positions)
    
    plt.tight_layout()
    plt.savefig('plots/figure1_minimum_constraint_multiple_humans.png', dpi=300)
    
    # Figure 2: Constraint condition
    fig2 = plt.figure(figsize=(15, 7))
    fig2.suptitle("Figure 2: Constraint Condition C_ji = ḣ_ji + α·h²_ji ≥ 0 (Multiple Humans)", fontsize=16)
    
    ax2_2d = fig2.add_subplot(1, 2, 1)
    ax2_3d = fig2.add_subplot(1, 2, 2, projection='3d')
    plot_constraint_function(ax2_2d, ax2_3d, C_min, "min{C_d, C_y, C_a, C_s}", human_positions)
    
    plt.tight_layout()
    plt.savefig('plots/figure2_constraint_condition_multiple_humans.png', dpi=300)
    
    # Figure 3: Individual constraint comparison for closest human
    fig3 = plt.figure(figsize=(20, 15))
    fig3.suptitle("Figure 3: Individual Constraint Functions Comparison", fontsize=16)
    
    # Create 2x2 subplot for individual constraints
    ax1 = fig3.add_subplot(2, 2, 1)
    contour1 = ax1.contourf(X, Y, h_d, cmap='viridis', levels=20)
    ax1.set_title("Distance Constraint (h_d)")
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.set_aspect('equal')
    plt.colorbar(contour1, ax=ax1)
    
    ax2 = fig3.add_subplot(2, 2, 2)
    contour2 = ax2.contourf(X, Y, h_y, cmap='viridis', levels=20)
    ax2.set_title("Yielding Constraint (h_y)")
    ax2.set_xlabel("x (m)")
    ax2.set_ylabel("y (m)")
    ax2.set_aspect('equal')
    plt.colorbar(contour2, ax=ax2)
    
    ax3 = fig3.add_subplot(2, 2, 3)
    contour3 = ax3.contourf(X, Y, h_s, cmap='viridis', levels=20)
    ax3.set_title("Speed Constraint (h_s)")
    ax3.set_xlabel("x (m)")
    ax3.set_ylabel("y (m)")
    ax3.set_aspect('equal')
    plt.colorbar(contour3, ax=ax3)
    
    ax4 = fig3.add_subplot(2, 2, 4)
    contour4 = ax4.contourf(X, Y, h_a, cmap='viridis', levels=20)
    ax4.set_title("Acceleration Constraint (h_a)")
    ax4.set_xlabel("x (m)")
    ax4.set_ylabel("y (m)")
    ax4.set_aspect('equal')
    plt.colorbar(contour4, ax=ax4)
    
    # Add human positions and safety circles to all subplots
    for ax in [ax1, ax2, ax3, ax4]:
        for human_pos in human_positions:
            circle_front = Circle((human_pos[0], human_pos[1]), rho_0, color='r', fill=False, linestyle='--', alpha=0.7)
            circle_side = Circle((human_pos[0], human_pos[1]), rho_1, color='orange', fill=False, linestyle=':', alpha=0.7)
            ax.add_patch(circle_front)
            ax.add_patch(circle_side)
            ax.plot(human_pos[0], human_pos[1], 'ro', markersize=8)
    
    plt.tight_layout()
    plt.savefig('plots/figure3_individual_constraints_multiple_humans.png', dpi=300)
    
    print("All figures have been saved to the 'plots/' directory.")
    print(f"Human positions: {human_positions}")
    print(f"Safety thresholds: ρ₀={rho_0}m (front), ρ₁={rho_1}m (side)")
    print(f"Angular threshold: θ₀={theta_0*180/np.pi:.1f}°")

if __name__ == "__main__":
    create_and_save_figures()
    # Restore original human positions in simulation
    plotter.restore_original_humans()

