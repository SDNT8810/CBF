#!/usr/bin/env python3
"""
Robot Constraints Simulation with Animation
- Robot (moving point) starts from random position and moves to random goal
- Multiple humans (obstacles) are positioned in the environment
- Shows animation of robot movement and constraint plots
"""

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.animation import FuncAnimation
import os

# Clear the console
os.system('cls' if os.name == 'nt' else 'clear')
# Create plots directory if it doesn't exist
os.makedirs('plots', exist_ok=True)

# Parameters
rho_0 = 3.0  # Safety threshold for front region (m)
rho_1 = 1.2  # Safety threshold for side region (m)
theta_0 = np.pi/4  # Critical angular range (45 degrees)
V_M = 1.0  # Robot's absolute maximum speed (m/s)
A_M = 1.0  # Robot's absolute maximum Acceleration (m/s^2)
robot_speed = 0.95  # Robot constant speed (m/s)
dt = 0.02  # Time step for animation (larger for fewer frames)
animation_duration = 2.0  # Total animation duration in seconds
save_sim = True  # Save simulation results to file
map_size = 8.0  # Size of the map (m), used for axis limits

alpha_C = 1.0  # Tuning parameter for constraint strength

class RobotSimulation:
    def __init__(self):
        # self.human_pos = np.array([[0.5, -1.0],[-1.0, 1.1]])  # 2 Humans
        # self.human_pos = np.array([[-3.0, 3.0], [3.0, 3.0], [0.0, -2.0]])  # 3 Humans at different positions
        self.human_pos = np.array([[2.0, 0.0]])  # 3 Humans at different positions

        self.simulation_complete = False  # Flag to track simulation completion
        
    def generate_random_trajectory(self):
        """Generate random start and goal points"""
        # Random start point anywhere in the space (excluding near human)
        start_x = -6
        start_y = np.random.uniform(-6, 6)
        # start_y = 0
        self.start_point = np.array([start_x, start_y])

        # Random goal point anywhere in the space (excluding near human)
        goal_x = 6
        # goal_y = random.uniform(-8, 8)
        goal_y = 0
        self.goal_point = np.array([goal_x, goal_y])

        # Calculate trajectory parameters
        self.direction = self.goal_point - self.start_point
        self.total_distance = np.linalg.norm(self.direction)
        self.unit_direction = self.direction / self.total_distance
        self.total_time = self.total_distance / robot_speed
        
        # Initialize data storage
        self.time_data = []
        self.h_d_data = []
        self.h_y_data = []
        self.h_s_data = []
        self.h_a_data = []
        self.h_min_data = []
        self.distance_data = []
        
        print(f"Start: ({self.start_point[0]:.2f}, {self.start_point[1]:.2f})")
        print(f"Goal: ({self.goal_point[0]:.2f}, {self.goal_point[1]:.2f})")
        print(f"Distance: {self.total_distance:.2f} m")
        print(f"Travel time: {self.total_time:.2f} s")
    
    def get_robot_state_at_time(self, t):
        """Get robot position and velocity at time t"""
        if t >= self.total_time:
            # Robot has reached goal
            robot_pos = self.goal_point.copy()
            robot_vel = np.array([0.0, 0.0])
        else:
            # Robot is moving
            traveled_distance = robot_speed * t
            robot_pos = self.start_point + self.unit_direction * traveled_distance
            robot_vel = self.unit_direction * robot_speed
        
        return robot_pos, robot_vel
    
    def calculate_constraints(self, robot_pos, robot_vel):
        """Calculate all 4 constraint functions for multiple humans"""
        v_magnitude = np.linalg.norm(robot_vel)
        
        # Initialize lists to store constraints for each human
        h_d_list = []
        h_y_list = []
        h_s_list = []
        h_a_list = []
        min_distance = float('inf')
        
        # Calculate constraints for each human
        for i, human_pos in enumerate(self.human_pos):
            # Distance to human (ρ_hi)
            robot_rho = np.linalg.norm(robot_pos - human_pos)
            min_distance = min(min_distance, robot_rho)
            
            # Robot heading and relative angle (θ_hi)
            if v_magnitude > 0.001:
                robot_heading = np.arctan2(robot_vel[1], robot_vel[0])
                
                # Angle from robot to human relative to robot's heading
                human_direction = np.arctan2(human_pos[1] - robot_pos[1], 
                                           human_pos[0] - robot_pos[0])
                relative_angle = human_direction - robot_heading
                relative_angle = np.arctan2(np.sin(relative_angle), np.cos(relative_angle))
                
            else:
                robot_heading = 0
                relative_angle = 0

            # Check if human is in front or side region
            is_front_region = np.abs(relative_angle) < theta_0    
            # is_front_region = np.abs(np.sin(relative_angle)) < np.abs(np.sin(theta_0))
            # is_front_region = np.sin(relative_angle** 2) < np.sin(theta_0** 2)
            
            # === Equation (7) - Distance Constraint ===
            # h_d,i(ρ_hi, θ_hi) = ρ_hi - ρ_0 if |θ_hi| < θ_0, else ρ_hi - ρ_1
            if is_front_region:
                h_d = robot_rho - rho_0
            else:
                h_d = robot_rho - rho_1
            h_d_list.append(h_d)
            
            # === Equation (8) - Yielding Constraint ===
            # h_y,i(dρ_hi/dt, ρ_hi, θ_hi) = dρ_hi/dt only in critical regions
            if robot_rho > 0.01:
                # Calculate rate of change of distance (dρ/dt)
                human_to_robot = robot_pos - human_pos
                drho_dt = np.dot(robot_vel, human_to_robot / robot_rho)
            else:
                drho_dt = 0
            
            # Apply yielding constraint only in critical regions as per equation (8)
            front_critical = (robot_rho <= rho_0) and is_front_region
            side_critical = (robot_rho <= rho_1) and not is_front_region
            
            if front_critical or side_critical:
                h_y = drho_dt  # Exact implementation: dρ/dt (can be negative!)
                h_y_list.append(h_y)
            
            # === Equation (9) - Speed Constraint ===
            # h_s,i(|v|, ν_M(ρ_hi)) = ν_M(ρ_hi) - |v|
            # where ν_M(ρ_hi) = V_M · tanh(ρ_hi)
            nu_M = V_M * np.tanh(robot_rho)
            h_s = nu_M - v_magnitude
            h_s_list.append(h_s)
            
            # === Equation (10) - Acceleration Constraint ===
            # h_a,i(d|v|/dt, a_max(ρ_hi)) = a_max(ρ_hi) - d|v|/dt
            a_max_allowed = A_M * np.tanh(robot_rho)  # distance-dependent acceleration limit
            a_current = 0.95  # Constant velocity motion (no acceleration)
            h_a = a_max_allowed - a_current
            h_a_list.append(h_a)
        
        # Take minimum constraint value across all humans for each type
        h_d = float(min(h_d_list))
        h_y = float(min(h_y_list)) if h_y_list else None
        h_s = float(min(h_s_list))
        h_a = float(min(h_a_list))
        
        # Collect all defined constraints for minimum calculation
        constraints = [h_d, h_s, h_a]
        if h_y is not None:
            constraints.append(h_y)
        
        h_min = float(min(constraints))
        
        # For plotting purposes, use a large positive value when h_y is undefined
        h_y_plot = h_y if h_y is not None else 5.0
        
        return h_d, h_y_plot, h_s, h_a, h_min, float(min_distance)
    
    def evaluate_cbf_conditions(self, h_d, h_y, h_s, h_a, alpha_C=1.0):
        """
        Evaluate CBF condition (Equation 11): C_ji = ḣ_ji + α·h²_ji ≥ 0
        This is for analysis purposes - showing when constraints are satisfied 
        """
        # For simplicity, assume ḣ_ji ≈ 0 (constraint derivative)
        # In practice, this would require computing time derivatives
        # of the constraints, but here we just use h_ji values directly.

        C_d = 0 + alpha_C * (h_d ** 2) if h_d is not None else None
        C_y = 0 + alpha_C * (h_y ** 2) if h_y is not None else None
        C_s = 0 + alpha_C * (h_s ** 2) if h_s is not None else None
        C_a = 0 + alpha_C * (h_a ** 2) if h_a is not None else None
    
        return C_d, C_y, C_s, C_a
    
    def setup_animation(self):
        """Setup the matplotlib figure and axes for animation"""
        self.fig, self.axes = plt.subplots(2, 3, figsize=(20, 14))
        self.fig.suptitle('Robot Constraint Simulation', fontsize=16)
        
        # Setup trajectory plot (top-left)
        self.ax_traj = self.axes[0, 0]
        self.ax_traj.set_xlim(-map_size, map_size)
        self.ax_traj.set_ylim(-map_size, map_size)
        self.ax_traj.set_title('Robot Trajectory')
        self.ax_traj.set_xlabel('X (m)')
        self.ax_traj.set_ylabel('Y (m)')
        self.ax_traj.grid(True, alpha=0.3)
        self.ax_traj.set_aspect('equal')
        
        # Plot static elements
        for i, human_pos in enumerate(self.human_pos):
            self.ax_traj.plot(human_pos[0], human_pos[1], 'ro', markersize=15, label=f'Human {i+1}' if i == 0 else '')
            
            # Add safety circles for each human
            circle_front = Circle((human_pos[0], human_pos[1]), rho_0, fill=False, color='red', linestyle='--', alpha=0.5)
            circle_side = Circle((human_pos[0], human_pos[1]), rho_1, fill=False, color='orange', linestyle='--', alpha=0.5)
            self.ax_traj.add_patch(circle_front)
            self.ax_traj.add_patch(circle_side)
        
        self.ax_traj.plot(self.start_point[0], self.start_point[1], 'go', markersize=10, label='Start')
        self.ax_traj.plot(self.goal_point[0], self.goal_point[1], 'b^', markersize=10, label='Goal')
        
        # Add safety circles (legend only - actual circles added above)
        if len(self.human_pos) > 0:
            self.ax_traj.plot([], [], 'r--', alpha=0.5, label=f'Front Safety (ρ₀={rho_0}m)')
            self.ax_traj.plot([], [], color='orange', linestyle='--', alpha=0.5, label=f'Side Safety (ρ₁={rho_1}m)')
        
        # Initialize robot point and trail
        self.robot_point, = self.ax_traj.plot([], [], 'ko', markersize=8, label='Robot')
        self.robot_trail, = self.ax_traj.plot([], [], 'k-', alpha=0.5, linewidth=2)
        
        # Initialize heading bound lines
        self.heading_line_upper, = self.ax_traj.plot([], [], 'g--', alpha=0.7, linewidth=2, label=f'Heading ±{theta_0*180/np.pi:.0f}°')
        self.heading_line_lower, = self.ax_traj.plot([], [], 'g--', alpha=0.7, linewidth=2)
        
        # Initialize human-to-robot line (to show relative angle to closest human)
        self.human_robot_line, = self.ax_traj.plot([], [], 'r:', alpha=0.8, linewidth=2, label='Closest Human Direction')
        
        self.ax_traj.legend()
        
        # Setup constraint plots
        self.constraint_axes = [self.axes[0, 1], self.axes[0, 2], self.axes[1, 0], self.axes[1, 1], self.axes[1, 2]]
        self.constraint_titles = ['Distance Constraint (h_d)', 'Yielding Constraint (h_y)', 'Speed Constraint (h_s)', 
                              'Acceleration Constraint (h_a)', 'Minimum Constraint (h_min)']
        self.constraint_colors = ['blue', 'green', 'orange', 'purple', 'black']
        self.constraint_lines = []
        
        for i, (ax, title, color) in enumerate(zip(self.constraint_axes, self.constraint_titles, self.constraint_colors)):
            ax.set_title(title)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Constraint Value')
            ax.grid(True, alpha=0.3)
            ax.axhline(y=0, color='red', linestyle='--', alpha=0.7)
            line, = ax.plot([], [], color=color, linewidth=2)
            self.constraint_lines.append(line)
            ax.set_xlim(0, self.total_time + 1)
            ax.set_ylim(-rho_0, 2.5)

        # Initialize trail data
        self.trail_x = []
        self.trail_y = []
    
    def update_heading_bounds(self, robot_pos, robot_vel):
        """Update the heading bound visualization lines"""
        v_magnitude = np.linalg.norm(robot_vel)
        if v_magnitude > 0.01:
            # Robot heading
            robot_heading = np.arctan2(robot_vel[1], robot_vel[0])
            
            # Calculate bound directions
            upper_heading = robot_heading + theta_0
            lower_heading = robot_heading - theta_0
            
            # Line length for visualization
            line_length = 4.0
            
            # Upper bound line
            upper_end_x = robot_pos[0] + line_length * np.cos(upper_heading)
            upper_end_y = robot_pos[1] + line_length * np.sin(upper_heading)
            self.heading_line_upper.set_data([robot_pos[0], upper_end_x], [robot_pos[1], upper_end_y])
            
            # Lower bound line
            lower_end_x = robot_pos[0] + line_length * np.cos(lower_heading)
            lower_end_y = robot_pos[1] + line_length * np.sin(lower_heading)
            self.heading_line_lower.set_data([robot_pos[0], lower_end_x], [robot_pos[1], lower_end_y])
            
            # Find closest human for visualization
            min_distance = float('inf')
            closest_human_pos = None
            closest_is_front = False
            
            for human_pos in self.human_pos:
                distance = np.linalg.norm(robot_pos - human_pos)
                if distance < min_distance:
                    min_distance = distance
                    closest_human_pos = human_pos
                    
                    # Check if closest human is in front region
                    human_direction = np.arctan2(human_pos[1] - robot_pos[1], 
                                               human_pos[0] - robot_pos[0])
                    relative_angle = human_direction - robot_heading
                    relative_angle = np.arctan2(np.sin(relative_angle), np.cos(relative_angle))
                    closest_is_front = np.abs(relative_angle) < theta_0
            
            # Show line to closest human with color indicating if it's in front region
            if closest_human_pos is not None:
                line_color = 'red' if closest_is_front else 'orange'
                self.human_robot_line.set_color(line_color)
                self.human_robot_line.set_data([robot_pos[0], closest_human_pos[0]], [robot_pos[1], closest_human_pos[1]])
        else:
            # No heading bounds when robot is stationary
            self.heading_line_upper.set_data([], [])
            self.heading_line_lower.set_data([], [])
            self.human_robot_line.set_data([], [])
    
    def animate(self, frame):
        """Animation function called for each frame"""
        # Map animation frame to actual simulation time
        animation_duration = 2.0  # seconds (match run_animation)
        num_frames = int(animation_duration / dt)
        current_time = (frame / num_frames) * self.total_time
        
        # Get current robot state
        robot_pos, robot_vel = self.get_robot_state_at_time(current_time)
        
        # Calculate constraints
        h_d, h_y, h_s, h_a, h_min, robot_rho = self.calculate_constraints(robot_pos, robot_vel)
        
        # Store data
        self.time_data.append(current_time)
        self.h_d_data.append(h_d)
        self.h_y_data.append(h_y)
        self.h_s_data.append(h_s)
        self.h_a_data.append(h_a)
        self.h_min_data.append(h_min)
        self.distance_data.append(robot_rho)
        
        # Update robot position and trail
        self.robot_point.set_data([robot_pos[0]], [robot_pos[1]])
        self.trail_x.append(robot_pos[0])
        self.trail_y.append(robot_pos[1])
        self.robot_trail.set_data(self.trail_x, self.trail_y)
        
        # Update heading bound lines
        self.update_heading_bounds(robot_pos, robot_vel)
        
        # Update constraint plots
        constraint_data = [self.h_d_data, self.h_y_data, self.h_s_data, self.h_a_data, self.h_min_data]
        for line, data in zip(self.constraint_lines, constraint_data):
            line.set_data(self.time_data, data)
        
        # Check if simulation is complete
        if (frame >= num_frames - 1 or current_time >= self.total_time) and not self.simulation_complete:
            self.simulation_complete = True
            print(f"\nSimulation completed!")
            print(f"Minimum constraint value: {min(self.h_min_data):.3f}")
            print(f"Minimum distance to human: {min(self.distance_data):.3f} m")
            if min(self.h_min_data) < 0:
                print("⚠️  WARNING: Constraint violated!")
            else:
                print("✅ All constraint satisfied")
            
            # Save the figure after simulation completes
            timestamp = __import__('datetime').datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"plots/simulation_result_{timestamp}.png"
            if save_sim:
                self.fig.savefig(filename, dpi=300, bbox_inches='tight')
                print(f"📁 Simulation results saved to: {filename}")
            
            # Close the figure immediately after saving
            plt.close(self.fig)
            print("Animation window closed automatically.")
                        
        return [self.robot_point, self.robot_trail, self.heading_line_upper, self.heading_line_lower, self.human_robot_line] + self.constraint_lines
    
    def run_animation(self, animation_duration=2.0):
        """Start the animation"""
        print("Starting simulation...")

        # Calculate number of frames for 5-second animation
        num_frames = int(animation_duration / dt)

        # Very fast frame interval (10ms per frame)
        frame_interval = 10  # milliseconds per frame
        
        # Create animation
        self.animation = FuncAnimation(
            self.fig, self.animate, frames=num_frames,
            interval=frame_interval, blit=False, repeat=False
        )
        
        # Make figure full screen
        try:
            manager = plt.get_current_fig_manager()
            if manager is not None and hasattr(manager, 'window'):
                window = getattr(manager, 'window', None)
                if window is not None and hasattr(window, 'showMaximized'):
                    window.showMaximized()
        except Exception:
            pass  # Silently fail if full screen doesn't work
        
        # Start the animation
        plt.tight_layout()
        plt.show()
        
        return self.animation

    def calculate_constraint_derivatives(self, robot_pos, robot_vel, robot_acc, human_vel=None, human_acc=None):
        """
        Calculate time derivatives of all 4 constraint functions for multiple humans
        Based on formulations from README.md
        
        Args:
            robot_pos: Robot position [x_r, y_r]
            robot_vel: Robot velocity [vx_r, vy_r]
            robot_acc: Robot acceleration [ax_r, ay_r]
            human_vel: Human velocities (optional, defaults to zero)
            human_acc: Human accelerations (optional, defaults to zero)
        
        Returns:
            h_d_dot, h_y_dot, h_s_dot, h_a_dot: Time derivatives of constraint functions
        """
        v_magnitude = np.linalg.norm(robot_vel)
        a_magnitude = np.linalg.norm(robot_acc)
        
        # Initialize lists to store derivatives for each human
        h_d_dot_list = []
        h_y_dot_list = []
        h_s_dot_list = []
        h_a_dot_list = []
        
        # Default human velocities and accelerations to zero if not provided
        if human_vel is None:
            human_vel = [np.array([0.0, 0.0]) for _ in self.human_pos]
        if human_acc is None:
            human_acc = [np.array([0.0, 0.0]) for _ in self.human_pos]
        
        # Calculate derivatives for each human
        for i, human_pos in enumerate(self.human_pos):
            # Distance to human (ρ_hi)
            robot_rho = np.linalg.norm(robot_pos - human_pos)
            
            # Avoid division by zero
            if robot_rho < 1e-6:
                robot_rho = 1e-6
            
            # Relative position and velocity
            rel_pos = robot_pos - human_pos  # p_r - p_hi
            rel_vel = robot_vel - human_vel[i]  # v_r - v_hi
            rel_acc = robot_acc - human_acc[i]  # a_r - a_hi
            
            # === h1: Distance Constraint Derivative (Equation 7) ===
            # ḣ_d,i = dρ_hi/dt = (p_r - p_hi)^T (v_r - v_hi) / ρ_hi
            h_d_dot = np.dot(rel_pos, rel_vel) / robot_rho
            h_d_dot_list.append(h_d_dot)
            
            # === h2: Yielding Constraint Derivative (Equation 8) ===
            # ḣ_y,i = d²ρ_hi/dt² = (1/ρ_hi) * [||v_r - v_hi||² + (p_r - p_hi)^T (a_r - a_hi) 
            #                                   - [(p_r - p_hi)^T (v_r - v_hi)]²/ρ_hi²]
            rel_vel_squared = np.dot(rel_vel, rel_vel)
            pos_acc_term = np.dot(rel_pos, rel_acc)
            pos_vel_dot = np.dot(rel_pos, rel_vel)
            
            h_y_dot = (1.0 / robot_rho) * (
                rel_vel_squared + pos_acc_term - (pos_vel_dot ** 2) / (robot_rho ** 2)
            )
            
            # Apply yielding constraint derivative only in critical regions
            robot_heading = np.arctan2(robot_vel[1], robot_vel[0]) if v_magnitude > 0.001 else 0
            human_direction = np.arctan2(human_pos[1] - robot_pos[1], 
                                       human_pos[0] - robot_pos[0])
            relative_angle = human_direction - robot_heading
            relative_angle = np.arctan2(np.sin(relative_angle), np.cos(relative_angle))
            is_front_region = np.abs(relative_angle) < theta_0
            
            front_critical = (robot_rho <= rho_0) and is_front_region
            side_critical = (robot_rho <= rho_1) and not is_front_region
            
            if front_critical or side_critical:
                h_y_dot_list.append(h_y_dot)
            
            # === h3: Speed Constraint Derivative (Equation 9) ===
            # ḣ_s,i = V_M * sech²(ρ_hi) * (p_r - p_hi)^T (v_r - v_hi) / ρ_hi - v_r^T a_r / ||v_r||
            sech_squared = 1.0 / (np.cosh(robot_rho) ** 2)  # sech²(ρ_hi)
            first_term = V_M * sech_squared * np.dot(rel_pos, rel_vel) / robot_rho
            
            if v_magnitude > 1e-6:
                second_term = np.dot(robot_vel, robot_acc) / v_magnitude
            else:
                second_term = 0.0
            
            h_s_dot = first_term - second_term
            h_s_dot_list.append(h_s_dot)
            
            # === h4: Acceleration Constraint Derivative (Equation 10) ===
            # ḣ_a,i = A_M * sech²(ρ_hi) * (p_r - p_hi)^T (v_r - v_hi) / ρ_hi - a_r^T ȧ_r / ||a_r||
            first_term_acc = A_M * sech_squared * np.dot(rel_pos, rel_vel) / robot_rho
            
            # For the second term, we need the jerk (ȧ_r). 
            # In practice, this would be computed from the control input derivative.
            # For now, we'll assume jerk is zero (constant acceleration scenario)
            if a_magnitude > 1e-6:
                # Assuming jerk = 0 for simplicity
                jerk = np.array([0.0, 0.0])
                second_term_acc = np.dot(robot_acc, jerk) / a_magnitude
            else:
                second_term_acc = 0.0
            
            h_a_dot = first_term_acc - second_term_acc
            h_a_dot_list.append(h_a_dot)
        
        # Take minimum constraint derivative value across all humans for each type
        h_d_dot = float(min(h_d_dot_list))
        h_y_dot = float(min(h_y_dot_list)) if h_y_dot_list else None
        h_s_dot = float(min(h_s_dot_list))
        h_a_dot = float(min(h_a_dot_list))
        
        return h_d_dot, h_y_dot, h_s_dot, h_a_dot

    def calculate_spatial_derivatives(self, robot_pos, robot_vel):
        """
        Calculate spatial derivatives (gradients) of all 4 constraint functions
        Based on formulations from README.md
        
        Args:
            robot_pos: Robot position [x_r, y_r]
            robot_vel: Robot velocity [vx_r, vy_r]
        
        Returns:
            Dictionary containing spatial derivatives for each constraint
        """
        v_magnitude = np.linalg.norm(robot_vel)
        
        # Initialize dictionaries to store gradients for each human
        gradients = {
            'h_d': {'x': [], 'y': [], 'theta': []},
            'h_y': {'x': [], 'y': [], 'theta': []},
            'h_s': {'x': [], 'y': [], 'theta': []},
            'h_a': {'x': [], 'y': [], 'theta': []}
        }
        
        # Calculate gradients for each human
        for i, human_pos in enumerate(self.human_pos):
            # Distance to human (ρ_hi)
            robot_rho = np.linalg.norm(robot_pos - human_pos)
            
            # Avoid division by zero
            if robot_rho < 1e-6:
                robot_rho = 1e-6
            
            # Relative position
            rel_pos = robot_pos - human_pos  # p_r - p_hi
            
            # === Distance Constraint Gradients ===
            # ∂h_d,i/∂x_r = (x_r - x_hi) / ρ_hi
            # ∂h_d,i/∂y_r = (y_r - y_hi) / ρ_hi
            # ∂h_d,i/∂θ_r = 0
            gradients['h_d']['x'].append(rel_pos[0] / robot_rho)
            gradients['h_d']['y'].append(rel_pos[1] / robot_rho)
            gradients['h_d']['theta'].append(0.0)
            
            # === Yielding Constraint Gradients ===
            # Same as distance constraint for spatial derivatives
            gradients['h_y']['x'].append(rel_pos[0] / robot_rho)
            gradients['h_y']['y'].append(rel_pos[1] / robot_rho)
            gradients['h_y']['theta'].append(0.0)
            
            # === Speed Constraint Gradients ===
            # ∂h_s,i/∂x_r = V_M * sech²(ρ_hi) * (x_r - x_hi) / ρ_hi
            # ∂h_s,i/∂y_r = V_M * sech²(ρ_hi) * (y_r - y_hi) / ρ_hi
            # ∂h_s,i/∂θ_r = 0
            sech_squared = 1.0 / (np.cosh(robot_rho) ** 2)
            gradients['h_s']['x'].append(V_M * sech_squared * rel_pos[0] / robot_rho)
            gradients['h_s']['y'].append(V_M * sech_squared * rel_pos[1] / robot_rho)
            gradients['h_s']['theta'].append(0.0)
            
            # === Acceleration Constraint Gradients ===
            # ∂h_a,i/∂x_r = A_M * sech²(ρ_hi) * (x_r - x_hi) / ρ_hi
            # ∂h_a,i/∂y_r = A_M * sech²(ρ_hi) * (y_r - y_hi) / ρ_hi
            # ∂h_a,i/∂θ_r = 0
            gradients['h_a']['x'].append(A_M * sech_squared * rel_pos[0] / robot_rho)
            gradients['h_a']['y'].append(A_M * sech_squared * rel_pos[1] / robot_rho)
            gradients['h_a']['theta'].append(0.0)
        
        # Take minimum gradient values across all humans for each constraint type
        min_gradients = {}
        for constraint in ['h_d', 'h_y', 'h_s', 'h_a']:
            min_gradients[constraint] = {
                'x': float(min(gradients[constraint]['x'])),
                'y': float(min(gradients[constraint]['y'])),
                'theta': float(min(gradients[constraint]['theta']))
            }
        
        return min_gradients
    
    def demonstrate_derivatives(self, t=1.0):
        """
        Demonstrate the usage of constraint derivative functions
        
        Args:
            t: Time at which to evaluate derivatives
        """
        print("\n" + "="*60)
        print("CONSTRAINT DERIVATIVES DEMONSTRATION")
        print("="*60)
        
        # Get robot state at specified time
        robot_pos, robot_vel = self.get_robot_state_at_time(t)
        
        # For demonstration, assume robot has some acceleration
        # In practice, this would come from the control input
        robot_acc = np.array([0.1, 0.05])  # Example acceleration
        
        print(f"Time: {t:.2f} s")
        print(f"Robot position: ({robot_pos[0]:.3f}, {robot_pos[1]:.3f}) m")
        print(f"Robot velocity: ({robot_vel[0]:.3f}, {robot_vel[1]:.3f}) m/s")
        print(f"Robot acceleration: ({robot_acc[0]:.3f}, {robot_acc[1]:.3f}) m/s²")
        print(f"Speed magnitude: {np.linalg.norm(robot_vel):.3f} m/s")
        
        # Calculate current constraint values
        h_d, h_y, h_s, h_a, h_min, min_distance = self.calculate_constraints(robot_pos, robot_vel)
        
        print(f"\nCurrent Constraint Values:")
        print(f"  h_d (Distance):     {h_d:.4f}")
        print(f"  h_y (Yielding):     {h_y:.4f}")
        print(f"  h_s (Speed):        {h_s:.4f}")
        print(f"  h_a (Acceleration): {h_a:.4f}")
        print(f"  h_min (Minimum):    {h_min:.4f}")
        print(f"  Min distance:       {min_distance:.4f} m")
        
        # Calculate time derivatives
        h_d_dot, h_y_dot, h_s_dot, h_a_dot = self.calculate_constraint_derivatives(
            robot_pos, robot_vel, robot_acc
        )
        
        print(f"\nTime Derivatives (ḣ):")
        print(f"  ḣ_d (Distance rate):     {h_d_dot:.4f}")
        if h_y_dot is not None:
            print(f"  ḣ_y (Yielding rate):     {h_y_dot:.4f}")
        else:
            print(f"  ḣ_y (Yielding rate):     N/A (not in critical region)")
        print(f"  ḣ_s (Speed rate):        {h_s_dot:.4f}")
        print(f"  ḣ_a (Acceleration rate): {h_a_dot:.4f}")
        
        # Calculate spatial derivatives
        gradients = self.calculate_spatial_derivatives(robot_pos, robot_vel)
        
        print(f"\nSpatial Derivatives (∇h):")
        for constraint in ['h_d', 'h_y', 'h_s', 'h_a']:
            grad = gradients[constraint]
            constraint_names = {
                'h_d': 'Distance',
                'h_y': 'Yielding', 
                'h_s': 'Speed',
                'h_a': 'Acceleration'
            }
            print(f"  ∇{constraint} ({constraint_names[constraint]}):")
            print(f"    ∂/∂x: {grad['x']:.4f}")
            print(f"    ∂/∂y: {grad['y']:.4f}")
            print(f"    ∂/∂θ: {grad['theta']:.4f}")
        
        # Evaluate CBF conditions using derivatives
        print(f"\nControl Barrier Function Conditions:")
        print(f"Using α = {alpha_C}")
        
        # CBF condition: ḣ + α·h ≥ 0
        cbf_conditions = {}
        constraints_data = [
            ('h_d', h_d, h_d_dot),
            ('h_s', h_s, h_s_dot),
            ('h_a', h_a, h_a_dot)
        ]
        
        if h_y_dot is not None:
            constraints_data.append(('h_y', h_y, h_y_dot))
        
        for name, h_val, h_dot_val in constraints_data:
            cbf_val = h_dot_val + alpha_C * h_val
            cbf_conditions[name] = cbf_val
            status = "✅ SATISFIED" if cbf_val >= 0 else "❌ VIOLATED"
            print(f"  {name}: ḣ + α·h = {h_dot_val:.4f} + {alpha_C}×{h_val:.4f} = {cbf_val:.4f} {status}")
        
        all_satisfied = all(val >= 0 for val in cbf_conditions.values())
        print(f"\nOverall CBF Status: {'✅ ALL SATISFIED' if all_satisfied else '❌ SOME VIOLATED'}")
        
        print("="*60)
        
def main():
    """Main function"""
    print("="*60)
    print("Robot Constraints Simulation")
    print("="*60)

    # Create and run simulation
    sim = RobotSimulation()
    sim.generate_random_trajectory()
    
    # Demonstrate derivative functions before animation
    print("\nDemonstrating constraint derivative functions...")
    sim.demonstrate_derivatives(t=0.5)  # Demonstrate at t=0.5 seconds
    sim.demonstrate_derivatives(t=1.0)  # Demonstrate at t=1.0 seconds
    
    print("\nStarting animation...")
    sim.setup_animation()
    anim = sim.run_animation(animation_duration)  # Hold reference to prevent garbage collection
    
    # Keep the animation reference alive
    return anim
    
if __name__ == "__main__":
    main()
