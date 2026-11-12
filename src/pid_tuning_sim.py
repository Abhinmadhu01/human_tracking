#!/usr/bin/env python3
"""
PID Tuning Simulator for Drone Distance Control
Run this to visualize why your current P-only controller will oscillate.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

class DronePhysics:
    def __init__(self):
        self.position = 0.0  # Drone position
        self.velocity = 0.0
        self.target_pos = 10.0  # Target at 10m
        self.dt = 0.05  # 50Hz control loop
        
        # Physical limits
        self.max_vel = 5.0  # m/s
        self.max_accel = 2.0  # m/s²
        
    def update_target(self, t):
        """Target moves erratically like a human"""
        self.target_pos = 10.0 + 3.0 * np.sin(0.5 * t) + 1.0 * np.sin(2.5 * t)
        
    def step(self, accel_cmd):
        """Physics integration with limits"""
        # Apply acceleration limits
        accel = np.clip(accel_cmd, -self.max_accel, self.max_accel)
        
        # Integrate to velocity
        self.velocity += accel * self.dt
        self.velocity = np.clip(self.velocity, -self.max_vel, self.max_vel)
        
        # Integrate to position
        self.position += self.velocity * self.dt
        
        return self.position, self.velocity

class PIDController:
    def __init__(self, kp=0.28, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        
    def compute(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        self.integral = np.clip(self.integral, -5.0, 5.0)  # Anti-windup
        
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        
        return self.kp * error + self.ki * self.integral + self.kd * derivative

def run_simulation(kp=0.28, ki=0.0, kd=0.0, duration=20.0):
    """Run simulation and return error history"""
    drone = DronePhysics()
    pid = PIDController(kp, ki, kd)
    
    dt = 0.05
    times = []
    errors = []
    drone_positions = []
    target_positions = []
    
    for i in range(int(duration / dt)):
        t = i * dt
        
        # Update target (human movement)
        drone.update_target(t)
        
        # Control loop: Maintain 3m distance behind target
        # Setpoint: target_pos - 3.0
        setpoint = drone.target_pos - 3.0
        measurement = drone.position
        
        # PID computes desired acceleration
        accel_cmd = pid.compute(setpoint, measurement, dt)
        
        # Physics step
        pos, vel = drone.step(accel_cmd)
        
        # Log
        times.append(t)
        errors.append(setpoint - pos)  # Distance error
        drone_positions.append(pos)
        target_positions.append(drone.target_pos)
        
    return times, errors, drone_positions, target_positions

# Run comparison: Your current P-only vs Proper PID
if __name__ == "__main__":
    print("Running P-only (your current setup)...")
    t1, e1, d1, tgt1 = run_simulation(kp=0.28, ki=0.0, kd=0.0)
    
    print("Running PID with D-gain...")
    t2, e2, d2, tgt2 = run_simulation(kp=0.28, ki=0.02, kd=0.15)
    
    # Plot
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # Position plot
    ax1 = axes[0]
    ax1.plot(t1, tgt1, 'g--', label='Target Position', linewidth=2)
    ax1.plot(t1, d1, 'r-', label='P-only Drone (yours)', alpha=0.7)
    ax1.plot(t2, d2, 'b-', label='PID Drone (fixed)', alpha=0.7)
    ax1.axhline(y=tgt1[0]-3.0, color='g', linestyle=':', alpha=0.5, label='3m offset line')
    ax1.set_ylabel('Position (m)')
    ax1.legend()
    ax1.grid(True)
    ax1.set_title('Drone Tracking Performance Comparison')
    
    # Error plot
    ax2 = axes[1]
    ax2.plot(t1, e1, 'r-', label='P-only Error (oscillating)', linewidth=2)
    ax2.plot(t2, e2, 'b-', label='PID Error (damped)', linewidth=2)
    ax2.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax2.fill_between(t1, e1, alpha=0.3, color='red')
    ax2.fill_between(t2, e2, alpha=0.3, color='blue')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Distance Error (m)')
    ax2.legend()
    ax2.grid(True)
    
    # Statistics
    rmse1 = np.sqrt(np.mean(np.array(e1)**2))
    rmse2 = np.sqrt(np.mean(np.array(e2)**2))
    max_overshoot1 = np.max(np.abs(e1))
    max_overshoot2 = np.max(np.abs(e2))
    
    print(f"\n=== RESULTS ===")
    print(f"Your P-only: RMSE={rmse1:.2f}m, Max Overshoot={max_overshoot1:.2f}m")
    print(f"Fixed PID:   RMSE={rmse2:.2f}m, Max Overshoot={max_overshoot2:.2f}m")
    print(f"Improvement: {(1-rmse2/rmse1)*100:.1f}% accuracy")
    
    plt.tight_layout()
    plt.savefig('pid_comparison.png')
    print("\nSaved plot to pid_comparison.png")
    plt.show()
