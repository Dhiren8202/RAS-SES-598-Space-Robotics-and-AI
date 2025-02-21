import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load logged data
data = pd.read_csv('lqr_performance.csv')

# Create subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))

# Plot cart position
ax1.plot(data['time'], data['x'], color='blue')
ax1.set_ylabel('Cart Position (m)')
ax1.axhline(y=0.5, color='red', linestyle='--', label='Safe Limit')
ax1.axhline(y=-0.5, color='red', linestyle='--')
ax1.legend()

# Plot pendulum angle (convert radians to degrees)
ax2.plot(data['time'], data['theta'] * 180/np.pi, color='orange')
ax2.set_ylabel('Pendulum Angle (°)')
ax2.axhline(y=3, color='red', linestyle='--', label='Stability Threshold')
ax2.axhline(y=-3, color='red', linestyle='--')

# Plot control force
ax3.plot(data['time'], data['force'], color='green')
ax3.set_ylabel('Control Force (N)')
ax3.set_xlabel('Time (s)')

plt.suptitle('LQR Controller Performance')
plt.tight_layout()
plt.savefig('lqr_performance.png')
plt.show()

# Plot Q and R impact (example data)
test_cases = [
    {'Q': [1, 1, 10, 10], 'R': 0.1, 'max_x': 2.5, 'max_theta': 20},
    {'Q': [50, 10, 50, 20], 'R': 0.1, 'max_x': 1.2, 'max_theta': 10},
    {'Q': [200, 50, 150, 75], 'R': 0.01, 'max_x': 0.4, 'max_theta': 3},
]

Q0_values = [case['Q'][0] for case in test_cases]
max_x = [case['max_x'] for case in test_cases]
max_theta = [case['max_theta'] for case in test_cases]
R_values = [case['R'] for case in test_cases]

# Plot Q[0] vs. max cart displacement
plt.figure(figsize=(10, 5))
plt.plot(Q0_values, max_x, marker='o', linestyle='--', color='blue')
plt.xlabel('Q[0] (Cart Position Penalty)')
plt.ylabel('Max Cart Displacement (m)')
plt.title('Impact of Q[0] on Cart Stability')
plt.grid(True)
plt.savefig('Q0_impact.png')
plt.show()

# Plot R vs. max pendulum angle
plt.figure(figsize=(10, 5))
plt.plot(R_values, max_theta, marker='o', linestyle='--', color='red')
plt.xlabel('R (Control Effort Penalty)')
plt.ylabel('Max Pendulum Angle (°)')
plt.title('Impact of R on Pendulum Stability')
plt.grid(True)
plt.savefig('R_impact.png')
plt.show()

# Phase portrait
plt.figure(figsize=(8, 6))
plt.scatter(data['theta'], data['theta_dot'], c=data['time'], cmap='viridis')
plt.xlabel('Pendulum Angle (rad)')
plt.ylabel('Angular Velocity (rad/s)')
plt.colorbar(label='Time (s)')
plt.title('Phase Portrait: Pendulum Dynamics')
plt.savefig('phase_portrait.png')
plt.show()

# Control effort distribution
plt.figure(figsize=(10, 5))
plt.hist(data['force'], bins=30, color='green', alpha=0.7)
plt.xlabel('Control Force (N)')
plt.ylabel('Frequency')
plt.title('Control Effort Distribution')
plt.savefig('force_histogram.png')
plt.show()