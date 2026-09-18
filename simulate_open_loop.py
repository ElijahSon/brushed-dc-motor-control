"""Open-loop step response of the brushed DC motor.

Applies a constant armature voltage from rest and integrates the plant with
forward Euler, recording current and speed. The final speed is compared
against the analytic no-load value Kt*V / (R*b + Kt*Ke) as a check that the
model and the integrator agree.

Step size is set from the electrical time constant L/R = 5 ms. Forward Euler
is stable on this plant only for dt below 10.26 ms, and needs roughly 1 ms or
less for accuracy during the transient.

Writes: figures/open_loop_step.png

Author: Fils Elie Boungoueres
"""

from motor import MotorParams, dc_gain, derivative, forward_euler_step
import matplotlib.pyplot as plt

motor_a = MotorParams()                    # All default parameters because we didn't specify any in the parantheses
motor_b = MotorParams(resistance=2.0)      # R =2.0, everything else default
motor_c = MotorParams(torque_constant=0.1, voltage_constant=0.1) # K_t = 0.1, K_e = 0.1, everything else default

# gain calculations for different motors
gain_a = dc_gain(motor_a)
gain_b = dc_gain(motor_b)
gain_c = dc_gain(motor_c)

print(f"DC gain (motor_a) = {gain_a:.2f} rad/s per V")
print(f"DC gain (motor_b) = {gain_b:.2f} rad/s per V")
print(f"DC gain (motor_c) = {gain_c:.2f} rad/s per V")

# armature current and shaft angular velocity initialization
armature_current = 0.0 
shaft_angular_velocity = 0.0

# Simulation parameters and initial conditions
voltage_test = 12.0  # Test voltage in volts
speed_steady_state = dc_gain(motor_a) * voltage_test  # Steady-state speed for motor_a at 12V
current_steady_state = (motor_a.friction_coefficient/motor_a.torque_constant) * speed_steady_state

print(f"Derivative of current and angular velocity for motor_a: {derivative(current_steady_state, speed_steady_state, voltage_test, 0.0, motor_a)}")

dt = 0.0001  # Time step in seconds
t_end = 2.0 
n_steps = int(t_end / dt)
print(f"Simulating motor_a for {t_end} seconds with time step {dt} seconds: {n_steps} steps")

# list to store the results for plotting or analysis
current_vector = []
angular_velocity_vector = []

for k in range(n_steps):  
    current_vector.append(armature_current)
    angular_velocity_vector.append(shaft_angular_velocity)
    armature_current, shaft_angular_velocity = forward_euler_step(armature_current, shaft_angular_velocity, voltage_test, 0.0, motor_a, dt)
 
print(f"Final state after simulation: Current = {armature_current:.4f} A, Angular Velocity = {shaft_angular_velocity:.4f} rad/s")

# Forward Euler step for motor_a at steady state
print(f"Forward Euler step for motor_a: {forward_euler_step(current_steady_state, speed_steady_state, voltage_test, 0.0, motor_a, 0.01)}")

# time vector for plotting
time_vector = [k * dt for k in range(n_steps)]

# peak current and angular velocity for analysis
peak_current = max(current_vector)
peak_index_current = current_vector.index(peak_current)
peak_time_current = time_vector[peak_index_current]

# Plotting the results for current and angular velocity over time
plt.figure(figsize=(14, 6))
plt.subplot(2, 1, 1)

plt.axhline(peak_current, color="gray", linestyle="--", linewidth=0.8)
plt.plot(peak_time_current, peak_current, "o", color="red", markersize=5)
plt.annotate(f"peak {peak_current:.2f} A at {peak_time_current*1000:.0f} ms",
             xy=(peak_time_current, peak_current),
             xytext=(peak_time_current + 0.1, peak_current))
plt.plot(time_vector, current_vector)
plt.xlabel("Time (s)")
plt.ylabel("Current (A)")
plt.title("Simulation Results")
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(time_vector, angular_velocity_vector, label="simulated")
plt.axhline(gain_a * voltage_test, color="gray", linestyle="--", linewidth=0.8,
            label=f"analytic {gain_a * voltage_test:.2f} rad/s")
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.legend(loc="lower right")
plt.grid(True)

plt.tight_layout()

plt.savefig("figures/open_loop_step.png", dpi=160)
plt.show()