"""Closed-loop step response: proportional speed control of the DC motor.

Commands a constant reference speed from rest and closes the loop through a
proportional control law, V = Kp * (reference - measured), saturated at the
supply limit. Records armature current, shaft speed and the commanded voltage.

The simulated final speed is compared against the analytic closed-loop steady
state. Substituting V = Kp * (w_ref - w) into the open-loop steady-state
relation w = Kt * V / (R*b + Kt*Ke) and solving for w gives

                            Kt * Kp
    w_ss  =  w_ref * ---------------------------
                      R*b + Kt*Ke + Kt*Kp

Equivalently w_ss = w_ref * L / (1 + L), with loop gain

    L = Kp * Kt / (R*b + Kt*Ke) = Kp * dc_gain

Since L is finite and positive, w_ss is strictly below w_ref for every Kp.
Proportional action alone cannot remove steady-state error. Raising Kp shrinks
it but demands a voltage the supply cannot deliver during the transient: at
Kp = 10.35 the command at t = 0.0, when the error is the full 200.0 rad/s, is
2070 V. That is the argument for integral action, added in the next stage.

The comparison is valid only while the controller is out of saturation at
steady state. If Kp * e_ss exceeds the voltage limit the loop is not linear
there and the expression above does not apply. At Kp = 1.0 and w_ref = 200.0 rad/s,
e_ss is 9.89 rad/s, so the command is 9.89 V and the condition holds for both a
12.0 V and a 24.0 V supply.

Plant and controller are imported from motor.py and controller.py; this script
holds only the specific case being run.

Writes: figures/closed_loop_step_response_<Kp>.png

Author: Fils Elie Boungoueres
"""

from motor import MotorParams, forward_euler_step, dc_gain
from controller import proportional_controller
import matplotlib.pyplot as plt

motor_a = MotorParams()

dt = 1e-4
t_end = 2.0
n_steps = int(t_end / dt)


Kp = 1.0  # Proportional gain [V.s/rad]
voltage_limit = 24.0  # Supply voltage limit [V]
reference_angular_velocity = 200.0  # Desired angular velocity [rad/s]

armature_current = 0.0
shaft_angular_velocity = 0.0

current_vector = []
angular_velocity_vector = []
voltage_command_vector = []  # List to store voltage commands for plotting

for k in range(n_steps):  
    voltage_command = proportional_controller(reference_angular_velocity, shaft_angular_velocity, Kp, voltage_limit)

    current_vector.append(armature_current)
    angular_velocity_vector.append(shaft_angular_velocity)
    voltage_command_vector.append(voltage_command)

    armature_current, shaft_angular_velocity = forward_euler_step(armature_current, shaft_angular_velocity, voltage_command, 0.0, motor_a, dt)

# prediction from the derivative function
loop_gain = Kp * dc_gain(motor_a)
predicted_speed = reference_angular_velocity * loop_gain / (1.0 + loop_gain)

# comparison of the final simulated speed with the predicted speed in a table
print(f"Final simulated speed: {shaft_angular_velocity:.4f} rad/s")
print(f"Predicted steady-state speed: {predicted_speed:.4f} rad/s")
print(f"Difference: {shaft_angular_velocity - predicted_speed:.4f} rad/s")

# panel plot of the results
time_vector = [k * dt for k in range(n_steps)]

plt.figure(figsize=(12, 8))
plt.subplot(3, 1, 1)
plt.plot(time_vector, angular_velocity_vector, label="Simulated Speed")
plt.axhline(reference_angular_velocity, color='g', linestyle='--', label="Reference Speed")
plt.axhline(y=predicted_speed, color='r', linestyle='--', label="Predicted Steady-State Speed") 
plt.ylabel("Angular Velocity [rad/s]")
plt.legend()
plt.grid(True)
plt.subplot(3, 1, 2)
plt.plot(time_vector, current_vector, label="Armature Current", color='orange')
plt.ylabel("Current [A]")
plt.legend()
plt.grid(True)
plt.subplot(3, 1, 3)
plt.plot(time_vector, voltage_command_vector, label="Voltage Command", color='green')
plt.ylabel("Voltage [V]")
plt.xlabel("Time [s]")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig(f"figures/closed_loop_step_response_{Kp}.png", dpi=300)
plt.show()