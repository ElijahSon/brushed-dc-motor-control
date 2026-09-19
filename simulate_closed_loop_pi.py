"""Anti-windup comparison: PI speed control of the brushed DC motor.

Runs the same closed loop twice, identical plant and identical gains, changing
only whether the controller suppresses integral accumulation while its output
is saturated. Everything else is held fixed so the difference between the two
traces is attributable to one variable.

Mechanism being demonstrated. A step to 200 rad/s demands roughly 200 V at
t = 0 from a 24 V supply, so the command sits on the rail for about 0.13 s.
Without protection the integral keeps accumulating through that period even
though extra command changes nothing, reaching about 5 times the value needed
to hold steady state. It can only discharge through sustained negative error,
which means the motor must overspeed. The overshoot is not a side effect of
windup, it is the mechanism by which the integrator unwinds.

Measured at Kp = 1.0, Ki = 5.0, 24 V limit, step to 200 rad/s:

    without anti-windup   peak 238.87 rad/s   19.44% overshoot
    with anti-windup      peak 200.00 rad/s    0.00% overshoot

Both reach the reference exactly, since integral action removes steady-state
error either way. Anti-windup also more than halves the settling time, so the
protection costs nothing in speed of response.

Plant, controller and integrator come from motor.py and controller.py; this
script holds only the specific case being run.

Writes: figures/anti_windup_comparison.png

Author: Fils Elie Boungoueres
"""

from motor import MotorParams, forward_euler_step, dc_gain
from controller import ProportionalIntegralController
import matplotlib.pyplot as plt

COLOUR_WITHOUT = "#c2410c"
COLOUR_WITH = "#0b6fb8"

motor_a = MotorParams()

dt = 1e-4
t_end = 2.0
n_steps = int(t_end / dt)


Kp = 1.0  # Proportional gain [V.s/rad]
Ki = 5.0  # Integral gain [V/rad]
voltage_limit = 24.0  # Supply voltage limit [V]
reference_angular_velocity = 200.0  # Desired angular velocity [rad/s]

def run_simulation(anti_windup: bool):
    """Run one closed-loop simulation from rest and return its histories.

    Builds a fresh controller each call, so the integral starts at zero and no
    reset() is needed between runs. The plant is integrated with forward Euler
    at the module-level dt; the controller is called once per step.

    anti_windup   if True, the controller suppresses integral accumulation
                  while its output is saturated and the error would push it
                  further into the limit

    Returns a tuple of four lists, each n_steps long, sampled before the state
    is advanced so element 0 is the initial condition:
        armature current [A]
        shaft angular velocity [rad/s]
        commanded voltage after saturation [V]
        accumulated integral of the error [rad]
    """
    armature_current = 0.0
    shaft_angular_velocity = 0.0

    current_vector = []
    angular_velocity_vector = []
    voltage_command_vector = []  # List to store voltage commands for plotting
    integral_error_vector = []      # with the other lists

    controller = ProportionalIntegralController(Kp=Kp, Ki=Ki, dt=dt, voltage_limit=voltage_limit, anti_windup=anti_windup)

    for k in range(n_steps):
        voltage_command = controller.update(reference_angular_velocity, shaft_angular_velocity)

        current_vector.append(armature_current)
        angular_velocity_vector.append(shaft_angular_velocity)
        voltage_command_vector.append(voltage_command)
        integral_error_vector.append(controller.integral_error)

        armature_current, shaft_angular_velocity = forward_euler_step(armature_current, shaft_angular_velocity, voltage_command, 0.0, motor_a, dt)

    return (current_vector, angular_velocity_vector, voltage_command_vector, integral_error_vector)

(current_vector_without, angular_velocity_vector_without,
 voltage_command_vector_without, integral_error_vector_without) = run_simulation(anti_windup=False)

(current_vector_with, angular_velocity_vector_with,
 voltage_command_vector_with, integral_error_vector_with) = run_simulation(anti_windup=True)

# predicted angular velocity and voltage command based on motor dynamics and plant gains
predicted_angular_velocity = reference_angular_velocity
predicted_voltage_command = reference_angular_velocity / dc_gain(motor_a)

time_vector = [k * dt for k in range(n_steps)]
peak_without = max(angular_velocity_vector_without)
k_peak_without = angular_velocity_vector_without.index(peak_without)

def report(label, angular_velocity_vector, voltage_command_vector, integral_error_vector):
    """Print the step-response numbers for one run, next to their predictions.

    Overshoot is measured against the reference, not the final value, so a
    negative figure means the response never reached the command rather than
    that it undershot on the way.

    label   short description of the run, printed as a header
    """
    peak = max(angular_velocity_vector)
    overshoot = (peak - reference_angular_velocity) / reference_angular_velocity * 100
    print(f"\n--- {label} ---")
    print(f"Final speed     {angular_velocity_vector[-1]:9.4f} rad/s   (predicted {predicted_angular_velocity:.4f})")
    print(f"Final voltage   {voltage_command_vector[-1]:9.4f} V       (predicted {predicted_voltage_command:.4f})")
    print(f"Peak speed      {peak:9.4f} rad/s  ({overshoot:+.2f}%)")
    print(f"Peak integral   {max(integral_error_vector):9.4f} rad     (required {predicted_voltage_command / Ki:.4f})")


report("without anti-windup", angular_velocity_vector_without,
       voltage_command_vector_without, integral_error_vector_without)
report("with anti-windup", angular_velocity_vector_with,
       voltage_command_vector_with, integral_error_vector_with)

# panel plot of the results 
plt.figure(figsize=(12, 8))

plt.subplot(2, 2, 1)
plt.plot(time_vector, angular_velocity_vector_without, color=COLOUR_WITHOUT, linestyle="-", label="without anti-windup")
plt.plot(time_vector, angular_velocity_vector_with, color=COLOUR_WITH, linestyle="--", label="with anti-windup")
plt.axhline(reference_angular_velocity, color="black", linestyle=":", linewidth=2.0, label=f"reference {reference_angular_velocity:.0f} rad/s")
plt.plot(time_vector[k_peak_without], peak_without, "o", color=COLOUR_WITHOUT, markersize=5)
plt.annotate(f"{peak_without:.1f} rad/s ({(peak_without - reference_angular_velocity) / reference_angular_velocity * 100:+.1f}%)",
             xy=(time_vector[k_peak_without], peak_without),
             xytext=(time_vector[k_peak_without] + 0.08, peak_without),
             color=COLOUR_WITHOUT)
plt.ylabel("Angular Velocity [rad/s]")
plt.xlabel("Time [s]")
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 2)
plt.plot(time_vector, current_vector_without, color=COLOUR_WITHOUT, linestyle="-", label="without anti-windup")
plt.plot(time_vector, current_vector_with, color=COLOUR_WITH, linestyle="--", label="with anti-windup")
plt.xlabel("Time [s]")
plt.ylabel("Armature Current [A]")
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 3)
plt.plot(time_vector, voltage_command_vector_without, color=COLOUR_WITHOUT, linestyle="-", label="without anti-windup")
plt.plot(time_vector, voltage_command_vector_with, color=COLOUR_WITH, linestyle="--", label="with anti-windup")
plt.axhline(predicted_voltage_command, color="black", linestyle=":", linewidth=2.0, label=f"required {predicted_voltage_command:.1f} V")
plt.ylabel("Voltage Command [V]")
plt.xlabel("Time [s]")
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 4)
plt.plot(time_vector, integral_error_vector_without, color=COLOUR_WITHOUT, linestyle="-", label="without anti-windup")
plt.plot(time_vector, integral_error_vector_with, color=COLOUR_WITH, linestyle="--", label="with anti-windup")
plt.axhline(predicted_voltage_command / Ki, color="black", linestyle=":", linewidth=2.0, label=f"required {predicted_voltage_command / Ki:.2f} rad")
peak_integral_without = max(integral_error_vector_without)
k_peak_integral = integral_error_vector_without.index(peak_integral_without)
plt.plot(time_vector[k_peak_integral], peak_integral_without, "o", color=COLOUR_WITHOUT, markersize=5)
plt.annotate(f"{peak_integral_without:.2f} rad ({peak_integral_without / (predicted_voltage_command / Ki):.1f}x required)",
             xy=(time_vector[k_peak_integral], peak_integral_without),
             xytext=(time_vector[k_peak_integral] + 0.1, peak_integral_without),
             color=COLOUR_WITHOUT)
plt.ylabel("Integral of Error [rad]")
plt.xlabel("Time [s]")
plt.legend()
plt.grid(True)

plt.suptitle(f"PI speed control, Kp={Kp} Ki={Ki}, {voltage_limit:.0f} V limit: effect of anti-windup")
plt.tight_layout()
plt.savefig("figures/anti_windup_comparison.png", dpi=200)
plt.show()