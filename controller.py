"""Speed controllers for the brushed DC motor.

Control laws that map a speed tracking error to an armature voltage command.
Each function is a pure function of its arguments: no state, no globals, no
dependency on the plant model or on the simulation. That is deliberate, so the
same law can run against any plant, and so it ports to C unchanged.

Saturation is applied inside the controller rather than by the caller, because
the limit is a property of the supply and driver that the control law must be
aware of. Actuator limits that the controller does not know about are how
integrator windup happens.

Conventions: speeds in rad/s, voltages in V, gains in V per rad/s.


Author: Fils Elie Boungoueres
"""

def proportional_controller(reference_angular_velocity: float, measured_angular_velocity: float, Kp: float, voltage_limit: float) -> float:
    """Return the saturated armature voltage for a proportional control law.

    V = Kp * (reference - measured), clamped to +/- voltage_limit.

    reference_angular_velocity  commanded shaft speed        [rad/s]
    measured_angular_velocity   measured shaft speed         [rad/s]
    Kp                          proportional gain            [V.s/rad]
    voltage_limit               supply limit, both polarities [V]

    Returns the voltage to apply [V]. Note that proportional action alone
    leaves a steady-state error of wref / (1 + Kp * dc_gain).
    """
    error = reference_angular_velocity - measured_angular_velocity
    control_voltage = Kp * error

    # Apply voltage limit
    control_voltage = max(-voltage_limit, min(voltage_limit, control_voltage))
    return control_voltage

class ProportionalIntegralController:
    """Discrete-time PI speed controller with output saturation.

    Implements, at each control step of length dt,

        e_k = reference - measured
        I_k = I_{k-1} + e_k * dt
        V_k = clamp(Kp * e_k + Ki * I_k,  -voltage_limit,  +voltage_limit)

    Unlike proportional control, the integral term can hold a nonzero output
    while the error is zero, so the steady-state error goes to zero rather
    than settling at w_ref / (1 + Kp * dc_gain).

    The accumulated integral is state held on the instance, which is why this
    is a class and not a function. One instance drives one loop; use a fresh
    instance or call reset() before each run, or the previous run's history
    carries over.

    Known limitation: the integral keeps accumulating while the output is
    saturated, even though extra command has no effect there. On a large step
    this overshoots badly on the way out. That is integrator windup, addressed
    in a later stage.

    Constructor arguments:
        Kp              proportional gain               [V.s/rad]
        Ki              integral gain                   [V/rad]
        dt              control period                  [s]
        voltage_limit   supply limit, both polarities   [V]

    Attributes:
        integral_error        accumulated error, sum of e * dt  [rad]
    """
    def __init__(self, Kp: float, Ki: float, dt: float, voltage_limit: float):
        """Initialize the PI controller with gains, time step, and voltage limit."""
        self.Kp = Kp
        self.Ki = Ki
        self.dt = dt
        self.voltage_limit = voltage_limit
        self.integral_error = 0.0  # Initialize the integral of the error

    def update(self, reference_angular_velocity: float, measured_angular_velocity: float) -> float:
        """Advance the controller one step and return the armature voltage.

        Call exactly once per control period. Calling it twice for the same
        instant integrates that error twice.

        reference_angular_velocity  commanded shaft speed  [rad/s]
        measured_angular_velocity   measured shaft speed   [rad/s]

        Returns the saturated voltage to apply [V].
        """
        error = reference_angular_velocity - measured_angular_velocity
        self.integral_error += error * self.dt  # Update the integral of the error
        control_voltage = self.Kp * error + self.Ki * self.integral_error

        # Apply voltage limit
        control_voltage = max(-self.voltage_limit, min(self.voltage_limit, control_voltage))
        return control_voltage
    
    def reset(self) -> None:
        """Clear the accumulated integral, returning the controller to its
        initial state. Call before re-running a simulation with the same
        instance."""
        self.integral_error = 0.0