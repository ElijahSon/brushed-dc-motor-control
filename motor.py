"""Brushed DC motor: plant model and fixed-step integration.

State-space model of a separately excited brushed DC motor, armature
controlled, with a load torque on the shaft.

    L di/dt     = V - R*i - Ke*w
    J dw/dt     = Kt*i - b*w - T_L

State is [i, w]: armature current [A] and shaft angular velocity [rad/s].
Input is the applied armature voltage V [V]; the load torque T_L [N.m] is a
disturbance input.

All quantities are SI. This module has no side effects: it defines the
parameter container and three pure functions, and computes nothing on import.

Author: Fils Elie Boungoueres
"""
from dataclasses import dataclass

@dataclass(frozen=True)
class MotorParams:
    """Motor parameters for a specific motor model."""
    torque_constant: float = 0.05 # K_t in Nm/A
    voltage_constant: float = 0.05  # K_e in V/(rad/s)
    resistance: float = 1.0  # R in Ohms
    inductance: float = 0.005  # L in Henrys
    friction_coefficient: float = 1e-4  # B in Nm/(rad/s)
    rotor_inertia: float = 5e-4  # J in kg*m^2


def dc_gain(p: MotorParams) -> float:
    """Calculate the DC gain of the motor."""
    return p.torque_constant / (p.resistance * p.friction_coefficient + p.torque_constant * p.voltage_constant)

# derivative of current and angular velocity for control purposes
def derivative(current: float, angular_velocity: float, voltage: float, torque_load: float, p: MotorParams) -> tuple[float, float]:
    """Calculate the derivatives of current and angular velocity."""
    derivative_current = (voltage - p.resistance * current - p.voltage_constant * angular_velocity) / p.inductance
    derivative_angular_velocity = (p.torque_constant * current - p.friction_coefficient * angular_velocity - torque_load) / p.rotor_inertia # acceleration angular of the motor shaft
    return derivative_current, derivative_angular_velocity


# current and angular velocity after Forward Euler integration step
def forward_euler_step(current: float, angular_velocity: float, voltage: float, torque_load: float, p: MotorParams, dt: float) -> tuple[float, float]:
    """Perform a single Forward Euler integration step."""
    d_current, d_angular_velocity = derivative(current, angular_velocity, voltage, torque_load, p)
    new_current = current + d_current * dt
    new_angular_velocity = angular_velocity + d_angular_velocity * dt
    return new_current, new_angular_velocity