# Results log: brushed DC motor speed control

Author: Fils Elie Boungoueres

Each entry records what was run, what came out, and what it was checked
against. A number with no reference to compare it to is not a result.

---

## Plant parameters

| symbol | name | value | units |
|---|---|---|---|
| Kt | torque constant | 0.05 | N.m/A |
| Ke | voltage constant | 0.05 | V.s/rad |
| R  | armature resistance | 1.0 | ohm |
| L  | armature inductance | 5.0e-3 | H |
| b  | viscous friction | 1.0e-4 | N.m.s/rad |
| J  | rotor inertia | 5.0e-4 | kg.m^2 |

Derived, analytic:

| quantity | expression | value |
|---|---|---|
| DC gain | Kt / (R.b + Kt.Ke) | 19.23 rad/s per V |
| droop | R / (R.b + Kt.Ke) | 384.6 rad/s per N.m |
| electrical time constant | L / R | 5.0 ms |
| mechanical time constant | J.R / (Kt.Ke + R.b) | 192 ms |

---

## 2026-09-17: open-loop step, 12 V, no load

**Setup.** Forward Euler, dt = 1e-4 s, t_end = 2.0 s, 20000 steps.
Initial state (0 A, 0 rad/s).

**Result.** Final speed 230.7638 rad/s, final current 0.4618 A.
Peak current 11.2 A at roughly 20 ms.

**Check.** Analytic no-load speed at 12 V is 19.23 x 12 = 230.77 rad/s.
Simulation agrees to 4 significant figures.

**Figure.** `figures/open_loop_step.png`

**Notes.** Peak current is 24 times the steady-state value, because at t = 0
there is no back-EMF and only R limits the current. A real motor of this size
would not survive it. Motivates a current limit later.

---

## Integrator study

| dt | speed at t = 20 ms | speed at t = 2 s |
|---|---|---|
| 10 ms | 24.00 | 230.765 |
| 5 ms | 17.83 | 230.764 |
| 1 ms | 17.67 | 230.764 |
| 0.1 ms | 17.656 | 230.764 |

**Conclusion.** The final value is insensitive to dt because steady state is a
fixed point of the Euler map. Integration error lives in the transient. Euler
stability limit for this plant is dt < 2/194.9 = 10.26 ms (fast eigenvalue
-194.9). Accuracy requires roughly 1 ms or less. Running at 1e-4 s.

---

## Template for new entries

## YYYY-MM-DD: <what was run>

**Setup.**

**Result.**

**Check.** (against what independent reference?)

**Figure.**

**Notes.**

---

## 2026-09-18: PI tuning by frequency synthesis (on paper, not yet implemented)

Parked mid-derivation so the PI simulation could be finished first. Resume here.
Worth carrying into the IEEE report as the analytic design section, to compare
against the hand-picked Kp = 1, Ki = 5.

**Plant transfer function**, Laplace of the two state equations with zero
initial conditions and T_L = 0, eliminating I(s):

    G(s) = w(s)/V(s) = Kt / [ L*J*s^2 + (R*J + L*b)*s + (R*b + Kt*Ke) ]

Check: G(0) = Kt/(R*b + Kt*Ke) = 19.23 rad/s/V, matching the time-domain
derivation.

**Poles.** Coefficients L*J = 2.5e-6, R*J + L*b = 5.005e-4,
R*b + Kt*Ke = 2.6e-3. Roots via np.roots:

    s1 = -194.86 rad/s   ->  tau1 = 5.13 ms
    s2 = -5.337 rad/s    ->  tau2 = 187.4 ms

Compare with the day-one decoupled approximations: L/R = 5.0 ms and
J*R/(Kt*Ke + R*b) = 192.3 ms. Both within a few percent, which quantifies how
good the decoupling assumption is for this machine.

    G(s) = K / [(1 + tau1*s)(1 + tau2*s)],  K = 19.23

**Method chosen: pole-zero cancellation.** Write the PI in time-constant form

    C(s) = Kp * (1 + Ti*s) / (Ti*s)

and set Ti = tau2 = 0.1874 s, so the controller zero cancels the dominant slow
pole. The open loop collapses to an integrator plus the one fast pole:

    L(s) = Kp*K / [ Ti*s*(1 + tau1*s) ]

**Remaining step.** One condition left, |L(jwc)| = 1, and one unknown, Kp.
Requires choosing a crossover frequency wc. Constraints: wc must sit well below
1/tau1 = 194.9 rad/s or the fast pole eats the phase margin, and higher wc means
a larger voltage demand against a 24 V rail that already saturates on a step to
200 rad/s. wc not yet chosen.

Then Kp = Ti*wc/K (valid while wc << 1/tau1), and Ki = Kp/Ti.

---

## 2026-09-19: PI control and the effect of anti-windup

**Setup.** Plant `motor.py` with default MotorParams, forward Euler, dt = 1e-4 s,
t_end = 2.0 s. Controller `ProportionalIntegralController` with Kp = 1.0,
Ki = 5.0, voltage limit 24 V, control period equal to dt. Step reference from
rest to 200 rad/s, no load torque. The same run executed twice, changing only
the `anti_windup` flag, so every difference below is attributable to that one
variable.

**Results.**

| quantity | without anti-windup | with anti-windup |
|---|---|---|
| peak speed | 238.87 rad/s | 200.00 rad/s |
| overshoot (vs reference) | +19.44% | 0.00% |
| settling to 2% band | 0.602 s | 0.280 s |
| final speed | 200.000 rad/s | 200.000 rad/s |
| final voltage command | 10.400 V | 10.400 V |
| peak integral | 10.66 rad | 2.08 rad |
| integral overcharge factor | 5.1x | 1.0x |
| time saturated at 24 V | 0.133 s | 0.095 s |

**Checks.** Both cases reach the reference exactly, since integral action
removes steady-state error regardless of windup protection. Both settle on
10.400 V, which equals reference / dc_gain = 200 / 19.23, the voltage the plant
requires. At steady state the error is zero, so the proportional term
contributes nothing and the entire 10.4 V comes from the integral term:
Ki * 2.08 = 10.40 V. The 238.87 figure was reproduced independently from a
separately written driver loop, confirming the behaviour belongs to the
controller and plant rather than to the simulation script.

**Mechanism.** The step demands roughly 200 V at t = 0 from a 24 V supply, so
the command sits on the rail for the first 0.13 s. During that period the
unprotected integral keeps accumulating even though additional command has no
effect on the motor, reaching 10.66 rad against the 2.08 rad needed to hold
steady state. Discharging that excess requires sustained negative error, and
negative error means the speed must exceed the reference.

**The overshoot is therefore not a side effect of windup: it is the mechanism
by which the integrator discharges.** The speed returns to 200 on the same
timescale the integral takes to fall to 2.08.

**Method used.** Conditional integration. Accumulation is skipped when the
tentative output exceeds the limit and the error would push it further in the
same direction. The direction check matters: blocking accumulation whenever
the output is merely saturated would also block the unwinding that must happen
once the error changes sign.

**Cost.** None observed at this operating point. Anti-windup removed the
overshoot and more than halved the settling time, at identical gains. The
protected run approaches the reference more slowly in the last 5% because its
integral starts from zero when it leaves saturation, which is the honest
argument for trying back-calculation later as a comparison.

**Figure.** `figures/anti_windup_comparison.png`

---

## 2026-09-19: detuning tested as an alternative to anti-windup, rejected

Lowering Ki was tested as a cheaper fix. Kp = 1.0 throughout, no anti-windup.

Step to 200 rad/s:

| Ki | overshoot |
|---|---|
| 5.0 | 19.44% |
| 2.0 | 5.67% |
| 1.0 | 0.68% |
| 5.0 with anti-windup | 0.00% |

At this operating point Ki = 1 looks excellent. Repeating with a step to
400 rad/s, still within what 24 V can reach (461 rad/s):

| Ki | overshoot |
|---|---|
| 5.0 | 15.15% |
| 1.0 | 7.39% |
| 5.0 with anti-windup | 0.00% |

**Conclusion.** Detuning reduces exposure to windup at the operating point
tested; it does not remove the mechanism. A larger step keeps the actuator
saturated for longer, the integral overcharges again, and 0.68% becomes 7.39%.
Anti-windup gives zero overshoot at both operating points while keeping the
faster integral action. Recorded because the negative result is worth as much
as the positive one in the report.

---

## 2026-09-19: derivative action tested as an alternative to anti-windup, rejected

Whether a D term could substitute for anti-windup. Kp = 1.0, Ki = 5.0,
derivative on measurement, no anti-windup, step to 200 rad/s.

| Kd | overshoot | time saturated |
|---|---|---|
| 0.0 | 19.44% | 0.133 s |
| 0.01 | 17.61% | 0.123 s |
| 0.05 | 15.23% | 0.071 s |
| 0.2 | 27.81% | 0.002 s |
| 0.0, anti-windup on | 0.00% | 0.095 s |

**Conclusion.** Derivative action barely helps and eventually hurts. While the
output is against the rail the applied voltage is 24 V regardless of what any
term computes, so D has no influence during the phase where the damage occurs,
and the integral overcharges exactly as before. At Kd = 0.2 the output leaves
saturation almost entirely but the approach slows, the error stays positive
longer, the integral accumulates more, and overshoot rises to 27.81%.

D shapes the linear response; anti-windup handles an actuator nonlinearity.
Neither substitutes for the other, and a PID with saturation and no anti-windup
is still a badly behaved controller.
