# Results log: brushed DC motor speed control

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
