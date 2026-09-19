# Phase 1 control requirements: brushed DC motor speed loop

Author: Fils Elie Boungoueres

Status: **draft.** Values marked TBD are engineering decisions that have not
been made. Everything else is a proposal with its reasoning stated, to be
accepted or changed, not a decision already taken.

Written 19 Sept 2026, after the anti-windup result and before frequency-domain
tuning. Until this point the controller had no specification: gains were chosen
by inspection of plots, which is why "is this controller good?" had no answer.
Every row below turns a preference into a claim that can be met or missed.

## Plant and actuator, given

These are not requirements, they are the facts the requirements must respect.

| quantity | value | source |
|---|---|---|
| DC gain | 19.23 rad/s per V | Kt / (R·b + Kt·Ke) |
| dominant time constant | 187.4 ms | pole at -5.337 rad/s |
| fast time constant | 5.13 ms | pole at -194.86 rad/s |
| supply voltage | 24 V, both polarities | hard limit |
| maximum reachable speed | 461 rad/s | 19.23 × 24 |
| steady-state current at 200 rad/s | 0.46 A | b·ω / Kt |
| open-loop droop | 384.6 rad/s per N.m | R / (R·b + Kt·Ke) |

## Functional requirements

| # | requirement | value | rationale | status |
|---|---|---|---|---|
| F1 | Steady-state error to a constant speed reference | 0 rad/s | Integral action provides it at no cost; anything else is a choice to be worse. | met, 0.004 rad/s residual at t = 2 s, converging |
| F2 | Overshoot, measured against the reference | ≤ 2% | TBD. Proposed on the grounds that a speed loop feeding a mechanical load should not exceed commanded speed meaningfully. A real number is needed; 2% is a placeholder until the load is characterised. | met with anti-windup (0.00%), failed without (19.44%) |
| F3 | Settling time to a 2% band | TBD | Cannot be set without knowing what the motor drives. Currently 0.280 s. | measured, not specified |
| F4 | Rise time, 10% to 90% of reference | TBD | Follows from F3 and the bandwidth choice rather than being set independently. | measured, not specified |

## Actuator and hardware constraints

| # | requirement | value | rationale | status |
|---|---|---|---|---|
| A1 | Commanded voltage within supply | \|V\| ≤ 24 V | Physical. Enforced by saturation inside the controller. | met by construction |
| A2 | Controller must remain well behaved while saturated | no windup-induced overshoot | Saturation is not an edge case here: a step to 200 rad/s saturates for 0.13 s, which is 13% of the settling time. | met by conditional integration |
| A3 | Peak armature current | TBD | **The largest open gap.** Currently 22 A peak against a 0.46 A steady-state current, a factor of 48. No element of the loop observes current, so nothing bounds it. A real motor of this size would not survive the step. Requires either a datasheet figure or a stated design assumption, then a cascaded current loop to enforce it. | not specified, not met |
| A4 | Control period | 1 ms or faster | TBD. Currently 0.1 ms, equal to the simulation step, which is not realistic for an embedded speed loop. Should be set from what the target MCU can sustain, then the plant simulated faster than the controller runs. | not yet separated from dt |

## Robustness

| # | requirement | value | rationale | status |
|---|---|---|---|---|
| R1 | Phase margin | ≥ 45° | Standard minimum for a loop expected to tolerate modelling error. | not evaluated |
| R2 | Gain margin | ≥ 6 dB | As above. | not evaluated |
| R3 | Crossover frequency ω_c | TBD | **This is the blocking decision.** The pole-zero cancellation design has one free parameter and cannot proceed without it. Constraints: ω_c well below 1/τ₁ = 194.9 rad/s or the fast pole eats the phase margin; higher ω_c means more voltage demand against a rail that already saturates. | blocks the frequency synthesis |
| R4 | Performance must hold across the operating range | TBD | The Ki = 1 experiment showed a fix that held at 200 rad/s and failed at 400. Any requirement verified at one operating point only is not verified. Range must be stated. | not specified |

## Disturbance rejection

| # | requirement | value | rationale | status |
|---|---|---|---|---|
| D1 | Speed dip for a step load torque | TBD | Open-loop droop is 384.6 rad/s per N.m, so 0.3 N.m costs 115 rad/s uncontrolled. The closed-loop target needs a number and a torque magnitude to be tested against. | not specified, not tested |
| D2 | Recovery time after a load step | TBD | As above. | not specified, not tested |

## Out of scope for Phase 1, stated so the omissions are deliberate

- Sensor model. Speed is currently measured perfectly: no quantisation, no
  noise, no delay. This flatters any derivative action and must be added before
  D is evaluated honestly.
- Temperature dependence of R and of the magnets.
- Coulomb friction and stiction. Only viscous friction is modelled, so
  behaviour near zero speed is not represented.
- Hardware validation. Everything here is simulation.

## Decisions needed, in priority order

1. **ω_c** (R3). Blocks the analytic tuning, which blocks the comparison
   against the hand-picked gains in the report.
2. **Peak current** (A3). Blocks the cascaded current loop, and it is the
   largest gap between this simulation and a motor that survives.
3. **Operating range** (R4). Determines whether any of the above has been
   verified or merely observed once.
4. **Control period** (A4). Determines whether the results transfer to the C
   implementation on real hardware.
