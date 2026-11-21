# Phase 7 — Simscape Multibody Validation Summary

## Phase 7.10N: Complete Mechanical Energy Validation

**Status: PASSED**

High-accuracy Simscape simulation demonstrated excellent mechanical energy conservation.

| Measurement | Result |
|---|---:|
| Initial energy | 4.8912213291 J |
| Final energy | 4.8912270304 J |
| RMS energy drift | 0.00003779 J |
| Maximum energy drift | 0.00024947 J |
| Maximum relative drift | 0.00510038% |

Rigid-body kinematics using measured angular velocity were used to calculate translational velocities, avoiding numerical differentiation noise.

## Phase 7.10O: MATLAB–Simscape Dynamic Validation

**Status: PASSED for short-term dynamics; PARTIAL for full-duration trajectory agreement.**

Simulation duration: 15 seconds.

- MATLAB samples: 1,501
- Simscape samples: 3,006
- Simscape solver: high accuracy
- Comparison performed after time alignment.

### Short-term comparison: 0–3 seconds

| Measurement | RMS error |
|---|---:|
| Cart position | 0.000320 m |
| Bob absolute position | 0.001562 m |
| Pendulum relative position | 0.001815 m |
| Angular velocity | 0.005133 rad/s |
| Inclination | 0.075662° |
| Azimuth | 0.631108° |

The two models show strong agreement during the initial transient.

### Full-duration comparison: 0–15 seconds

| Measurement | RMS error |
|---|---:|
| Cart position | 0.030131 m |
| Bob absolute position | 0.120573 m |
| Pendulum relative position | 0.150690 m |
| Cart velocity | 0.124689 m/s |
| Bob velocity | 0.499209 m/s |
| Angular velocity | 0.892277 rad/s |
| Inclination | 0.324710° |
| Azimuth | 14.986550° |

Long-term differences are attributed to the unstable uncontrolled inverted-pendulum dynamics, numerical integration differences, and physical-model differences.

Azimuth becomes particularly sensitive when the pendulum's horizontal projection is small.

## Overall Phase 7 Assessment

The following have been demonstrated:

1. Correct X-Y cart assembly.
2. Correct inverted spherical-pendulum geometry.
3. Correct initial orientation and rod length.
4. Successful Simscape model compilation and simulation.
5. Correct angular velocity measurement.
6. Excellent mechanical energy conservation.
7. Strong short-term agreement with the independent MATLAB model.

The Simscape model is suitable for proceeding to the next validation and control stages.

**Important limitation:** Long-term uncontrolled trajectory agreement has not been established. This must remain documented in the final project report.