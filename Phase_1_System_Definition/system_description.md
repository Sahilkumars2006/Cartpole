# System Description
## Mathematical Modelling, Simulation and Analysis of a 3D X-Y Cart with an Inverted Spherical Pendulum

## 1. Introduction

This project focuses on the mathematical modelling, simulation, and analysis of a three-dimensional inverted spherical pendulum mounted on a two-axis translating cart.

The system consists of a cart that can independently translate along the horizontal X and Y directions. An inverted pendulum is mounted on the cart through a spherical joint. The pendulum consists of a rigid, massless rod with a point mass attached at its upper end.

Unlike a conventional hanging pendulum, the rod extends upward from the cart. The pendulum is initially displaced slightly from its upright equilibrium and is allowed to move freely under gravity.

The objective is to develop a nonlinear mathematical model, simulate its dynamic behaviour, verify the derived equations, and later investigate control strategies for stabilizing the inverted pendulum.

## 2. System Configuration

The physical system consists of the following components:

| Component | Description |
|---|---|
| Cart | Rigid body of mass M |
| Pendulum rod | Rigid, massless rod of length L |
| Pendulum bob | Point mass m attached to the upper end of the rod |
| Spherical joint | Connects the pendulum rod to the cart |
| Gravity | Acts vertically downward along the negative Z-axis |

The cart is constrained to move in the horizontal X-Y plane, while the pendulum can rotate freely in three-dimensional space.

The system has four independent degrees of freedom.

## 3. Coordinate System

A fixed Cartesian coordinate system is defined:

- X-axis: Horizontal direction.
- Y-axis: Horizontal direction perpendicular to X.
- Z-axis: Vertically upward.

Gravity acts along the negative Z direction.

The cart pivot position is:

r_c = [x, y, 0]^T

The pendulum inclination and azimuth angles describe the orientation of the rod relative to the inertial coordinate frame.

## 4. Generalized Coordinates

The generalized coordinate vector is:

q = [x, y, theta, phi]^T

where:

| Coordinate | Description | Unit |
|---|---|---|
| x | Cart displacement along X | m |
| y | Cart displacement along Y | m |
| theta | Pendulum inclination measured from upward vertical | rad |
| phi | Pendulum azimuth measured from positive X towards positive Y | rad |

The upright equilibrium configuration is defined by:

theta = 0

At this position, the pendulum rod points vertically upward.

The corresponding generalized velocity vector is:

q_dot = [dx, dy, dtheta, dphi]^T

The complete state vector is:

z = [x, y, theta, phi, dx, dy, dtheta, dphi]^T

Therefore, the system has four degrees of freedom and eight first-order state variables.

## 5. Position of the Pendulum Bob

The position of the cart pivot is:

r_c = [x, y, 0]^T

The position of the bob relative to the pivot is:

r_rel = [
    L sin(theta) cos(phi),
    L sin(theta) sin(phi),
    L cos(theta)
]^T

The absolute position of the pendulum bob is:

r_p = r_c + r_rel

Therefore:

x_p = x + L sin(theta) cos(phi)

y_p = y + L sin(theta) sin(phi)

z_p = L cos(theta)

These equations describe the three-dimensional motion of the pendulum bob.

## 6. System Parameters

The initial system parameters are:

| Parameter | Symbol | Value | Unit |
|---|---|---:|---|
| Cart mass | M | 2 | kg |
| Pendulum bob mass | m | 0.5 | kg |
| Rod length | L | 1 | m |
| Gravitational acceleration | g | 9.81 | m/s² |
| X-direction external force | Fx | 0 | N |
| Y-direction external force | Fy | 0 | N |

These parameters will be used for the initial MATLAB simulation.

## 7. Initial Conditions

The pendulum starts slightly displaced from its upright equilibrium.

| Variable | Symbol | Initial value |
|---|---|---:|
| Cart X position | x0 | 0 m |
| Cart Y position | y0 | 0 m |
| Pendulum inclination | theta0 | 5° |
| Pendulum azimuth | phi0 | 30° |
| Cart X velocity | dx0 | 0 m/s |
| Cart Y velocity | dy0 | 0 m/s |
| Angular velocity | dtheta0 | 0 rad/s |
| Azimuthal velocity | dphi0 | 0 rad/s |

The initial state vector is:

z0 = [
    0,
    0,
    5*pi/180,
    30*pi/180,
    0,
    0,
    0,
    0
]^T

## 8. Modelling Assumptions

The following assumptions are considered during mathematical modelling:

1. The cart is a rigid body.
2. The pendulum rod is rigid and massless.
3. The bob is modelled as a point mass.
4. The spherical joint is ideal and frictionless.
5. Gravity is uniform and constant.
6. Cart movement is restricted to the horizontal X-Y plane.
7. The pendulum rod maintains a constant length.
8. Friction and damping are neglected in the initial model.
9. External forces Fx and Fy act on the cart.
10. No controller is applied during the initial open-loop simulation.

## 9. Degrees of Freedom

The system has four independent generalized coordinates:

1. Translation along X.
2. Translation along Y.
3. Pendulum inclination.
4. Pendulum azimuth.

Therefore:

