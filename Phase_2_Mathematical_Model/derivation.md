# Mathematical Derivation
## X-Y Cart with an Inverted Spherical Pendulum

## 1. Introduction

This document presents the mathematical modelling and dynamic equations of an inverted spherical pendulum mounted on a cart capable of translating independently in two horizontal directions.

The system consists of:
- A cart of mass \(M\), translating along the X and Y axes.
- A massless rigid rod of length \(L\).
- A point mass \(m\) attached to the upper end of the rod.
- A spherical joint connecting the pendulum rod to the cart.
- Gravitational acceleration \(g\), acting in the negative Z direction.

The pendulum is configured in an inverted position, with the rod extending upward from the cart. The upright equilibrium is unstable in the absence of control.

The equations of motion are derived using the Lagrangian formulation.

---

## 2. Modelling Assumptions

The following assumptions are considered:

1. The cart is a rigid body.
2. The pendulum rod is rigid and massless.
3. The bob is modelled as a point mass.
4. The spherical joint is ideal and frictionless.
5. Gravitational acceleration is constant.
6. The cart is restricted to translation in the horizontal X-Y plane.
7. Friction and damping are neglected initially.
8. External forces \(F_x\) and \(F_y\) act on the cart.
9. The rod length remains constant throughout the motion.

---

## 3. Generalized Coordinates

The system has four degrees of freedom.

The generalized coordinate vector is:

\[
q=
\begin{bmatrix}
x & y & \theta & \phi
\end{bmatrix}^{T}
\]

where:

- \(x\): Cart displacement along the X-axis.
- \(y\): Cart displacement along the Y-axis.
- \(\theta\): Pendulum inclination measured from the upward vertical.
- \(\phi\): Pendulum azimuth measured from the positive X-axis towards the positive Y-axis.

The generalized velocity vector is:

\[
\dot q=
\begin{bmatrix}
\dot x & \dot y & \dot\theta & \dot\phi
\end{bmatrix}^{T}
\]

The upright equilibrium corresponds to:

\[
\theta=0
\]

At this configuration, the pendulum rod points vertically upward.

The complete first-order state vector is:

\[
z=
\begin{bmatrix}
x&y&\theta&\phi&
\dot x&\dot y&\dot\theta&\dot\phi
\end{bmatrix}^{T}
\]

---

## 4. Position Analysis

The position of the cart pivot in the inertial coordinate frame is:

\[
r_c=
\begin{bmatrix}
x\\y\\0
\end{bmatrix}
\]

Since the pendulum rod extends upward, the position of the bob relative to the pivot is:

\[
r_{rel}=
\begin{bmatrix}
L\sin\theta\cos\phi\\
L\sin\theta\sin\phi\\
L\cos\theta
\end{bmatrix}
\]

Therefore, the absolute position of the bob is:

\[
r_p=r_c+r_{rel}
\]

Hence:

\[
\boxed{
r_p=
\begin{bmatrix}
x+L\sin\theta\cos\phi\\
y+L\sin\theta\sin\phi\\
L\cos\theta
\end{bmatrix}
}
\]

The Cartesian coordinates of the bob are:

\[
x_p=x+L\sin\theta\cos\phi
\]

\[
y_p=y+L\sin\theta\sin\phi
\]

\[
z_p=L\cos\theta
\]

---

## 5. Velocity Analysis

The bob velocity is obtained by differentiating its position vector with respect to time.

\[
v_p=\frac{dr_p}{dt}
\]

Differentiating each component:

### X-direction velocity

\[
\boxed{
v_x=\dot x+
L\cos\theta\cos\phi\dot\theta
-L\sin\theta\sin\phi\dot\phi
}
\]

### Y-direction velocity

\[
\boxed{
v_y=\dot y+
L\cos\theta\sin\phi\dot\theta
+L\sin\theta\cos\phi\dot\phi
}
\]

### Z-direction velocity

\[
\boxed{
v_z=-L\sin\theta\dot\theta
}
\]

Therefore:

\[
v_p=
\begin{bmatrix}
v_x\\v_y\\v_z
\end{bmatrix}
\]

The square of the bob velocity is:

\[
v_p^2=v_x^2+v_y^2+v_z^2
\]

After simplification:

\[
\begin{aligned}
v_p^2={}&
\dot x^2+\dot y^2\\
&+2L\cos\theta
(\dot x\cos\phi+\dot y\sin\phi)\dot\theta\\
&+2L\sin\theta
(-\dot x\sin\phi+\dot y\cos\phi)\dot\phi\\
&+L^2\dot\theta^2
+L^2\sin^2\theta\dot\phi^2
\end{aligned}
\]

---

## 6. Kinetic Energy

The total kinetic energy is the sum of the cart and bob kinetic energies.

\[
T=T_c+T_p
\]

The cart kinetic energy is:

\[
T_c=\frac12M(\dot x^2+\dot y^2)
\]

The bob kinetic energy is:

\[
T_p=\frac12m(v_x^2+v_y^2+v_z^2)
\]

Substituting the velocity expressions and simplifying:

\[
\boxed{
\begin{aligned}
T={}&
\frac12(M+m)(\dot x^2+\dot y^2)\\
&+mL\cos\theta
(\dot x\cos\phi+\dot y\sin\phi)\dot\theta\\
&+mL\sin\theta
(-\dot x\sin\phi+\dot y\cos\phi)\dot\phi\\
&+\frac12mL^2
(\dot\theta^2+\sin^2\theta\dot\phi^2)
\end{aligned}
}
\]

This expression represents the total kinetic energy of the X-Y cart and the inverted spherical pendulum.

---

## 7. Potential Energy

The gravitational potential energy is defined relative to the cart pivot height.

\[
V=mgz_p
\]

Since:

\[
z_p=L\cos\theta
\]

the potential energy becomes:

\[
\boxed{V=mgL\cos\theta}
\]

At the upright position:

\[
\theta=0
\]

Therefore:

\[
V=mgL
\]

This represents the maximum gravitational potential energy.

As the pendulum moves away from the upright position, its potential energy decreases. Consequently, the upright equilibrium is unstable without control.

---

## 8. Lagrangian Formulation

The Lagrangian is defined as:

\[
\mathcal L=T-V
\]

Substituting the kinetic and potential energy expressions:

\[
\boxed{
\begin{aligned}
\mathcal L={}&
\frac12(M+m)(\dot x^2+\dot y^2)\\
&+mL\cos\theta
(\dot x\cos\phi+\dot y\sin\phi)\dot\theta\\
&+mL\sin\theta
(-\dot x\sin\phi+\dot y\cos\phi)\dot\phi\\
&+\frac12mL^2
(\dot\theta^2+\sin^2\theta\dot\phi^2)\\
&-mgL\cos\theta
\end{aligned}
}
\]

---

## 9. Euler-Lagrange Equations

The equations of motion are derived using the Euler-Lagrange equation:

\[
\boxed{
\frac{d}{dt}
\left(
\frac{\partial\mathcal L}{\partial\dot q_i}
\right)
-
\frac{\partial\mathcal L}{\partial q_i}
=Q_i
}
\]

where \(q_i\) represents each generalized coordinate and \(Q_i\) is its corresponding generalized force.

The generalized force vector is:

\[
Q=
\begin{bmatrix}
F_x\\
F_y\\
0\\
0
\end{bmatrix}
\]

The pendulum coordinates have no directly applied generalized forces.

Applying the Euler-Lagrange equation to all four coordinates and collecting acceleration terms gives:

\[
\boxed{M(q)\ddot q=R(q,\dot q)}
\]

---

## 10. Mass Matrix

The mass matrix is:

\[
\boxed{
M(q)=
\begin{bmatrix}
M+m & 0 & mL\cos\theta\cos\phi & -mL\sin\theta\sin\phi\\
0 & M+m & mL\cos\theta\sin\phi & mL\sin\theta\cos\phi\\
mL\cos\theta\cos\phi & mL\cos\theta\sin\phi & mL^2 & 0\\
-mL\sin\theta\sin\phi & mL\sin\theta\cos\phi & 0 & mL^2\sin^2\theta
\end{bmatrix}
}
\]

The matrix is symmetric:

\[
M(q)=M(q)^T
\]

---

## 11. Nonlinear Right-Hand-Side Vector

The nonlinear right-hand-side vector is:

\[
R=
\begin{bmatrix}
R_1\\
R_2\\
R_3\\
R_4
\end{bmatrix}
\]

where:

### First equation

\[
\boxed{
\begin{aligned}
R_1={}&F_x+
mL\sin\theta\cos\phi
(\dot\theta^2+\dot\phi^2)\\
&+2mL\cos\theta\sin\phi
\dot\theta\dot\phi
\end{aligned}
}
\]

### Second equation

\[
\boxed{
\begin{aligned}
R_2={}&F_y+
mL\sin\theta\sin\phi
(\dot\theta^2+\dot\phi^2)\\
&-2mL\cos\theta\cos\phi
\dot\theta\dot\phi
\end{aligned}
}
\]

### Third equation

\[
\boxed{
R_3=
mL^2\sin\theta\cos\theta\dot\phi^2
+mgL\sin\theta
}
\]

### Fourth equation

\[
\boxed{
R_4=
-2mL^2\sin\theta\cos\theta
\dot\theta\dot\phi
}
\]

The positive gravitational term in \(R_3\) is essential for the inverted configuration.

---

## 12. Final Equations of Motion

The complete nonlinear equations of motion are:

\[
\boxed{
\begin{bmatrix}
M+m & 0 & mL c_\theta c_\phi & -mL s_\theta s_\phi\\
0 & M+m & mL c_\theta s_\phi & mL s_\theta c_\phi\\
mL c_\theta c_\phi & mL c_\theta s_\phi & mL^2 & 0\\
-mL s_\theta s_\phi & mL s_\theta c_\phi & 0 & mL^2s_\theta^2
\end{bmatrix}
\begin{bmatrix}
\ddot x\\
\ddot y\\
\ddot\theta\\
\ddot\phi
\end{bmatrix}
=
\begin{bmatrix}
R_1\\
R_2\\
R_3\\
R_4
\end{bmatrix}
}
\]

where \(c_\theta=\cos\theta\), \(s_\theta=\sin\theta\), \(c_\phi=\cos\phi\), and \(s_\phi=\sin\phi\).

The accelerations are calculated by solving:

\[
\boxed{
\ddot q=M(q)^{-1}R(q,\dot q)
}
\]

In MATLAB, the numerically preferred implementation is:

```matlab
qdd = Mq \ R;
```

rather than explicitly calculating the inverse.

---

## 13. Mechanical Energy

The total mechanical energy is the sum of kinetic and potential energy:

\[
E=T+V
\]

Therefore:

\[
\boxed{
E=T+mgL\cos\theta
}
\]

For an unforced and undamped system:

\[
F_x=0,\qquad F_y=0
\]

The mechanical energy must remain constant:

\[
\boxed{\frac{dE}{dt}=0}
\]

For nonzero external forces, the energy balance is:

\[
\boxed{
\frac{dE}{dt}=F_x\dot x+F_y\dot y
}
\]

This relationship will be used to verify the numerical simulation.

---

## 14. Coordinate Singularity

The determinant of the mass matrix is:

\[
\boxed{
\det(M(q))=
M(M+m)^2m^3L^4\sin^2\theta
}
\]

The matrix becomes singular when:

\[
\sin\theta=0
\]

which occurs at:

\[
\theta=0,\pi
\]

At these configurations, the pendulum is vertically aligned and the azimuth angle \(\phi\) is undefined.

This is a coordinate singularity associated with spherical coordinates. It does not represent a physical loss of a degree of freedom.

For the initial numerical simulation, a small nonzero inclination is used to avoid the singularity.

For precise control near the upright equilibrium, a nonsingular coordinate representation will be considered in the later control-design phase.

---

## 15. Initial Conditions and Simulation Parameters

The initial parameters are:

| Parameter | Symbol | Value |
|---|---|---:|
| Cart mass | \(M\) | 2 kg |
| Bob mass | \(m\) | 0.5 kg |
| Rod length | \(L\) | 1 m |
| Gravity | \(g\) | 9.81 m/s² |
| X-direction force | \(F_x\) | 0 N |
| Y-direction force | \(F_y\) | 0 N |
| Initial X position | \(x_0\) | 0 m |
| Initial Y position | \(y_0\) | 0 m |
| Initial inclination | \(\theta_0\) | 5 degrees |
| Initial azimuth | \(\phi_0\) | 30 degrees |
| Initial X velocity | \(\dot x_0\) | 0 m/s |
| Initial Y velocity | \(\dot y_0\) | 0 m/s |
| Initial angular velocity | \(\dot\theta_0\) | 0 rad/s |
| Initial azimuthal velocity | \(\dot\phi_0\) | 0 rad/s |

The initial state vector is:

\[
\boxed{
z_0=
\begin{bmatrix}
0\\
0\\
5\pi/180\\
30\pi/180\\
0\\
0\\
0\\
0
\end{bmatrix}
}
\]

The simulation will initially run for 15 seconds using MATLAB's `ode45` solver.

---

## 16. Conclusion

The nonlinear mathematical model of an inverted spherical pendulum mounted on an X-Y cart has been derived using the Lagrangian formulation.

The model includes:
- Four generalized coordinates.
- Eight first-order state variables.
- Nonlinear coupling between cart translation and pendulum motion.
- Gravitational effects associated with the inverted configuration.
- A symmetric mass matrix.
- A mechanical energy relationship for numerical verification.

The resulting equations provide the mathematical foundation for MATLAB simulation, 3D visualization, Simulink implementation, Simscape Multibody modelling, and subsequent controller design.