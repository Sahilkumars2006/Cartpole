
function dz = equations_of_motion(~, z, p)
% EQUATIONS_OF_MOTION
% Dynamics of a 3D X-Y Cart with an Inverted Spherical Pendulum
%
% State vector:
% z = [x; y; theta; phi; dx; dy; dtheta; dphi]
%
% Generalized coordinates:
% q = [x; y; theta; phi]
%
% Equation:
% M(q)*qdd = R(q,qd)

%% 1. Extract state variables

theta = z(3);
phi   = z(4);

dx    = z(5);
dy    = z(6);
dtheta = z(7);
dphi   = z(8);

%% 2. Extract physical parameters

M = p.M;
m = p.m;
L = p.L;
g = p.g;

Fx = p.Fx;
Fy = p.Fy;

%% 3. Trigonometric terms

st = sin(theta);
ct = cos(theta);

sp = sin(phi);
cp = cos(phi);

%% 4. Construct mass matrix

MassMatrix = [
    M+m,       0,       m*L*ct*cp,  -m*L*st*sp;
    0,         M+m,     m*L*ct*sp,   m*L*st*cp;
    m*L*ct*cp, m*L*ct*sp, m*L^2,     0;
    -m*L*st*sp, m*L*st*cp, 0,         m*L^2*st^2
    ];

%% 5. Construct nonlinear RHS

R = [
    Fx + m*L*st*cp*(dtheta^2+dphi^2) ...
    + 2*m*L*ct*sp*dtheta*dphi;

    Fy + m*L*st*sp*(dtheta^2+dphi^2) ...
    - 2*m*L*ct*cp*dtheta*dphi;

    m*L^2*st*ct*dphi^2 ...
    + m*g*L*st;

    -2*m*L^2*st*ct*dtheta*dphi
    ];

%% 6. Solve for generalized accelerations

qdd = MassMatrix \ R;

%% 7. Construct first-order state derivatives

dz = [
    dx;
    dy;
    dtheta;
    dphi;
    qdd
    ];

end
