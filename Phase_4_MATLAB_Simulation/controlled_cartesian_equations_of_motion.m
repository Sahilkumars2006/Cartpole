
function [dz, controlForce] = ...
    controlled_cartesian_equations_of_motion(~, z, p, K)

% CONTROLLED CARTESIAN EQUATIONS OF MOTION
% Nonlinear constrained dynamics with two-axis LQR feedback.
%
% Plant state:
% z = [x; y; X; Y; Z; dx; dy; dX; dY; dZ]
%
% Controller state:
% s = [x; y; xi; eta; dx; dy; dxi; deta]

%% 1. Extract position and velocity

q = z(1:5);
v = z(6:10);

x = q(1);
y = q(2);

X = q(3);
Y = q(4);
Z = q(5);

dx = v(1);
dy = v(2);

dX = v(3);
dY = v(4);
dZ = v(5);

%% 2. Relative bob coordinates

xi = X - x;
eta = Y - y;

dxi = dX - dx;
deta = dY - dy;

%% 3. Construct controller state

s = [x;
     y;
     xi;
     eta;
     dx;
     dy;
     dxi;
     deta];

%% 4. Calculate LQR control forces

controlForce = -K*s;

% Apply actuator force limits

maximumForce = p.maximum_force;

controlForce = max(-maximumForce, ...
    min(maximumForce, controlForce));

Fx = controlForce(1);
Fy = controlForce(2);

%% 5. Relative position and velocity

rx = X-x;
ry = Y-y;
rz = Z;

vrx = dX-dx;
vry = dY-dy;
vrz = dZ;

%% 6. Constraint Jacobian

J = [-rx, -ry, rx, ry, rz];

constraint = 0.5 * ...
    (rx^2 + ry^2 + rz^2 - p.L^2);

constraintVelocity = J*v;

%% 7. Mass matrix

A = diag([p.M, p.M, p.m, p.m, p.m]);

%% 8. External generalized forces

Q = [Fx;
     Fy;
     0;
     0;
    -p.m*p.g];

%% 9. Baumgarte constraint stabilization

alpha = 10;
beta = 10;

relativeSpeedSquared = ...
    vrx^2 + vry^2 + vrz^2;

constraintAcceleration = ...
    -relativeSpeedSquared ...
    -2*alpha*constraintVelocity ...
    -beta^2*constraint;

%% 10. Solve constrained equations

KKT = [A, -J';
       J,  0];

rhs = [Q;
       constraintAcceleration];

solution = KKT\rhs;

qdd = solution(1:5);

%% 11. Return derivatives

dz = [v;
      qdd];

end
