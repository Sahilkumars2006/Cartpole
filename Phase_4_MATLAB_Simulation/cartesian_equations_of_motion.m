
function dz = cartesian_equations_of_motion(~, z, p)
% CARTESIAN_EQUATIONS_OF_MOTION
% Dynamics of an inverted spherical pendulum using Cartesian coordinates.
%
% State vector:
% z = [x; y; X; Y; Z; dx; dy; dX; dY; dZ]
%
% x,y : cart position
% X,Y,Z : absolute bob position
%
% The rigid rod constraint is:
% (X-x)^2 + (Y-y)^2 + Z^2 = L^2

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

%% 2. Relative bob position

rx = X - x;
ry = Y - y;
rz = Z;

%% 3. Relative bob velocity

vrx = dX - dx;
vry = dY - dy;
vrz = dZ;

%% 4. Constraint equation

constraint = 0.5 * (rx^2 + ry^2 + rz^2 - p.L^2);

%% 5. Constraint Jacobian

J = [-rx, -ry, rx, ry, rz];

%% 6. Constraint velocity

constraint_velocity = J * v;

%% 7. Mass matrix

A = diag([p.M, p.M, p.m, p.m, p.m]);

%% 8. Generalized external forces

Q = [p.Fx;
    p.Fy;
    0;
    0;
    -p.m * p.g];

%% 9. Baumgarte stabilization

alpha = 10;
beta = 10;

relative_speed_squared = vrx^2 + vry^2 + vrz^2;

constraint_acceleration = ...
    -relative_speed_squared ...
    - 2 * alpha * constraint_velocity ...
    - beta^2 * constraint;

%% 10. Augmented equations

K = [A, -J';
    J,  0];

rhs = [Q;
    constraint_acceleration];

%% 11. Solve for accelerations and Lagrange multiplier

solution = K \ rhs;

qdd = solution(1:5);

%% 12. Return state derivatives

dz = [v;
    qdd];

end
