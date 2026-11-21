
function p = system_parameters()
% SYSTEM_PARAMETERS
% Parameters for a 3D X-Y Cart with an Inverted Spherical Pendulum

%% Physical parameters

p.M = 2.0;          % Cart mass (kg)
p.m = 0.5;          % Pendulum bob mass (kg)
p.L = 1.0;          % Pendulum rod length (m)
p.g = 9.81;         % Gravitational acceleration (m/s^2)

%% External forces

p.Fx = 0.0;         % Force along X (N)
p.Fy = 0.0;         % Force along Y (N)

%% Initial positions

p.x0 = 0.0;         
p.y0 = 0.0;

% Small initial inclination from upward vertical
p.theta0 = deg2rad(5);

% Initial azimuth
p.phi0 = deg2rad(30);

%% Initial velocities

p.dx0 = 0.0;
p.dy0 = 0.0;
p.dtheta0 = 0.0;
p.dphi0 = 0.0;

%% Simulation settings

p.t_start = 0.0;
p.t_end = 15.0;
p.sample_time = 0.01;

%% Numerical solver settings

p.relative_tolerance = 1e-9;
p.absolute_tolerance = 1e-11;

end
