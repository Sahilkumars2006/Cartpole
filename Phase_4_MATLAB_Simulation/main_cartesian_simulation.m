
%% MAIN CARTESIAN SIMULATION
% Inverted Spherical Pendulum on an X-Y Cart
% Cartesian coordinates with a rigid rod constraint

clear;
clc;
close all;

fprintf('\n============================================\n');
fprintf(' CARTESIAN INVERTED SPHERICAL PENDULUM\n');
fprintf('============================================\n');

%% 1. Project paths

scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);

addpath(scriptDir);

%% 2. Load system parameters

p = system_parameters();

%% 3. Initial angular configuration

theta0 = p.theta0;
phi0 = p.phi0;

%% 4. Convert initial position to Cartesian coordinates

x0 = p.x0;
y0 = p.y0;

X0 = x0 + p.L * sin(theta0) * cos(phi0);
Y0 = y0 + p.L * sin(theta0) * sin(phi0);
Z0 = p.L * cos(theta0);

%% 5. Convert initial velocities

dx0 = p.dx0;
dy0 = p.dy0;

dtheta0 = p.dtheta0;
dphi0 = p.dphi0;

dX0 = dx0 + p.L * ( ...
    cos(theta0) * cos(phi0) * dtheta0 ...
    - sin(theta0) * sin(phi0) * dphi0);

dY0 = dy0 + p.L * ( ...
    cos(theta0) * sin(phi0) * dtheta0 ...
    + sin(theta0) * cos(phi0) * dphi0);

dZ0 = -p.L * sin(theta0) * dtheta0;

%% 6. Construct initial state

q0 = [x0; y0; X0; Y0; Z0];

v0 = [dx0; dy0; dX0; dY0; dZ0];

z0 = [q0; v0];

%% 7. Check initial rod constraint

initial_rod_length = norm([X0-x0; Y0-y0; Z0]);

fprintf('\nInitial rod length: %.12f m\n', initial_rod_length);
fprintf('Specified rod length: %.12f m\n', p.L);

if abs(initial_rod_length - p.L) > 1e-10
    error('Initial rod constraint is not satisfied.');
end

%% 8. Simulation time

timeVector = p.t_start:p.sample_time:p.t_end;

if timeVector(end) < p.t_end
    timeVector = [timeVector, p.t_end];
end

%% 9. Numerical integration

options = odeset( ...
    'RelTol', p.relative_tolerance, ...
    'AbsTol', p.absolute_tolerance, ...
    'MaxStep', p.sample_time);

fprintf('\nStarting Cartesian simulation...\n');

tic;

[time, state] = ode45( ...
    @(t,z) cartesian_equations_of_motion(t,z,p), ...
    timeVector, z0, options);

simulationTime = toc;

fprintf('Integration finished in %.4f seconds.\n', simulationTime);

%% 10. Extract Cartesian states

cart_position = state(:,1:2);
bob_position = state(:,3:5);

cart_velocity = state(:,6:7);
bob_velocity = state(:,8:10);

%% 11. Calculate pendulum angles for analysis

cart_position_3D = [cart_position, ...
    zeros(size(cart_position,1),1)];

relative_position = bob_position - cart_position_3D;

relative_x = relative_position(:,1);
relative_y = relative_position(:,2);
relative_z = relative_position(:,3);

theta = acos(max(-1, min(1, relative_z / p.L)));
phi = atan2(relative_y, relative_x);

%% 12. Calculate kinetic energy

cart_speed_squared = sum(cart_velocity.^2, 2);
bob_speed_squared = sum(bob_velocity.^2, 2);

T_cart = 0.5 * p.M * cart_speed_squared;
T_bob = 0.5 * p.m * bob_speed_squared;

kinetic_energy = T_cart + T_bob;

%% 13. Calculate potential energy

potential_energy = p.m * p.g * bob_position(:,3);

total_energy = kinetic_energy + potential_energy;

%% 14. Calculate energy balance

input_power = ...
    p.Fx * cart_velocity(:,1) ...
    + p.Fy * cart_velocity(:,2);

input_work = cumtrapz(time, input_power);

energy_error = ...
    total_energy - total_energy(1) - input_work;

max_energy_error = max(abs(energy_error));

%% 15. Verify rod length

rod_length = sqrt( ...
    relative_x.^2 + ...
    relative_y.^2 + ...
    relative_z.^2);

rod_length_error = rod_length - p.L;

max_rod_error = max(abs(rod_length_error));

%% 16. Simulation completion check

simulationCompleted = ...
    abs(time(end) - p.t_end) <= 1e-8;

fprintf('\n--- SIMULATION RESULTS ---\n');

fprintf('Requested duration: %.4f s\n', p.t_end);
fprintf('Actual duration:    %.4f s\n', time(end));

fprintf('Number of samples:  %d\n', length(time));

fprintf('Initial inclination: %.4f degrees\n', ...
    rad2deg(theta(1)));

fprintf('Final inclination:   %.4f degrees\n', ...
    rad2deg(theta(end)));

fprintf('\n--- CONSTRAINT VERIFICATION ---\n');

fprintf('Specified rod length: %.12f m\n', p.L);
fprintf('Final rod length:      %.12f m\n', rod_length(end));

fprintf('Maximum rod length error: %.6e m\n', ...
    max_rod_error);

fprintf('\n--- ENERGY VERIFICATION ---\n');

fprintf('Initial energy: %.12f J\n', total_energy(1));
fprintf('Final energy:   %.12f J\n', total_energy(end));

fprintf('Maximum energy balance error: %.6e J\n', ...
    max_energy_error);

%% 17. Store simulation results

results.time = time;
results.states = state;

results.cart_position = cart_position;
results.bob_position = bob_position;

results.cart_velocity = cart_velocity;
results.bob_velocity = bob_velocity;

results.theta = theta;
results.phi = phi;

results.kinetic_energy = kinetic_energy;
results.potential_energy = potential_energy;
results.total_energy = total_energy;

results.energy_error = energy_error;

results.rod_length = rod_length;
results.rod_length_error = rod_length_error;

results.parameters = p;
results.simulation_time = simulationTime;

results.max_energy_error = max_energy_error;
results.max_rod_error = max_rod_error;
results.simulation_completed = simulationCompleted;

%% 18. Save results separately from spherical simulation

resultsDir = fullfile(projectDir, 'Results', 'MATLAB');

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

save(fullfile(resultsDir, ...
    'cartesian_simulation_results.mat'), 'results');

fprintf('\nResults saved successfully.\n');
fprintf('Location:\n%s\n', fullfile(resultsDir, ...
    'cartesian_simulation_results.mat'));

%% 19. Plot cart motion

figure('Name','Cart Motion','Color','w');

plot(time, cart_position(:,1), 'LineWidth', 1.5);
hold on;
plot(time, cart_position(:,2), 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Position (m)');
legend('X position','Y position');
title('X-Y Cart Motion');
grid on;

%% 20. Plot bob position

figure('Name','Bob Position','Color','w');

plot3(bob_position(:,1), ...
      bob_position(:,2), ...
      bob_position(:,3), ...
      'LineWidth', 1.5);

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

title('Bob Trajectory in 3D');
grid on;
axis equal;
view(3);

%% 21. Plot pendulum angles

figure('Name','Pendulum Angles','Color','w');

subplot(2,1,1);

plot(time, rad2deg(theta), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('\theta (degrees)');
title('Pendulum Inclination');
grid on;

subplot(2,1,2);

plot(time, rad2deg(phi), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('\phi (degrees)');
title('Pendulum Azimuth');
grid on;

%% 22. Plot energy

figure('Name','Energy','Color','w');

plot(time, kinetic_energy, 'LineWidth', 1.5);
hold on;

plot(time, potential_energy, 'LineWidth', 1.5);
plot(time, total_energy, 'k--', 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Energy (J)');

legend('Kinetic','Potential','Total');
title('System Energy');
grid on;

%% 23. Plot constraint error

figure('Name','Rod Constraint Error','Color','w');

plot(time, rod_length_error, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Rod length error (m)');
title('Rigid Rod Constraint Error');
grid on;

%% 24. Plot energy balance error

figure('Name','Energy Balance Error','Color','w');

plot(time, energy_error, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Energy error (J)');
title('Energy Conservation Verification');
grid on;

%% 25. Final status

fprintf('\n============================================\n');

if simulationCompleted
    fprintf('SIMULATION: COMPLETED\n');
else
    fprintf('SIMULATION: INCOMPLETE\n');
end

fprintf('Maximum rod error: %.6e m\n', max_rod_error);
fprintf('Maximum energy error: %.6e J\n', max_energy_error);

fprintf('============================================\n');
