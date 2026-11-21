
%% MAIN MATLAB SIMULATION
% 3D X-Y Cart with an Inverted Spherical Pendulum
%
% Numerical simulation using ode45.

clear;
clc;
close all;

fprintf('\n============================================\n');
fprintf(' INVERTED SPHERICAL PENDULUM SIMULATION\n');
fprintf('============================================\n\n');

%% 1. PROJECT PATHS

scriptDir = fileparts(mfilename('fullpath'));

projectDir = fileparts(scriptDir);

addpath(scriptDir);

%% 2. LOAD SYSTEM PARAMETERS

p = system_parameters();

fprintf('System parameters loaded.\n');

%% 3. INITIAL STATE VECTOR

z0 = [
    p.x0;
    p.y0;
    p.theta0;
    p.phi0;
    p.dx0;
    p.dy0;
    p.dtheta0;
    p.dphi0
];

fprintf('\nInitial state:\n');
disp(z0);

%% 4. SIMULATION TIME

time_vector = ...
    (p.t_start:p.sample_time:p.t_end).';

if time_vector(end) < p.t_end
    time_vector = [time_vector; p.t_end];
end

%% 5. ODE45 SOLVER OPTIONS

options = odeset( ...
    'RelTol', p.relative_tolerance, ...
    'AbsTol', p.absolute_tolerance, ...
    'MaxStep', p.sample_time);

%% 6. SOLVE EQUATIONS OF MOTION

fprintf('\nStarting ode45 simulation...\n');

tic;

[time, states] = ode45( ...
    @(t,z) equations_of_motion(t,z,p), ...
    time_vector, ...
    z0, ...
    options);

simulation_time = toc;

fprintf('Simulation completed.\n');
fprintf('Computation time: %.4f seconds\n', simulation_time);

%% 7. EXTRACT STATE VARIABLES

x = states(:,1);
y = states(:,2);
theta = states(:,3);
phi = states(:,4);

dx = states(:,5);
dy = states(:,6);
dtheta = states(:,7);
dphi = states(:,8);

%% 8. CALCULATE BOB POSITION

xp = x + p.L.*sin(theta).*cos(phi);

yp = y + p.L.*sin(theta).*sin(phi);

zp = p.L.*cos(theta);

bob_position = [xp, yp, zp];

fprintf('\nBob position calculated.\n');

%% 9. CALCULATE BOB VELOCITY

vpx = dx ...
    + p.L.*cos(theta).*cos(phi).*dtheta ...
    - p.L.*sin(theta).*sin(phi).*dphi;

vpy = dy ...
    + p.L.*cos(theta).*sin(phi).*dtheta ...
    + p.L.*sin(theta).*cos(phi).*dphi;

vpz = -p.L.*sin(theta).*dtheta;

%% 10. CALCULATE KINETIC ENERGY

T_cart = 0.5*p.M.*(dx.^2 + dy.^2);

T_bob = 0.5*p.m.*(vpx.^2 + vpy.^2 + vpz.^2);

kinetic_energy = T_cart + T_bob;

%% 11. CALCULATE POTENTIAL ENERGY

potential_energy = p.m*p.g*p.L.*cos(theta);

%% 12. TOTAL MECHANICAL ENERGY

total_energy = kinetic_energy + potential_energy;

%% 13. ENERGY BALANCE VERIFICATION

% Expected energy change:
% E(t)-E(0) = integral(Fx*dx + Fy*dy) dt

input_power = p.Fx.*dx + p.Fy.*dy;

work_done = cumtrapz(time, input_power);

energy_error = ...
    total_energy - total_energy(1) - work_done;

max_energy_error = max(abs(energy_error));

fprintf('\n--- ENERGY VERIFICATION ---\n');

fprintf('Initial energy: %.12f J\n', total_energy(1));

fprintf('Final energy:   %.12f J\n', total_energy(end));

fprintf('Maximum energy balance error: %.6e J\n', ...
    max_energy_error);

%% 14. SIMULATION INFORMATION

fprintf('\n--- SIMULATION INFORMATION ---\n');

fprintf('Number of time points: %d\n', length(time));

fprintf('Simulation duration: %.2f seconds\n', ...
    time(end)-time(1));

fprintf('Initial inclination: %.2f degrees\n', ...
    rad2deg(theta(1)));

fprintf('Final inclination: %.2f degrees\n', ...
    rad2deg(theta(end)));

%% 15. CREATE RESULTS STRUCTURE

results = struct();

results.time = time;

results.states = states;

results.bob_position = bob_position;

results.kinetic_energy = kinetic_energy;

results.potential_energy = potential_energy;

results.total_energy = total_energy;

results.energy_error = energy_error;

results.parameters = p;

results.simulation_time = simulation_time;

results.max_energy_error = max_energy_error;

%% 16. SAVE SIMULATION RESULTS

resultsDir = fullfile(projectDir, 'Results', 'MATLAB');

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

save(fullfile(resultsDir, 'simulation_results.mat'), ...
    'results');

fprintf('\nSimulation results saved successfully.\n');

fprintf('File location:\n%s\n', ...
    fullfile(resultsDir, 'simulation_results.mat'));

%% 17. PLOT CART POSITION

figure('Name','Cart Position','Color','w');

subplot(2,1,1);

plot(time, x, 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('X position (m)');
title('Cart X Position');

subplot(2,1,2);

plot(time, y, 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Y position (m)');
title('Cart Y Position');

%% 18. PLOT PENDULUM ANGLES

figure('Name','Pendulum Angles','Color','w');

subplot(2,1,1);

plot(time, rad2deg(theta), 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('\theta (degrees)');
title('Pendulum Inclination');

subplot(2,1,2);

plot(time, rad2deg(phi), 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('\phi (degrees)');
title('Pendulum Azimuth');

%% 19. PLOT ENERGY

figure('Name','Mechanical Energy','Color','w');

plot(time, kinetic_energy, 'LineWidth', 1.5);
hold on;

plot(time, potential_energy, 'LineWidth', 1.5);

plot(time, total_energy, '--', 'LineWidth', 1.5);

grid on;

xlabel('Time (s)');
ylabel('Energy (J)');

title('Mechanical Energy');

legend( ...
    'Kinetic Energy', ...
    'Potential Energy', ...
    'Total Energy', ...
    'Location','best');

%% 20. PLOT ENERGY BALANCE ERROR

figure('Name','Energy Balance Error','Color','w');

plot(time, energy_error, 'LineWidth', 1.5);

grid on;

xlabel('Time (s)');
ylabel('Energy error (J)');

title('Mechanical Energy Balance Error');

%% 21. FINAL STATUS

fprintf('\n============================================\n');
fprintf('       SIMULATION SUMMARY\n');
fprintf('============================================\n');

fprintf('Simulation: COMPLETED\n');

fprintf('Number of states: %d\n', size(states,2));

fprintf('Number of samples: %d\n', size(states,1));

fprintf('Maximum energy error: %.6e J\n', ...
    max_energy_error);

fprintf('Results saved: YES\n');

fprintf('============================================\n');
