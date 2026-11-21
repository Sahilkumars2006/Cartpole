
%% PHASE 10.5: BASELINE CONTROLLED MATLAB REPRODUCTION
%
% Reproduce the nominal controlled MATLAB simulation.
% Uses the existing fixed LQR gain.
% Does not modify the original controller or Simscape model.

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 10.5: BASELINE CONTROL REPRODUCTION\n');
fprintf('============================================\n');

%% 1. PROJECT PATHS

scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);

phase4Dir = fullfile(projectDir, ...
    'Phase_4_MATLAB_Simulation');

addpath(phase4Dir);

%% 2. LOAD PARAMETERS AND CONTROLLER

p = system_parameters();

controllerFile = fullfile(scriptDir, ...
    'controller_data.mat');

if ~isfile(controllerFile)
    error('Controller data file not found.');
end

loaded = load(controllerFile);

controller = loaded.controller;

K = controller.K;

p.maximum_force = controller.maximum_force;

fprintf('\n--- NOMINAL PARAMETERS ---\n');

fprintf('Cart mass: %.4f kg\n',p.M);
fprintf('Bob mass: %.4f kg\n',p.m);
fprintf('Rod length: %.4f m\n',p.L);
fprintf('Gravity: %.4f m/s^2\n',p.g);
fprintf('Maximum force: %.4f N\n',p.maximum_force);

%% 3. INITIAL CONDITIONS

theta0 = p.theta0;
phi0 = p.phi0;

x0 = p.x0;
y0 = p.y0;

X0 = x0 + p.L*sin(theta0)*cos(phi0);
Y0 = y0 + p.L*sin(theta0)*sin(phi0);
Z0 = p.L*cos(theta0);

dx0 = p.dx0;
dy0 = p.dy0;

dX0 = 0;
dY0 = 0;
dZ0 = 0;

z0 = [
    x0;
    y0;
    X0;
    Y0;
    Z0;
    dx0;
    dy0;
    dX0;
    dY0;
    dZ0
];

fprintf('\nInitial inclination: %.4f deg\n', ...
    rad2deg(theta0));

fprintf('Initial azimuth: %.4f deg\n', ...
    rad2deg(phi0));

%% 4. SIMULATION

fprintf('\nRunning controlled nonlinear simulation...\n');

time = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol',p.relative_tolerance, ...
    'AbsTol',p.absolute_tolerance);

[t,z] = ode45( ...
    @(t,z) controlled_cartesian_equations_of_motion( ...
    t,z,p,K), ...
    time,z0,options);

fprintf('Simulation completed.\n');
fprintf('Samples: %d\n',numel(t));

%% 5. RECONSTRUCT CONTROL FORCES

controlForce = zeros(numel(t),2);

for k = 1:numel(t)

    [~,u] = controlled_cartesian_equations_of_motion( ...
        t(k),z(k,:)',p,K);

    controlForce(k,:) = u';

end

Fx = controlForce(:,1);
Fy = controlForce(:,2);

%% 6. CART AND BOB POSITIONS

cartPosition = z(:,1:2);
bobPosition = z(:,3:5);

relativePosition = [
    bobPosition(:,1)-cartPosition(:,1), ...
    bobPosition(:,2)-cartPosition(:,2), ...
    bobPosition(:,3)
];

rodLength = sqrt(sum(relativePosition.^2,2));

inclination = acosd(max(-1,min(1, ...
    relativePosition(:,3)./rodLength)));

radialCartDisplacement = sqrt(sum(cartPosition.^2,2));

cartSpeed = sqrt(z(:,6).^2+z(:,7).^2);

%% 7. PERFORMANCE METRICS

finalInclination = inclination(end);

maximumInclination = max(inclination);

maximumCartDisplacement = max(radialCartDisplacement);

maximumCartSpeed = max(cartSpeed);

maximumFx = max(abs(Fx));
maximumFy = max(abs(Fy));

maximumRodError = max(abs(rodLength-p.L));

finalCartPosition = cartPosition(end,:);

finalBobRelativeError = norm( ...
    relativePosition(end,:) - [0 0 p.L]);

fprintf('\n============================================\n');
fprintf('BASELINE CONTROL PERFORMANCE\n');
fprintf('============================================\n');

fprintf('Final inclination: %.10f deg\n',finalInclination);

fprintf('Maximum inclination: %.10f deg\n', ...
    maximumInclination);

fprintf('Maximum cart displacement: %.10f m\n', ...
    maximumCartDisplacement);

fprintf('Maximum cart speed: %.10f m/s\n', ...
    maximumCartSpeed);

fprintf('Maximum |Fx|: %.10f N\n',maximumFx);

fprintf('Maximum |Fy|: %.10f N\n',maximumFy);

fprintf('Maximum rod length error: %.12e m\n', ...
    maximumRodError);

fprintf('Final cart X: %.10e m\n',finalCartPosition(1));

fprintf('Final cart Y: %.10e m\n',finalCartPosition(2));

fprintf('Final bob relative position error: %.10e m\n', ...
    finalBobRelativeError);

%% 8. PLOTS

figure('Name','Phase 10.5 Inclination');

plot(t,inclination,'LineWidth',1.5);

xlabel('Time (s)');
ylabel('Inclination (deg)');
title('Controlled Pendulum Inclination');
grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_5_Baseline_Inclination.png'));

figure('Name','Phase 10.5 Cart Trajectory');

plot(cartPosition(:,1),cartPosition(:,2), ...
    'LineWidth',1.5);

xlabel('Cart X (m)');
ylabel('Cart Y (m)');
title('Controlled Cart X-Y Trajectory');
axis equal;
grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_5_Baseline_Cart_Trajectory.png'));

figure('Name','Phase 10.5 Control Forces');

plot(t,Fx,'LineWidth',1.3);
hold on;
plot(t,Fy,'LineWidth',1.3);

xlabel('Time (s)');
ylabel('Control force (N)');
title('LQR Control Forces');

legend('F_x','F_y');
grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_5_Baseline_Control_Forces.png'));

%% 9. SAVE RESULTS

baseline = struct();

baseline.time = t;
baseline.states = z;

baseline.cart_position = cartPosition;
baseline.bob_position = bobPosition;
baseline.relative_position = relativePosition;

baseline.control_force = controlForce;

baseline.inclination = inclination;
baseline.rod_length = rodLength;

baseline.final_inclination = finalInclination;
baseline.maximum_inclination = maximumInclination;

baseline.maximum_cart_displacement = ...
    maximumCartDisplacement;

baseline.maximum_cart_speed = maximumCartSpeed;

baseline.maximum_Fx = maximumFx;
baseline.maximum_Fy = maximumFy;

baseline.maximum_rod_error = maximumRodError;

baseline.final_cart_position = finalCartPosition;

baseline.final_bob_relative_error = ...
    finalBobRelativeError;

baseline.parameters = p;
baseline.controller_gain = K;

outputFile = fullfile(scriptDir, ...
    'phase_10_5_baseline_results.mat');

save(outputFile,'baseline');

fprintf('\nResults saved:\n%s\n',outputFile);

fprintf('\n============================================\n');
fprintf('PHASE 10.5 COMPLETED\n');
fprintf('============================================\n');
