
%% CONTROLLED INVERTED SPHERICAL PENDULUM
% Nonlinear Cartesian simulation using two-axis LQR control

clear;
clc;
close all;

fprintf('\n============================================\n');
fprintf(' CONTROLLED INVERTED SPHERICAL PENDULUM\n');
fprintf('============================================\n');

%% 1. Locate project

scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);

phase10Dir = fullfile(projectDir,'Phase_10_Control');

addpath(scriptDir);

%% 2. Load physical parameters and controller

p = system_parameters();

controllerFile = fullfile(phase10Dir,'controller_data.mat');

if ~isfile(controllerFile)
    error('Controller data not found. Run control_design.m first.');
end

loadedController = load(controllerFile,'controller');

controller = loadedController.controller;
K = controller.K;

p.maximum_force = controller.maximum_force;

%% 3. Initial conditions

theta0 = p.theta0;
phi0 = p.phi0;

x0 = p.x0;
y0 = p.y0;

X0 = x0 + p.L*sin(theta0)*cos(phi0);
Y0 = y0 + p.L*sin(theta0)*sin(phi0);
Z0 = p.L*cos(theta0);

dx0 = p.dx0;
dy0 = p.dy0;

dtheta0 = p.dtheta0;
dphi0 = p.dphi0;

dX0 = dx0 + p.L*( ...
    cos(theta0)*cos(phi0)*dtheta0 ...
    - sin(theta0)*sin(phi0)*dphi0);

dY0 = dy0 + p.L*( ...
    cos(theta0)*sin(phi0)*dtheta0 ...
    + sin(theta0)*cos(phi0)*dphi0);

dZ0 = -p.L*sin(theta0)*dtheta0;

z0 = [x0; y0; X0; Y0; Z0; ...
      dx0; dy0; dX0; dY0; dZ0];

%% 4. Simulation settings

timeVector = p.t_start:p.sample_time:p.t_end;

if timeVector(end) < p.t_end
    timeVector = [timeVector, p.t_end];
end

options = odeset( ...
    'RelTol',p.relative_tolerance, ...
    'AbsTol',p.absolute_tolerance, ...
    'MaxStep',p.sample_time);

%% 5. Run controlled simulation

fprintf('\nStarting controlled simulation...\n');

tic;

[time,state] = ode45( ...
    @(t,z) controlled_cartesian_equations_of_motion( ...
    t,z,p,K), ...
    timeVector,z0,options);

simulationTime = toc;

fprintf('Integration time: %.4f seconds\n',simulationTime);

%% 6. Extract states

cartPosition = state(:,1:2);
bobPosition = state(:,3:5);

cartVelocity = state(:,6:7);
bobVelocity = state(:,8:10);

relativePosition = bobPosition - ...
    [cartPosition,zeros(length(time),1)];

relativeVelocity = bobVelocity - ...
    [cartVelocity,zeros(length(time),1)];

xi = relativePosition(:,1);
eta = relativePosition(:,2);

%% 7. Calculate pendulum angles

theta = acos(max(-1,min(1, ...
    relativePosition(:,3)/p.L)));

phi = atan2(eta,xi);

%% 8. Calculate control forces at output times

controlForce = zeros(length(time),2);

for k = 1:length(time)

    [~,controlForce(k,:)] = ...
        controlled_cartesian_equations_of_motion( ...
        time(k),state(k,:)',p,K);

end

Fx = controlForce(:,1);
Fy = controlForce(:,2);

%% 9. Energy calculations

cartSpeedSquared = sum(cartVelocity.^2,2);
bobSpeedSquared = sum(bobVelocity.^2,2);

kineticEnergy = ...
    0.5*p.M*cartSpeedSquared + ...
    0.5*p.m*bobSpeedSquared;

potentialEnergy = p.m*p.g*bobPosition(:,3);

totalEnergy = kineticEnergy + potentialEnergy;

inputPower = ...
    Fx.*cartVelocity(:,1) + ...
    Fy.*cartVelocity(:,2);

inputWork = cumtrapz(time,inputPower);

energyError = ...
    totalEnergy-totalEnergy(1)-inputWork;

maxEnergyError = max(abs(energyError));

%% 10. Rod constraint verification

rodLength = sqrt(sum(relativePosition.^2,2));

rodLengthError = rodLength-p.L;

maxRodError = max(abs(rodLengthError));

%% 11. Simulation completion

simulationCompleted = ...
    abs(time(end)-p.t_end) <= 1e-8;

%% 12. Print results

fprintf('\n--- CONTROLLED SIMULATION RESULTS ---\n');

fprintf('Requested duration: %.4f s\n',p.t_end);
fprintf('Actual duration:    %.4f s\n',time(end));

fprintf('Initial inclination: %.4f degrees\n', ...
    rad2deg(theta(1)));

fprintf('Final inclination:   %.4f degrees\n', ...
    rad2deg(theta(end)));

fprintf('Final X displacement: %.6f m\n',xi(end));
fprintf('Final Y displacement: %.6f m\n',eta(end));

fprintf('\n--- CONTROL FORCE ---\n');

fprintf('Maximum |Fx|: %.4f N\n',max(abs(Fx)));
fprintf('Maximum |Fy|: %.4f N\n',max(abs(Fy)));

fprintf('\n--- CONSTRAINT VERIFICATION ---\n');

fprintf('Maximum rod length error: %.6e m\n',maxRodError);

fprintf('\n--- ENERGY VERIFICATION ---\n');

fprintf('Initial energy: %.12f J\n',totalEnergy(1));
fprintf('Final energy:   %.12f J\n',totalEnergy(end));

fprintf('Maximum energy balance error: %.6e J\n', ...
    maxEnergyError);

%% 13. Save results

results.time = time;
results.states = state;

results.cart_position = cartPosition;
results.bob_position = bobPosition;

results.cart_velocity = cartVelocity;
results.bob_velocity = bobVelocity;

results.relative_position = relativePosition;
results.relative_velocity = relativeVelocity;

results.theta = theta;
results.phi = phi;

results.control_force = controlForce;

results.kinetic_energy = kineticEnergy;
results.potential_energy = potentialEnergy;
results.total_energy = totalEnergy;

results.energy_error = energyError;

results.rod_length = rodLength;
results.rod_length_error = rodLengthError;

results.max_energy_error = maxEnergyError;
results.max_rod_error = maxRodError;

results.parameters = p;
results.controller = controller;
results.simulation_time = simulationTime;
results.simulation_completed = simulationCompleted;

resultsDir = fullfile(projectDir,'Results','MATLAB');

if ~exist(resultsDir,'dir')
    mkdir(resultsDir);
end

save(fullfile(resultsDir, ...
    'controlled_simulation_results.mat'),'results');

fprintf('\nControlled results saved successfully.\n');

%% 14. Plot inclination

figure('Name','Controlled Pendulum Angles','Color','w');

subplot(2,1,1);

plot(time,rad2deg(theta),'LineWidth',1.5);
hold on;
yline(0,'k--');

xlabel('Time (s)');
ylabel('\theta (degrees)');
title('Pendulum Inclination');
grid on;

subplot(2,1,2);

plot(time,rad2deg(phi),'LineWidth',1.5);

xlabel('Time (s)');
ylabel('\phi (degrees)');
title('Pendulum Azimuth');
grid on;

%% 15. Plot cart movement

figure('Name','Controlled Cart Motion','Color','w');

subplot(2,1,1);

plot(time,cartPosition(:,1),'LineWidth',1.5);
xlabel('Time (s)');
ylabel('X (m)');
title('Cart X Position');
grid on;

subplot(2,1,2);

plot(time,cartPosition(:,2),'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Y (m)');
title('Cart Y Position');
grid on;

%% 16. Plot control forces

figure('Name','LQR Control Forces','Color','w');

subplot(2,1,1);

plot(time,Fx,'LineWidth',1.5);
xlabel('Time (s)');
ylabel('F_x (N)');
title('X-Axis Control Force');
grid on;

subplot(2,1,2);

plot(time,Fy,'LineWidth',1.5);
xlabel('Time (s)');
ylabel('F_y (N)');
title('Y-Axis Control Force');
grid on;

%% 17. Plot 3D bob trajectory

figure('Name','Controlled 3D Trajectory','Color','w');

plot3(bobPosition(:,1), ...
      bobPosition(:,2), ...
      bobPosition(:,3),'LineWidth',1.5);

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

title('Controlled Bob Trajectory');
grid on;
axis equal;
view(3);

%% 18. Plot energy

figure('Name','Controlled Energy','Color','w');

plot(time,kineticEnergy,'LineWidth',1.5);
hold on;

plot(time,potentialEnergy,'LineWidth',1.5);
plot(time,totalEnergy,'k--','LineWidth',1.5);

xlabel('Time (s)');
ylabel('Energy (J)');

legend('Kinetic','Potential','Total');
title('Controlled System Energy');
grid on;

%% 19. Plot rod constraint error

figure('Name','Controlled Rod Constraint','Color','w');

plot(time,rodLengthError,'LineWidth',1.5);

xlabel('Time (s)');
ylabel('Rod length error (m)');
title('Rigid Rod Constraint Error');
grid on;

%% 20. Final status

fprintf('\n============================================\n');

if simulationCompleted
    fprintf('SIMULATION: COMPLETED\n');
else
    fprintf('SIMULATION: INCOMPLETE\n');
end

fprintf('Maximum rod error: %.6e m\n',maxRodError);
fprintf('Maximum energy error: %.6e J\n',maxEnergyError);

fprintf('============================================\n');
