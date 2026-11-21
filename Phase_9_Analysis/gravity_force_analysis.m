
%% PHASE 9.13: GRAVITY SENSITIVITY UNDER EXTERNAL FORCE
% 3D X-Y Cart with Inverted Spherical Pendulum

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 9.13: GRAVITY SENSITIVITY ANALYSIS\n');
fprintf('============================================\n');

%% 1. PROJECT PATHS

analysisFolder = fileparts(mfilename('fullpath'));
projectRoot = fileparts(analysisFolder);

addpath(genpath(fullfile(projectRoot, ...
    'Phase_4_MATLAB_Simulation')));

resultsFolder = fullfile(projectRoot, ...
    'Results', 'Phase_9');

if ~exist(resultsFolder, 'dir')
    mkdir(resultsFolder);
end

%% 2. BASELINE PARAMETERS

p.M = 2.0;
p.m = 0.5;
p.L = 1.0;
p.g = 9.81;

p.Fx = 1.0;
p.Fy = 0.0;

p.x0 = 0;
p.y0 = 0;

p.theta0 = deg2rad(5);
p.phi0 = deg2rad(30);

p.dx0 = 0;
p.dy0 = 0;
p.dtheta0 = 0;
p.dphi0 = 0;

p.t_start = 0;
p.t_end = 15;
p.sample_time = 0.01;

%% 3. GRAVITY VALUES

gravityValues = [7.81 8.81 9.81 10.81 11.81];

numberOfCases = numel(gravityValues);

fprintf('\n--- BASELINE PARAMETERS ---\n');
fprintf('Cart mass: %.3f kg\n', p.M);
fprintf('Bob mass: %.3f kg\n', p.m);
fprintf('Rod length: %.3f m\n', p.L);
fprintf('Initial inclination: %.3f deg\n', rad2deg(p.theta0));
fprintf('Initial azimuth: %.3f deg\n', rad2deg(p.phi0));
fprintf('External force: Fx = %.2f N, Fy = %.2f N\n', ...
    p.Fx, p.Fy);

fprintf('\n--- GRAVITY CASES ---\n');

for k = 1:numberOfCases
    fprintf('Case %d: g = %.2f m/s^2\n', ...
        k, gravityValues(k));
end

%% 4. INITIAL CARTESIAN STATE

theta = p.theta0;
phi = p.phi0;

X0 = p.L*sin(theta)*cos(phi);
Y0 = p.L*sin(theta)*sin(phi);
Z0 = p.L*cos(theta);

z0 = [ ...
    p.x0;
    p.y0;
    X0;
    Y0;
    Z0;
    p.dx0;
    p.dy0;
    0;
    0;
    0];

%% 5. SOLVER SETTINGS

timeVector = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol', 1e-11, ...
    'AbsTol', 1e-13, ...
    'MaxStep', p.sample_time/5);

%% 6. RUN GRAVITY SENSITIVITY SIMULATIONS

fprintf('\n============================================\n');
fprintf('RUNNING GRAVITY SENSITIVITY SIMULATIONS\n');
fprintf('============================================\n');

results = struct([]);

for k = 1:numberOfCases

    pCase = p;
    pCase.g = gravityValues(k);

    fprintf('\nCase %d: Gravity %.2f m/s^2\n', ...
        k, pCase.g);

    [t, states] = ode45( ...
        @(t,z) cartesian_equations_of_motion(t,z,pCase), ...
        timeVector, z0, options);

    %% Extract positions

    x = states(:,1);
    y = states(:,2);

    X = states(:,3);
    Y = states(:,4);
    Z = states(:,5);

    %% Extract velocities

    dx = states(:,6);
    dy = states(:,7);

    dX = states(:,8);
    dY = states(:,9);
    dZ = states(:,10);

    %% Cart motion

    cartRadial = sqrt( ...
        (x-pCase.x0).^2 + (y-pCase.y0).^2);

    cartSpeed = sqrt(dx.^2 + dy.^2);

    %% Bob displacement

    bobDisplacement = sqrt( ...
        (X-X0).^2 + ...
        (Y-Y0).^2 + ...
        (Z-Z0).^2);

    %% Relative pendulum position

    rx = X-x;
    ry = Y-y;
    rz = Z;

    relativeLength = sqrt(rx.^2 + ry.^2 + rz.^2);

    %% Pendulum inclination

    inclination = acosd(max(-1, min(1, rz/pCase.L)));

    %% Energy components

    cartKE = 0.5*pCase.M*(dx.^2 + dy.^2);

    bobKE = 0.5*pCase.m*( ...
        dX.^2 + dY.^2 + dZ.^2);

    potentialEnergy = pCase.m*pCase.g*Z;

    totalEnergy = cartKE + bobKE + potentialEnergy;

    %% External work

    externalWork = ...
        pCase.Fx*(x-pCase.x0) + ...
        pCase.Fy*(y-pCase.y0);

    energyResidual = ...
        totalEnergy-totalEnergy(1)-externalWork;

    %% Rod constraint

    rodError = abs(relativeLength-pCase.L);

    %% Store results

    results(k).gravity = pCase.g;

    results(k).time = t;
    results(k).states = states;

    results(k).cart_x = x;
    results(k).cart_y = y;

    results(k).bob_x = X;
    results(k).bob_y = Y;
    results(k).bob_z = Z;

    results(k).cart_radial = cartRadial;
    results(k).cart_speed = cartSpeed;

    results(k).bob_displacement = bobDisplacement;

    results(k).inclination = inclination;

    results(k).cart_KE = cartKE;
    results(k).bob_KE = bobKE;
    results(k).potential_energy = potentialEnergy;
    results(k).total_energy = totalEnergy;

    results(k).external_work = externalWork;
    results(k).energy_residual = energyResidual;

    %% Metrics

    results(k).max_cart_displacement = max(cartRadial);
    results(k).final_cart_displacement = cartRadial(end);

    results(k).max_cart_speed = max(cartSpeed);

    results(k).max_bob_displacement = max(bobDisplacement);

    results(k).max_inclination = max(inclination);
    results(k).final_inclination = inclination(end);

    results(k).final_external_work = externalWork(end);

    results(k).max_cart_KE = max(cartKE);
    results(k).max_bob_KE = max(bobKE);

    results(k).max_potential_energy = max(potentialEnergy);
    results(k).min_potential_energy = min(potentialEnergy);

    results(k).max_energy_residual = max(abs(energyResidual));
    results(k).max_rod_error = max(rodError);

    %% Display metrics

    fprintf('Maximum cart displacement: %.6f m\n', ...
        results(k).max_cart_displacement);

    fprintf('Maximum cart speed: %.6f m/s\n', ...
        results(k).max_cart_speed);

    fprintf('Maximum bob displacement: %.6f m\n', ...
        results(k).max_bob_displacement);

    fprintf('Maximum inclination: %.6f deg\n', ...
        results(k).max_inclination);

    fprintf('Final inclination: %.6f deg\n', ...
        results(k).final_inclination);

    fprintf('Final external work: %.6f J\n', ...
        results(k).final_external_work);

    fprintf('Maximum energy residual: %.6e J\n', ...
        results(k).max_energy_residual);

    fprintf('Maximum rod error: %.6e m\n', ...
        results(k).max_rod_error);

end

%% 7. SUMMARY TABLE

gravityColumn = [results.gravity]';

maxCartDisplacement = [results.max_cart_displacement]';
finalCartDisplacement = [results.final_cart_displacement]';

maxCartSpeed = [results.max_cart_speed]';

maxBobDisplacement = [results.max_bob_displacement]';

maxInclination = [results.max_inclination]';
finalInclination = [results.final_inclination]';

finalExternalWork = [results.final_external_work]';

maxCartKE = [results.max_cart_KE]';
maxBobKE = [results.max_bob_KE]';

maxPotentialEnergy = [results.max_potential_energy]';
minPotentialEnergy = [results.min_potential_energy]';

energyError = [results.max_energy_residual]';
rodError = [results.max_rod_error]';

summaryTable = table( ...
    gravityColumn, ...
    maxCartDisplacement, ...
    finalCartDisplacement, ...
    maxCartSpeed, ...
    maxBobDisplacement, ...
    maxInclination, ...
    finalInclination, ...
    finalExternalWork, ...
    maxCartKE, ...
    maxBobKE, ...
    maxPotentialEnergy, ...
    minPotentialEnergy, ...
    energyError, ...
    rodError, ...
    'VariableNames', { ...
    'Gravity_mps2', ...
    'Max_Cart_Displacement_m', ...
    'Final_Cart_Displacement_m', ...
    'Max_Cart_Speed_mps', ...
    'Max_Bob_Displacement_m', ...
    'Max_Inclination_deg', ...
    'Final_Inclination_deg', ...
    'Final_External_Work_J', ...
    'Max_Cart_KE_J', ...
    'Max_Bob_KE_J', ...
    'Max_Potential_Energy_J', ...
    'Min_Potential_Energy_J', ...
    'Max_Energy_Residual_J', ...
    'Max_Rod_Error_m'});

fprintf('\n============================================\n');
fprintf('GRAVITY SENSITIVITY SUMMARY\n');
fprintf('============================================\n');

disp(summaryTable);

%% 8. CART DISPLACEMENT COMPARISON

figure('Name','Gravity Cart Displacement');
hold on;

for k = 1:numberOfCases
    plot(results(k).time, results(k).cart_radial, ...
        'LineWidth', 1.3);
end

grid on;
xlabel('Time (s)');
ylabel('Radial cart displacement (m)');
title('Effect of Gravity on Cart Displacement');

legend(compose('g = %.2f m/s^2', gravityValues), ...
    'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_13_Gravity_Cart_Displacement.png'));

%% 9. CART SPEED COMPARISON

figure('Name','Gravity Cart Speed');
hold on;

for k = 1:numberOfCases
    plot(results(k).time, results(k).cart_speed, ...
        'LineWidth', 1.3);
end

grid on;
xlabel('Time (s)');
ylabel('Cart speed (m/s)');
title('Effect of Gravity on Cart Speed');

legend(compose('g = %.2f m/s^2', gravityValues), ...
    'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_13_Gravity_Cart_Speed.png'));

%% 10. PENDULUM INCLINATION COMPARISON

figure('Name','Gravity Pendulum Inclination');
hold on;

for k = 1:numberOfCases
    plot(results(k).time, results(k).inclination, ...
        'LineWidth', 1.3);
end

grid on;
xlabel('Time (s)');
ylabel('Inclination (deg)');
title('Effect of Gravity on Pendulum Inclination');

legend(compose('g = %.2f m/s^2', gravityValues), ...
    'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_13_Gravity_Inclination.png'));

%% 11. BOB DISPLACEMENT COMPARISON

figure('Name','Gravity Bob Displacement');
hold on;

for k = 1:numberOfCases
    plot(results(k).time, results(k).bob_displacement, ...
        'LineWidth', 1.3);
end

grid on;
xlabel('Time (s)');
ylabel('Bob displacement from initial position (m)');
title('Effect of Gravity on Bob Displacement');

legend(compose('g = %.2f m/s^2', gravityValues), ...
    'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_13_Gravity_Bob_Displacement.png'));

%% 12. ENERGY COMPONENT COMPARISON

figure('Name','Gravity Energy Components');

subplot(3,1,1);
hold on;
for k = 1:numberOfCases
    plot(results(k).time, results(k).cart_KE, ...
        'LineWidth', 1.1);
end
grid on;
ylabel('Cart KE (J)');
title('Cart Kinetic Energy');

subplot(3,1,2);
hold on;
for k = 1:numberOfCases
    plot(results(k).time, results(k).bob_KE, ...
        'LineWidth', 1.1);
end
grid on;
ylabel('Bob KE (J)');
title('Bob Kinetic Energy');

subplot(3,1,3);
hold on;
for k = 1:numberOfCases
    plot(results(k).time, results(k).potential_energy, ...
        'LineWidth', 1.1);
end
grid on;
xlabel('Time (s)');
ylabel('Potential energy (J)');
title('Gravitational Potential Energy');

legend(compose('g = %.2f m/s^2', gravityValues), ...
    'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_13_Gravity_Energy_Components.png'));

%% 13. ENERGY BALANCE

figure('Name','Gravity Energy Balance');
hold on;

for k = 1:numberOfCases
    plot(results(k).time, results(k).energy_residual, ...
        'LineWidth', 1.1);
end

grid on;
xlabel('Time (s)');
ylabel('Energy balance residual (J)');
title('Work-Energy Balance for Different Gravity Values');

legend(compose('g = %.2f m/s^2', gravityValues), ...
    'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_13_Gravity_Energy_Balance.png'));

%% 14. VALIDATION

maximumEnergyError = max(energyError);
maximumRodError = max(rodError);

fprintf('\n============================================\n');
fprintf('PHASE 9.13 VALIDATION\n');
fprintf('============================================\n');

fprintf('Number of simulations: %d\n', numberOfCases);
fprintf('Simulation duration: %.2f seconds\n', p.t_end);
fprintf('Applied X force: %.2f N\n', p.Fx);

fprintf('Maximum energy balance residual: %.6e J\n', ...
    maximumEnergyError);

fprintf('Maximum rod constraint error: %.6e m\n', ...
    maximumRodError);

if maximumEnergyError < 1e-5
    fprintf('Work-energy validation: PASSED\n');
else
    fprintf('Work-energy validation: REVIEW REQUIRED\n');
end

if maximumRodError < 1e-7
    fprintf('Rigid rod constraint: PASSED\n');
else
    fprintf('Rigid rod constraint: REVIEW REQUIRED\n');
end

%% 15. SAVE RESULTS

save(fullfile(resultsFolder, ...
    'phase_9_13_gravity_force_results.mat'), ...
    'results', 'summaryTable', ...
    'gravityValues', 'p', 'timeVector');

writetable(summaryTable, fullfile(resultsFolder, ...
    'Phase_9_13_Gravity_Sensitivity_Summary.csv'));

fprintf('\n============================================\n');
fprintf('PHASE 9.13 ANALYSIS COMPLETED\n');
fprintf('============================================\n');

fprintf('\nResults directory:\n%s\n', resultsFolder);

fprintf('\nSaved files:\n');
fprintf('1. phase_9_13_gravity_force_results.mat\n');
fprintf('2. Phase_9_13_Gravity_Sensitivity_Summary.csv\n');

fprintf('\nGenerated figures:\n');
fprintf('1. Phase_9_13_Gravity_Cart_Displacement.png\n');
fprintf('2. Phase_9_13_Gravity_Cart_Speed.png\n');
fprintf('3. Phase_9_13_Gravity_Inclination.png\n');
fprintf('4. Phase_9_13_Gravity_Bob_Displacement.png\n');
fprintf('5. Phase_9_13_Gravity_Energy_Components.png\n');
fprintf('6. Phase_9_13_Gravity_Energy_Balance.png\n');

fprintf('\n============================================\n');
fprintf('PHASE 9.13 SCRIPT FINISHED\n');
fprintf('============================================\n');

