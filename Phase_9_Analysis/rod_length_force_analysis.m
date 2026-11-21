
%% PHASE 9.12: ROD LENGTH SENSITIVITY UNDER EXTERNAL FORCE
% 3D X-Y Cart with Inverted Spherical Pendulum

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 9.12: ROD LENGTH SENSITIVITY ANALYSIS\n');
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

%% 3. ROD LENGTH VALUES

rodLengths = [0.5 0.75 1.0 1.25 1.5];

numberOfCases = numel(rodLengths);

fprintf('\n--- BASELINE PARAMETERS ---\n');
fprintf('Cart mass: %.3f kg\n', p.M);
fprintf('Bob mass: %.3f kg\n', p.m);
fprintf('Gravity: %.3f m/s^2\n', p.g);
fprintf('Initial inclination: %.3f deg\n', rad2deg(p.theta0));
fprintf('Initial azimuth: %.3f deg\n', rad2deg(p.phi0));
fprintf('External force: Fx = %.2f N, Fy = %.2f N\n', ...
    p.Fx, p.Fy);

fprintf('\n--- ROD LENGTH CASES ---\n');

for k = 1:numberOfCases
    fprintf('Case %d: L = %.2f m\n', k, rodLengths(k));
end

%% 4. SOLVER SETTINGS

timeVector = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol', 1e-11, ...
    'AbsTol', 1e-13, ...
    'MaxStep', p.sample_time/5);

%% 5. RUN SIMULATIONS

fprintf('\n============================================\n');
fprintf('RUNNING ROD LENGTH SENSITIVITY SIMULATIONS\n');
fprintf('============================================\n');

results = struct([]);

for k = 1:numberOfCases

    pCase = p;
    pCase.L = rodLengths(k);

    %% Initial Cartesian state for this rod length

    theta = pCase.theta0;
    phi = pCase.phi0;

    X0 = pCase.L*sin(theta)*cos(phi);
    Y0 = pCase.L*sin(theta)*sin(phi);
    Z0 = pCase.L*cos(theta);

    z0 = [ ...
        pCase.x0;
        pCase.y0;
        X0;
        Y0;
        Z0;
        pCase.dx0;
        pCase.dy0;
        0;
        0;
        0];

    fprintf('\nCase %d: Rod length %.2f m\n', ...
        k, pCase.L);

    [t, states] = ode45( ...
        @(t,z) cartesian_equations_of_motion(t,z,pCase), ...
        timeVector, z0, options);

    %% Extract Cartesian positions and velocities

    x = states(:,1);
    y = states(:,2);

    X = states(:,3);
    Y = states(:,4);
    Z = states(:,5);

    dx = states(:,6);
    dy = states(:,7);

    dX = states(:,8);
    dY = states(:,9);
    dZ = states(:,10);

    %% Cart motion

    cartRadial = sqrt( ...
        (x-pCase.x0).^2 + (y-pCase.y0).^2);

    cartSpeed = sqrt(dx.^2 + dy.^2);

    %% Bob displacement from initial position

    bobDisplacement = sqrt( ...
        (X-X0).^2 + ...
        (Y-Y0).^2 + ...
        (Z-Z0).^2);

    %% Relative pendulum coordinates

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

    %% Rod constraint error

    rodError = abs(relativeLength-pCase.L);

    %% Save case results

    results(k).rod_length = pCase.L;
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

    results(k).max_energy_residual = max(abs(energyResidual));
    results(k).max_rod_error = max(rodError);

    %% Display case metrics

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

%% 6. SUMMARY TABLE

rodLengthColumn = [results.rod_length]';

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

energyError = [results.max_energy_residual]';
rodError = [results.max_rod_error]';

summaryTable = table( ...
    rodLengthColumn, ...
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
    energyError, ...
    rodError, ...
    'VariableNames', { ...
    'Rod_Length_m', ...
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
    'Max_Energy_Residual_J', ...
    'Max_Rod_Error_m'});

fprintf('\n============================================\n');
fprintf('ROD LENGTH SENSITIVITY SUMMARY\n');
fprintf('============================================\n');

disp(summaryTable);

%% 7. CART DISPLACEMENT COMPARISON

figure('Name','Rod Length Cart Displacement');
hold on;

for k = 1:numberOfCases
    plot(results(k).time, results(k).cart_radial, ...
        'LineWidth', 1.3);
end

grid on;
xlabel('Time (s)');
ylabel('Radial cart displacement (m)');
title('Effect of Rod Length on Cart Displacement');

legend(compose('L = %.2f m', rodLengths), ...
    'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_12_Rod_Length_Cart_Displacement.png'));

%% 8. CART SPEED COMPARISON

figure('Name','Rod Length Cart Speed');
hold on;

for k = 1:numberOfCases
    plot(results(k).time, results(k).cart_speed, ...
        'LineWidth', 1.3);
end

grid on;
xlabel('Time (s)');
ylabel('Cart speed (m/s)');
title('Effect of Rod Length on Cart Speed');

legend(compose('L = %.2f m', rodLengths), ...
    'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_12_Rod_Length_Cart_Speed.png'));

%% 9. PENDULUM INCLINATION COMPARISON

figure('Name','Rod Length Pendulum Inclination');
hold on;

for k = 1:numberOfCases
    plot(results(k).time, results(k).inclination, ...
        'LineWidth', 1.3);
end

grid on;
xlabel('Time (s)');
ylabel('Inclination (deg)');
title('Effect of Rod Length on Pendulum Inclination');

legend(compose('L = %.2f m', rodLengths), ...
    'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_12_Rod_Length_Inclination.png'));

%% 10. BOB DISPLACEMENT COMPARISON

figure('Name','Rod Length Bob Displacement');
hold on;

for k = 1:numberOfCases
    plot(results(k).time, results(k).bob_displacement, ...
        'LineWidth', 1.3);
end

grid on;
xlabel('Time (s)');
ylabel('Bob displacement from initial position (m)');
title('Effect of Rod Length on Bob Displacement');

legend(compose('L = %.2f m', rodLengths), ...
    'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_12_Rod_Length_Bob_Displacement.png'));

%% 11. ENERGY COMPONENT COMPARISON

figure('Name','Rod Length Energy Components');

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

legend(compose('L = %.2f m', rodLengths), ...
    'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_12_Rod_Length_Energy_Components.png'));

%% 12. ENERGY BALANCE

figure('Name','Rod Length Energy Balance');
hold on;

for k = 1:numberOfCases
    plot(results(k).time, results(k).energy_residual, ...
        'LineWidth', 1.1);
end

grid on;
xlabel('Time (s)');
ylabel('Energy balance residual (J)');
title('Work-Energy Balance for Different Rod Lengths');

legend(compose('L = %.2f m', rodLengths), ...
    'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_12_Rod_Length_Energy_Balance.png'));

%% 13. VALIDATION

maximumEnergyError = max(energyError);
maximumRodError = max(rodError);

fprintf('\n============================================\n');
fprintf('PHASE 9.12 VALIDATION\n');
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

%% 14. SAVE RESULTS

save(fullfile(resultsFolder, ...
    'phase_9_12_rod_length_force_results.mat'), ...
    'results', 'summaryTable', ...
    'rodLengths', 'p', 'timeVector');

writetable(summaryTable, fullfile(resultsFolder, ...
    'Phase_9_12_Rod_Length_Summary.csv'));

fprintf('\n============================================\n');
fprintf('PHASE 9.12 ANALYSIS COMPLETED\n');
fprintf('============================================\n');

fprintf('\nResults directory:\n%s\n', resultsFolder);

fprintf('\nSaved files:\n');
fprintf('1. phase_9_12_rod_length_force_results.mat\n');
fprintf('2. Phase_9_12_Rod_Length_Summary.csv\n');

fprintf('\nGenerated figures:\n');
fprintf('1. Phase_9_12_Rod_Length_Cart_Displacement.png\n');
fprintf('2. Phase_9_12_Rod_Length_Cart_Speed.png\n');
fprintf('3. Phase_9_12_Rod_Length_Inclination.png\n');
fprintf('4. Phase_9_12_Rod_Length_Bob_Displacement.png\n');
fprintf('5. Phase_9_12_Rod_Length_Energy_Components.png\n');
fprintf('6. Phase_9_12_Rod_Length_Energy_Balance.png\n');

fprintf('\n============================================\n');
fprintf('PHASE 9.12 SCRIPT FINISHED\n');
fprintf('============================================\n');
