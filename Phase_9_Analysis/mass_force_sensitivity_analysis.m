
%% PHASE 9.11: MASS SENSITIVITY UNDER EXTERNAL FORCE
% 3D X-Y Cart with Inverted Spherical Pendulum
%
% Five cart-mass cases and five bob-mass cases.
% Constant external force: Fx = +1 N, Fy = 0 N.

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 9.11: MASS SENSITIVITY ANALYSIS\n');
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

fprintf('\n--- BASELINE PARAMETERS ---\n');
fprintf('Cart mass: %.3f kg\n', p.M);
fprintf('Bob mass: %.3f kg\n', p.m);
fprintf('Rod length: %.3f m\n', p.L);
fprintf('Gravity: %.3f m/s^2\n', p.g);
fprintf('External force: Fx = %.2f N, Fy = %.2f N\n', ...
    p.Fx, p.Fy);

%% 3. DEFINE MASS VARIATIONS

cartMassValues = [1.0 1.5 2.0 2.5 3.0];

bobMassValues = [0.25 0.375 0.50 0.625 0.75];

caseType = [repmat({'Cart mass'}, 1, 5), ...
            repmat({'Bob mass'}, 1, 5)];

caseValues = [cartMassValues, bobMassValues];

caseNames = { ...
    'Cart mass 1.0 kg', ...
    'Cart mass 1.5 kg', ...
    'Cart mass 2.0 kg', ...
    'Cart mass 2.5 kg', ...
    'Cart mass 3.0 kg', ...
    'Bob mass 0.25 kg', ...
    'Bob mass 0.375 kg', ...
    'Bob mass 0.50 kg', ...
    'Bob mass 0.625 kg', ...
    'Bob mass 0.75 kg'};

numberOfCases = numel(caseValues);

%% 4. INITIAL CARTESIAN STATE

theta = p.theta0;
phi = p.phi0;

X0 = p.L * sin(theta) * cos(phi);
Y0 = p.L * sin(theta) * sin(phi);
Z0 = p.L * cos(theta);

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

timeVector = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol', 1e-11, ...
    'AbsTol', 1e-13, ...
    'MaxStep', p.sample_time/5);

%% 5. RUN MASS SENSITIVITY SIMULATIONS

fprintf('\n============================================\n');
fprintf('RUNNING MASS SENSITIVITY SIMULATIONS\n');
fprintf('============================================\n');

results = struct([]);

for k = 1:numberOfCases

    pCase = p;

    if strcmp(caseType{k}, 'Cart mass')
        pCase.M = caseValues(k);
    else
        pCase.m = caseValues(k);
    end

    fprintf('\nCase %d: %s\n', k, caseNames{k});

    [t, states] = ode45( ...
        @(t,z) cartesian_equations_of_motion(t,z,pCase), ...
        timeVector, z0, options);

    %% Extract states

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

    %% Cart displacement and velocity

    cartRadial = sqrt( ...
        (x-p.x0).^2 + (y-p.y0).^2);

    cartSpeed = sqrt(dx.^2 + dy.^2);

    %% Pendulum inclination

    rx = X-x;
    ry = Y-y;
    rz = Z;

    relativeLength = sqrt(rx.^2 + ry.^2 + rz.^2);

    inclination = acosd(max(-1, min(1, rz/pCase.L)));

    %% Energy calculation

    cartKE = 0.5*pCase.M*(dx.^2 + dy.^2);

    bobKE = 0.5*pCase.m*( ...
        dX.^2 + dY.^2 + dZ.^2);

    potentialEnergy = pCase.m*pCase.g*Z;

    totalEnergy = cartKE + bobKE + potentialEnergy;

    %% Exact external work for constant force

    externalWork = ...
        pCase.Fx*(x-p.x0) + ...
        pCase.Fy*(y-p.y0);

    energyResidual = ...
        totalEnergy-totalEnergy(1)-externalWork;

    %% Rod constraint

    rodError = abs(relativeLength-pCase.L);

    %% Store results

    results(k).case_name = caseNames{k};
    results(k).case_type = caseType{k};
    results(k).mass_value = caseValues(k);

    results(k).time = t;
    results(k).states = states;

    results(k).cart_x = x;
    results(k).cart_y = y;

    results(k).cart_radial = cartRadial;
    results(k).cart_speed = cartSpeed;

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

    results(k).max_inclination = max(inclination);
    results(k).final_inclination = inclination(end);

    results(k).final_external_work = externalWork(end);

    results(k).max_energy_residual = max(abs(energyResidual));
    results(k).max_rod_error = max(rodError);

    fprintf('Cart mass: %.3f kg\n', pCase.M);
    fprintf('Bob mass: %.3f kg\n', pCase.m);
    fprintf('Maximum cart displacement: %.6f m\n', ...
        results(k).max_cart_displacement);
    fprintf('Maximum cart speed: %.6f m/s\n', ...
        results(k).max_cart_speed);
    fprintf('Maximum inclination: %.6f deg\n', ...
        results(k).max_inclination);
    fprintf('Maximum energy residual: %.6e J\n', ...
        results(k).max_energy_residual);
    fprintf('Maximum rod error: %.6e m\n', ...
        results(k).max_rod_error);

end

%% 6. CREATE SUMMARY TABLE

caseNameColumn = string({results.case_name})';

massTypeColumn = string({results.case_type})';

massValueColumn = [results.mass_value]';

maxDisplacement = [results.max_cart_displacement]';
maxSpeed = [results.max_cart_speed]';
maxInclination = [results.max_inclination]';
finalInclination = [results.final_inclination]';
finalWork = [results.final_external_work]';
energyError = [results.max_energy_residual]';
rodError = [results.max_rod_error]';

summaryTable = table( ...
    caseNameColumn, ...
    massTypeColumn, ...
    massValueColumn, ...
    maxDisplacement, ...
    maxSpeed, ...
    maxInclination, ...
    finalInclination, ...
    finalWork, ...
    energyError, ...
    rodError, ...
    'VariableNames', { ...
    'Case', ...
    'Mass_Type', ...
    'Mass_kg', ...
    'Max_Cart_Displacement_m', ...
    'Max_Cart_Speed_mps', ...
    'Max_Inclination_deg', ...
    'Final_Inclination_deg', ...
    'Final_External_Work_J', ...
    'Max_Energy_Residual_J', ...
    'Max_Rod_Error_m'});

fprintf('\n============================================\n');
fprintf('MASS SENSITIVITY SUMMARY\n');
fprintf('============================================\n');

disp(summaryTable);

%% 7. CART MASS SENSITIVITY FIGURE

figure('Name','Cart Mass Sensitivity');

subplot(2,1,1);
plot(cartMassValues, ...
    maxDisplacement(1:5), '-o', 'LineWidth', 1.5);
grid on;
xlabel('Cart mass (kg)');
ylabel('Maximum radial displacement (m)');
title('Cart Mass vs Maximum Cart Displacement');

subplot(2,1,2);
plot(cartMassValues, ...
    maxSpeed(1:5), '-o', 'LineWidth', 1.5);
grid on;
xlabel('Cart mass (kg)');
ylabel('Maximum cart speed (m/s)');
title('Cart Mass vs Maximum Cart Speed');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_11_Cart_Mass_Sensitivity.png'));

%% 8. BOB MASS SENSITIVITY FIGURE

figure('Name','Bob Mass Sensitivity');

subplot(2,1,1);
plot(bobMassValues, ...
    maxDisplacement(6:10), '-o', 'LineWidth', 1.5);
grid on;
xlabel('Bob mass (kg)');
ylabel('Maximum radial displacement (m)');
title('Bob Mass vs Maximum Cart Displacement');

subplot(2,1,2);
plot(bobMassValues, ...
    maxSpeed(6:10), '-o', 'LineWidth', 1.5);
grid on;
xlabel('Bob mass (kg)');
ylabel('Maximum cart speed (m/s)');
title('Bob Mass vs Maximum Cart Speed');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_11_Bob_Mass_Sensitivity.png'));

%% 9. PENDULUM INCLINATION COMPARISON

figure('Name','Mass and Pendulum Inclination');

subplot(2,1,1);
hold on;
for k = 1:5
    plot(results(k).time, results(k).inclination, ...
        'LineWidth', 1.2);
end
grid on;
xlabel('Time (s)');
ylabel('Inclination (deg)');
title('Effect of Cart Mass on Pendulum Inclination');
legend(caseNames(1:5), 'Location', 'best');

subplot(2,1,2);
hold on;
for k = 6:10
    plot(results(k).time, results(k).inclination, ...
        'LineWidth', 1.2);
end
grid on;
xlabel('Time (s)');
ylabel('Inclination (deg)');
title('Effect of Bob Mass on Pendulum Inclination');
legend(caseNames(6:10), 'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_11_Mass_Inclination.png'));

%% 10. ENERGY BALANCE COMPARISON

figure('Name','Mass Sensitivity Energy Balance');
hold on;

for k = 1:numberOfCases
    plot(results(k).time, results(k).energy_residual, ...
        'LineWidth', 1.0);
end

grid on;
xlabel('Time (s)');
ylabel('Energy balance residual (J)');
title('Work-Energy Balance Across Mass Cases');
legend(caseNames, 'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_11_Energy_Balance.png'));

%% 11. VALIDATION

maximumEnergyError = max(energyError);
maximumRodError = max(rodError);

fprintf('\n============================================\n');
fprintf('PHASE 9.11 VALIDATION\n');
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

%% 12. SAVE RESULTS

save(fullfile(resultsFolder, ...
    'phase_9_11_mass_force_sensitivity_results.mat'), ...
    'results', 'summaryTable', ...
    'cartMassValues', 'bobMassValues', ...
    'p', 'timeVector');

writetable(summaryTable, fullfile(resultsFolder, ...
    'Phase_9_11_Mass_Sensitivity_Summary.csv'));

fprintf('\n============================================\n');
fprintf('PHASE 9.11 ANALYSIS COMPLETED\n');
fprintf('============================================\n');

fprintf('\nResults directory:\n%s\n', resultsFolder);

fprintf('\nSaved files:\n');
fprintf('1. phase_9_11_mass_force_sensitivity_results.mat\n');
fprintf('2. Phase_9_11_Mass_Sensitivity_Summary.csv\n');

fprintf('\nGenerated figures:\n');
fprintf('1. Phase_9_11_Cart_Mass_Sensitivity.png\n');
fprintf('2. Phase_9_11_Bob_Mass_Sensitivity.png\n');
fprintf('3. Phase_9_11_Mass_Inclination.png\n');
fprintf('4. Phase_9_11_Energy_Balance.png\n');

fprintf('\n============================================\n');
fprintf('PHASE 9.11 SCRIPT FINISHED\n');
fprintf('============================================\n');
