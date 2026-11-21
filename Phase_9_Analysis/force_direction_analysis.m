
%% PHASE 9.14: EXTERNAL FORCE MAGNITUDE AND DIRECTION SENSITIVITY
% 3D X-Y Cart with Inverted Spherical Pendulum

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 9.14: FORCE MAGNITUDE AND DIRECTION\n');
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

p.x0 = 0;
p.y0 = 0;

p.theta0 = deg2rad(5);
p.phi0 = deg2rad(30);

p.dx0 = 0;
p.dy0 = 0;

p.t_start = 0;
p.t_end = 15;
p.sample_time = 0.01;

%% 3. FORCE MAGNITUDES AND DIRECTIONS

forceMagnitudes = [0 0.5 1.0 1.5 2.0];

forceDirections = [0 30 60 90 120 150];

numberOfMagnitudes = numel(forceMagnitudes);
numberOfDirections = numel(forceDirections);

numberOfCases = numberOfMagnitudes * numberOfDirections;

fprintf('\n--- BASELINE PARAMETERS ---\n');
fprintf('Cart mass: %.3f kg\n', p.M);
fprintf('Bob mass: %.3f kg\n', p.m);
fprintf('Rod length: %.3f m\n', p.L);
fprintf('Gravity: %.3f m/s^2\n', p.g);
fprintf('Initial inclination: %.3f deg\n', rad2deg(p.theta0));
fprintf('Initial azimuth: %.3f deg\n', rad2deg(p.phi0));

fprintf('\n--- FORCE MAGNITUDES ---\n');
disp(forceMagnitudes);

fprintf('--- FORCE DIRECTIONS (degrees) ---\n');
disp(forceDirections);

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

%% 6. RUN SIMULATIONS

fprintf('\n============================================\n');
fprintf('RUNNING FORCE SENSITIVITY SIMULATIONS\n');
fprintf('============================================\n');

results = struct([]);
caseNumber = 0;

for i = 1:numberOfMagnitudes

    for j = 1:numberOfDirections

        caseNumber = caseNumber + 1;

        magnitude = forceMagnitudes(i);
        direction = forceDirections(j);

        pCase = p;

        pCase.Fx = magnitude*cosd(direction);
        pCase.Fy = magnitude*sind(direction);

        fprintf('\nCase %d of %d\n', caseNumber, numberOfCases);
        fprintf('Force magnitude: %.2f N\n', magnitude);
        fprintf('Force direction: %.2f deg\n', direction);
        fprintf('Fx = %.6f N, Fy = %.6f N\n', ...
            pCase.Fx, pCase.Fy);

        %% Solve Cartesian equations of motion

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

        %% Pendulum geometry

        rx = X-x;
        ry = Y-y;
        rz = Z;

        relativeLength = sqrt(rx.^2 + ry.^2 + rz.^2);

        inclination = acosd(max(-1, min(1, rz/pCase.L)));

        %% Bob displacement from initial position

        bobDisplacement = sqrt( ...
            (X-X0).^2 + ...
            (Y-Y0).^2 + ...
            (Z-Z0).^2);

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

        %% Store simulation data

        results(caseNumber).force_magnitude = magnitude;
        results(caseNumber).force_direction = direction;

        results(caseNumber).Fx = pCase.Fx;
        results(caseNumber).Fy = pCase.Fy;

        results(caseNumber).time = t;
        results(caseNumber).states = states;

        results(caseNumber).cart_x = x;
        results(caseNumber).cart_y = y;

        results(caseNumber).bob_x = X;
        results(caseNumber).bob_y = Y;
        results(caseNumber).bob_z = Z;

        results(caseNumber).cart_radial = cartRadial;
        results(caseNumber).cart_speed = cartSpeed;

        results(caseNumber).inclination = inclination;
        results(caseNumber).bob_displacement = bobDisplacement;

        results(caseNumber).cart_KE = cartKE;
        results(caseNumber).bob_KE = bobKE;
        results(caseNumber).potential_energy = potentialEnergy;
        results(caseNumber).total_energy = totalEnergy;

        results(caseNumber).external_work = externalWork;
        results(caseNumber).energy_residual = energyResidual;

        results(caseNumber).max_cart_displacement = max(cartRadial);
        results(caseNumber).final_cart_displacement = cartRadial(end);

        results(caseNumber).max_cart_speed = max(cartSpeed);

        results(caseNumber).max_inclination = max(inclination);
        results(caseNumber).final_inclination = inclination(end);

        results(caseNumber).max_bob_displacement = max(bobDisplacement);

        results(caseNumber).final_external_work = externalWork(end);

        results(caseNumber).max_cart_KE = max(cartKE);
        results(caseNumber).max_bob_KE = max(bobKE);

        results(caseNumber).max_potential_energy = max(potentialEnergy);

        results(caseNumber).max_energy_residual = max(abs(energyResidual));
        results(caseNumber).max_rod_error = max(rodError);

        fprintf('Maximum cart displacement: %.6f m\n', ...
            results(caseNumber).max_cart_displacement);

        fprintf('Maximum cart speed: %.6f m/s\n', ...
            results(caseNumber).max_cart_speed);

        fprintf('Maximum inclination: %.6f deg\n', ...
            results(caseNumber).max_inclination);

        fprintf('Final inclination: %.6f deg\n', ...
            results(caseNumber).final_inclination);

        fprintf('Maximum energy residual: %.6e J\n', ...
            results(caseNumber).max_energy_residual);

        fprintf('Maximum rod error: %.6e m\n', ...
            results(caseNumber).max_rod_error);

    end
end

%% 7. SUMMARY TABLE

forceMagnitudeColumn = [results.force_magnitude]';
forceDirectionColumn = [results.force_direction]';

FxColumn = [results.Fx]';
FyColumn = [results.Fy]';

maxCartDisplacement = [results.max_cart_displacement]';
finalCartDisplacement = [results.final_cart_displacement]';

maxCartSpeed = [results.max_cart_speed]';

maxInclination = [results.max_inclination]';
finalInclination = [results.final_inclination]';

maxBobDisplacement = [results.max_bob_displacement]';

finalExternalWork = [results.final_external_work]';

maxCartKE = [results.max_cart_KE]';
maxBobKE = [results.max_bob_KE]';

maxPotentialEnergy = [results.max_potential_energy]';

energyError = [results.max_energy_residual]';
rodError = [results.max_rod_error]';

summaryTable = table( ...
    forceMagnitudeColumn, ...
    forceDirectionColumn, ...
    FxColumn, ...
    FyColumn, ...
    maxCartDisplacement, ...
    finalCartDisplacement, ...
    maxCartSpeed, ...
    maxInclination, ...
    finalInclination, ...
    maxBobDisplacement, ...
    finalExternalWork, ...
    maxCartKE, ...
    maxBobKE, ...
    maxPotentialEnergy, ...
    energyError, ...
    rodError, ...
    'VariableNames', { ...
    'Force_Magnitude_N', ...
    'Force_Direction_deg', ...
    'Fx_N', ...
    'Fy_N', ...
    'Max_Cart_Displacement_m', ...
    'Final_Cart_Displacement_m', ...
    'Max_Cart_Speed_mps', ...
    'Max_Inclination_deg', ...
    'Final_Inclination_deg', ...
    'Max_Bob_Displacement_m', ...
    'Final_External_Work_J', ...
    'Max_Cart_KE_J', ...
    'Max_Bob_KE_J', ...
    'Max_Potential_Energy_J', ...
    'Max_Energy_Residual_J', ...
    'Max_Rod_Error_m'});

fprintf('\n============================================\n');
fprintf('FORCE SENSITIVITY SUMMARY\n');
fprintf('============================================\n');

disp(summaryTable);

%% 8. CART TRAJECTORY COMPARISON

figure('Name','Force Direction XY Trajectories');
hold on;

for k = 1:numberOfCases

    if results(k).force_magnitude > 0
        plot(results(k).cart_x, results(k).cart_y, ...
            'LineWidth', 1.0);
    end

end

grid on;
axis equal;

xlabel('Cart X position (m)');
ylabel('Cart Y position (m)');
title('Cart Trajectories Under Different Force Conditions');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_14_Force_XY_Trajectories.png'));

%% 9. MAXIMUM CART DISPLACEMENT

figure('Name','Force Magnitude Displacement');

hold on;

for j = 1:numberOfDirections

    indices = j:numberOfDirections:numberOfCases;

    displacementValues = [results(indices).max_cart_displacement];

    plot(forceMagnitudes, displacementValues, ...
        '-o', 'LineWidth', 1.3);

end

grid on;

xlabel('Force magnitude (N)');
ylabel('Maximum radial cart displacement (m)');
title('Cart Displacement vs Force Magnitude');

legend(compose('Direction = %d deg', forceDirections), ...
    'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_14_Force_Displacement.png'));

%% 10. MAXIMUM CART SPEED

figure('Name','Force Magnitude Speed');

hold on;

for j = 1:numberOfDirections

    indices = j:numberOfDirections:numberOfCases;

    speedValues = [results(indices).max_cart_speed];

    plot(forceMagnitudes, speedValues, ...
        '-o', 'LineWidth', 1.3);

end

grid on;

xlabel('Force magnitude (N)');
ylabel('Maximum cart speed (m/s)');
title('Cart Speed vs Force Magnitude');

legend(compose('Direction = %d deg', forceDirections), ...
    'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_14_Force_Speed.png'));

%% 11. MAXIMUM PENDULUM INCLINATION

figure('Name','Force Magnitude Inclination');

hold on;

for j = 1:numberOfDirections

    indices = j:numberOfDirections:numberOfCases;

    inclinationValues = [results(indices).max_inclination];

    plot(forceMagnitudes, inclinationValues, ...
        '-o', 'LineWidth', 1.3);

end

grid on;

xlabel('Force magnitude (N)');
ylabel('Maximum inclination (deg)');
title('Pendulum Inclination vs Force Magnitude');

legend(compose('Direction = %d deg', forceDirections), ...
    'Location', 'best');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_14_Force_Inclination.png'));

%% 12. ENERGY BALANCE

figure('Name','Force Work Energy Balance');

hold on;

for k = 1:numberOfCases

    plot(results(k).time, results(k).energy_residual, ...
        'LineWidth', 0.8);

end

grid on;

xlabel('Time (s)');
ylabel('Energy residual (J)');
title('Work-Energy Balance Across Force Cases');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_14_Force_Energy_Balance.png'));

%% 13. VALIDATION

maximumEnergyError = max(energyError);
maximumRodError = max(rodError);

fprintf('\n============================================\n');
fprintf('PHASE 9.14 VALIDATION\n');
fprintf('============================================\n');

fprintf('Number of simulations: %d\n', numberOfCases);

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
    'phase_9_14_force_direction_results.mat'), ...
    'results', 'summaryTable', ...
    'forceMagnitudes', 'forceDirections', ...
    'p', 'timeVector');

writetable(summaryTable, fullfile(resultsFolder, ...
    'Phase_9_14_Force_Direction_Summary.csv'));

fprintf('\n============================================\n');
fprintf('PHASE 9.14 ANALYSIS COMPLETED\n');
fprintf('============================================\n');

fprintf('\nResults directory:\n%s\n', resultsFolder);

fprintf('\nSaved files:\n');
fprintf('1. phase_9_14_force_direction_results.mat\n');
fprintf('2. Phase_9_14_Force_Direction_Summary.csv\n');

fprintf('\nGenerated figures:\n');
fprintf('1. Phase_9_14_Force_XY_Trajectories.png\n');
fprintf('2. Phase_9_14_Force_Displacement.png\n');
fprintf('3. Phase_9_14_Force_Speed.png\n');
fprintf('4. Phase_9_14_Force_Inclination.png\n');
fprintf('5. Phase_9_14_Force_Energy_Balance.png\n');

fprintf('\n============================================\n');
fprintf('PHASE 9.14 SCRIPT FINISHED\n');
fprintf('============================================\n');
