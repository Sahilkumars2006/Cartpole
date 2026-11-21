
%% PHASE 9.15: INITIAL AZIMUTH AND EXTERNAL FORCE INTERACTION
% 3D X-Y Cart with Inverted Spherical Pendulum

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 9.15: AZIMUTH-FORCE INTERACTION\n');
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

p.Fx = 0;
p.Fy = 0;

p.x0 = 0;
p.y0 = 0;

p.theta0 = deg2rad(5);

p.dx0 = 0;
p.dy0 = 0;

p.t_start = 0;
p.t_end = 15;
p.sample_time = 0.01;

%% 3. EXPERIMENTAL PARAMETERS

initialAzimuths = [0 30 60 90 120 150];

forceDirections = [0 45 90 135 180 225 270 315];

forceMagnitude = 1.0;

numberOfAzimuths = numel(initialAzimuths);
numberOfDirections = numel(forceDirections);

numberOfCases = numberOfAzimuths * numberOfDirections;

fprintf('\n--- BASELINE PARAMETERS ---\n');

fprintf('Cart mass: %.3f kg\n', p.M);
fprintf('Bob mass: %.3f kg\n', p.m);
fprintf('Rod length: %.3f m\n', p.L);
fprintf('Gravity: %.3f m/s^2\n', p.g);
fprintf('Initial inclination: %.3f deg\n', rad2deg(p.theta0));
fprintf('Force magnitude: %.3f N\n', forceMagnitude);

fprintf('\nInitial azimuths (deg):\n');
disp(initialAzimuths);

fprintf('Force directions (deg):\n');
disp(forceDirections);

fprintf('Total simulations: %d\n', numberOfCases);

%% 4. SOLVER SETTINGS

timeVector = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol', 1e-11, ...
    'AbsTol', 1e-13, ...
    'MaxStep', p.sample_time/5);

%% 5. RUN SIMULATIONS

fprintf('\n============================================\n');
fprintf('RUNNING AZIMUTH-FORCE INTERACTION STUDY\n');
fprintf('============================================\n');

results = struct([]);
caseNumber = 0;

for i = 1:numberOfAzimuths

    for j = 1:numberOfDirections

        caseNumber = caseNumber + 1;

        initialAzimuth = initialAzimuths(i);
        forceDirection = forceDirections(j);

        %% Case-specific parameters

        pCase = p;

        pCase.phi0 = deg2rad(initialAzimuth);

        pCase.Fx = forceMagnitude*cosd(forceDirection);
        pCase.Fy = forceMagnitude*sind(forceDirection);

        %% Initial Cartesian state

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

        %% Solve equations of motion

        fprintf('\nCase %d of %d\n', caseNumber, numberOfCases);

        fprintf('Initial azimuth: %.1f deg\n', initialAzimuth);
        fprintf('Force direction: %.1f deg\n', forceDirection);

        fprintf('Fx = %.6f N, Fy = %.6f N\n', ...
            pCase.Fx, pCase.Fy);

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

        %% Cart displacement

        cartRadial = sqrt( ...
            (x-pCase.x0).^2 + (y-pCase.y0).^2);

        cartSpeed = sqrt(dx.^2 + dy.^2);

        %% Force-aligned coordinates

        forceAngle = deg2rad(forceDirection);

        forceParallel = ...
            (x-pCase.x0)*cos(forceAngle) + ...
            (y-pCase.y0)*sin(forceAngle);

        forcePerpendicular = ...
            -(x-pCase.x0)*sin(forceAngle) + ...
            (y-pCase.y0)*cos(forceAngle);

        %% Pendulum relative position

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

        %% Energy calculations

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

        %% Relative azimuth between force and pendulum

        relativeAzimuth = mod( ...
            forceDirection-initialAzimuth, 360);

        %% Store simulation data

        results(caseNumber).initial_azimuth = initialAzimuth;
        results(caseNumber).force_direction = forceDirection;
        results(caseNumber).relative_azimuth = relativeAzimuth;

        results(caseNumber).force_magnitude = forceMagnitude;
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

        results(caseNumber).force_parallel = forceParallel;
        results(caseNumber).force_perpendicular = forcePerpendicular;

        results(caseNumber).inclination = inclination;
        results(caseNumber).bob_displacement = bobDisplacement;

        results(caseNumber).cart_KE = cartKE;
        results(caseNumber).bob_KE = bobKE;
        results(caseNumber).potential_energy = potentialEnergy;
        results(caseNumber).total_energy = totalEnergy;

        results(caseNumber).external_work = externalWork;
        results(caseNumber).energy_residual = energyResidual;

        %% Calculate metrics

        results(caseNumber).max_cart_displacement = max(cartRadial);

        results(caseNumber).max_force_parallel = max(abs(forceParallel));

        results(caseNumber).max_force_perpendicular = ...
            max(abs(forcePerpendicular));

        results(caseNumber).final_force_parallel = forceParallel(end);

        results(caseNumber).final_force_perpendicular = ...
            forcePerpendicular(end);

        results(caseNumber).max_cart_speed = max(cartSpeed);

        results(caseNumber).max_inclination = max(inclination);

        results(caseNumber).final_inclination = inclination(end);

        results(caseNumber).max_bob_displacement = max(bobDisplacement);

        results(caseNumber).final_external_work = externalWork(end);

        results(caseNumber).max_energy_residual = max(abs(energyResidual));

        results(caseNumber).max_rod_error = max(rodError);

        %% Display case results

        fprintf('Relative azimuth: %.1f deg\n', relativeAzimuth);

        fprintf('Maximum radial displacement: %.6f m\n', ...
            results(caseNumber).max_cart_displacement);

        fprintf('Maximum force-parallel displacement: %.6f m\n', ...
            results(caseNumber).max_force_parallel);

        fprintf('Maximum force-perpendicular displacement: %.6f m\n', ...
            results(caseNumber).max_force_perpendicular);

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

%% 6. SUMMARY TABLE

initialAzimuthColumn = [results.initial_azimuth]';
forceDirectionColumn = [results.force_direction]';
relativeAzimuthColumn = [results.relative_azimuth]';

FxColumn = [results.Fx]';
FyColumn = [results.Fy]';

maxRadialDisplacement = [results.max_cart_displacement]';

maxParallelDisplacement = [results.max_force_parallel]';
maxPerpendicularDisplacement = [results.max_force_perpendicular]';

finalParallelDisplacement = [results.final_force_parallel]';
finalPerpendicularDisplacement = [results.final_force_perpendicular]';

maxCartSpeed = [results.max_cart_speed]';

maxInclination = [results.max_inclination]';
finalInclination = [results.final_inclination]';

maxBobDisplacement = [results.max_bob_displacement]';

finalExternalWork = [results.final_external_work]';

energyError = [results.max_energy_residual]';
rodError = [results.max_rod_error]';

summaryTable = table( ...
    initialAzimuthColumn, ...
    forceDirectionColumn, ...
    relativeAzimuthColumn, ...
    FxColumn, ...
    FyColumn, ...
    maxRadialDisplacement, ...
    maxParallelDisplacement, ...
    maxPerpendicularDisplacement, ...
    finalParallelDisplacement, ...
    finalPerpendicularDisplacement, ...
    maxCartSpeed, ...
    maxInclination, ...
    finalInclination, ...
    maxBobDisplacement, ...
    finalExternalWork, ...
    energyError, ...
    rodError, ...
    'VariableNames', { ...
    'Initial_Azimuth_deg', ...
    'Force_Direction_deg', ...
    'Relative_Azimuth_deg', ...
    'Fx_N', ...
    'Fy_N', ...
    'Max_Radial_Displacement_m', ...
    'Max_Parallel_Displacement_m', ...
    'Max_Perpendicular_Displacement_m', ...
    'Final_Parallel_Displacement_m', ...
    'Final_Perpendicular_Displacement_m', ...
    'Max_Cart_Speed_mps', ...
    'Max_Inclination_deg', ...
    'Final_Inclination_deg', ...
    'Max_Bob_Displacement_m', ...
    'Final_External_Work_J', ...
    'Max_Energy_Residual_J', ...
    'Max_Rod_Error_m'});

fprintf('\n============================================\n');
fprintf('AZIMUTH-FORCE INTERACTION SUMMARY\n');
fprintf('============================================\n');

disp(summaryTable);

%% 7. CREATE RESPONSE MATRICES

% Rows: initial pendulum azimuth
% Columns: applied force direction

radialMatrix = zeros(numberOfAzimuths, numberOfDirections);
parallelMatrix = zeros(numberOfAzimuths, numberOfDirections);
perpendicularMatrix = zeros(numberOfAzimuths, numberOfDirections);
speedMatrix = zeros(numberOfAzimuths, numberOfDirections);
inclinationMatrix = zeros(numberOfAzimuths, numberOfDirections);

for i = 1:numberOfAzimuths

    for j = 1:numberOfDirections

        index = (i-1)*numberOfDirections+j;

        radialMatrix(i,j) = results(index).max_cart_displacement;

        parallelMatrix(i,j) = results(index).max_force_parallel;

        perpendicularMatrix(i,j) = ...
            results(index).max_force_perpendicular;

        speedMatrix(i,j) = results(index).max_cart_speed;

        inclinationMatrix(i,j) = results(index).max_inclination;

    end
end

%% 8. RADIAL DISPLACEMENT HEATMAP

figure('Name','Azimuth Force Radial Displacement');

imagesc(forceDirections, initialAzimuths, radialMatrix);

set(gca,'YDir','normal');

colorbar;
xlabel('Force direction (deg)');
ylabel('Initial pendulum azimuth (deg)');

title('Maximum Radial Cart Displacement (m)');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_15_Azimuth_Radial_Displacement.png'));

%% 9. FORCE-PARALLEL DISPLACEMENT HEATMAP

figure('Name','Force Parallel Displacement');

imagesc(forceDirections, initialAzimuths, parallelMatrix);

set(gca,'YDir','normal');

colorbar;
xlabel('Force direction (deg)');
ylabel('Initial pendulum azimuth (deg)');

title('Maximum Force-Parallel Displacement (m)');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_15_Azimuth_Parallel_Displacement.png'));

%% 10. FORCE-PERPENDICULAR DISPLACEMENT HEATMAP

figure('Name','Force Perpendicular Displacement');

imagesc(forceDirections, initialAzimuths, perpendicularMatrix);

set(gca,'YDir','normal');

colorbar;
xlabel('Force direction (deg)');
ylabel('Initial pendulum azimuth (deg)');

title('Maximum Force-Perpendicular Displacement (m)');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_15_Azimuth_Perpendicular_Displacement.png'));

%% 11. CART SPEED HEATMAP

figure('Name','Azimuth Force Cart Speed');

imagesc(forceDirections, initialAzimuths, speedMatrix);

set(gca,'YDir','normal');

colorbar;
xlabel('Force direction (deg)');
ylabel('Initial pendulum azimuth (deg)');

title('Maximum Cart Speed (m/s)');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_15_Azimuth_Cart_Speed.png'));

%% 12. PENDULUM INCLINATION HEATMAP

figure('Name','Azimuth Force Inclination');

imagesc(forceDirections, initialAzimuths, inclinationMatrix);

set(gca,'YDir','normal');

colorbar;
xlabel('Force direction (deg)');
ylabel('Initial pendulum azimuth (deg)');

title('Maximum Pendulum Inclination (deg)');

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_15_Azimuth_Inclination.png'));

%% 13. RELATIVE AZIMUTH COMPARISON

relativeAzimuthValues = [results.relative_azimuth]';

uniqueRelativeAzimuths = unique(relativeAzimuthValues);

relativeResponse = zeros(numel(uniqueRelativeAzimuths),1);
relativeSpeed = zeros(numel(uniqueRelativeAzimuths),1);

for k = 1:numel(uniqueRelativeAzimuths)

    indices = find(relativeAzimuthValues == uniqueRelativeAzimuths(k));

    relativeResponse(k) = mean(maxParallelDisplacement(indices));

    relativeSpeed(k) = mean(maxCartSpeed(indices));

end

relativeSummary = table( ...
    uniqueRelativeAzimuths, ...
    relativeResponse, ...
    relativeSpeed, ...
    'VariableNames', { ...
    'Relative_Azimuth_deg', ...
    'Mean_Max_Parallel_Displacement_m', ...
    'Mean_Max_Cart_Speed_mps'});

fprintf('\n============================================\n');
fprintf('RELATIVE AZIMUTH RESPONSE\n');
fprintf('============================================\n');

disp(relativeSummary);

figure('Name','Relative Azimuth Response');

yyaxis left;

plot(uniqueRelativeAzimuths, relativeResponse, ...
    '-o','LineWidth',1.5);

ylabel('Mean maximum parallel displacement (m)');

yyaxis right;

plot(uniqueRelativeAzimuths, relativeSpeed, ...
    '-s','LineWidth',1.5);

ylabel('Mean maximum cart speed (m/s)');

xlabel('Relative azimuth (deg)');
title('System Response vs Relative Azimuth');
grid on;

saveas(gcf, fullfile(resultsFolder, ...
    'Phase_9_15_Relative_Azimuth_Response.png'));

%% 14. VALIDATION

maximumEnergyError = max(energyError);
maximumRodError = max(rodError);

fprintf('\n============================================\n');
fprintf('PHASE 9.15 VALIDATION\n');
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

%% 15. SAVE RESULTS

save(fullfile(resultsFolder, ...
    'phase_9_15_azimuth_force_results.mat'), ...
    'results', ...
    'summaryTable', ...
    'relativeSummary', ...
    'radialMatrix', ...
    'parallelMatrix', ...
    'perpendicularMatrix', ...
    'speedMatrix', ...
    'inclinationMatrix', ...
    'initialAzimuths', ...
    'forceDirections', ...
    'forceMagnitude', ...
    'p', ...
    'timeVector');

writetable(summaryTable, fullfile(resultsFolder, ...
    'Phase_9_15_Azimuth_Force_Summary.csv'));

writetable(relativeSummary, fullfile(resultsFolder, ...
    'Phase_9_15_Relative_Azimuth_Summary.csv'));

fprintf('\n============================================\n');
fprintf('PHASE 9.15 ANALYSIS COMPLETED\n');
fprintf('============================================\n');

fprintf('\nResults directory:\n%s\n', resultsFolder);

fprintf('\nSaved files:\n');

fprintf('1. phase_9_15_azimuth_force_results.mat\n');
fprintf('2. Phase_9_15_Azimuth_Force_Summary.csv\n');
fprintf('3. Phase_9_15_Relative_Azimuth_Summary.csv\n');

fprintf('\nGenerated figures:\n');

fprintf('1. Phase_9_15_Azimuth_Radial_Displacement.png\n');
fprintf('2. Phase_9_15_Azimuth_Parallel_Displacement.png\n');
fprintf('3. Phase_9_15_Azimuth_Perpendicular_Displacement.png\n');
fprintf('4. Phase_9_15_Azimuth_Cart_Speed.png\n');
fprintf('5. Phase_9_15_Azimuth_Inclination.png\n');
fprintf('6. Phase_9_15_Relative_Azimuth_Response.png\n');

fprintf('\n============================================\n');
fprintf('PHASE 9.15 SCRIPT FINISHED\n');
fprintf('============================================\n');
