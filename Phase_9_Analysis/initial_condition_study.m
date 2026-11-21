
%% PHASE 9.7: INITIAL CONDITION SENSITIVITY ANALYSIS
%
% 3D X-Y Cart with an Inverted Spherical Pendulum
%
% Investigates the effects of:
% 1. Initial inclination
% 2. Initial azimuth
% 3. Initial cart X velocity
% 4. Initial cart Y velocity
% 5. Initial pendulum inclination rate
% 6. Initial pendulum azimuth rate
%
% Uses the validated 10-state Cartesian equations of motion.

clc;
close all;

fprintf('\n');
fprintf('============================================\n');
fprintf('PHASE 9.7: INITIAL CONDITION SENSITIVITY\n');
fprintf('============================================\n');

%% 1. PROJECT PATHS

scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);

phase4Dir = fullfile(projectDir, ...
    'Phase_4_MATLAB_Simulation');

resultsDir = fullfile(projectDir, ...
    'Results', 'Phase_9');

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

addpath(phase4Dir);

%% 2. BASELINE PARAMETERS

p = system_parameters();

fprintf('\n--- BASELINE PARAMETERS ---\n');
fprintf('Cart mass: %.4f kg\n', p.M);
fprintf('Bob mass: %.4f kg\n', p.m);
fprintf('Rod length: %.4f m\n', p.L);
fprintf('Gravity: %.4f m/s^2\n', p.g);
fprintf('Baseline inclination: %.4f deg\n', ...
    rad2deg(p.theta0));
fprintf('Baseline azimuth: %.4f deg\n', ...
    rad2deg(p.phi0));

%% 3. SIMULATION SETTINGS

time = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol', p.relative_tolerance, ...
    'AbsTol', p.absolute_tolerance, ...
    'MaxStep', p.sample_time);

% Each experiment varies one initial condition.
% All other initial conditions remain at baseline.

experiments = struct([]);

experiments(1).name = 'Initial Inclination';
experiments(1).field = 'theta0';
experiments(1).values = deg2rad([1 3 5 7 10]);
experiments(1).unit = 'degrees';
experiments(1).displayValues = [1 3 5 7 10];

experiments(2).name = 'Initial Azimuth';
experiments(2).field = 'phi0';
experiments(2).values = deg2rad([0 30 60 90 120]);
experiments(2).unit = 'degrees';
experiments(2).displayValues = [0 30 60 90 120];

experiments(3).name = 'Initial Cart X Velocity';
experiments(3).field = 'dx0';
experiments(3).values = [-0.2 -0.1 0 0.1 0.2];
experiments(3).unit = 'm/s';
experiments(3).displayValues = experiments(3).values;

experiments(4).name = 'Initial Cart Y Velocity';
experiments(4).field = 'dy0';
experiments(4).values = [-0.2 -0.1 0 0.1 0.2];
experiments(4).unit = 'm/s';
experiments(4).displayValues = experiments(4).values;

experiments(5).name = 'Initial Inclination Rate';
experiments(5).field = 'dtheta0';
experiments(5).values = [-0.5 -0.25 0 0.25 0.5];
experiments(5).unit = 'rad/s';
experiments(5).displayValues = experiments(5).values;

experiments(6).name = 'Initial Azimuth Rate';
experiments(6).field = 'dphi0';
experiments(6).values = [-0.5 -0.25 0 0.25 0.5];
experiments(6).unit = 'rad/s';
experiments(6).displayValues = experiments(6).values;

numExperiments = numel(experiments);
numCases = numel(experiments(1).values);

%% 4. RESULT STORAGE

% Metrics:
% 1. Maximum inclination (degrees)
% 2. Final inclination (degrees)
% 3. Maximum radial cart displacement (m)
% 4. Maximum X cart displacement (m)
% 5. Maximum Y cart displacement (m)
% 6. Maximum bob displacement from initial position (m)
% 7. Maximum rod constraint error (m)
% 8. Maximum absolute energy drift (J)

metricNames = {
    'Maximum inclination'
    'Final inclination'
    'Maximum radial cart displacement'
    'Maximum X cart displacement'
    'Maximum Y cart displacement'
    'Maximum bob displacement'
    'Maximum rod constraint error'
    'Maximum energy drift'
};

numMetrics = numel(metricNames);

metrics = zeros(numExperiments, numCases, numMetrics);

allResults = struct([]);

%% 5. RUN INITIAL CONDITION EXPERIMENTS

for e = 1:numExperiments

    fprintf('\n');
    fprintf('============================================\n');
    fprintf('EXPERIMENT: %s\n', experiments(e).name);
    fprintf('============================================\n');

    for k = 1:numCases

        % Copy baseline parameters
        pk = p;

        % Change only the selected initial condition
        pk.(experiments(e).field) = ...
            experiments(e).values(k);

        theta = pk.theta0;
        phi = pk.phi0;

        % Initial cart position
        x0 = pk.x0;
        y0 = pk.y0;

        % Initial cart velocity
        dx0 = pk.dx0;
        dy0 = pk.dy0;

        % Initial bob position
        X0 = x0 + pk.L*sin(theta)*cos(phi);
        Y0 = y0 + pk.L*sin(theta)*sin(phi);
        Z0 = pk.L*cos(theta);

        % Derivative of bob position with respect to theta
        rTheta = pk.L * [
            cos(theta)*cos(phi);
            cos(theta)*sin(phi);
            -sin(theta)
        ];

        % Derivative of bob position with respect to phi
        rPhi = pk.L * [
            -sin(theta)*sin(phi);
             sin(theta)*cos(phi);
             0
        ];

        % Absolute bob velocity:
        % Cart translational velocity plus pendulum-relative velocity
        bobVelocity = [
            dx0;
            dy0;
            0
        ] + pk.dtheta0*rTheta + pk.dphi0*rPhi;

        % Cartesian initial state:
        % [x; y; X; Y; Z; dx; dy; dX; dY; dZ]

        z0 = [
            x0;
            y0;
            X0;
            Y0;
            Z0;
            dx0;
            dy0;
            bobVelocity
        ];

        fprintf('\nRunning %s = %.6f %s ...\n', ...
            experiments(e).field, ...
            experiments(e).displayValues(k), ...
            experiments(e).unit);

        %% Numerical simulation

        [~, states] = ode45( ...
            @(t,z) cartesian_equations_of_motion(t,z,pk), ...
            time, z0, options);

        %% Position calculations

        cartX = states(:,1) - x0;
        cartY = states(:,2) - y0;

        cartRadius = sqrt(cartX.^2 + cartY.^2);

        bobDisplacement = sqrt( ...
            (states(:,3)-X0).^2 + ...
            (states(:,4)-Y0).^2 + ...
            (states(:,5)-Z0).^2);

        %% Pendulum inclination

        relativeZ = states(:,5) - states(:,2)*0;

        inclination = acosd(max(-1, min(1, ...
            relativeZ/pk.L)));

        %% Rod constraint

        rodLength = sqrt( ...
            (states(:,3)-states(:,1)).^2 + ...
            (states(:,4)-states(:,2)).^2 + ...
            states(:,5).^2);

        rodError = abs(rodLength - pk.L);

        %% Mechanical energy

        cartKineticEnergy = 0.5*pk.M*( ...
            states(:,6).^2 + states(:,7).^2);

        bobKineticEnergy = 0.5*pk.m*( ...
            states(:,8).^2 + ...
            states(:,9).^2 + ...
            states(:,10).^2);

        potentialEnergy = pk.m*pk.g*states(:,5);

        totalEnergy = ...
            cartKineticEnergy + ...
            bobKineticEnergy + ...
            potentialEnergy;

        energyDrift = abs(totalEnergy - totalEnergy(1));

        %% Store metrics

        metrics(e,k,1) = max(inclination);
        metrics(e,k,2) = inclination(end);
        metrics(e,k,3) = max(cartRadius);
        metrics(e,k,4) = max(abs(cartX));
        metrics(e,k,5) = max(abs(cartY));
        metrics(e,k,6) = max(bobDisplacement);
        metrics(e,k,7) = max(rodError);
        metrics(e,k,8) = max(energyDrift);

        %% Store full simulation data

        allResults(e).name = experiments(e).name;
        allResults(e).field = experiments(e).field;
        allResults(e).values = experiments(e).displayValues;

        allResults(e).states(:,:,k) = states;
        allResults(e).energy(:,k) = totalEnergy;
        allResults(e).inclination(:,k) = inclination;

        fprintf('  Maximum inclination: %.6f deg\n', ...
            metrics(e,k,1));

        fprintf('  Final inclination:   %.6f deg\n', ...
            metrics(e,k,2));

        fprintf('  Maximum cart radius: %.8f m\n', ...
            metrics(e,k,3));

        fprintf('  Maximum rod error:   %.3e m\n', ...
            metrics(e,k,7));

        fprintf('  Maximum energy drift: %.3e J\n', ...
            metrics(e,k,8));

    end

end

%% 6. NUMERICAL SUMMARY

fprintf('\n');
fprintf('============================================\n');
fprintf('INITIAL CONDITION SENSITIVITY SUMMARY\n');
fprintf('============================================\n');

for e = 1:numExperiments

    fprintf('\n%s\n', experiments(e).name);

    fprintf('%12s %14s %14s %14s\n', ...
        'Value', 'Max Tilt', 'Final Tilt', 'Cart Radius');

    for k = 1:numCases

        fprintf('%12.5f %14.6f %14.6f %14.8f\n', ...
            experiments(e).displayValues(k), ...
            metrics(e,k,1), ...
            metrics(e,k,2), ...
            metrics(e,k,3));

    end

end

%% 7. VALIDATION SUMMARY

maximumRodError = max(metrics(:,:,7), [], 'all');
maximumEnergyDrift = max(metrics(:,:,8), [], 'all');

fprintf('\n');
fprintf('============================================\n');
fprintf('NUMERICAL VALIDATION\n');
fprintf('============================================\n');

fprintf('Maximum rod constraint error: %.6e m\n', ...
    maximumRodError);

fprintf('Maximum absolute energy drift: %.6e J\n', ...
    maximumEnergyDrift);

if maximumRodError < 1e-7
    fprintf('Rigid rod constraint: PASSED\n');
else
    fprintf('Rigid rod constraint: REVIEW REQUIRED\n');
end

if maximumEnergyDrift < 1e-5
    fprintf('Energy conservation: PASSED\n');
else
    fprintf('Energy conservation: REVIEW REQUIRED\n');
end

%% 8. GENERATE FIGURES

for e = 1:numExperiments

    figure('Name', ...
        ['Phase 9.7 - ' experiments(e).name], ...
        'Color', 'w');

    tiledlayout(2,2);

    nexttile;

    plot(experiments(e).displayValues, ...
        squeeze(metrics(e,:,1)), '-o', ...
        'LineWidth', 1.5);

    grid on;
    xlabel(experiments(e).unit);
    ylabel('Maximum inclination (deg)');
    title('Maximum inclination');

    nexttile;

    plot(experiments(e).displayValues, ...
        squeeze(metrics(e,:,3)), '-o', ...
        'LineWidth', 1.5);

    grid on;
    xlabel(experiments(e).unit);
    ylabel('Maximum cart radius (m)');
    title('Maximum cart displacement');

    nexttile;

    plot(experiments(e).displayValues, ...
        squeeze(metrics(e,:,6)), '-o', ...
        'LineWidth', 1.5);

    grid on;
    xlabel(experiments(e).unit);
    ylabel('Maximum bob displacement (m)');
    title('Maximum bob displacement');

    nexttile;

    semilogy(experiments(e).displayValues, ...
        max(squeeze(metrics(e,:,8)), eps), '-o', ...
        'LineWidth', 1.5);

    grid on;
    xlabel(experiments(e).unit);
    ylabel('Maximum energy drift (J)');
    title('Energy conservation error');

    sgtitle(['Initial Condition Study: ' ...
        experiments(e).name]);

    figureName = sprintf( ...
        'Phase_9_7_Initial_Condition_%02d.png', e);

    saveas(gcf, fullfile(resultsDir, figureName));

end

%% 9. SAVE RESULTS

phase9_7_results.time = time;
phase9_7_results.experiments = experiments;
phase9_7_results.metricNames = metricNames;
phase9_7_results.metrics = metrics;
phase9_7_results.allResults = allResults;
phase9_7_results.maximumRodError = maximumRodError;
phase9_7_results.maximumEnergyDrift = maximumEnergyDrift;

save(fullfile(resultsDir, ...
    'phase_9_7_initial_condition_results.mat'), ...
    'phase9_7_results', '-v7.3');

%% 10. COMPLETION SUMMARY

fprintf('\n');
fprintf('============================================\n');
fprintf('PHASE 9.7 COMPLETION SUMMARY\n');
fprintf('============================================\n');

fprintf('Initial condition groups: %d\n', numExperiments);
fprintf('Cases per group: %d\n', numCases);
fprintf('Total simulations: %d\n', ...
    numExperiments*numCases);

fprintf('Simulation duration: %.2f seconds\n', ...
    p.t_end-p.t_start);

fprintf('Maximum rod constraint error: %.6e m\n', ...
    maximumRodError);

fprintf('Maximum energy drift: %.6e J\n', ...
    maximumEnergyDrift);

fprintf('\nResults directory:\n%s\n', resultsDir);

fprintf('\nSaved file:\n');
fprintf('phase_9_7_initial_condition_results.mat\n');

fprintf('\nGenerated figures:\n');

for e = 1:numExperiments
    fprintf('%d. Phase_9_7_Initial_Condition_%02d.png\n', e, e);
end

fprintf('\n============================================\n');
fprintf('PHASE 9.7 ANALYSIS COMPLETED\n');
fprintf('============================================\n');
