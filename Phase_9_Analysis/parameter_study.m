
%% PHASE 9.5: PARAMETER SENSITIVITY ANALYSIS
% 3D X-Y Cart with an Inverted Spherical Pendulum
%
% Uses the existing validated Cartesian equations of motion.
% Each experiment changes one parameter at a time.

clc;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 9.5: PARAMETER SENSITIVITY ANALYSIS\n');
fprintf('============================================\n');

%% 1. Project paths

scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);

phase4Dir = fullfile(projectDir, 'Phase_4_MATLAB_Simulation');

addpath(phase4Dir);

resultsDir = fullfile(projectDir, 'Results', 'Phase_9');

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

%% 2. Load baseline parameters

p0 = system_parameters();

fprintf('\n--- BASELINE PARAMETERS ---\n');

fprintf('Cart mass: %.3f kg\n', p0.M);
fprintf('Bob mass: %.3f kg\n', p0.m);
fprintf('Rod length: %.3f m\n', p0.L);
fprintf('Gravity: %.3f m/s^2\n', p0.g);

fprintf('Initial inclination: %.3f degrees\n', ...
    rad2deg(p0.theta0));

fprintf('Initial azimuth: %.3f degrees\n', ...
    rad2deg(p0.phi0));

%% 3. Parameter test values

parameterNames = {'Cart Mass'; 'Bob Mass'; ...
                  'Rod Length'; 'Gravity'};

parameterSymbols = {'M'; 'm'; 'L'; 'g'};

parameterValues = {
    [1.0 1.5 2.0 2.5 3.0]
    [0.25 0.375 0.5 0.625 0.75]
    [0.5 0.75 1.0 1.25 1.5]
    [7.81 8.81 9.81 10.81 11.81]
    };

baselineValues = [p0.M p0.m p0.L p0.g];

%% 4. Simulation settings

% Use the same simulation duration as the baseline project.
tStart = p0.t_start;
tEnd = p0.t_end;

sampleTime = p0.sample_time;

timeVector = tStart:sampleTime:tEnd;

if timeVector(end) < tEnd
    timeVector = [timeVector tEnd];
end

options = odeset( ...
    'RelTol', p0.relative_tolerance, ...
    'AbsTol', p0.absolute_tolerance, ...
    'MaxStep', sampleTime);

%% 5. Initial Cartesian state

theta0 = p0.theta0;
phi0 = p0.phi0;

x0 = p0.x0;
y0 = p0.y0;

X0 = x0 + p0.L*sin(theta0)*cos(phi0);
Y0 = y0 + p0.L*sin(theta0)*sin(phi0);
Z0 = p0.L*cos(theta0);

dx0 = p0.dx0;
dy0 = p0.dy0;

dtheta0 = p0.dtheta0;
dphi0 = p0.dphi0;

dX0 = dx0 + p0.L*( ...
    cos(theta0)*cos(phi0)*dtheta0 ...
    - sin(theta0)*sin(phi0)*dphi0);

dY0 = dy0 + p0.L*( ...
    cos(theta0)*sin(phi0)*dtheta0 ...
    + sin(theta0)*cos(phi0)*dphi0);

dZ0 = -p0.L*sin(theta0)*dtheta0;

z0 = [x0; y0; X0; Y0; Z0; ...
      dx0; dy0; dX0; dY0; dZ0];

%% 6. Allocate result storage

numParameters = numel(parameterNames);

numCases = numel(parameterValues{1});

% Columns:
% 1: maximum inclination
% 2: final inclination
% 3: maximum cart X displacement
% 4: maximum cart Y displacement
% 5: maximum cart radial displacement
% 6: maximum bob displacement from initial position
% 7: maximum rod length error

metrics = zeros(numParameters, numCases, 7);

simulationTimes = zeros(numParameters, numCases);

%% 7. Run parameter sensitivity experiments

for i = 1:numParameters

    fprintf('\n============================================\n');
    fprintf('PARAMETER: %s (%s)\n', ...
        parameterNames{i}, parameterSymbols{i});
    fprintf('============================================\n');

    values = parameterValues{i};

    for j = 1:numel(values)

        p = p0;

        % Modify exactly one parameter.
        switch i
            case 1
                p.M = values(j);
            case 2
                p.m = values(j);
            case 3
                p.L = values(j);
            case 4
                p.g = values(j);
        end

        % Recalculate the initial bob position using the
        % modified rod length, preserving initial angles.
        X = p.x0 + p.L*sin(p.theta0)*cos(p.phi0);
        Y = p.y0 + p.L*sin(p.theta0)*sin(p.phi0);
        Z = p.L*cos(p.theta0);

        dX = p.dx0 + p.L*( ...
            cos(p.theta0)*cos(p.phi0)*p.dtheta0 ...
            - sin(p.theta0)*sin(p.phi0)*p.dphi0);

        dY = p.dy0 + p.L*( ...
            cos(p.theta0)*sin(p.phi0)*p.dtheta0 ...
            + sin(p.theta0)*cos(p.phi0)*p.dphi0);

        dZ = -p.L*sin(p.theta0)*p.dtheta0;

        zInitial = [p.x0; p.y0; X; Y; Z; ...
                    p.dx0; p.dy0; dX; dY; dZ];

        fprintf('\nRunning %s = %.4f ...\n', ...
            parameterSymbols{i}, values(j));

        tic;

        [t, state] = ode45( ...
            @(t,z) cartesian_equations_of_motion(t,z,p), ...
            timeVector, zInitial, options);

        simulationTimes(i,j) = toc;

        %% Extract positions

        cartX = state(:,1);
        cartY = state(:,2);

        bobX = state(:,3);
        bobY = state(:,4);
        bobZ = state(:,5);

        %% Pendulum inclination

        rx = bobX - cartX;
        ry = bobY - cartY;
        rz = bobZ;

        rodLength = sqrt(rx.^2 + ry.^2 + rz.^2);

        theta = acos(max(-1, min(1, rz/p.L)));

        %% Calculate displacement metrics

        cartRadial = sqrt( ...
            (cartX-p.x0).^2 + (cartY-p.y0).^2);

        bobInitial = [X; Y; Z];

        bobDisplacement = sqrt( ...
            (bobX-bobInitial(1)).^2 + ...
            (bobY-bobInitial(2)).^2 + ...
            (bobZ-bobInitial(3)).^2);

        %% Store metrics

        metrics(i,j,1) = max(rad2deg(theta));
        metrics(i,j,2) = rad2deg(theta(end));

        metrics(i,j,3) = max(abs(cartX-p.x0));
        metrics(i,j,4) = max(abs(cartY-p.y0));

        metrics(i,j,5) = max(cartRadial);

        metrics(i,j,6) = max(bobDisplacement);

        metrics(i,j,7) = max(abs(rodLength-p.L));

        fprintf('  Maximum inclination: %.6f deg\n', ...
            metrics(i,j,1));

        fprintf('  Final inclination:   %.6f deg\n', ...
            metrics(i,j,2));

        fprintf('  Maximum cart motion: %.6f m\n', ...
            metrics(i,j,5));

        fprintf('  Maximum rod error:   %.3e m\n', ...
            metrics(i,j,7));

    end
end

%% 8. Display numerical results

fprintf('\n============================================\n');
fprintf('PARAMETER SENSITIVITY SUMMARY\n');
fprintf('============================================\n');

for i = 1:numParameters

    fprintf('\n%s (%s)\n', ...
        parameterNames{i}, parameterSymbols{i});

    fprintf('%12s %18s %18s %18s\n', ...
        'Value', 'Max Tilt (deg)', ...
        'Final Tilt (deg)', 'Cart Motion (m)');

    for j = 1:numel(parameterValues{i})

        fprintf('%12.4f %18.6f %18.6f %18.6f\n', ...
            parameterValues{i}(j), ...
            metrics(i,j,1), ...
            metrics(i,j,2), ...
            metrics(i,j,5));

    end
end

%% 9. Plot parameter sensitivity

metricNames = {
    'Maximum Inclination (deg)'
    'Final Inclination (deg)'
    'Maximum Cart Radial Displacement (m)'
    'Maximum Bob Displacement (m)'
    };

metricIndices = [1 2 5 6];

for i = 1:numParameters

    figure('Color','w', ...
        'Name',['Sensitivity - ' parameterNames{i}]);

    for k = 1:numel(metricIndices)

        subplot(2,2,k);

        plot(parameterValues{i}, ...
            squeeze(metrics(i,:,metricIndices(k))), ...
            '-o', 'LineWidth',1.5, 'MarkerSize',6);

        hold on;

        xlabel([parameterSymbols{i} ' value']);
        ylabel(metricNames{k});

        title([parameterNames{i} ': ' metricNames{k}]);

        grid on;

    end

    sgtitle(['Parameter Sensitivity: ' parameterNames{i}]);

    saveas(gcf, fullfile(resultsDir, ...
        ['Phase_9_Sensitivity_' parameterSymbols{i} '.png']));

end

%% 10. Save results

sensitivity.parameter_names = parameterNames;
sensitivity.parameter_symbols = parameterSymbols;

sensitivity.parameter_values = parameterValues;
sensitivity.baseline_values = baselineValues;

sensitivity.metrics = metrics;
sensitivity.simulation_times = simulationTimes;

sensitivity.metric_names = {
    'Maximum inclination (degrees)'
    'Final inclination (degrees)'
    'Maximum X cart displacement (m)'
    'Maximum Y cart displacement (m)'
    'Maximum radial cart displacement (m)'
    'Maximum bob displacement (m)'
    'Maximum rod length error (m)'
    };

sensitivity.baseline_parameters = p0;

sensitivity.simulation_duration = tEnd-tStart;

save(fullfile(resultsDir, ...
    'phase_9_parameter_sensitivity.mat'), ...
    'sensitivity');

%% 11. Final verification

fprintf('\n============================================\n');
fprintf('PHASE 9.5 COMPLETION SUMMARY\n');
fprintf('============================================\n');

fprintf('Parameters investigated: %d\n', numParameters);

fprintf('Total simulations: %d\n', ...
    sum(cellfun(@numel, parameterValues)));

fprintf('Simulation duration: %.2f seconds\n', tEnd-tStart);

fprintf('Maximum rod constraint error: %.6e m\n', ...
    max(metrics(:,:,7), [], 'all'));

fprintf('\nResults directory:\n%s\n', resultsDir);

fprintf('\nSaved file:\n');
fprintf('phase_9_parameter_sensitivity.mat\n');

fprintf('\n============================================\n');
fprintf('PHASE 9.5 ANALYSIS COMPLETED\n');
fprintf('============================================\n');
