
%% PHASE 9.9: UNCONTROLLED SYSTEM STABILITY ANALYSIS
%
% Project:
% 3D X-Y Cart with an Inverted Spherical Pendulum
%
% Objective:
% Investigate the stability of the upright equilibrium by
% comparing trajectories with different initial inclinations.
%
% Model:
% Validated 10-state Cartesian equations of motion.

clc;
close all;

fprintf('\n');
fprintf('============================================\n');
fprintf('PHASE 9.9: UNCONTROLLED STABILITY ANALYSIS\n');
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

%% 2. LOAD BASELINE PARAMETERS

p = system_parameters();

fprintf('\n--- BASELINE PARAMETERS ---\n');
fprintf('Cart mass: %.4f kg\n', p.M);
fprintf('Bob mass: %.4f kg\n', p.m);
fprintf('Rod length: %.4f m\n', p.L);
fprintf('Gravity: %.4f m/s^2\n', p.g);
fprintf('Azimuth: %.4f deg\n', rad2deg(p.phi0));

%% 3. SIMULATION SETTINGS

initialAnglesDeg = [0, 0.1, 0.5, 1, 2, 5];

numberOfCases = length(initialAnglesDeg);

timeVector = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol', p.relative_tolerance, ...
    'AbsTol', p.absolute_tolerance, ...
    'MaxStep', p.sample_time);

% Inclination thresholds to investigate
thresholdsDeg = [10, 45, 90, 135, 170];

numberOfThresholds = length(thresholdsDeg);

%% 4. INITIALIZE RESULT ARRAYS

allAngles = cell(numberOfCases, 1);
allStates = cell(numberOfCases, 1);

maximumInclination = zeros(numberOfCases, 1);
finalInclination = zeros(numberOfCases, 1);
maximumRadialCartDisplacement = zeros(numberOfCases, 1);
maximumCartXDisplacement = zeros(numberOfCases, 1);
maximumCartYDisplacement = zeros(numberOfCases, 1);
maximumRodError = zeros(numberOfCases, 1);

thresholdTimes = NaN(numberOfCases, numberOfThresholds);

%% 5. RUN SIMULATIONS

fprintf('\n');
fprintf('============================================\n');
fprintf('RUNNING INITIAL PERTURBATION CASES\n');
fprintf('============================================\n');

for k = 1:numberOfCases

    theta0 = deg2rad(initialAnglesDeg(k));
    phi0 = p.phi0;

    % Initial cart position
    x0 = p.x0;
    y0 = p.y0;

    % Initial bob position
    X0 = x0 + p.L*sin(theta0)*cos(phi0);
    Y0 = y0 + p.L*sin(theta0)*sin(phi0);
    Z0 = p.L*cos(theta0);

    % Initial cart and bob velocities are zero.
    z0 = [
        x0;
        y0;
        X0;
        Y0;
        Z0;
        0;
        0;
        0;
        0;
        0
    ];

    fprintf('\nCase %d: Initial inclination = %.2f deg\n', ...
        k, initialAnglesDeg(k));

    [t, states] = ode45( ...
        @(t,z) cartesian_equations_of_motion(t,z,p), ...
        timeVector, z0, options);

    % Relative bob position
    rx = states(:,3) - states(:,1);
    ry = states(:,4) - states(:,2);
    rz = states(:,5);

    % Actual pendulum inclination from vertical upright
    cosTheta = rz/p.L;
    cosTheta = max(-1, min(1, cosTheta));

    theta = acos(cosTheta);
    thetaDeg = rad2deg(theta);

    % Cart displacement relative to its initial position
    cartX = states(:,1) - x0;
    cartY = states(:,2) - y0;

    radialCartDisplacement = sqrt(cartX.^2 + cartY.^2);

    % Rigid rod constraint error
    rodLength = sqrt(rx.^2 + ry.^2 + rz.^2);
    rodError = abs(rodLength - p.L);

    % Store trajectories
    allAngles{k} = thetaDeg;
    allStates{k} = states;

    % Calculate metrics
    maximumInclination(k) = max(thetaDeg);
    finalInclination(k) = thetaDeg(end);

    maximumRadialCartDisplacement(k) = ...
        max(radialCartDisplacement);

    maximumCartXDisplacement(k) = max(abs(cartX));
    maximumCartYDisplacement(k) = max(abs(cartY));

    maximumRodError(k) = max(rodError);

    % First time each inclination threshold is reached
    for j = 1:numberOfThresholds

        index = find(thetaDeg >= thresholdsDeg(j), 1, 'first');

        if ~isempty(index)
            thresholdTimes(k,j) = t(index);
        end

    end

    fprintf('Maximum inclination: %.6f deg\n', ...
        maximumInclination(k));

    fprintf('Final inclination: %.6f deg\n', ...
        finalInclination(k));

    fprintf('Maximum radial cart displacement: %.8f m\n', ...
        maximumRadialCartDisplacement(k));

    fprintf('Maximum rod constraint error: %.6e m\n', ...
        maximumRodError(k));

end

%% 6. PRINT THRESHOLD CROSSING TIMES

fprintf('\n');
fprintf('============================================\n');
fprintf('INCLINATION THRESHOLD CROSSING TIMES\n');
fprintf('============================================\n');

fprintf('Times are first crossing times in seconds.\n');
fprintf('NaN means the threshold was not reached.\n\n');

fprintf('%12s', 'Initial deg');

for j = 1:numberOfThresholds
    fprintf('%12.0f deg', thresholdsDeg(j));
end

fprintf('\n');

for k = 1:numberOfCases

    fprintf('%12.2f', initialAnglesDeg(k));

    for j = 1:numberOfThresholds
        fprintf('%12.4f', thresholdTimes(k,j));
    end

    fprintf('\n');

end

%% 7. PRINT STABILITY METRICS

fprintf('\n');
fprintf('============================================\n');
fprintf('STABILITY METRICS\n');
fprintf('============================================\n');

fprintf('%12s %16s %16s %18s\n', ...
    'Initial deg', 'Maximum deg', 'Final deg', 'Max radial m');

for k = 1:numberOfCases

    fprintf('%12.2f %16.6f %16.6f %18.8f\n', ...
        initialAnglesDeg(k), ...
        maximumInclination(k), ...
        finalInclination(k), ...
        maximumRadialCartDisplacement(k));

end

%% 8. VALIDATE EXACT UPRIGHT EQUILIBRIUM

fprintf('\n');
fprintf('============================================\n');
fprintf('UPRIGHT EQUILIBRIUM VALIDATION\n');
fprintf('============================================\n');

uprightIndex = find(initialAnglesDeg == 0, 1);

uprightMaximumAngle = maximumInclination(uprightIndex);
uprightMaximumCartMotion = ...
    maximumRadialCartDisplacement(uprightIndex);

fprintf('Maximum inclination from upright: %.12f deg\n', ...
    uprightMaximumAngle);

fprintf('Maximum radial cart displacement: %.12e m\n', ...
    uprightMaximumCartMotion);

if uprightMaximumAngle < 1e-6 && ...
        uprightMaximumCartMotion < 1e-8

    fprintf('Exact upright equilibrium: PASSED\n');

else

    fprintf('Exact upright equilibrium: REVIEW REQUIRED\n');

end

%% 9. VALIDATE PERTURBATION GROWTH

fprintf('\n');
fprintf('============================================\n');
fprintf('PERTURBATION GROWTH ANALYSIS\n');
fprintf('============================================\n');

perturbedIndices = find(initialAnglesDeg > 0);

growthFactors = maximumInclination(perturbedIndices) ./ ...
    initialAnglesDeg(perturbedIndices);

for k = 1:length(perturbedIndices)

    idx = perturbedIndices(k);

    fprintf(['Initial angle: %.2f deg | ' ...
        'Maximum angle: %.6f deg | Growth factor: %.4f\n'], ...
        initialAnglesDeg(idx), ...
        maximumInclination(idx), ...
        growthFactors(k));

end

if any(growthFactors > 1.5)

    fprintf('\nSignificant perturbation growth observed.\n');
    fprintf('Evidence of upright open-loop instability: YES\n');

else

    fprintf('\nNo strong perturbation growth detected.\n');
    fprintf('Further investigation may be required.\n');

end

%% 10. RIGID ROD CONSTRAINT VALIDATION

fprintf('\n');
fprintf('============================================\n');
fprintf('RIGID ROD CONSTRAINT VALIDATION\n');
fprintf('============================================\n');

overallMaximumRodError = max(maximumRodError);

fprintf('Maximum rod length error: %.6e m\n', ...
    overallMaximumRodError);

if overallMaximumRodError < 1e-7
    fprintf('Rod constraint: PASSED\n');
else
    fprintf('Rod constraint: REVIEW REQUIRED\n');
end

%% 11. FIGURE 1: INCLINATION RESPONSE

figure('Name', 'Phase 9.9 - Inclination Response', ...
    'Color', 'w');

hold on;

for k = 1:numberOfCases

    plot(timeVector, allAngles{k}, ...
        'LineWidth', 1.5, ...
        'DisplayName', sprintf('Initial %.1f deg', ...
        initialAnglesDeg(k)));

end

yline(90, 'k--', '90 deg', ...
    'HandleVisibility', 'off');

grid on;

xlabel('Time (s)');
ylabel('Inclination from upright (deg)');

title('Uncontrolled Pendulum Inclination');

legend('Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_Stability_Inclination.png'));

%% 12. FIGURE 2: CART RADIAL DISPLACEMENT

figure('Name', 'Phase 9.9 - Cart Displacement', ...
    'Color', 'w');

hold on;

for k = 1:numberOfCases

    states = allStates{k};

    cartX = states(:,1) - p.x0;
    cartY = states(:,2) - p.y0;

    radialDisplacement = sqrt(cartX.^2 + cartY.^2);

    plot(timeVector, radialDisplacement, ...
        'LineWidth', 1.5, ...
        'DisplayName', sprintf('Initial %.1f deg', ...
        initialAnglesDeg(k)));

end

grid on;

xlabel('Time (s)');
ylabel('Radial cart displacement (m)');

title('Cart Motion Under Initial Pendulum Perturbations');

legend('Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_Stability_Cart_Displacement.png'));

%% 13. FIGURE 3: MAXIMUM INCLINATION VS INITIAL PERTURBATION

figure('Name', 'Phase 9.9 - Maximum Inclination', ...
    'Color', 'w');

plot(initialAnglesDeg, maximumInclination, ...
    'o-', 'LineWidth', 1.5, 'MarkerSize', 7);

hold on;

plot(initialAnglesDeg, initialAnglesDeg, ...
    'k--', 'LineWidth', 1.2);

grid on;

xlabel('Initial inclination (deg)');
ylabel('Maximum inclination (deg)');

title('Maximum Response vs Initial Perturbation');

legend('Maximum response', ...
    'Initial inclination reference', ...
    'Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_Stability_Perturbation_Growth.png'));

%% 14. FIGURE 4: THRESHOLD CROSSING TIMES

figure('Name', 'Phase 9.9 - Threshold Times', ...
    'Color', 'w');

hold on;

for j = 1:numberOfThresholds

    plot(initialAnglesDeg, thresholdTimes(:,j), ...
        'o-', 'LineWidth', 1.4, ...
        'DisplayName', sprintf('%.0f deg threshold', ...
        thresholdsDeg(j)));

end

grid on;

xlabel('Initial inclination (deg)');
ylabel('First crossing time (s)');

title('Time to Reach Inclination Thresholds');

legend('Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_Stability_Threshold_Times.png'));

%% 15. SAVE RESULTS

phase9_9_results.initialAnglesDeg = initialAnglesDeg;
phase9_9_results.time = timeVector;

phase9_9_results.allAngles = allAngles;
phase9_9_results.allStates = allStates;

phase9_9_results.maximumInclination = maximumInclination;
phase9_9_results.finalInclination = finalInclination;

phase9_9_results.maximumRadialCartDisplacement = ...
    maximumRadialCartDisplacement;

phase9_9_results.maximumCartXDisplacement = ...
    maximumCartXDisplacement;

phase9_9_results.maximumCartYDisplacement = ...
    maximumCartYDisplacement;

phase9_9_results.thresholdsDeg = thresholdsDeg;
phase9_9_results.thresholdTimes = thresholdTimes;

phase9_9_results.maximumRodError = maximumRodError;
phase9_9_results.overallMaximumRodError = ...
    overallMaximumRodError;

phase9_9_results.growthFactors = growthFactors;
phase9_9_results.parameters = p;

save(fullfile(resultsDir, ...
    'phase_9_9_stability_results.mat'), ...
    'phase9_9_results', '-v7.3');

%% 16. COMPLETION SUMMARY

fprintf('\n');
fprintf('============================================\n');
fprintf('PHASE 9.9 COMPLETION SUMMARY\n');
fprintf('============================================\n');

fprintf('Number of initial conditions: %d\n', numberOfCases);

fprintf('Simulation duration: %.2f seconds\n', ...
    p.t_end-p.t_start);

fprintf('Maximum rod constraint error: %.6e m\n', ...
    overallMaximumRodError);

fprintf('\nResults directory:\n%s\n', resultsDir);

fprintf('\nSaved file:\n');
fprintf('phase_9_9_stability_results.mat\n');

fprintf('\nGenerated figures:\n');
fprintf('1. Phase_9_Stability_Inclination.png\n');
fprintf('2. Phase_9_Stability_Cart_Displacement.png\n');
fprintf('3. Phase_9_Stability_Perturbation_Growth.png\n');
fprintf('4. Phase_9_Stability_Threshold_Times.png\n');

fprintf('\n============================================\n');
fprintf('PHASE 9.9 ANALYSIS COMPLETED\n');
fprintf('============================================\n');
