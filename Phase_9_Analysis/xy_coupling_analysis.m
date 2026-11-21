
%% PHASE 9.6: X-Y COUPLING AND ROTATIONAL SYMMETRY ANALYSIS
%
% Project:
% Mathematical Modelling, Simulation and Analysis of a
% 3D X-Y Cart with an Inverted Spherical Pendulum
%
% Purpose:
% 1. Investigate X-Y cart motion for different initial azimuths.
% 2. Verify horizontal rotational symmetry.
% 3. Compare canonicalized trajectories.
% 4. Quantify numerical symmetry errors.

clc;
close all;

fprintf('\n');
fprintf('============================================\n');
fprintf('PHASE 9.6: X-Y COUPLING ANALYSIS\n');
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
fprintf('Initial inclination: %.4f deg\n', ...
    rad2deg(p.theta0));

%% 3. SIMULATION SETTINGS

azimuths = [0 30 60 90 120 150];

numCases = numel(azimuths);

time = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol', p.relative_tolerance, ...
    'AbsTol', p.absolute_tolerance, ...
    'MaxStep', p.sample_time);

% Store state histories:
% dimensions = time x state x azimuth case

allStates = zeros(length(time), 10, numCases);

% Store canonicalized trajectories.
canonicalStates = zeros(length(time), 10, numCases);

%% 4. RUN SIMULATIONS

for i = 1:numCases

    phi = deg2rad(azimuths(i));

    fprintf('\nRunning initial azimuth = %.1f degrees...\n', ...
        azimuths(i));

    % Initial cart position
    x0 = p.x0;
    y0 = p.y0;

    % Initial bob position
    X0 = x0 + p.L*sin(p.theta0)*cos(phi);
    Y0 = y0 + p.L*sin(p.theta0)*sin(phi);
    Z0 = p.L*cos(p.theta0);

    % Initial state:
    % [x; y; X; Y; Z; dx; dy; dX; dY; dZ]

    z0 = [
        x0;
        y0;
        X0;
        Y0;
        Z0;
        p.dx0;
        p.dy0;
        0;
        0;
        0
    ];

    % Run Cartesian equations of motion
    [~, states] = ode45( ...
        @(t,z) cartesian_equations_of_motion(t,z,p), ...
        time, z0, options);

    allStates(:,:,i) = states;

    %% Canonical rotation

    % Rotate horizontal coordinates by -phi.
    % This transforms each case to an equivalent
    % reference orientation with initial azimuth zero.

    c = cos(phi);
    s = sin(phi);

    R = [c s; -s c];

    canonical = states;

    % Cart position
    canonical(:,1:2) = ...
        (R * states(:,1:2)')';

    % Bob absolute horizontal position
    canonical(:,3:4) = ...
        (R * states(:,3:4)')';

    % Cart velocity
    canonical(:,6:7) = ...
        (R * states(:,6:7)')';

    % Bob horizontal velocity
    canonical(:,8:9) = ...
        (R * states(:,8:9)')';

    canonicalStates(:,:,i) = canonical;

    %% Calculate metrics

    cartX = states(:,1) - x0;
    cartY = states(:,2) - y0;

    cartRadius = sqrt(cartX.^2 + cartY.^2);

    relativeZ = states(:,5);

    inclination = acosd(max(-1, min(1, ...
        relativeZ/p.L)));

    fprintf('  Maximum X displacement: %.8f m\n', ...
        max(abs(cartX)));

    fprintf('  Maximum Y displacement: %.8f m\n', ...
        max(abs(cartY)));

    fprintf('  Maximum radial displacement: %.8f m\n', ...
        max(cartRadius));

    fprintf('  Maximum inclination: %.8f deg\n', ...
        max(inclination));

end

%% 5. ROTATIONAL SYMMETRY ERROR

% Azimuth zero is the reference case.
reference = canonicalStates(:,:,1);

symmetryError = zeros(numCases,1);

for i = 1:numCases

    difference = canonicalStates(:,:,i) - reference;

    symmetryError(i) = max(abs(difference), [], 'all');

end

%% 6. SUMMARY METRICS

maxX = zeros(numCases,1);
maxY = zeros(numCases,1);
maxRadial = zeros(numCases,1);
maxTilt = zeros(numCases,1);
finalTilt = zeros(numCases,1);

for i = 1:numCases

    states = allStates(:,:,i);

    cartX = states(:,1) - p.x0;
    cartY = states(:,2) - p.y0;

    maxX(i) = max(abs(cartX));
    maxY(i) = max(abs(cartY));

    maxRadial(i) = max(sqrt(cartX.^2 + cartY.^2));

    inclination = acosd(max(-1, min(1, ...
        states(:,5)/p.L)));

    maxTilt(i) = max(inclination);
    finalTilt(i) = inclination(end);

end

fprintf('\n');
fprintf('============================================\n');
fprintf('ROTATIONAL SYMMETRY RESULTS\n');
fprintf('============================================\n');

fprintf('\n');
fprintf('%12s %14s %14s %14s %14s\n', ...
    'Azimuth', 'Max X', 'Max Y', ...
    'Max Radius', 'Symmetry Error');

for i = 1:numCases

    fprintf('%12.1f %14.8f %14.8f %14.8f %14.4e\n', ...
        azimuths(i), maxX(i), maxY(i), ...
        maxRadial(i), symmetryError(i));

end

%% 7. FIGURE 1: X-Y CART TRAJECTORIES

figure('Name', 'Phase 9.6 - XY Trajectories', ...
    'Color', 'w');

hold on;
grid on;
axis equal;

for i = 1:numCases

    states = allStates(:,:,i);

    plot(states(:,1), states(:,2), ...
        'LineWidth', 1.5, ...
        'DisplayName', ...
        sprintf('Azimuth %.0f deg', azimuths(i)));

end

xlabel('Cart X Position (m)');
ylabel('Cart Y Position (m)');

title('X-Y Cart Trajectories for Different Initial Azimuths');

legend('Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_XY_Trajectories.png'));

%% 8. FIGURE 2: MAXIMUM X AND Y DISPLACEMENT

figure('Name', 'Phase 9.6 - XY Displacement', ...
    'Color', 'w');

plot(azimuths, maxX, '-o', ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Maximum |X|');

hold on;

plot(azimuths, maxY, '-s', ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Maximum |Y|');

grid on;

xlabel('Initial Azimuth (degrees)');
ylabel('Maximum Cart Displacement (m)');

title('Maximum X and Y Cart Displacement');

legend('Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_XY_Displacement.png'));

%% 9. FIGURE 3: CANONICALIZED TRAJECTORY COMPARISON

figure('Name', 'Phase 9.6 - Canonical Trajectories', ...
    'Color', 'w');

hold on;
grid on;
axis equal;

for i = 1:numCases

    states = canonicalStates(:,:,i);

    plot(states(:,1), states(:,2), ...
        'LineWidth', 1.3, ...
        'DisplayName', ...
        sprintf('Azimuth %.0f deg', azimuths(i)));

end

xlabel('Canonical X Position (m)');
ylabel('Canonical Y Position (m)');

title('Trajectories After Removing Initial Azimuth Rotation');

legend('Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_Canonical_Trajectories.png'));

%% 10. FIGURE 4: ROTATIONAL SYMMETRY ERROR

figure('Name', 'Phase 9.6 - Symmetry Error', ...
    'Color', 'w');

semilogy(azimuths, max(symmetryError, eps), ...
    '-o', 'LineWidth', 1.5);

grid on;

xlabel('Initial Azimuth (degrees)');
ylabel('Maximum State Difference');

title('Rotational Symmetry Numerical Error');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_Rotational_Symmetry_Error.png'));

%% 11. VALIDATION

fprintf('\n');
fprintf('============================================\n');
fprintf('PHASE 9.6 VALIDATION\n');
fprintf('============================================\n');

maximumSymmetryError = max(symmetryError);

fprintf('Maximum symmetry error: %.6e\n', ...
    maximumSymmetryError);

if maximumSymmetryError < 1e-6
    fprintf('Rotational symmetry test: PASSED\n');
else
    fprintf('Rotational symmetry test: REVIEW REQUIRED\n');
end

fprintf('\nMaximum radial cart displacement range:\n');
fprintf('Minimum: %.10f m\n', min(maxRadial));
fprintf('Maximum: %.10f m\n', max(maxRadial));

fprintf('\nMaximum inclination range:\n');
fprintf('Minimum: %.8f deg\n', min(maxTilt));
fprintf('Maximum: %.8f deg\n', max(maxTilt));

%% 12. SAVE RESULTS

phase9_6_results.azimuths = azimuths;
phase9_6_results.time = time;
phase9_6_results.allStates = allStates;
phase9_6_results.canonicalStates = canonicalStates;
phase9_6_results.maxX = maxX;
phase9_6_results.maxY = maxY;
phase9_6_results.maxRadial = maxRadial;
phase9_6_results.maxTilt = maxTilt;
phase9_6_results.finalTilt = finalTilt;
phase9_6_results.symmetryError = symmetryError;
phase9_6_results.maximumSymmetryError = maximumSymmetryError;

save(fullfile(resultsDir, ...
    'phase_9_6_xy_coupling_results.mat'), ...
    'phase9_6_results');

fprintf('\n');
fprintf('============================================\n');
fprintf('PHASE 9.6 COMPLETED\n');
fprintf('============================================\n');

fprintf('Number of simulations: %d\n', numCases);
fprintf('Simulation duration: %.2f seconds\n', ...
    p.t_end - p.t_start);

fprintf('\nResults directory:\n%s\n', resultsDir);

fprintf('\nSaved file:\n');
fprintf('phase_9_6_xy_coupling_results.mat\n');

fprintf('\nGenerated figures:\n');
fprintf('1. Phase_9_XY_Trajectories.png\n');
fprintf('2. Phase_9_XY_Displacement.png\n');
fprintf('3. Phase_9_Canonical_Trajectories.png\n');
fprintf('4. Phase_9_Rotational_Symmetry_Error.png\n');

fprintf('\n============================================\n');
