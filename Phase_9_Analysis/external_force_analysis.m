
%% PHASE 9.10: EXTERNAL FORCE RESPONSE ANALYSIS
%
% Project:
% 3D X-Y Cart with an Inverted Spherical Pendulum
%
% Objective:
% Investigate the response of the uncontrolled system to
% constant external forces applied to the cart in X and Y.
%
% Model:
% Validated 10-state Cartesian equations of motion.
%
% Energy balance:
% E(t) - E(0) = W_external(t)
%
% where:
% W_external = integral(Fx*dx + Fy*dy) dt

clc;
close all;

fprintf('\n');
fprintf('============================================\n');
fprintf('PHASE 9.10: EXTERNAL FORCE RESPONSE ANALYSIS\n');
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
fprintf('Initial azimuth: %.4f deg\n', ...
    rad2deg(p.phi0));

%% 3. DEFINE EXTERNAL FORCE CASES

% Each row contains:
% [Fx, Fy] in Newtons

forceCases = [
     0,  0;
     1,  0;
    -1,  0;
     0,  1;
     0, -1;
     1,  1;
    -1, -1
];

caseNames = {
    'No external force'
    'Positive X force'
    'Negative X force'
    'Positive Y force'
    'Negative Y force'
    'Positive diagonal force'
    'Negative diagonal force'
};

numberOfCases = size(forceCases, 1);

fprintf('\n--- EXTERNAL FORCE CASES ---\n');

for k = 1:numberOfCases

    fprintf('%d. %-25s Fx = %+.2f N, Fy = %+.2f N\n', ...
        k, caseNames{k}, ...
        forceCases(k,1), forceCases(k,2));

end

%% 4. SIMULATION SETTINGS

timeVector = (p.t_start:p.sample_time:p.t_end)';


options = odeset( ...
    'RelTol', 1e-11, ...
    'AbsTol', 1e-13, ...
    'MaxStep', p.sample_time/5);


%% 5. INITIAL CARTESIAN STATE

theta0 = p.theta0;
phi0 = p.phi0;

x0 = p.x0;
y0 = p.y0;

X0 = x0 + p.L*sin(theta0)*cos(phi0);
Y0 = y0 + p.L*sin(theta0)*sin(phi0);
Z0 = p.L*cos(theta0);

% Initial bob velocity from angular rates
rTheta = p.L * [
    cos(theta0)*cos(phi0);
    cos(theta0)*sin(phi0);
    -sin(theta0)
];

rPhi = p.L * [
    -sin(theta0)*sin(phi0);
     sin(theta0)*cos(phi0);
     0
];

bobVelocity = [
    p.dx0;
    p.dy0;
    0
] + p.dtheta0*rTheta + p.dphi0*rPhi;

z0 = [
    x0;
    y0;
    X0;
    Y0;
    Z0;
    p.dx0;
    p.dy0;
    bobVelocity
];

%% 6. INITIALIZE RESULT ARRAYS

allStates = cell(numberOfCases, 1);
allInclinations = cell(numberOfCases, 1);
allCartX = cell(numberOfCases, 1);
allCartY = cell(numberOfCases, 1);

allCartKE = cell(numberOfCases, 1);
allBobKE = cell(numberOfCases, 1);
allPotentialEnergy = cell(numberOfCases, 1);
allTotalEnergy = cell(numberOfCases, 1);

allExternalWork = cell(numberOfCases, 1);
allEnergyResidual = cell(numberOfCases, 1);

maximumInclination = zeros(numberOfCases, 1);
finalInclination = zeros(numberOfCases, 1);

maximumCartX = zeros(numberOfCases, 1);
maximumCartY = zeros(numberOfCases, 1);
maximumRadialCart = zeros(numberOfCases, 1);

finalCartX = zeros(numberOfCases, 1);
finalCartY = zeros(numberOfCases, 1);

finalExternalWork = zeros(numberOfCases, 1);
maximumExternalWork = zeros(numberOfCases, 1);

maximumEnergyResidual = zeros(numberOfCases, 1);
rmsEnergyResidual = zeros(numberOfCases, 1);

maximumRodError = zeros(numberOfCases, 1);

%% 7. RUN ALL FORCE CASES

fprintf('\n');
fprintf('============================================\n');
fprintf('RUNNING EXTERNAL FORCE SIMULATIONS\n');
fprintf('============================================\n');

for k = 1:numberOfCases

    % Apply the selected force
    pCase = p;

    pCase.Fx = forceCases(k,1);
    pCase.Fy = forceCases(k,2);

    fprintf('\nCase %d: %s\n', k, caseNames{k});

    [t, states] = ode45( ...
        @(t,z) cartesian_equations_of_motion(t,z,pCase), ...
        timeVector, z0, options);

    %% 7.1 CART AND BOB POSITIONS

    cartX = states(:,1) - x0;
    cartY = states(:,2) - y0;

    radialCart = sqrt(cartX.^2 + cartY.^2);

    rx = states(:,3) - states(:,1);
    ry = states(:,4) - states(:,2);
    rz = states(:,5);

    %% 7.2 PENDULUM INCLINATION

    cosTheta = rz/p.L;
    cosTheta = max(-1, min(1, cosTheta));

    thetaDeg = rad2deg(acos(cosTheta));

    %% 7.3 MECHANICAL ENERGY

    cartVelocitySquared = ...
        states(:,6).^2 + states(:,7).^2;

    bobVelocitySquared = ...
        states(:,8).^2 + ...
        states(:,9).^2 + ...
        states(:,10).^2;

    cartKE = 0.5*p.M*cartVelocitySquared;

    bobKE = 0.5*p.m*bobVelocitySquared;

    potentialEnergy = p.m*p.g*states(:,5);

    totalEnergy = cartKE + bobKE + potentialEnergy;


    %% 7.4 EXTERNAL WORK

    % The applied forces are constant.
    % Therefore, external work can be calculated directly
    % from cart displacement without numerical integration.

    externalWork = ...
        pCase.Fx*(states(:,1) - x0) + ...
        pCase.Fy*(states(:,2) - y0);

    %% 7.5 ENERGY BALANCE RESIDUAL

    % For ideal dynamics:
    % E(t) - E(0) - W_external(t) = 0

    energyResidual = ...
        totalEnergy - totalEnergy(1) - externalWork;

    %% 7.6 ROD CONSTRAINT

    rodLength = sqrt(rx.^2 + ry.^2 + rz.^2);

    rodError = abs(rodLength - p.L);

    %% 7.7 STORE RESULTS

    allStates{k} = states;
    allInclinations{k} = thetaDeg;

    allCartX{k} = cartX;
    allCartY{k} = cartY;

    allCartKE{k} = cartKE;
    allBobKE{k} = bobKE;
    allPotentialEnergy{k} = potentialEnergy;
    allTotalEnergy{k} = totalEnergy;

    allExternalWork{k} = externalWork;
    allEnergyResidual{k} = energyResidual;

    %% 7.8 CALCULATE METRICS

    maximumInclination(k) = max(thetaDeg);
    finalInclination(k) = thetaDeg(end);

    maximumCartX(k) = max(abs(cartX));
    maximumCartY(k) = max(abs(cartY));
    maximumRadialCart(k) = max(radialCart);

    finalCartX(k) = cartX(end);
    finalCartY(k) = cartY(end);

    finalExternalWork(k) = externalWork(end);
    maximumExternalWork(k) = max(abs(externalWork));

    maximumEnergyResidual(k) = max(abs(energyResidual));

    rmsEnergyResidual(k) = ...
        sqrt(mean(energyResidual.^2));

    maximumRodError(k) = max(rodError);

    %% 7.9 PRINT CASE RESULTS

    fprintf('Maximum inclination: %.6f deg\n', ...
        maximumInclination(k));

    fprintf('Final inclination: %.6f deg\n', ...
        finalInclination(k));

    fprintf('Maximum cart X displacement: %.8f m\n', ...
        maximumCartX(k));

    fprintf('Maximum cart Y displacement: %.8f m\n', ...
        maximumCartY(k));

    fprintf('Maximum radial cart displacement: %.8f m\n', ...
        maximumRadialCart(k));

    fprintf('Final external work: %.10f J\n', ...
        finalExternalWork(k));

    fprintf('Maximum energy balance residual: %.6e J\n', ...
        maximumEnergyResidual(k));

    fprintf('Maximum rod constraint error: %.6e m\n', ...
        maximumRodError(k));

end

%% 8. FORCE RESPONSE COMPARISON

fprintf('\n');
fprintf('============================================\n');
fprintf('FORCE RESPONSE SUMMARY\n');
fprintf('============================================\n');

fprintf('%-24s %12s %12s %14s %14s\n', ...
    'Force case', 'Max tilt', 'Final tilt', ...
    'Max X (m)', 'Max Y (m)');

for k = 1:numberOfCases

    fprintf('%-24s %12.4f %12.4f %14.6f %14.6f\n', ...
        caseNames{k}, ...
        maximumInclination(k), ...
        finalInclination(k), ...
        maximumCartX(k), ...
        maximumCartY(k));

end

%% 9. EXTERNAL WORK SUMMARY

fprintf('\n');
fprintf('============================================\n');
fprintf('EXTERNAL WORK SUMMARY\n');
fprintf('============================================\n');

fprintf('%-24s %16s %16s\n', ...
    'Force case', 'Final work (J)', 'Max |work| (J)');

for k = 1:numberOfCases

    fprintf('%-24s %16.8f %16.8f\n', ...
        caseNames{k}, ...
        finalExternalWork(k), ...
        maximumExternalWork(k));

end

%% 10. ENERGY BALANCE VALIDATION

fprintf('\n');
fprintf('============================================\n');
fprintf('WORK-ENERGY VALIDATION\n');
fprintf('============================================\n');

overallMaximumEnergyResidual = max(maximumEnergyResidual);

overallRmsEnergyResidual = sqrt(mean( ...
    rmsEnergyResidual.^2));

fprintf('Maximum energy balance residual: %.6e J\n', ...
    overallMaximumEnergyResidual);

fprintf('Combined RMS energy balance residual: %.6e J\n', ...
    overallRmsEnergyResidual);

if overallMaximumEnergyResidual < 1e-5
    fprintf('Work-energy balance: PASSED\n');
else
    fprintf('Work-energy balance: REVIEW REQUIRED\n');
end

%% 11. RIGID ROD CONSTRAINT VALIDATION

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

%% 12. FIGURE 1: CART X-Y TRAJECTORIES

figure('Name', 'Phase 9.10 - Cart XY Trajectories', ...
    'Color', 'w');

hold on;

for k = 1:numberOfCases

    plot(allCartX{k}, allCartY{k}, ...
        'LineWidth', 1.4, ...
        'DisplayName', caseNames{k});

end

grid on;
axis equal;

xlabel('Cart X displacement (m)');
ylabel('Cart Y displacement (m)');

title('Cart X-Y Trajectories Under External Forces');

legend('Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_10_Force_XY_Trajectories.png'));

%% 13. FIGURE 2: PENDULUM INCLINATION

figure('Name', 'Phase 9.10 - Inclination Response', ...
    'Color', 'w');

hold on;

for k = 1:numberOfCases

    plot(timeVector, allInclinations{k}, ...
        'LineWidth', 1.4, ...
        'DisplayName', caseNames{k});

end

grid on;

xlabel('Time (s)');
ylabel('Inclination from upright (deg)');

title('Pendulum Inclination Under External Forces');

legend('Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_10_Force_Inclination.png'));

%% 14. FIGURE 3: CART DISPLACEMENT

figure('Name', 'Phase 9.10 - Cart Displacement', ...
    'Color', 'w');

subplot(2,1,1);

hold on;

for k = 1:numberOfCases

    plot(timeVector, allCartX{k}, ...
        'LineWidth', 1.3, ...
        'DisplayName', caseNames{k});

end

grid on;
xlabel('Time (s)');
ylabel('X displacement (m)');
title('Cart X Displacement');
legend('Location', 'best');

subplot(2,1,2);

hold on;

for k = 1:numberOfCases

    plot(timeVector, allCartY{k}, ...
        'LineWidth', 1.3, ...
        'DisplayName', caseNames{k});

end

grid on;
xlabel('Time (s)');
ylabel('Y displacement (m)');
title('Cart Y Displacement');
legend('Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_10_Force_Cart_Displacement.png'));

%% 15. FIGURE 4: ENERGY AND EXTERNAL WORK

figure('Name', 'Phase 9.10 - Energy and Work', ...
    'Color', 'w');

hold on;

for k = 1:numberOfCases

    plot(timeVector, allTotalEnergy{k}, ...
        'LineWidth', 1.3, ...
        'DisplayName', caseNames{k});

end

grid on;

xlabel('Time (s)');
ylabel('Total mechanical energy (J)');

title('Mechanical Energy Under External Forces');

legend('Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_10_Force_Total_Energy.png'));

%% 16. FIGURE 5: WORK-ENERGY BALANCE

figure('Name', 'Phase 9.10 - Work Energy Balance', ...
    'Color', 'w');

hold on;

for k = 1:numberOfCases

    plot(timeVector, allEnergyResidual{k}, ...
        'LineWidth', 1.3, ...
        'DisplayName', caseNames{k});

end

grid on;

xlabel('Time (s)');
ylabel('Energy balance residual (J)');

title('Energy Change Minus External Work');

legend('Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_10_Force_Energy_Balance.png'));

%% 17. FIGURE 6: FINAL CART DISPLACEMENT

figure('Name', 'Phase 9.10 - Final Cart Displacement', ...
    'Color', 'w');

bar([finalCartX, finalCartY]);

grid on;

xlabel('Force case');
ylabel('Final displacement (m)');

title('Final Cart Position Under External Forces');

xticks(1:numberOfCases);
xticklabels(caseNames);
xtickangle(25);

legend('X displacement', 'Y displacement', ...
    'Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_10_Force_Final_Displacement.png'));

%% 18. SAVE RESULTS

phase9_10_results.forceCases = forceCases;
phase9_10_results.caseNames = caseNames;

phase9_10_results.time = timeVector;
phase9_10_results.allStates = allStates;

phase9_10_results.allInclinations = allInclinations;
phase9_10_results.allCartX = allCartX;
phase9_10_results.allCartY = allCartY;

phase9_10_results.allCartKE = allCartKE;
phase9_10_results.allBobKE = allBobKE;
phase9_10_results.allPotentialEnergy = allPotentialEnergy;
phase9_10_results.allTotalEnergy = allTotalEnergy;

phase9_10_results.allExternalWork = allExternalWork;
phase9_10_results.allEnergyResidual = allEnergyResidual;

phase9_10_results.maximumInclination = maximumInclination;
phase9_10_results.finalInclination = finalInclination;

phase9_10_results.maximumCartX = maximumCartX;
phase9_10_results.maximumCartY = maximumCartY;
phase9_10_results.maximumRadialCart = maximumRadialCart;

phase9_10_results.finalCartX = finalCartX;
phase9_10_results.finalCartY = finalCartY;

phase9_10_results.finalExternalWork = finalExternalWork;
phase9_10_results.maximumExternalWork = maximumExternalWork;

phase9_10_results.maximumEnergyResidual = maximumEnergyResidual;
phase9_10_results.rmsEnergyResidual = rmsEnergyResidual;

phase9_10_results.maximumRodError = maximumRodError;

phase9_10_results.overallMaximumEnergyResidual = ...
    overallMaximumEnergyResidual;

phase9_10_results.overallMaximumRodError = ...
    overallMaximumRodError;

phase9_10_results.parameters = p;

save(fullfile(resultsDir, ...
    'phase_9_10_external_force_results.mat'), ...
    'phase9_10_results', '-v7.3');

%% 19. COMPLETION SUMMARY

fprintf('\n');
fprintf('============================================\n');
fprintf('PHASE 9.10 COMPLETION SUMMARY\n');
fprintf('============================================\n');

fprintf('Number of force cases: %d\n', numberOfCases);

fprintf('Simulation duration: %.2f seconds\n', ...
    p.t_end-p.t_start);

fprintf('Applied force magnitude per axis: 1.00 N\n');

fprintf('Maximum energy balance residual: %.6e J\n', ...
    overallMaximumEnergyResidual);

fprintf('Maximum rod constraint error: %.6e m\n', ...
    overallMaximumRodError);

fprintf('\nResults directory:\n%s\n', resultsDir);

fprintf('\nSaved file:\n');
fprintf('phase_9_10_external_force_results.mat\n');

fprintf('\nGenerated figures:\n');
fprintf('1. Phase_9_10_Force_XY_Trajectories.png\n');
fprintf('2. Phase_9_10_Force_Inclination.png\n');
fprintf('3. Phase_9_10_Force_Cart_Displacement.png\n');
fprintf('4. Phase_9_10_Force_Total_Energy.png\n');
fprintf('5. Phase_9_10_Force_Energy_Balance.png\n');
fprintf('6. Phase_9_10_Force_Final_Displacement.png\n');

fprintf('\n============================================\n');
fprintf('PHASE 9.10 ANALYSIS COMPLETED\n');
fprintf('============================================\n');
