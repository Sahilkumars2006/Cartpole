
%% PHASE 10.16: COMBINED INITIAL CONDITION ROBUSTNESS

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 10.16: COMBINED INITIAL CONDITION ROBUSTNESS\n');
fprintf('============================================\n');

%% 1. PATHS AND CONTROLLER

scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);

addpath(fullfile(projectDir, ...
    'Phase_4_MATLAB_Simulation'));

p = system_parameters();

loaded = load(fullfile(scriptDir, ...
    'controller_data.mat'));

K = loaded.controller.K;
p.maximum_force = loaded.controller.maximum_force;

fprintf('\nController information:\n');
fprintf('LQR gain dimensions: %d x %d\n',size(K,1),size(K,2));
fprintf('Maximum componentwise force: %.2f N\n', ...
    p.maximum_force);

%% 2. DEFINE COMBINED INITIAL CONDITION CASES

caseNames = {
    'Nominal'
    'Moderate'
    'Positive_Disturbance'
    'Negative_Disturbance'
    'Mixed_Disturbance'
};

% Each row:
% [theta_deg, phi_deg, dx0, dy0, dtheta0, dphi0]

caseValues = [
     5,  30,  0.00,  0.00,  0.00,  0.00
     7,  45,  0.10, -0.10,  0.10,  0.10
    10,  30,  0.20,  0.20,  0.25,  0.25
    10, 120, -0.20, -0.20, -0.25, -0.25
     8,  75,  0.15, -0.15,  0.30, -0.30
];

numberOfCases = size(caseValues,1);

%% 3. SIMULATION SETTINGS

time = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol',p.relative_tolerance, ...
    'AbsTol',p.absolute_tolerance);

%% 4. RESULT ARRAYS

results = struct();

results.case_names = caseNames;
results.case_values = caseValues;

results.final_inclination = zeros(numberOfCases,1);
results.maximum_inclination = zeros(numberOfCases,1);

results.maximum_cart_displacement = zeros(numberOfCases,1);
results.maximum_cart_speed = zeros(numberOfCases,1);

results.maximum_Fx = zeros(numberOfCases,1);
results.maximum_Fy = zeros(numberOfCases,1);
results.maximum_radial_force = zeros(numberOfCases,1);

results.maximum_rod_error = zeros(numberOfCases,1);

results.final_cart_position = zeros(numberOfCases,2);

results.force_limit_pass = false(numberOfCases,1);

results.simulation_states = cell(numberOfCases,1);
results.control_forces = cell(numberOfCases,1);
results.inclination_history = cell(numberOfCases,1);
results.cart_trajectories = cell(numberOfCases,1);

%% 5. RUN ALL COMBINED CASES

for i = 1:numberOfCases

    fprintf('\n--------------------------------------------\n');
    fprintf('CASE %d / %d: %s\n', ...
        i,numberOfCases,caseNames{i});

    %% Set case-specific initial conditions

    pCase = p;

    theta0 = deg2rad(caseValues(i,1));
    phi0 = deg2rad(caseValues(i,2));

    pCase.theta0 = theta0;
    pCase.phi0 = phi0;

    pCase.dx0 = caseValues(i,3);
    pCase.dy0 = caseValues(i,4);

    pCase.dtheta0 = caseValues(i,5);
    pCase.dphi0 = caseValues(i,6);

    fprintf('Initial inclination: %.2f deg\n',rad2deg(theta0));
    fprintf('Initial azimuth: %.2f deg\n',rad2deg(phi0));
    fprintf('Initial X velocity: %.3f m/s\n',pCase.dx0);
    fprintf('Initial Y velocity: %.3f m/s\n',pCase.dy0);
    fprintf('Initial inclination rate: %.3f rad/s\n', ...
        pCase.dtheta0);
    fprintf('Initial azimuth rate: %.3f rad/s\n', ...
        pCase.dphi0);

    %% Initial bob position relative to cart

    rx0 = pCase.L*sin(theta0)*cos(phi0);
    ry0 = pCase.L*sin(theta0)*sin(phi0);
    rz0 = pCase.L*cos(theta0);

    X0 = pCase.x0 + rx0;
    Y0 = pCase.y0 + ry0;
    Z0 = rz0;

    %% Initial relative bob velocity

    thetaRate = pCase.dtheta0;
    phiRate = pCase.dphi0;

    drx0 = pCase.L*( ...
        cos(theta0)*thetaRate*cos(phi0) ...
        - sin(theta0)*phiRate*sin(phi0));

    dry0 = pCase.L*( ...
        cos(theta0)*thetaRate*sin(phi0) ...
        + sin(theta0)*phiRate*cos(phi0));

    drz0 = -pCase.L*sin(theta0)*thetaRate;

    %% Initial absolute bob velocity

    dX0 = pCase.dx0 + drx0;
    dY0 = pCase.dy0 + dry0;
    dZ0 = drz0;

    %% Cartesian initial state

    z0 = [
        pCase.x0;
        pCase.y0;
        X0;
        Y0;
        Z0;
        pCase.dx0;
        pCase.dy0;
        dX0;
        dY0;
        dZ0
    ];

    %% Controlled simulation

    [t,z] = ode45( ...
        @(t,z) controlled_cartesian_equations_of_motion( ...
        t,z,pCase,K), ...
        time,z0,options);

    %% Calculate control forces

    force = zeros(numel(t),2);

    for j = 1:numel(t)

        [~,u] = controlled_cartesian_equations_of_motion( ...
            t(j),z(j,:)',pCase,K);

        force(j,:) = u';

    end

    %% Pendulum geometry

    relative = [
        z(:,3)-z(:,1), ...
        z(:,4)-z(:,2), ...
        z(:,5)
    ];

    rodLength = sqrt(sum(relative.^2,2));

    inclination = acosd(max(-1,min(1, ...
        relative(:,3)./rodLength)));

    %% Cart response

    cart = z(:,1:2);

    displacement = sqrt(sum(cart.^2,2));

    speed = sqrt(z(:,6).^2+z(:,7).^2);

    radialForce = sqrt(sum(force.^2,2));

    %% Store performance metrics

    results.final_inclination(i) = inclination(end);

    results.maximum_inclination(i) = max(inclination);

    results.maximum_cart_displacement(i) = max(displacement);

    results.maximum_cart_speed(i) = max(speed);

    results.maximum_Fx(i) = max(abs(force(:,1)));

    results.maximum_Fy(i) = max(abs(force(:,2)));

    results.maximum_radial_force(i) = max(radialForce);

    results.maximum_rod_error(i) = ...
        max(abs(rodLength-pCase.L));

    results.final_cart_position(i,:) = cart(end,:);

    results.force_limit_pass(i) = ...
        all(abs(force(:,1)) <= p.maximum_force + 1e-9) && ...
        all(abs(force(:,2)) <= p.maximum_force + 1e-9);

    results.simulation_states{i} = z;
    results.control_forces{i} = force;
    results.inclination_history{i} = inclination;
    results.cart_trajectories{i} = cart;

    %% Display case results

    fprintf('\nSimulation results:\n');

    fprintf('Final inclination: %.8f deg\n', ...
        results.final_inclination(i));

    fprintf('Maximum inclination: %.8f deg\n', ...
        results.maximum_inclination(i));

    fprintf('Maximum cart displacement: %.8f m\n', ...
        results.maximum_cart_displacement(i));

    fprintf('Maximum cart speed: %.8f m/s\n', ...
        results.maximum_cart_speed(i));

    fprintf('Maximum |Fx|: %.8f N\n', ...
        results.maximum_Fx(i));

    fprintf('Maximum |Fy|: %.8f N\n', ...
        results.maximum_Fy(i));

    fprintf('Maximum radial force: %.8f N\n', ...
        results.maximum_radial_force(i));

    fprintf('Maximum rod error: %.4e m\n', ...
        results.maximum_rod_error(i));

    fprintf('Force limit: %s\n', ...
        string(results.force_limit_pass(i)));

end

%% 6. RESULTS TABLE

fprintf('\n============================================\n');
fprintf('COMBINED INITIAL CONDITION RESULTS\n');
fprintf('============================================\n');

T = table( ...
    caseNames, ...
    caseValues(:,1), ...
    caseValues(:,2), ...
    caseValues(:,3), ...
    caseValues(:,4), ...
    caseValues(:,5), ...
    caseValues(:,6), ...
    results.final_inclination, ...
    results.maximum_inclination, ...
    results.maximum_cart_displacement, ...
    results.maximum_cart_speed, ...
    results.maximum_Fx, ...
    results.maximum_Fy, ...
    results.maximum_radial_force, ...
    results.maximum_rod_error, ...
    results.force_limit_pass, ...
    'VariableNames',{ ...
    'Case', ...
    'InitialInclination_deg', ...
    'InitialAzimuth_deg', ...
    'InitialXVelocity_mps', ...
    'InitialYVelocity_mps', ...
    'InitialThetaRate_radps', ...
    'InitialPhiRate_radps', ...
    'FinalInclination_deg', ...
    'MaxInclination_deg', ...
    'MaxDisplacement_m', ...
    'MaxSpeed_mps', ...
    'MaxFx_N', ...
    'MaxFy_N', ...
    'MaxRadialForce_N', ...
    'MaxRodError_m', ...
    'ForceLimitPass'});

disp(T);

%% 7. INCLINATION RESPONSE

figure('Name','Combined Initial Condition Stabilization');

hold on;

for i = 1:numberOfCases

    plot(t,results.inclination_history{i}, ...
        'LineWidth',1.4);

end

xlabel('Time (s)');
ylabel('Pendulum inclination (deg)');

title('Combined Initial Condition Stabilization');

legend(caseNames,'Interpreter','none','Location','best');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_16_Combined_Stabilization.png'));

%% 8. CART TRAJECTORIES

figure('Name','Combined Initial Condition Cart Trajectories');

hold on;

for i = 1:numberOfCases

    cart = results.cart_trajectories{i};

    plot(cart(:,1),cart(:,2),'LineWidth',1.4);

end

xlabel('Cart X position (m)');
ylabel('Cart Y position (m)');

title('Cart Trajectories Under Combined Initial Conditions');

legend(caseNames,'Interpreter','none','Location','best');

axis equal;
grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_16_Combined_CartTrajectories.png'));

%% 9. CART DISPLACEMENT COMPARISON

figure('Name','Combined Initial Condition Displacement');

bar(results.maximum_cart_displacement);

set(gca,'XTick',1:numberOfCases, ...
    'XTickLabel',caseNames);

xtickangle(20);

ylabel('Maximum cart displacement (m)');
title('Maximum Cart Displacement by Combined Case');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_16_Combined_Displacement.png'));

%% 10. CONTROL FORCE COMPARISON

figure('Name','Combined Initial Condition Control Forces');

bar([results.maximum_Fx,results.maximum_Fy]);

hold on;

yline(p.maximum_force,'--');

set(gca,'XTick',1:numberOfCases, ...
    'XTickLabel',caseNames);

xtickangle(20);

ylabel('Maximum force magnitude (N)');

title('Maximum Componentwise Control Force');

legend('Maximum |Fx|','Maximum |Fy|','30 N limit', ...
    'Location','best');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_16_Combined_ControlForces.png'));

%% 11. SUMMARY CHECKS

allStabilized = all(results.final_inclination < 0.01);

allForceLimitsPassed = all(results.force_limit_pass);

maximumObservedRodError = max(results.maximum_rod_error);

fprintf('\n============================================\n');
fprintf('PHASE 10.16 SUMMARY CHECKS\n');
fprintf('============================================\n');

fprintf('All final inclinations below 0.01 deg: %s\n', ...
    string(allStabilized));

fprintf('All componentwise force limits satisfied: %s\n', ...
    string(allForceLimitsPassed));

fprintf('Largest rod-length error: %.4e m\n', ...
    maximumObservedRodError);

%% 12. SAVE RESULTS

results.time = time;
results.parameters = p;
results.controller_gain = K;
results.maximum_force = p.maximum_force;
results.results_table = T;

results.all_stabilized = allStabilized;
results.all_force_limits_passed = allForceLimitsPassed;

save(fullfile(scriptDir, ...
    'phase_10_16_combined_initial_condition_results.mat'), ...
    'results');

writetable(T,fullfile(scriptDir, ...
    'Phase_10_16_CombinedInitialCondition_Robustness.csv'));

fprintf('\nResults saved:\n');

fprintf('%s\n',fullfile(scriptDir, ...
    'phase_10_16_combined_initial_condition_results.mat'));

fprintf('\n============================================\n');
fprintf('PHASE 10.16 COMPLETED\n');
fprintf('============================================\n');
