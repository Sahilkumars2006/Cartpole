
%% PHASE 10.20: MAXIMUM STABILIZABLE INITIAL INCLINATION

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 10.20: MAXIMUM STABILIZABLE INCLINATION\n');
fprintf('============================================\n');

%% 1. LOAD PARAMETERS AND CONTROLLER

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

%% 2. INITIAL INCLINATION CASES

inclinationValues = [30 40 50 60 70 80];

numberOfCases = numel(inclinationValues);

phi0 = deg2rad(30);

time = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol',p.relative_tolerance, ...
    'AbsTol',p.absolute_tolerance);

%% 3. INITIALIZE RESULTS

results = struct();

results.initial_inclination = inclinationValues;

results.final_inclination = zeros(numberOfCases,1);
results.maximum_inclination = zeros(numberOfCases,1);

results.maximum_cart_displacement = zeros(numberOfCases,1);
results.maximum_cart_speed = zeros(numberOfCases,1);

results.maximum_Fx = zeros(numberOfCases,1);
results.maximum_Fy = zeros(numberOfCases,1);
results.maximum_radial_force = zeros(numberOfCases,1);

results.maximum_rod_error = zeros(numberOfCases,1);

results.force_saturation = false(numberOfCases,1);
results.stabilization_pass = false(numberOfCases,1);

results.simulation_states = cell(numberOfCases,1);
results.control_forces = cell(numberOfCases,1);
results.inclination_history = cell(numberOfCases,1);

%% 4. RUN SIMULATIONS

for i = 1:numberOfCases

    fprintf('\n--------------------------------------------\n');
    fprintf('CASE %d / %d\n',i,numberOfCases);

    theta0 = deg2rad(inclinationValues(i));

    fprintf('Initial inclination: %.2f deg\n', ...
        inclinationValues(i));

    pCase = p;

    pCase.theta0 = theta0;
    pCase.phi0 = phi0;

    pCase.dx0 = 0;
    pCase.dy0 = 0;

    pCase.dtheta0 = 0;
    pCase.dphi0 = 0;

    %% Initial bob position

    X0 = pCase.x0 + pCase.L*sin(theta0)*cos(phi0);

    Y0 = pCase.y0 + pCase.L*sin(theta0)*sin(phi0);

    Z0 = pCase.L*cos(theta0);

    %% Initial bob velocity

    dX0 = 0;
    dY0 = 0;
    dZ0 = 0;

    %% Initial Cartesian state

    z0 = [
        pCase.x0;
        pCase.y0;
        X0;
        Y0;
        Z0;
        0;
        0;
        dX0;
        dY0;
        dZ0
    ];

    %% Controlled simulation

    [t,z] = ode45( ...
        @(t,z) controlled_cartesian_equations_of_motion( ...
        t,z,pCase,K), ...
        time,z0,options);

    %% Extract control forces

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

    %% Store results

    results.final_inclination(i) = inclination(end);

    results.maximum_inclination(i) = max(inclination);

    results.maximum_cart_displacement(i) = max(displacement);

    results.maximum_cart_speed(i) = max(speed);

    results.maximum_Fx(i) = max(abs(force(:,1)));

    results.maximum_Fy(i) = max(abs(force(:,2)));

    results.maximum_radial_force(i) = max(radialForce);

    results.maximum_rod_error(i) = ...
        max(abs(rodLength-pCase.L));

    results.force_saturation(i) = ...
        any(abs(force(:,1)) >= p.maximum_force-1e-8) || ...
        any(abs(force(:,2)) >= p.maximum_force-1e-8);

    results.stabilization_pass(i) = ...
        inclination(end) < 0.01;

    results.simulation_states{i} = z;
    results.control_forces{i} = force;
    results.inclination_history{i} = inclination;

    %% Display results

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

    fprintf('Force saturation: %s\n', ...
        string(results.force_saturation(i)));

    fprintf('Stabilization: %s\n', ...
        string(results.stabilization_pass(i)));

end

%% 5. RESULTS TABLE

T = table( ...
    inclinationValues', ...
    results.final_inclination, ...
    results.maximum_inclination, ...
    results.maximum_cart_displacement, ...
    results.maximum_cart_speed, ...
    results.maximum_Fx, ...
    results.maximum_Fy, ...
    results.maximum_radial_force, ...
    results.maximum_rod_error, ...
    results.force_saturation, ...
    results.stabilization_pass, ...
    'VariableNames',{ ...
    'InitialInclination_deg', ...
    'FinalInclination_deg', ...
    'MaxInclination_deg', ...
    'MaxDisplacement_m', ...
    'MaxSpeed_mps', ...
    'MaxFx_N', ...
    'MaxFy_N', ...
    'MaxRadialForce_N', ...
    'MaxRodError_m', ...
    'ForceSaturation', ...
    'StabilizationPass'});

fprintf('\n============================================\n');
fprintf('MAXIMUM STABILIZABLE INCLINATION RESULTS\n');
fprintf('============================================\n');

disp(T);

%% 6. INCLINATION RESPONSE

figure('Name','Large Initial Inclination Stabilization');

hold on;

for i = 1:numberOfCases

    plot(t,results.inclination_history{i}, ...
        'LineWidth',1.4);

end

xlabel('Time (s)');
ylabel('Pendulum inclination (deg)');

title('LQR Stabilization at Large Initial Inclinations');

legend(compose('%g deg',inclinationValues), ...
    'Location','best');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_20_LargeInclination_Stabilization.png'));

%% 7. CART DISPLACEMENT

figure('Name','Large Initial Inclination Cart Displacement');

plot(inclinationValues, ...
    results.maximum_cart_displacement,'o-', ...
    'LineWidth',1.5);

xlabel('Initial inclination (deg)');
ylabel('Maximum cart displacement (m)');

title('Initial Inclination vs Maximum Cart Displacement');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_20_LargeInclination_Displacement.png'));

%% 8. CONTROL FORCE

figure('Name','Large Initial Inclination Control Force');

plot(inclinationValues,results.maximum_Fx,'o-', ...
    'LineWidth',1.5);

hold on;

plot(inclinationValues,results.maximum_Fy,'s-', ...
    'LineWidth',1.5);

yline(p.maximum_force,'--');

xlabel('Initial inclination (deg)');
ylabel('Maximum force (N)');

title('Large Initial Inclination vs Control Force');

legend('Maximum |Fx|','Maximum |Fy|','30 N limit', ...
    'Location','best');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_20_LargeInclination_ControlForces.png'));

%% 9. SUMMARY CHECKS

fprintf('\n============================================\n');
fprintf('PHASE 10.20 SUMMARY CHECKS\n');
fprintf('============================================\n');

fprintf('Cases stabilized: %d / %d\n', ...
    sum(results.stabilization_pass),numberOfCases);

fprintf('Cases without force saturation: %d / %d\n', ...
    sum(~results.force_saturation),numberOfCases);

fprintf('Largest rod-length error: %.4e m\n', ...
    max(results.maximum_rod_error));

fprintf('\nIndividual stabilization results:\n');

for i = 1:numberOfCases

    fprintf('%2.0f deg: Stabilization = %s, Saturation = %s\n', ...
        inclinationValues(i), ...
        string(results.stabilization_pass(i)), ...
        string(results.force_saturation(i)));

end

%% 10. SAVE RESULTS

results.time = time;
results.parameters = p;
results.controller_gain = K;
results.maximum_force = p.maximum_force;
results.results_table = T;

save(fullfile(scriptDir, ...
    'phase_10_20_maximum_stabilizable_inclination_results.mat'), ...
    'results');

writetable(T,fullfile(scriptDir, ...
    'Phase_10_20_MaximumStabilizableInclination.csv'));

fprintf('\nResults saved successfully.\n');

fprintf('\n============================================\n');
fprintf('PHASE 10.20 COMPLETED\n');
fprintf('============================================\n');
