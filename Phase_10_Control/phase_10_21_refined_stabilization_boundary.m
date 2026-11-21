
%% PHASE 10.21: REFINED STABILIZATION BOUNDARY

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 10.21: REFINED STABILIZATION BOUNDARY\n');
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

%% 2. TEST ANGLES

inclinationValues = 41:49;

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

results.inclination_history = cell(numberOfCases,1);

%% 4. RUN SIMULATIONS

for i = 1:numberOfCases

    fprintf('\n--------------------------------------------\n');
    fprintf('CASE %d / %d\n',i,numberOfCases);

    theta0 = deg2rad(inclinationValues(i));

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

    %% Initial state

    z0 = [
        pCase.x0;
        pCase.y0;
        X0;
        Y0;
        Z0;
        0;
        0;
        0;
        0;
        0
    ];

    %% Controlled simulation

    [t,z] = ode45( ...
        @(t,z) controlled_cartesian_equations_of_motion( ...
        t,z,pCase,K), ...
        time,z0,options);

    %% Extract control force

    force = zeros(numel(t),2);

    for j = 1:numel(t)

        [~,u] = controlled_cartesian_equations_of_motion( ...
            t(j),z(j,:)',pCase,K);

        force(j,:) = u';

    end

    %% Pendulum geometry

    rx = z(:,3)-z(:,1);
    ry = z(:,4)-z(:,2);
    rz = z(:,5);

    rodLength = sqrt(rx.^2+ry.^2+rz.^2);

    inclination = acosd(max(-1,min(1, ...
        rz./rodLength)));

    %% Cart response

    displacement = sqrt(z(:,1).^2+z(:,2).^2);

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

    results.inclination_history{i} = inclination;

    %% Display results

    fprintf('Initial inclination: %.2f deg\n', ...
        inclinationValues(i));

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
fprintf('REFINED STABILIZATION BOUNDARY RESULTS\n');
fprintf('============================================\n');

disp(T);

%% 6. SUMMARY

fprintf('\n============================================\n');
fprintf('PHASE 10.21 SUMMARY\n');
fprintf('============================================\n');

fprintf('Cases stabilized: %d / %d\n', ...
    sum(results.stabilization_pass),numberOfCases);

fprintf('Cases saturated: %d / %d\n', ...
    sum(results.force_saturation),numberOfCases);

successful = inclinationValues(results.stabilization_pass);

failed = inclinationValues(~results.stabilization_pass);

if ~isempty(successful)

    fprintf('Largest successful tested angle: %.2f deg\n', ...
        max(successful));

end

if ~isempty(failed)

    fprintf('Smallest failed tested angle: %.2f deg\n', ...
        min(failed));

end

%% 7. INCLINATION PLOT

figure('Name','Refined Stabilization Boundary');

hold on;

for i = 1:numberOfCases

    plot(t,results.inclination_history{i}, ...
        'LineWidth',1.3);

end

xlabel('Time (s)');
ylabel('Inclination (deg)');

title('LQR Response Between 40 and 50 Degrees');

legend(compose('%g deg',inclinationValues), ...
    'Location','best');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_21_RefinedStabilizationBoundary.png'));

%% 8. SAVE RESULTS

results.time = time;
results.parameters = p;
results.controller_gain = K;
results.maximum_force = p.maximum_force;
results.results_table = T;

save(fullfile(scriptDir, ...
    'phase_10_21_refined_stabilization_boundary_results.mat'), ...
    'results');

writetable(T,fullfile(scriptDir, ...
    'Phase_10_21_RefinedStabilizationBoundary.csv'));

fprintf('\nResults saved successfully.\n');

fprintf('\n============================================\n');
fprintf('PHASE 10.21 COMPLETED\n');
fprintf('============================================\n');
