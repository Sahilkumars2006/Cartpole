
%% PHASE 10.18: ACTUATOR SATURATION BOUNDARY

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 10.18: ACTUATOR SATURATION BOUNDARY\n');
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

inclinationValues = [20 21 22 23 24 25];

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

results.maximum_Fx = zeros(numberOfCases,1);
results.maximum_Fy = zeros(numberOfCases,1);

results.maximum_displacement = zeros(numberOfCases,1);
results.maximum_speed = zeros(numberOfCases,1);

results.force_saturation = false(numberOfCases,1);
results.stabilization_pass = false(numberOfCases,1);

results.inclination_history = cell(numberOfCases,1);

%% 4. RUN TEST CASES

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

    %% Simulation

    [t,z] = ode45( ...
        @(t,z) controlled_cartesian_equations_of_motion( ...
        t,z,pCase,K), ...
        time,z0,options);

    %% Extract forces

    force = zeros(numel(t),2);

    for j = 1:numel(t)

        [~,u] = controlled_cartesian_equations_of_motion( ...
            t(j),z(j,:)',pCase,K);

        force(j,:) = u';

    end

    %% Inclination

    rx = z(:,3)-z(:,1);
    ry = z(:,4)-z(:,2);
    rz = z(:,5);

    rodLength = sqrt(rx.^2+ry.^2+rz.^2);

    inclination = acosd(max(-1,min(1, ...
        rz./rodLength)));

    %% Cart response

    displacement = sqrt(z(:,1).^2+z(:,2).^2);

    speed = sqrt(z(:,6).^2+z(:,7).^2);

    %% Results

    results.final_inclination(i) = inclination(end);

    results.maximum_Fx(i) = max(abs(force(:,1)));

    results.maximum_Fy(i) = max(abs(force(:,2)));

    results.maximum_displacement(i) = max(displacement);

    results.maximum_speed(i) = max(speed);

    results.force_saturation(i) = ...
        any(abs(force(:,1)) >= p.maximum_force-1e-8) || ...
        any(abs(force(:,2)) >= p.maximum_force-1e-8);

    results.stabilization_pass(i) = ...
        inclination(end) < 0.01;

    results.inclination_history{i} = inclination;

    %% Display

    fprintf('Initial inclination: %.2f deg\n', ...
        inclinationValues(i));

    fprintf('Final inclination: %.8f deg\n', ...
        results.final_inclination(i));

    fprintf('Maximum |Fx|: %.8f N\n', ...
        results.maximum_Fx(i));

    fprintf('Maximum |Fy|: %.8f N\n', ...
        results.maximum_Fy(i));

    fprintf('Maximum cart displacement: %.8f m\n', ...
        results.maximum_displacement(i));

    fprintf('Maximum cart speed: %.8f m/s\n', ...
        results.maximum_speed(i));

    fprintf('Force saturation: %s\n', ...
        string(results.force_saturation(i)));

    fprintf('Stabilization: %s\n', ...
        string(results.stabilization_pass(i)));

end

%% 5. RESULTS TABLE

T = table( ...
    inclinationValues', ...
    results.final_inclination, ...
    results.maximum_Fx, ...
    results.maximum_Fy, ...
    results.maximum_displacement, ...
    results.maximum_speed, ...
    results.force_saturation, ...
    results.stabilization_pass, ...
    'VariableNames',{ ...
    'InitialInclination_deg', ...
    'FinalInclination_deg', ...
    'MaxFx_N', ...
    'MaxFy_N', ...
    'MaxDisplacement_m', ...
    'MaxSpeed_mps', ...
    'ForceSaturation', ...
    'StabilizationPass'});

fprintf('\n============================================\n');
fprintf('SATURATION BOUNDARY RESULTS\n');
fprintf('============================================\n');

disp(T);

%% 6. FORCE VS INITIAL INCLINATION

figure('Name','Actuator Saturation Boundary');

plot(inclinationValues,results.maximum_Fx,'o-', ...
    'LineWidth',1.5);

hold on;

plot(inclinationValues,results.maximum_Fy,'s-', ...
    'LineWidth',1.5);

yline(p.maximum_force,'--');

xlabel('Initial inclination (deg)');
ylabel('Maximum force component (N)');

title('Actuator Force vs Initial Inclination');

legend('Maximum |Fx|','Maximum |Fy|','30 N limit', ...
    'Location','best');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_18_ActuatorSaturationBoundary.png'));

%% 7. SUMMARY

fprintf('\n============================================\n');
fprintf('PHASE 10.18 SUMMARY\n');
fprintf('============================================\n');

unsaturated = ~results.force_saturation & ...
    results.stabilization_pass;

if any(unsaturated)

    maximumUnsaturatedAngle = ...
        max(inclinationValues(unsaturated));

    fprintf('Largest tested unsaturated stabilization angle: %.2f deg\n', ...
        maximumUnsaturatedAngle);

else

    fprintf('No unsaturated stabilization case found.\n');

end

if any(results.force_saturation)

    firstSaturatedAngle = ...
        min(inclinationValues(results.force_saturation));

    fprintf('First tested saturated inclination: %.2f deg\n', ...
        firstSaturatedAngle);

end

%% 8. SAVE

results.results_table = T;
results.parameters = p;
results.controller_gain = K;
results.maximum_force = p.maximum_force;

save(fullfile(scriptDir, ...
    'phase_10_18_actuator_saturation_boundary_results.mat'), ...
    'results');

writetable(T,fullfile(scriptDir, ...
    'Phase_10_18_ActuatorSaturationBoundary.csv'));

fprintf('\nResults saved successfully.\n');

fprintf('\n============================================\n');
fprintf('PHASE 10.18 COMPLETED\n');
fprintf('============================================\n');

