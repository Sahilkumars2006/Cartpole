
%% PHASE 10.10: INITIAL INCLINATION ROBUSTNESS
%
% Tests fixed-gain LQR performance for different
% initial pendulum inclination angles.

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 10.10: INITIAL INCLINATION ROBUSTNESS\n');
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

%% 2. INITIAL INCLINATION VARIATIONS

inclinationValues = [1 3 5 7 10];

numberOfCases = numel(inclinationValues);

%% 3. INITIAL CONDITIONS

phi0 = deg2rad(30);

x0 = p.x0;
y0 = p.y0;

time = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol',p.relative_tolerance, ...
    'AbsTol',p.absolute_tolerance);

%% 4. RESULT ARRAYS

results = struct();

results.initial_inclination = inclinationValues;

results.final_inclination = zeros(numberOfCases,1);
results.maximum_inclination = zeros(numberOfCases,1);

results.maximum_cart_displacement = zeros(numberOfCases,1);
results.maximum_cart_speed = zeros(numberOfCases,1);

results.maximum_Fx = zeros(numberOfCases,1);
results.maximum_Fy = zeros(numberOfCases,1);

results.final_cart_position = zeros(numberOfCases,2);
results.maximum_rod_error = zeros(numberOfCases,1);

results.simulation_states = cell(numberOfCases,1);
results.control_forces = cell(numberOfCases,1);
results.inclination_history = cell(numberOfCases,1);

%% 5. RUN ROBUSTNESS CASES

for i = 1:numberOfCases

    fprintf('\n--------------------------------------------\n');
    fprintf('CASE %d / %d\n',i,numberOfCases);
    fprintf('Initial inclination: %.2f deg\n', ...
        inclinationValues(i));

    pCase = p;

    theta0 = deg2rad(inclinationValues(i));

    pCase.theta0 = theta0;
    pCase.phi0 = phi0;

    % Initial bob position

    X0 = x0 + pCase.L*sin(theta0)*cos(phi0);
    Y0 = y0 + pCase.L*sin(theta0)*sin(phi0);
    Z0 = pCase.L*cos(theta0);

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

    [t,z] = ode45( ...
        @(t,z) controlled_cartesian_equations_of_motion( ...
        t,z,pCase,K), ...
        time,z0,options);

    %% Control forces

    force = zeros(numel(t),2);

    for j = 1:numel(t)

        [~,u] = controlled_cartesian_equations_of_motion( ...
            t(j),z(j,:)',pCase,K);

        force(j,:) = u';

    end

    %% Positions and velocities

    cart = z(:,1:2);

    relative = [
        z(:,3)-z(:,1), ...
        z(:,4)-z(:,2), ...
        z(:,5)
    ];

    rodLength = sqrt(sum(relative.^2,2));

    inclination = acosd(max(-1,min(1, ...
        relative(:,3)./rodLength)));

    displacement = sqrt(sum(cart.^2,2));

    speed = sqrt(z(:,6).^2+z(:,7).^2);

    %% Store results

    results.final_inclination(i) = inclination(end);

    results.maximum_inclination(i) = max(inclination);

    results.maximum_cart_displacement(i) = max(displacement);

    results.maximum_cart_speed(i) = max(speed);

    results.maximum_Fx(i) = max(abs(force(:,1)));

    results.maximum_Fy(i) = max(abs(force(:,2)));

    results.final_cart_position(i,:) = cart(end,:);

    results.maximum_rod_error(i) = ...
        max(abs(rodLength-pCase.L));

    results.simulation_states{i} = z;
    results.control_forces{i} = force;
    results.inclination_history{i} = inclination;

    fprintf('Final inclination: %.8f deg\n', ...
        results.final_inclination(i));

    fprintf('Maximum cart displacement: %.8f m\n', ...
        results.maximum_cart_displacement(i));

    fprintf('Maximum cart speed: %.8f m/s\n', ...
        results.maximum_cart_speed(i));

    fprintf('Maximum |Fx|: %.8f N\n', ...
        results.maximum_Fx(i));

    fprintf('Maximum |Fy|: %.8f N\n', ...
        results.maximum_Fy(i));

end

%% 6. RESULTS TABLE

fprintf('\n============================================\n');
fprintf('INITIAL INCLINATION ROBUSTNESS RESULTS\n');
fprintf('============================================\n');

T = table( ...
    inclinationValues', ...
    results.final_inclination, ...
    results.maximum_cart_displacement, ...
    results.maximum_cart_speed, ...
    results.maximum_Fx, ...
    results.maximum_Fy, ...
    results.maximum_rod_error, ...
    'VariableNames',{ ...
    'InitialInclination_deg', ...
    'FinalInclination_deg', ...
    'MaxDisplacement_m', ...
    'MaxSpeed_mps', ...
    'MaxFx_N', ...
    'MaxFy_N', ...
    'MaxRodError_m'});

disp(T);

%% 7. PLOTS

figure('Name','Initial Inclination vs Stabilization');

plot(inclinationValues, ...
    results.final_inclination,'o-', ...
    'LineWidth',1.5);

xlabel('Initial inclination (deg)');
ylabel('Final inclination (deg)');
title('Initial Inclination vs Final Pendulum Inclination');
grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_10_InitialInclination_Stabilization.png'));

figure('Name','Initial Inclination vs Displacement');

plot(inclinationValues, ...
    results.maximum_cart_displacement,'o-', ...
    'LineWidth',1.5);

xlabel('Initial inclination (deg)');
ylabel('Maximum cart displacement (m)');
title('Initial Inclination vs Maximum Cart Displacement');
grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_10_InitialInclination_Displacement.png'));

figure('Name','Initial Inclination vs Control Force');

plot(inclinationValues,results.maximum_Fx,'o-', ...
    'LineWidth',1.5);

hold on;

plot(inclinationValues,results.maximum_Fy,'s-', ...
    'LineWidth',1.5);

yline(p.maximum_force,'--');

xlabel('Initial inclination (deg)');
ylabel('Maximum control force (N)');
title('Initial Inclination vs Control Effort');

legend('Maximum |Fx|','Maximum |Fy|','Force limit');
grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_10_InitialInclination_ControlForce.png'));

%% 8. SAVE RESULTS

results.time = time;
results.parameters = p;
results.controller_gain = K;
results.results_table = T;

outputFile = fullfile(scriptDir, ...
    'phase_10_10_initial_inclination_robustness_results.mat');

save(outputFile,'results');

writetable(T,fullfile(scriptDir, ...
    'Phase_10_10_InitialInclination_Robustness.csv'));

fprintf('\nResults saved:\n%s\n',outputFile);

fprintf('\n============================================\n');
fprintf('PHASE 10.10 COMPLETED\n');
fprintf('============================================\n');
