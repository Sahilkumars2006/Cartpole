
%% PHASE 10.8: ROD LENGTH ROBUSTNESS ANALYSIS
%
% Fixed-gain LQR robustness against rod length variations.
% Controller gain remains unchanged.

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 10.8: ROD LENGTH ROBUSTNESS ANALYSIS\n');
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

%% 2. ROD LENGTH VARIATIONS

rodLengthValues = [0.5 0.75 1.0 1.25 1.5];

numberOfCases = numel(rodLengthValues);

%% 3. INITIAL CONDITIONS

theta0 = p.theta0;
phi0 = p.phi0;

x0 = p.x0;
y0 = p.y0;

time = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol',p.relative_tolerance, ...
    'AbsTol',p.absolute_tolerance);

%% 4. RESULT ARRAYS

results = struct();

results.rod_length = rodLengthValues;

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
    fprintf('Rod length: %.2f m\n',rodLengthValues(i));

    pCase = p;
    pCase.L = rodLengthValues(i);

    % Reconstruct initial bob position using the
    % current rod length to preserve the initial angle.

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
fprintf('ROD LENGTH ROBUSTNESS RESULTS\n');
fprintf('============================================\n');

T = table( ...
    rodLengthValues', ...
    results.final_inclination, ...
    results.maximum_cart_displacement, ...
    results.maximum_cart_speed, ...
    results.maximum_Fx, ...
    results.maximum_Fy, ...
    results.maximum_rod_error, ...
    'VariableNames',{ ...
    'RodLength_m', ...
    'FinalInclination_deg', ...
    'MaxDisplacement_m', ...
    'MaxSpeed_mps', ...
    'MaxFx_N', ...
    'MaxFy_N', ...
    'MaxRodError_m'});

disp(T);

%% 7. PLOTS

figure('Name','Rod Length vs Stabilization');

plot(rodLengthValues, ...
    results.final_inclination,'o-', ...
    'LineWidth',1.5);

xlabel('Rod length (m)');
ylabel('Final inclination (deg)');
title('Rod Length vs Final Pendulum Inclination');
grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_8_RodLength_Stabilization.png'));

figure('Name','Rod Length vs Displacement');

plot(rodLengthValues, ...
    results.maximum_cart_displacement,'o-', ...
    'LineWidth',1.5);

xlabel('Rod length (m)');
ylabel('Maximum cart displacement (m)');
title('Rod Length vs Maximum Cart Displacement');
grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_8_RodLength_Displacement.png'));

figure('Name','Rod Length vs Control Force');

plot(rodLengthValues,results.maximum_Fx,'o-', ...
    'LineWidth',1.5);

hold on;

plot(rodLengthValues,results.maximum_Fy,'s-', ...
    'LineWidth',1.5);

yline(p.maximum_force,'--');

xlabel('Rod length (m)');
ylabel('Maximum control force (N)');
title('Rod Length vs Control Effort');

legend('Maximum |Fx|','Maximum |Fy|','Force limit');
grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_8_RodLength_ControlForce.png'));

%% 8. SAVE RESULTS

results.time = time;
results.parameters = p;
results.controller_gain = K;
results.results_table = T;

outputFile = fullfile(scriptDir, ...
    'phase_10_8_rod_length_robustness_results.mat');

save(outputFile,'results');

writetable(T,fullfile(scriptDir, ...
    'Phase_10_8_RodLength_Robustness.csv'));

fprintf('\nResults saved:\n%s\n',outputFile);

fprintf('\n============================================\n');
fprintf('PHASE 10.8 COMPLETED\n');
fprintf('============================================\n');

