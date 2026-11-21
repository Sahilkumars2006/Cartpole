
%% PHASE 10.11: INITIAL AZIMUTH ROBUSTNESS
%
% Tests fixed-gain LQR performance for different
% initial pendulum azimuth angles.

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 10.11: INITIAL AZIMUTH ROBUSTNESS\n');
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

%% 2. INITIAL AZIMUTH VARIATIONS

azimuthValues = [0 30 60 90 120];

numberOfCases = numel(azimuthValues);

%% 3. FIXED INITIAL CONDITIONS

theta0 = deg2rad(5);

x0 = p.x0;
y0 = p.y0;

time = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol',p.relative_tolerance, ...
    'AbsTol',p.absolute_tolerance);

%% 4. RESULT ARRAYS

results = struct();

results.initial_azimuth = azimuthValues;

results.final_inclination = zeros(numberOfCases,1);
results.maximum_inclination = zeros(numberOfCases,1);

results.maximum_cart_displacement = zeros(numberOfCases,1);
results.maximum_cart_speed = zeros(numberOfCases,1);

results.maximum_Fx = zeros(numberOfCases,1);
results.maximum_Fy = zeros(numberOfCases,1);

results.maximum_radial_force = zeros(numberOfCases,1);

results.final_cart_position = zeros(numberOfCases,2);
results.maximum_rod_error = zeros(numberOfCases,1);

results.simulation_states = cell(numberOfCases,1);
results.control_forces = cell(numberOfCases,1);
results.inclination_history = cell(numberOfCases,1);

%% 5. RUN ROBUSTNESS CASES

for i = 1:numberOfCases

    fprintf('\n--------------------------------------------\n');
    fprintf('CASE %d / %d\n',i,numberOfCases);
    fprintf('Initial azimuth: %.2f deg\n',azimuthValues(i));

    pCase = p;

    phi0 = deg2rad(azimuthValues(i));

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

    radialForce = sqrt(sum(force.^2,2));

    %% Store results

    results.final_inclination(i) = inclination(end);

    results.maximum_inclination(i) = max(inclination);

    results.maximum_cart_displacement(i) = max(displacement);

    results.maximum_cart_speed(i) = max(speed);

    results.maximum_Fx(i) = max(abs(force(:,1)));

    results.maximum_Fy(i) = max(abs(force(:,2)));

    results.maximum_radial_force(i) = max(radialForce);

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

    fprintf('Maximum radial force: %.8f N\n', ...
        results.maximum_radial_force(i));

end

%% 6. RESULTS TABLE

fprintf('\n============================================\n');
fprintf('INITIAL AZIMUTH ROBUSTNESS RESULTS\n');
fprintf('============================================\n');

T = table( ...
    azimuthValues', ...
    results.final_inclination, ...
    results.maximum_cart_displacement, ...
    results.maximum_cart_speed, ...
    results.maximum_Fx, ...
    results.maximum_Fy, ...
    results.maximum_radial_force, ...
    results.maximum_rod_error, ...
    'VariableNames',{ ...
    'InitialAzimuth_deg', ...
    'FinalInclination_deg', ...
    'MaxDisplacement_m', ...
    'MaxSpeed_mps', ...
    'MaxFx_N', ...
    'MaxFy_N', ...
    'MaxRadialForce_N', ...
    'MaxRodError_m'});

disp(T);

%% 7. PLOTS

figure('Name','Azimuth vs Stabilization');

plot(azimuthValues, ...
    results.final_inclination,'o-', ...
    'LineWidth',1.5);

xlabel('Initial azimuth (deg)');
ylabel('Final inclination (deg)');
title('Initial Azimuth vs Final Pendulum Inclination');
grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_11_Azimuth_Stabilization.png'));

figure('Name','Azimuth vs Displacement');

plot(azimuthValues, ...
    results.maximum_cart_displacement,'o-', ...
    'LineWidth',1.5);

xlabel('Initial azimuth (deg)');
ylabel('Maximum cart displacement (m)');
title('Initial Azimuth vs Maximum Cart Displacement');
grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_11_Azimuth_Displacement.png'));

figure('Name','Azimuth vs Control Force');

plot(azimuthValues,results.maximum_Fx,'o-', ...
    'LineWidth',1.5);

hold on;

plot(azimuthValues,results.maximum_Fy,'s-', ...
    'LineWidth',1.5);

yline(p.maximum_force,'--');

xlabel('Initial azimuth (deg)');
ylabel('Maximum control force (N)');
title('Initial Azimuth vs Control Effort');

legend('Maximum |Fx|','Maximum |Fy|','Force limit');
grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_11_Azimuth_ControlForce.png'));

%% 8. CART TRAJECTORY COMPARISON

figure('Name','Azimuth Cart Trajectory');

hold on;

for i = 1:numberOfCases

    z = results.simulation_states{i};

    plot(z(:,1),z(:,2),'LineWidth',1.5);

end

xlabel('Cart X position (m)');
ylabel('Cart Y position (m)');
title('Cart Trajectories for Different Initial Azimuths');

legend(compose('Azimuth %g deg',azimuthValues), ...
    'Location','best');

axis equal;
grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_11_Azimuth_CartTrajectories.png'));

%% 9. SAVE RESULTS

results.time = time;
results.parameters = p;
results.controller_gain = K;
results.results_table = T;

outputFile = fullfile(scriptDir, ...
    'phase_10_11_initial_azimuth_robustness_results.mat');

save(outputFile,'results');

writetable(T,fullfile(scriptDir, ...
    'Phase_10_11_InitialAzimuth_Robustness.csv'));

fprintf('\nResults saved:\n%s\n',outputFile);

fprintf('\n============================================\n');
fprintf('PHASE 10.11 COMPLETED\n');
fprintf('============================================\n');

