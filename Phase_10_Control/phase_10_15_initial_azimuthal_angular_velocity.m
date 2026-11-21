
%% PHASE 10.15: INITIAL AZIMUTHAL ANGULAR VELOCITY ROBUSTNESS

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 10.15: INITIAL AZIMUTHAL ANGULAR VELOCITY\n');
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

%% 2. INITIAL AZIMUTHAL RATE VALUES

azimuthRateValues = [-0.5 -0.25 0 0.25 0.5];

numberOfCases = numel(azimuthRateValues);

%% 3. FIXED INITIAL CONDITIONS

theta0 = p.theta0;
phi0 = p.phi0;

time = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol',p.relative_tolerance, ...
    'AbsTol',p.absolute_tolerance);

%% 4. RESULT ARRAYS

results = struct();

results.initial_phi_rate = azimuthRateValues;

results.final_inclination = zeros(numberOfCases,1);
results.maximum_inclination = zeros(numberOfCases,1);

results.maximum_cart_displacement = zeros(numberOfCases,1);
results.maximum_cart_speed = zeros(numberOfCases,1);

results.maximum_Fx = zeros(numberOfCases,1);
results.maximum_Fy = zeros(numberOfCases,1);
results.maximum_radial_force = zeros(numberOfCases,1);

results.maximum_rod_error = zeros(numberOfCases,1);

results.simulation_states = cell(numberOfCases,1);
results.control_forces = cell(numberOfCases,1);
results.inclination_history = cell(numberOfCases,1);

%% 5. RUN SIMULATIONS

for i = 1:numberOfCases

    fprintf('\n--------------------------------------------\n');
    fprintf('CASE %d / %d\n',i,numberOfCases);

    fprintf('Initial azimuthal rate: %.3f rad/s\n', ...
        azimuthRateValues(i));

    pCase = p;

    pCase.dx0 = 0;
    pCase.dy0 = 0;

    pCase.dtheta0 = 0;
    pCase.dphi0 = azimuthRateValues(i);

    %% Initial bob position

    X0 = pCase.x0 + pCase.L*sin(theta0)*cos(phi0);

    Y0 = pCase.y0 + pCase.L*sin(theta0)*sin(phi0);

    Z0 = pCase.L*cos(theta0);

    %% Initial bob velocity

    thetaRate = pCase.dtheta0;
    phiRate = pCase.dphi0;

    dX0 = pCase.L*( ...
        cos(theta0)*thetaRate*cos(phi0) ...
        - sin(theta0)*phiRate*sin(phi0));

    dY0 = pCase.L*( ...
        cos(theta0)*thetaRate*sin(phi0) ...
        + sin(theta0)*phiRate*cos(phi0));

    dZ0 = -pCase.L*sin(theta0)*thetaRate;

    %% Initial Cartesian state

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

    %% Control forces

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

    results.simulation_states{i} = z;
    results.control_forces{i} = force;
    results.inclination_history{i} = inclination;

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

end

%% 6. RESULTS TABLE

fprintf('\n============================================\n');
fprintf('INITIAL AZIMUTHAL VELOCITY RESULTS\n');
fprintf('============================================\n');

T = table( ...
    azimuthRateValues', ...
    results.final_inclination, ...
    results.maximum_inclination, ...
    results.maximum_cart_displacement, ...
    results.maximum_cart_speed, ...
    results.maximum_Fx, ...
    results.maximum_Fy, ...
    results.maximum_radial_force, ...
    results.maximum_rod_error, ...
    'VariableNames',{ ...
    'InitialPhiRate_radps', ...
    'FinalInclination_deg', ...
    'MaxInclination_deg', ...
    'MaxDisplacement_m', ...
    'MaxSpeed_mps', ...
    'MaxFx_N', ...
    'MaxFy_N', ...
    'MaxRadialForce_N', ...
    'MaxRodError_m'});

disp(T);

%% 7. STABILIZATION PLOT

figure('Name','Initial Azimuthal Velocity Stabilization');

hold on;

for i = 1:numberOfCases

    plot(t,results.inclination_history{i}, ...
        'LineWidth',1.4);

end

xlabel('Time (s)');
ylabel('Pendulum inclination (deg)');

title('Stabilization Under Initial Azimuthal Angular Velocity');

legend(compose('Initial rate %g rad/s', ...
    azimuthRateValues),'Location','best');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_15_AzimuthalVelocity_Stabilization.png'));

%% 8. CART DISPLACEMENT PLOT

figure('Name','Azimuthal Velocity Cart Displacement');

plot(azimuthRateValues, ...
    results.maximum_cart_displacement,'o-', ...
    'LineWidth',1.5);

xlabel('Initial azimuthal rate (rad/s)');
ylabel('Maximum cart displacement (m)');

title('Azimuthal Angular Velocity vs Cart Displacement');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_15_AzimuthalVelocity_Displacement.png'));

%% 9. CONTROL FORCE PLOT

figure('Name','Azimuthal Velocity Control Force');

plot(azimuthRateValues,results.maximum_Fx,'o-', ...
    'LineWidth',1.5);

hold on;

plot(azimuthRateValues,results.maximum_Fy,'s-', ...
    'LineWidth',1.5);

yline(p.maximum_force,'--');

xlabel('Initial azimuthal rate (rad/s)');
ylabel('Maximum control force (N)');

title('Azimuthal Angular Velocity vs Control Effort');

legend('Maximum |Fx|','Maximum |Fy|','Force limit');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_15_AzimuthalVelocity_ControlForce.png'));

%% 10. CART TRAJECTORIES

figure('Name','Azimuthal Velocity Cart Trajectories');

hold on;

for i = 1:numberOfCases

    z = results.simulation_states{i};

    plot(z(:,1),z(:,2),'LineWidth',1.4);

end

xlabel('Cart X position (m)');
ylabel('Cart Y position (m)');

title('Cart Trajectories Under Initial Azimuthal Velocity');

legend(compose('Initial rate %g rad/s', ...
    azimuthRateValues),'Location','best');

axis equal;
grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_15_AzimuthalVelocity_CartTrajectories.png'));

%% 11. SAVE RESULTS

results.time = time;
results.parameters = p;
results.controller_gain = K;
results.results_table = T;

save(fullfile(scriptDir, ...
    'phase_10_15_azimuthal_velocity_robustness_results.mat'), ...
    'results');

writetable(T,fullfile(scriptDir, ...
    'Phase_10_15_AzimuthalVelocity_Robustness.csv'));

fprintf('\nResults saved successfully.\n');

fprintf('\n============================================\n');
fprintf('PHASE 10.15 COMPLETED\n');
fprintf('============================================\n');

