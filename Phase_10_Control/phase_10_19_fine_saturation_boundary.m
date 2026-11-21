
%% PHASE 10.19: FINE ACTUATOR SATURATION BOUNDARY

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 10.19: FINE SATURATION BOUNDARY\n');
fprintf('============================================\n');

%% 1. LOAD CONTROLLER

scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);

addpath(fullfile(projectDir, ...
    'Phase_4_MATLAB_Simulation'));

p = system_parameters();

loaded = load(fullfile(scriptDir,'controller_data.mat'));

K = loaded.controller.K;

p.maximum_force = loaded.controller.maximum_force;

%% 2. TEST ANGLES

inclinationValues = 20:0.1:21;

n = numel(inclinationValues);

phi0 = deg2rad(30);

time = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol',p.relative_tolerance, ...
    'AbsTol',p.absolute_tolerance);

%% 3. RESULT ARRAYS

maxFx = zeros(n,1);
maxFy = zeros(n,1);

finalInclination = zeros(n,1);

maxDisplacement = zeros(n,1);

saturation = false(n,1);

%% 4. SIMULATION LOOP

for i = 1:n

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

    %% Simulate

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

    %% Inclination

    rx = z(:,3)-z(:,1);
    ry = z(:,4)-z(:,2);
    rz = z(:,5);

    rodLength = sqrt(rx.^2+ry.^2+rz.^2);

    inclination = acosd(max(-1,min(1, ...
        rz./rodLength)));

    %% Store results

    maxFx(i) = max(abs(force(:,1)));

    maxFy(i) = max(abs(force(:,2)));

    finalInclination(i) = inclination(end);

    maxDisplacement(i) = max(sqrt( ...
        z(:,1).^2+z(:,2).^2));

    saturation(i) = ...
        any(abs(force(:,1)) >= p.maximum_force-1e-8) || ...
        any(abs(force(:,2)) >= p.maximum_force-1e-8);

    fprintf(['Angle: %5.1f deg | Fx: %9.6f N | ' ...
        'Fy: %9.6f N | Final tilt: %.8f deg | Saturation: %s\n'], ...
        inclinationValues(i),maxFx(i),maxFy(i), ...
        finalInclination(i),string(saturation(i)));

end

%% 5. RESULTS TABLE

T = table( ...
    inclinationValues', ...
    maxFx, ...
    maxFy, ...
    finalInclination, ...
    maxDisplacement, ...
    saturation, ...
    'VariableNames',{ ...
    'InitialInclination_deg', ...
    'MaxFx_N', ...
    'MaxFy_N', ...
    'FinalInclination_deg', ...
    'MaxDisplacement_m', ...
    'ForceSaturation'});

fprintf('\n============================================\n');
fprintf('FINE SATURATION BOUNDARY RESULTS\n');
fprintf('============================================\n');

disp(T);

%% 6. DETERMINE FIRST SATURATED CASE

firstSaturated = find(saturation,1,'first');

if isempty(firstSaturated)

    fprintf('No saturation detected in the tested interval.\n');

else

    fprintf('First tested saturated angle: %.1f deg\n', ...
        inclinationValues(firstSaturated));

    if firstSaturated > 1

        fprintf('Previous unsaturated angle: %.1f deg\n', ...
            inclinationValues(firstSaturated-1));

    end

end

%% 7. FORCE PLOT

figure('Name','Fine Saturation Boundary');

plot(inclinationValues,maxFx,'o-', ...
    'LineWidth',1.5);

hold on;

plot(inclinationValues,maxFy,'s-', ...
    'LineWidth',1.5);

yline(p.maximum_force,'--');

xlabel('Initial inclination (deg)');
ylabel('Maximum force component (N)');

title('Fine Actuator Saturation Boundary');

legend('Maximum |Fx|','Maximum |Fy|','30 N limit', ...
    'Location','best');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_19_FineSaturationBoundary.png'));

%% 8. SAVE RESULTS

results = struct();

results.results_table = T;
results.initial_inclination = inclinationValues;
results.maximum_Fx = maxFx;
results.maximum_Fy = maxFy;
results.final_inclination = finalInclination;
results.maximum_displacement = maxDisplacement;
results.force_saturation = saturation;
results.parameters = p;
results.controller_gain = K;

save(fullfile(scriptDir, ...
    'phase_10_19_fine_saturation_boundary_results.mat'), ...
    'results');

writetable(T,fullfile(scriptDir, ...
    'Phase_10_19_FineSaturationBoundary.csv'));

fprintf('\nResults saved successfully.\n');

fprintf('\n============================================\n');
fprintf('PHASE 10.19 COMPLETED\n');
fprintf('============================================\n');
