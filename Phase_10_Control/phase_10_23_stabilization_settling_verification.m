
%% PHASE 10.23: STABILIZATION AND SETTLING VERIFICATION

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 10.23: SETTLING VERIFICATION\n');
fprintf('============================================\n');

%% 1. LOAD MODEL AND CONTROLLER

scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);

addpath(fullfile(projectDir,'Phase_4_MATLAB_Simulation'));

p = system_parameters();

loaded = load(fullfile(scriptDir,'controller_data.mat'));

K = loaded.controller.K;
p.maximum_force = loaded.controller.maximum_force;

%% 2. TEST CASES

angles = 46.0:0.1:46.7;

numberOfCases = numel(angles);

phi0 = deg2rad(30);

time = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol',p.relative_tolerance, ...
    'AbsTol',p.absolute_tolerance);

%% 3. SETTLING CRITERIA

tiltLimit = 0.01;             % degrees
angularSpeedLimit = 0.001;    % rad/s
cartSpeedLimit = 0.01;        % m/s

%% 4. INITIALIZE RESULTS

finalTilt = zeros(numberOfCases,1);
lastWindowMaxTilt = zeros(numberOfCases,1);
lastWindowMaxAngularSpeed = zeros(numberOfCases,1);
finalCartSpeed = zeros(numberOfCases,1);
finalCartDisplacement = zeros(numberOfCases,1);

tiltPass = false(numberOfCases,1);
angularSpeedPass = false(numberOfCases,1);
cartSpeedPass = false(numberOfCases,1);
settlingPass = false(numberOfCases,1);

%% 5. RUN SIMULATIONS

for i = 1:numberOfCases

    theta0 = deg2rad(angles(i));

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

    %% Simulate

    [t,z] = ode45( ...
        @(t,z) controlled_cartesian_equations_of_motion( ...
        t,z,pCase,K), ...
        time,z0,options);

    %% Relative position and velocity

    rx = z(:,3)-z(:,1);
    ry = z(:,4)-z(:,2);
    rz = z(:,5);

    vx = z(:,8)-z(:,6);
    vy = z(:,9)-z(:,7);
    vz = z(:,10);

    rodLength = sqrt(rx.^2+ry.^2+rz.^2);

    %% Inclination

    inclination = acosd(max(-1,min(1,rz./rodLength)));

    %% Bob angular velocity

    cx = ry.*vz-rz.*vy;
    cy = rz.*vx-rx.*vz;
    cz = rx.*vy-ry.*vx;

    angularSpeed = sqrt(cx.^2+cy.^2+cz.^2)./(rodLength.^2);

    %% Final one-second interval

    lastWindow = t >= (p.t_end-1);

    finalTilt(i) = inclination(end);

    lastWindowMaxTilt(i) = max(inclination(lastWindow));

    lastWindowMaxAngularSpeed(i) = ...
        max(angularSpeed(lastWindow));

    finalCartSpeed(i) = hypot(z(end,6),z(end,7));

    finalCartDisplacement(i) = ...
        hypot(z(end,1),z(end,2));

    %% Individual criteria

    tiltPass(i) = lastWindowMaxTilt(i) < tiltLimit;

    angularSpeedPass(i) = ...
        lastWindowMaxAngularSpeed(i) < angularSpeedLimit;

    cartSpeedPass(i) = ...
        finalCartSpeed(i) < cartSpeedLimit;

    settlingPass(i) = ...
        tiltPass(i) && angularSpeedPass(i) && cartSpeedPass(i);

    %% Display

    fprintf('\n--------------------------------------------\n');
    fprintf('Initial inclination: %.1f deg\n',angles(i));

    fprintf('Final inclination: %.8f deg\n',finalTilt(i));

    fprintf('Maximum inclination in final second: %.8f deg\n', ...
        lastWindowMaxTilt(i));

    fprintf('Maximum angular speed in final second: %.8e rad/s\n', ...
        lastWindowMaxAngularSpeed(i));

    fprintf('Final cart speed: %.8e m/s\n',finalCartSpeed(i));

    fprintf('Final cart displacement: %.8f m\n', ...
        finalCartDisplacement(i));

    fprintf('Tilt criterion: %s\n',string(tiltPass(i)));

    fprintf('Angular speed criterion: %s\n', ...
        string(angularSpeedPass(i)));

    fprintf('Cart speed criterion: %s\n', ...
        string(cartSpeedPass(i)));

    fprintf('Settling verification: %s\n', ...
        string(settlingPass(i)));

end

%% 6. RESULTS TABLE

T = table( ...
    angles', ...
    finalTilt, ...
    lastWindowMaxTilt, ...
    lastWindowMaxAngularSpeed, ...
    finalCartSpeed, ...
    finalCartDisplacement, ...
    tiltPass, ...
    angularSpeedPass, ...
    cartSpeedPass, ...
    settlingPass, ...
    'VariableNames',{ ...
    'InitialInclination_deg', ...
    'FinalTilt_deg', ...
    'LastSecondMaxTilt_deg', ...
    'LastSecondMaxAngularSpeed_radps', ...
    'FinalCartSpeed_mps', ...
    'FinalCartDisplacement_m', ...
    'TiltPass', ...
    'AngularSpeedPass', ...
    'CartSpeedPass', ...
    'SettlingPass'});

fprintf('\n============================================\n');
fprintf('PHASE 10.23 SETTLING VERIFICATION RESULTS\n');
fprintf('============================================\n');

disp(T);

%% 7. SUMMARY

fprintf('\n============================================\n');
fprintf('PHASE 10.23 SUMMARY\n');
fprintf('============================================\n');

fprintf('Cases passing all settling criteria: %d / %d\n', ...
    sum(settlingPass),numberOfCases);

successful = angles(settlingPass);
failed = angles(~settlingPass);

if ~isempty(successful)
    fprintf('Largest verified settling angle: %.1f deg\n', ...
        max(successful));
end

if ~isempty(failed)
    fprintf('Smallest case failing settling verification: %.1f deg\n', ...
        min(failed));
end

%% 8. SAVE RESULTS

results.angles = angles;
results.final_tilt = finalTilt;
results.last_window_max_tilt = lastWindowMaxTilt;
results.last_window_max_angular_speed = lastWindowMaxAngularSpeed;
results.final_cart_speed = finalCartSpeed;
results.final_cart_displacement = finalCartDisplacement;

results.tilt_pass = tiltPass;
results.angular_speed_pass = angularSpeedPass;
results.cart_speed_pass = cartSpeedPass;
results.settling_pass = settlingPass;

results.tilt_limit = tiltLimit;
results.angular_speed_limit = angularSpeedLimit;
results.cart_speed_limit = cartSpeedLimit;

results.results_table = T;

results.parameters = p;
results.controller_gain = K;

save(fullfile(scriptDir, ...
    'phase_10_23_stabilization_settling_verification_results.mat'), ...
    'results');

writetable(T,fullfile(scriptDir, ...
    'Phase_10_23_SettlingVerification.csv'));

fprintf('\nResults saved successfully.\n');

fprintf('\n============================================\n');
fprintf('PHASE 10.23 COMPLETED\n');
fprintf('============================================\n');

