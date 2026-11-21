
%% PHASE 10.24: CONTROL PERFORMANCE AND EFFICIENCY ASSESSMENT

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 10.24: CONTROL PERFORMANCE ASSESSMENT\n');
fprintf('============================================\n');

%% 1. LOAD EXISTING RESULTS

scriptDir = fileparts(mfilename('fullpath'));

fileFine = fullfile(scriptDir, ...
    'phase_10_22_fine_stabilization_boundary_results.mat');

fileSettling = fullfile(scriptDir, ...
    'phase_10_23_stabilization_settling_verification_results.mat');

if ~isfile(fileFine)
    error('Phase 10.22 results file not found: %s',fileFine);
end

if ~isfile(fileSettling)
    error('Phase 10.23 results file not found: %s',fileSettling);
end

fineData = load(fileFine);
settlingData = load(fileSettling);

if ~isfield(fineData,'results') || ~isfield(settlingData,'results')
    error('Expected results structure is missing from a MAT file.');
end

fine = fineData.results;
settling = settlingData.results;

fprintf('\nExisting results loaded successfully.\n');

%% 2. VERIFY DATA CONSISTENCY

requiredFine = { ...
    'initial_inclination', ...
    'maximum_cart_displacement', ...
    'maximum_cart_speed', ...
    'maximum_Fx', ...
    'maximum_Fy', ...
    'maximum_force', ...
    'parameters'};

requiredSettling = { ...
    'angles', ...
    'last_window_max_tilt', ...
    'last_window_max_angular_speed', ...
    'final_cart_speed', ...
    'final_cart_displacement', ...
    'settling_pass', ...
    'tilt_limit', ...
    'angular_speed_limit', ...
    'cart_speed_limit'};

for k = 1:numel(requiredFine)

    if ~isfield(fine,requiredFine{k})
        error('Missing Phase 10.22 field: %s',requiredFine{k});
    end

end

for k = 1:numel(requiredSettling)

    if ~isfield(settling,requiredSettling{k})
        error('Missing Phase 10.23 field: %s',requiredSettling{k});
    end

end

% Phase 10.22 uses initial_inclination.
% Phase 10.23 uses angles.

% Phase 10.22 contains 11 cases (46.0 to 47.0 deg).
% Phase 10.23 contains 8 cases (46.0 to 46.7 deg).
% Match the common inclination cases.

fineAngles = fine.initial_inclination(:);
settlingAngles = settling.angles(:);

fineIdx = zeros(numel(settlingAngles),1);

for k = 1:numel(settlingAngles)

    [difference,idx] = min(abs(fineAngles-settlingAngles(k)));

    if difference > 1e-8
        error('No matching Phase 10.22 case for %.4f degrees.', ...
            settlingAngles(k));
    end

    fineIdx(k) = idx;

end

angles = settlingAngles;
numberOfCases = numel(angles);

fprintf('\nDataset alignment successful.\n');
fprintf('Phase 10.22 cases available: %d\n',numel(fineAngles));
fprintf('Phase 10.23 cases available: %d\n',numel(settlingAngles));
fprintf('Common cases selected: %d\n',numberOfCases);

if numel(angles) ~= numel(settlingAngles)
    error('Phase 10.22 and Phase 10.23 datasets have different sizes.');
end

if any(abs(angles-settlingAngles) > 1e-8)
    error('Initial inclination values do not match between datasets.');
end

numberOfCases = numel(angles);

%% 3. EXTRACT PERFORMANCE DATA

maximumForce = fine.maximum_force;

if isempty(maximumForce) || ~isscalar(maximumForce) || maximumForce <= 0
    error('Maximum actuator force is missing or invalid.');
end

if ~isfield(fine.parameters,'L')
    error('Rod length L is missing from Phase 10.22 parameters.');
end

% Extract only Phase 10.22 cases matching Phase 10.23.

maxDisplacement = fine.maximum_cart_displacement(fineIdx);
maxSpeed = fine.maximum_cart_speed(fineIdx);

maxFx = fine.maximum_Fx(fineIdx);
maxFy = fine.maximum_Fy(fineIdx);

lastSecondTilt = settling.last_window_max_tilt(:);
lastSecondAngularSpeed = ...
    settling.last_window_max_angular_speed(:);

finalCartSpeed = settling.final_cart_speed(:);
finalCartDisplacement = settling.final_cart_displacement(:);

settlingPass = logical(settling.settling_pass(:));

% Check all extracted vectors have consistent dimensions.

dataVectors = { ...
    maxDisplacement, maxSpeed, maxFx, maxFy, ...
    lastSecondTilt, lastSecondAngularSpeed, ...
    finalCartSpeed, finalCartDisplacement, settlingPass};

for k = 1:numel(dataVectors)

    if numel(dataVectors{k}) ~= numberOfCases
        error('Performance data vector %d has an inconsistent size.',k);
    end

end

%% 4. CALCULATE PERFORMANCE INDICATORS

% Peak componentwise actuator utilization.

utilizationX = 100*maxFx/maximumForce;
utilizationY = 100*maxFy/maximumForce;

% Highest individual component utilization.
% X and Y peaks may occur at different times.

peakComponentUtilization = max(utilizationX,utilizationY);

% Detect whether either actuator component reached its limit.

saturationObserved = ...
    maxFx >= maximumForce-1e-8 | ...
    maxFy >= maximumForce-1e-8;

% Normalize displacement using rod length.

rodLength = fine.parameters.L;

normalizedDisplacement = maxDisplacement/rodLength;

% Settling margins relative to specified criteria.

tiltLimit = settling.tilt_limit;
angularSpeedLimit = settling.angular_speed_limit;
cartSpeedLimit = settling.cart_speed_limit;

tiltMargin = 100*(1-lastSecondTilt/tiltLimit);

angularSpeedMargin = ...
    100*(1-lastSecondAngularSpeed/angularSpeedLimit);

cartSpeedMargin = ...
    100*(1-finalCartSpeed/cartSpeedLimit);

%% 5. CREATE PERFORMANCE TABLE

T = table( ...
    angles, ...
    maxDisplacement, ...
    normalizedDisplacement, ...
    maxSpeed, ...
    maxFx, ...
    maxFy, ...
    utilizationX, ...
    utilizationY, ...
    peakComponentUtilization, ...
    saturationObserved, ...
    lastSecondTilt, ...
    lastSecondAngularSpeed, ...
    finalCartSpeed, ...
    finalCartDisplacement, ...
    tiltMargin, ...
    angularSpeedMargin, ...
    cartSpeedMargin, ...
    settlingPass, ...
    'VariableNames',{ ...
    'InitialInclination_deg', ...
    'MaxDisplacement_m', ...
    'DisplacementOverRodLength', ...
    'MaxCartSpeed_mps', ...
    'MaxFx_N', ...
    'MaxFy_N', ...
    'FxUtilization_percent', ...
    'FyUtilization_percent', ...
    'PeakComponentUtilization_percent', ...
    'SaturationObserved', ...
    'LastSecondMaxTilt_deg', ...
    'LastSecondMaxAngularSpeed_radps', ...
    'FinalCartSpeed_mps', ...
    'FinalCartDisplacement_m', ...
    'TiltMargin_percent', ...
    'AngularSpeedMargin_percent', ...
    'CartSpeedMargin_percent', ...
    'SettlingPass'});

fprintf('\n============================================\n');
fprintf('CONTROL PERFORMANCE TABLE\n');
fprintf('============================================\n');

disp(T);

%% 6. PERFORMANCE TREND ANALYSIS

fprintf('\n============================================\n');
fprintf('PERFORMANCE TREND ANALYSIS\n');
fprintf('============================================\n');

fprintf('Number of tested cases: %d\n',numberOfCases);

fprintf('Verified settling cases: %d / %d\n', ...
    sum(settlingPass),numberOfCases);

fprintf('Cases reaching actuator saturation: %d / %d\n', ...
    sum(saturationObserved),numberOfCases);

fprintf('\nMaximum cart displacement: %.6f m\n', ...
    max(maxDisplacement));

fprintf('Maximum cart speed: %.6f m/s\n', ...
    max(maxSpeed));

fprintf('Maximum X force: %.6f N\n',max(maxFx));

fprintf('Maximum Y force: %.6f N\n',max(maxFy));

fprintf('Maximum X utilization: %.4f %%\n',max(utilizationX));

fprintf('Maximum Y utilization: %.4f %%\n',max(utilizationY));

if any(settlingPass)

    fprintf('Largest tested settling angle: %.2f deg\n', ...
        max(angles(settlingPass)));

else

    fprintf('No tested case satisfied the settling criteria.\n');

end

if any(~settlingPass)

    fprintf('Smallest tested failing angle: %.2f deg\n', ...
        min(angles(~settlingPass)));

else

    fprintf('All tested cases satisfied the settling criteria.\n');

end

%% 7. PERFORMANCE CHANGE FROM 46.0 TO 46.6 DEGREES

firstCase = find(abs(angles-46.0)<1e-8,1);
lastCase = find(abs(angles-46.6)<1e-8,1);

if isempty(firstCase) || isempty(lastCase)

    warning('Could not locate both 46.0 and 46.6 degree cases.');

else

    displacementChange = 100*( ...
        maxDisplacement(lastCase)/maxDisplacement(firstCase)-1);

    speedChange = 100*( ...
        maxSpeed(lastCase)/maxSpeed(firstCase)-1);

    fprintf('\n--------------------------------------------\n');
    fprintf('PERFORMANCE CHANGE: 46.0 TO 46.6 DEGREES\n');
    fprintf('--------------------------------------------\n');

    fprintf('Cart displacement increase: %.4f %%\n', ...
        displacementChange);

    fprintf('Maximum cart speed increase: %.4f %%\n', ...
        speedChange);

    fprintf('46.6 degree final-second tilt: %.8f deg\n', ...
        lastSecondTilt(lastCase));

    fprintf('46.6 degree final cart speed: %.8e m/s\n', ...
        finalCartSpeed(lastCase));

end

%% 8. FIGURE 1: CART DISPLACEMENT

figure('Name','Phase 10.24 Cart Displacement');

plot(angles,maxDisplacement,'o-', ...
    'LineWidth',1.6,'MarkerSize',6);

xlabel('Initial inclination (deg)');
ylabel('Maximum cart displacement (m)');

title('Cart Displacement Near Stabilization Boundary');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_24_CartDisplacement.png'));

%% 9. FIGURE 2: CART SPEED

figure('Name','Phase 10.24 Cart Speed');

plot(angles,maxSpeed,'o-', ...
    'LineWidth',1.6,'MarkerSize',6);

xlabel('Initial inclination (deg)');
ylabel('Maximum cart speed (m/s)');

title('Maximum Cart Speed Near Stabilization Boundary');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_24_CartSpeed.png'));

%% 10. FIGURE 3: ACTUATOR UTILIZATION

figure('Name','Phase 10.24 Actuator Utilization');

plot(angles,utilizationX,'o-', ...
    'LineWidth',1.6,'MarkerSize',6);

hold on;

plot(angles,utilizationY,'s-', ...
    'LineWidth',1.6,'MarkerSize',6);

yline(100,'--','Actuator limit');

xlabel('Initial inclination (deg)');
ylabel('Peak actuator utilization (%)');

title('X and Y Actuator Peak Utilization');

legend('X actuator','Y actuator','Location','best');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_24_ActuatorUtilization.png'));

%% 11. FIGURE 4: SETTLING PERFORMANCE

figure('Name','Phase 10.24 Settling Performance');

plot(angles,lastSecondTilt,'o-', ...
    'LineWidth',1.6,'MarkerSize',6);

hold on;

yline(tiltLimit,'--','Tilt criterion');

xlabel('Initial inclination (deg)');
ylabel('Maximum final-second inclination (deg)');

title('Final-Second Pendulum Settling Performance');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_24_SettlingPerformance.png'));

%% 12. FIGURE 5: SUCCESS AND FAILURE

figure('Name','Phase 10.24 Stabilization Classification');

hold on;

plot(angles(settlingPass), ...
    ones(sum(settlingPass),1),'o', ...
    'MarkerSize',8,'LineWidth',1.5);

plot(angles(~settlingPass), ...
    zeros(sum(~settlingPass),1),'x', ...
    'MarkerSize',9,'LineWidth',1.8);

xlabel('Initial inclination (deg)');
ylabel('Settling classification');

yticks([0 1]);
yticklabels({'Failed','Settled'});

ylim([-0.25 1.25]);

title('Verified Stabilization Classification');

legend('Settled','Failed','Location','best');

grid on;

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_24_StabilizationClassification.png'));

%% 13. SAVE CONSOLIDATED ASSESSMENT

assessment = struct();

assessment.performance_table = T;

assessment.initial_inclination = angles;

assessment.maximum_force = maximumForce;

assessment.rod_length = rodLength;

assessment.maximum_cart_displacement = maxDisplacement;
assessment.maximum_cart_speed = maxSpeed;

assessment.maximum_Fx = maxFx;
assessment.maximum_Fy = maxFy;

assessment.X_utilization_percent = utilizationX;
assessment.Y_utilization_percent = utilizationY;

assessment.peak_component_utilization_percent = ...
    peakComponentUtilization;

assessment.saturation_observed = saturationObserved;

assessment.last_second_max_tilt = lastSecondTilt;
assessment.last_second_max_angular_speed = lastSecondAngularSpeed;

assessment.final_cart_speed = finalCartSpeed;
assessment.final_cart_displacement = finalCartDisplacement;

assessment.settling_pass = settlingPass;

assessment.tilt_margin_percent = tiltMargin;
assessment.angular_speed_margin_percent = angularSpeedMargin;
assessment.cart_speed_margin_percent = cartSpeedMargin;

assessment.note = ...
    ['Peak force utilization is componentwise. X and Y peak forces ' ...
     'may occur at different times. Exact saturation duration and ' ...
     'stabilization time are unavailable from the saved summaries.'];

save(fullfile(scriptDir, ...
    'phase_10_24_control_performance_assessment_results.mat'), ...
    'assessment');

writetable(T,fullfile(scriptDir, ...
    'Phase_10_24_ControlPerformanceAssessment.csv'));

%% 14. COMPLETION

fprintf('\n============================================\n');
fprintf('PHASE 10.24 COMPLETED\n');
fprintf('============================================\n');

fprintf('Existing simulation results reused.\n');
fprintf('No new simulations performed.\n');
fprintf('Controller gain unchanged.\n');
fprintf('Actuator limits unchanged.\n');

fprintf('\nAssessment MAT, CSV and figures saved.\n');

fprintf('\nOutput directory:\n%s\n',scriptDir);

fprintf('\n============================================\n');
fprintf('END OF PHASE 10.24\n');
fprintf('============================================\n');
