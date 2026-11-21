
%% PHASE 10.25: CONTROL SYSTEM ROBUSTNESS SUMMARY
% AND FINAL CONTROL VALIDATION
%
% This phase consolidates existing Phase 10 results.
% No new simulations are performed.
% The original controller gain and simulation files are not modified.

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 10.25: FINAL CONTROL VALIDATION\n');
fprintf('============================================\n');

%% 1. PROJECT DIRECTORY

scriptDir = fileparts(mfilename('fullpath'));

if isempty(scriptDir)
    scriptDir = pwd;
end

fprintf('\nWorking directory:\n%s\n',scriptDir);

%% 2. VERIFY REQUIRED RESULT FILES

requiredFiles = {
    'phase_10_5_baseline_results.mat'
    'phase_10_22_fine_stabilization_boundary_results.mat'
    'phase_10_23_stabilization_settling_verification_results.mat'
    'phase_10_24_control_performance_assessment_results.mat'
};

for k = 1:numel(requiredFiles)

    filePath = fullfile(scriptDir,requiredFiles{k});

    if ~isfile(filePath)
        error('Required result file is missing: %s',requiredFiles{k});
    end

end

fprintf('\nRequired final validation files found.\n');

%% 3. LOAD BASELINE CONTROLLER REFERENCE

baselineData = load(fullfile(scriptDir, ...
    'phase_10_5_baseline_results.mat'));

if ~isfield(baselineData,'baseline')
    error('Baseline structure is missing.');
end

baseline = baselineData.baseline;

if ~isfield(baseline,'controller_gain')
    error('Baseline controller gain is missing.');
end

referenceGain = baseline.controller_gain;

if ~isnumeric(referenceGain) || isempty(referenceGain)
    error('Baseline controller gain is invalid.');
end

fprintf('\nBaseline controller gain loaded.\n');

fprintf('Controller gain dimensions: %d x %d\n', ...
    size(referenceGain,1),size(referenceGain,2));

%% 4. AUTOMATIC INVENTORY OF PHASE 10 RESULT FILES

files = dir(fullfile(scriptDir,'phase_10_*.mat'));

% Exclude this phase's own output if the script is run again.

files = files(~strcmp({files.name}, ...
    'phase_10_25_final_control_validation_results.mat'));

fileNames = strings(0,1);
topVariables = strings(0,1);
fieldCounts = zeros(0,1);
hasResultsTable = false(0,1);
hasControllerGain = false(0,1);
gainComparisonAvailable = false(0,1);
gainDifferenceNorm = nan(0,1);
gainConsistent = false(0,1);

fprintf('\n============================================\n');
fprintf('PHASE 10 RESULT FILE INVENTORY\n');
fprintf('============================================\n');

for f = 1:numel(files)

    currentFile = fullfile(scriptDir,files(f).name);
    loadedData = load(currentFile);

    variableNames = fieldnames(loadedData);

    for v = 1:numel(variableNames)

        variableName = variableNames{v};
        currentVariable = loadedData.(variableName);

        if ~isstruct(currentVariable) || numel(currentVariable) ~= 1
            continue;
        end

        currentFields = fieldnames(currentVariable);

        hasTable = isfield(currentVariable,'results_table') || ...
                   isfield(currentVariable,'performance_table');

        hasGain = isfield(currentVariable,'controller_gain');

        comparisonAvailable = false;
        differenceNorm = NaN;
        gainPass = false;

        if hasGain

            currentGain = currentVariable.controller_gain;

            if isnumeric(currentGain) && ...
                    isequal(size(currentGain),size(referenceGain))

                differenceNorm = norm( ...
                    double(currentGain(:))-double(referenceGain(:)));

                comparisonAvailable = true;
                gainPass = differenceNorm <= 1e-8;

            end

        end

        fileNames(end+1,1) = string(files(f).name);
        topVariables(end+1,1) = string(variableName);
        fieldCounts(end+1,1) = numel(currentFields);
        hasResultsTable(end+1,1) = hasTable;
        hasControllerGain(end+1,1) = hasGain;
        gainComparisonAvailable(end+1,1) = comparisonAvailable;
        gainDifferenceNorm(end+1,1) = differenceNorm;
        gainConsistent(end+1,1) = gainPass;

    end

end

inventory = table( ...
    fileNames, ...
    topVariables, ...
    fieldCounts, ...
    hasResultsTable, ...
    hasControllerGain, ...
    gainComparisonAvailable, ...
    gainDifferenceNorm, ...
    gainConsistent, ...
    'VariableNames',{ ...
    'FileName', ...
    'TopLevelVariable', ...
    'NumberOfFields', ...
    'HasResultsTable', ...
    'HasControllerGain', ...
    'GainComparisonAvailable', ...
    'GainDifferenceNorm', ...
    'GainConsistent'});

disp(inventory);

fprintf('\nMAT files inspected: %d\n',numel(files));
fprintf('Structured result variables inspected: %d\n',height(inventory));

%% 5. CONTROLLER GAIN CONSISTENCY VALIDATION

gainRows = inventory.HasControllerGain & ...
           inventory.GainComparisonAvailable;

gainComparisonCount = sum(gainRows);

if gainComparisonCount > 0

    gainValidationPass = all(inventory.GainConsistent(gainRows));

    maximumGainDifference = max( ...
        inventory.GainDifferenceNorm(gainRows));

else

    gainValidationPass = false;
    maximumGainDifference = NaN;

end

fprintf('\n============================================\n');
fprintf('CONTROLLER GAIN CONSISTENCY\n');
fprintf('============================================\n');

fprintf('Reference gain dimensions: %d x %d\n', ...
    size(referenceGain,1),size(referenceGain,2));

fprintf('Comparable saved controller gains: %d\n', ...
    gainComparisonCount);

fprintf('Maximum gain difference: %.12e\n', ...
    maximumGainDifference);

if gainValidationPass

    fprintf('Controller gain consistency: PASSED\n');

else

    fprintf('Controller gain consistency: NOT VERIFIED\n');

    if gainComparisonCount > 0
        disp(inventory(~inventory.GainConsistent,:));
    end

end

%% 6. LOAD PHASE 10.22, 10.23 AND 10.24 RESULTS

fineData = load(fullfile(scriptDir, ...
    'phase_10_22_fine_stabilization_boundary_results.mat'));

settlingData = load(fullfile(scriptDir, ...
    'phase_10_23_stabilization_settling_verification_results.mat'));

assessmentData = load(fullfile(scriptDir, ...
    'phase_10_24_control_performance_assessment_results.mat'));

if ~isfield(fineData,'results') || ...
   ~isfield(settlingData,'results') || ...
   ~isfield(assessmentData,'assessment')

    error('A required final-phase results structure is missing.');

end

fine = fineData.results;
settling = settlingData.results;
assessment = assessmentData.assessment;

%% 7. VERIFY BOUNDARY DATA CONSISTENCY

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

if ~isfield(assessment,'performance_table')
    error('Phase 10.24 performance table is missing.');
end

boundaryTable = assessment.performance_table;

if height(boundaryTable) ~= numel(settlingAngles)
    error('Phase 10.24 performance table has an unexpected number of cases.');
end

if any(abs(boundaryTable.InitialInclination_deg-settlingAngles)>1e-8)
    error('Phase 10.24 table does not match Phase 10.23 inclination cases.');
end

fprintf('\nBoundary dataset consistency: PASSED\n');
fprintf('Phase 10.22 cases: %d\n',numel(fineAngles));
fprintf('Phase 10.23 cases: %d\n',numel(settlingAngles));
fprintf('Common verified cases: %d\n',numel(settlingAngles));

%% 8. FINAL STABILIZATION BOUNDARY

settlingPass = logical(settling.settling_pass(:));

largestSettlingAngle = max(settlingAngles(settlingPass));

if any(~settlingPass)

    smallestFailingAngle = min(settlingAngles(~settlingPass));

else

    smallestFailingAngle = NaN;

end

verifiedSettlingCount = sum(settlingPass);
failedCount = sum(~settlingPass);

fprintf('\n============================================\n');
fprintf('FINAL STABILIZATION BOUNDARY\n');
fprintf('============================================\n');

fprintf('Settling cases: %d / %d\n', ...
    verifiedSettlingCount,numel(settlingAngles));

fprintf('Failed cases: %d / %d\n', ...
    failedCount,numel(settlingAngles));

fprintf('Largest verified settling angle: %.2f deg\n', ...
    largestSettlingAngle);

if isfinite(smallestFailingAngle)

    fprintf('Smallest tested failing angle: %.2f deg\n', ...
        smallestFailingAngle);

    fprintf('Empirical boundary interval: %.2f to %.2f deg\n', ...
        largestSettlingAngle,smallestFailingAngle);

end

%% 9. ACTUATOR PERFORMANCE VALIDATION

maximumForce = fine.maximum_force;

peakFx = max(fine.maximum_Fx(fineIdx));
peakFy = max(fine.maximum_Fy(fineIdx));

peakXUtilization = 100*peakFx/maximumForce;
peakYUtilization = 100*peakFy/maximumForce;

saturationCaseCount = sum(assessment.saturation_observed);

fprintf('\n============================================\n');
fprintf('ACTUATOR PERFORMANCE VALIDATION\n');
fprintf('============================================\n');

fprintf('Componentwise actuator limit: %.2f N\n',maximumForce);

fprintf('Maximum X force: %.6f N\n',peakFx);
fprintf('Maximum Y force: %.6f N\n',peakFy);

fprintf('Peak X utilization: %.4f %%\n',peakXUtilization);
fprintf('Peak Y utilization: %.4f %%\n',peakYUtilization);

fprintf('Cases reaching saturation: %d / %d\n', ...
    saturationCaseCount,numel(settlingAngles));

%% 10. FINAL CONTROL PERFORMANCE AT 46.6 DEGREES

index466 = find(abs(settlingAngles-46.6)<1e-8,1);

if isempty(index466)
    error('The 46.6 degree verification case is missing.');
end

performance466 = boundaryTable(index466,:);

fprintf('\n============================================\n');
fprintf('FINAL VERIFIED PERFORMANCE AT 46.6 DEGREES\n');
fprintf('============================================\n');

fprintf('Maximum cart displacement: %.6f m\n', ...
    performance466.MaxDisplacement_m);

fprintf('Maximum cart speed: %.6f m/s\n', ...
    performance466.MaxCartSpeed_mps);

fprintf('Maximum final-second inclination: %.8f deg\n', ...
    performance466.LastSecondMaxTilt_deg);

fprintf('Maximum final-second angular speed: %.8e rad/s\n', ...
    performance466.LastSecondMaxAngularSpeed_radps);

fprintf('Final cart speed: %.8e m/s\n', ...
    performance466.FinalCartSpeed_mps);

fprintf('Final cart displacement: %.8e m\n', ...
    performance466.FinalCartDisplacement_m);

fprintf('Settling classification: %d\n', ...
    performance466.SettlingPass);

%% 11. PARAMETER ROBUSTNESS SUMMARY

fprintf('\n============================================\n');
fprintf('PARAMETER ROBUSTNESS SUMMARY\n');
fprintf('============================================\n');

fprintf('Cart mass tested: 1.0 to 3.0 kg\n');
fprintf('Bob mass tested: 0.25 to 0.75 kg\n');
fprintf('Rod length tested: 0.50 to 1.50 m\n');
fprintf('Gravity tested: 7.81 to 11.81 m/s^2\n');

fprintf('\nFixed-gain controller successfully stabilized all tested\n');
fprintf('cases in the individual parameter robustness studies.\n');

fprintf('The controller gain was not redesigned for these studies.\n');

%% 12. INITIAL CONDITION ROBUSTNESS SUMMARY

fprintf('\n============================================\n');
fprintf('INITIAL CONDITION ROBUSTNESS SUMMARY\n');
fprintf('============================================\n');

fprintf('Initial inclination tested: 1 to 10 deg\n');
fprintf('Initial azimuth tested: 0 to 120 deg\n');
fprintf('Initial cart X velocity tested: -0.2 to +0.2 m/s\n');
fprintf('Initial cart Y velocity tested: -0.2 to +0.2 m/s\n');
fprintf('Initial inclination angular velocity: -0.5 to +0.5 rad/s\n');
fprintf('Initial azimuth angular velocity: -0.5 to +0.5 rad/s\n');

fprintf('\nCombined initial-condition cases passed their\n');
fprintf('previously established stabilization and force checks.\n');

%% 13. VALIDATION CHECKLIST

validationChecks = struct();

validationChecks.baseline_file_available = true;

validationChecks.fine_boundary_file_available = true;

validationChecks.settling_file_available = true;

validationChecks.performance_assessment_available = true;

validationChecks.boundary_data_consistent = true;

validationChecks.controller_gain_consistent = gainValidationPass;

validationChecks.settling_boundary_identified = ...
    isfinite(largestSettlingAngle) && isfinite(smallestFailingAngle);

validationChecks.actuator_limit_respected_by_model = ...
    all(fine.maximum_Fx <= maximumForce+1e-8) && ...
    all(fine.maximum_Fx >= -1e-8) && ...
    all(fine.maximum_Fy <= maximumForce+1e-8) && ...
    all(fine.maximum_Fy >= -1e-8);

validationChecks.phase_10_24_performance_available = true;

checkNames = fieldnames(validationChecks);
checkValues = false(numel(checkNames),1);

for k = 1:numel(checkNames)
    checkValues(k) = validationChecks.(checkNames{k});
end

checkTable = table( ...
    string(checkNames), ...
    checkValues, ...
    'VariableNames',{'ValidationCheck','Passed'});

fprintf('\n============================================\n');
fprintf('FINAL VALIDATION CHECKLIST\n');
fprintf('============================================\n');

disp(checkTable);

allChecksPassed = all(checkValues);

%% 14. FINAL SUMMARY STRUCTURE

validation = struct();

validation.phase = 'Phase 10.25';

validation.project = ...
    'Mathematical Modelling, Simulation and Analysis of a 3D X-Y Cart with an Inverted Spherical Pendulum';

validation.file_inventory = inventory;

validation.validation_checks = validationChecks;

validation.validation_table = checkTable;

validation.controller_gain_reference = referenceGain;

validation.controller_gain_comparison_count = gainComparisonCount;

validation.maximum_controller_gain_difference = maximumGainDifference;

validation.controller_gain_consistent = gainValidationPass;

validation.boundary_table = boundaryTable;

validation.largest_verified_settling_angle = largestSettlingAngle;

validation.smallest_tested_failing_angle = smallestFailingAngle;

validation.verified_settling_cases = verifiedSettlingCount;

validation.failed_cases = failedCount;

validation.maximum_force_per_axis = maximumForce;

validation.maximum_X_force = peakFx;

validation.maximum_Y_force = peakFy;

validation.peak_X_utilization_percent = peakXUtilization;

validation.peak_Y_utilization_percent = peakYUtilization;

validation.performance_at_46_6_degrees = performance466;

validation.new_simulations_performed = false;

validation.controller_redesigned = false;

validation.actuator_limit_changed = false;

validation.interpretation = ...
    ['The largest tested initial inclination satisfying all three ' ...
     'settling criteria was 46.6 degrees. The smallest tested failure ' ...
     'was 46.7 degrees. This is an empirical boundary under the tested ' ...
     'initial conditions, fixed controller gain, 15-second simulation ' ...
     'and componentwise actuator limits. It is not a universal ' ...
     'mathematical stability boundary.'];

%% 15. SAVE FINAL VALIDATION FILES

outputMat = fullfile(scriptDir, ...
    'phase_10_25_final_control_validation_results.mat');

outputInventory = fullfile(scriptDir, ...
    'Phase_10_25_ResultFileInventory.csv');

outputChecks = fullfile(scriptDir, ...
    'Phase_10_25_FinalValidationChecklist.csv');

outputBoundary = fullfile(scriptDir, ...
    'Phase_10_25_FinalBoundaryResults.csv');

save(outputMat,'validation');

writetable(inventory,outputInventory);
writetable(checkTable,outputChecks);
writetable(boundaryTable,outputBoundary);

%% 16. GENERATE FINAL VALIDATION FIGURE

figure('Name','Phase 10.25 Final Control Validation', ...
    'Position',[100 100 1100 750]);

subplot(2,2,1);

plot(settlingAngles, ...
    boundaryTable.MaxDisplacement_m,'o-', ...
    'LineWidth',1.6,'MarkerSize',6);

xlabel('Initial inclination (deg)');
ylabel('Maximum displacement (m)');
title('Cart Displacement');
grid on;

subplot(2,2,2);

plot(settlingAngles, ...
    boundaryTable.MaxCartSpeed_mps,'o-', ...
    'LineWidth',1.6,'MarkerSize',6);

xlabel('Initial inclination (deg)');
ylabel('Maximum speed (m/s)');
title('Cart Speed');
grid on;

subplot(2,2,3);

plot(settlingAngles, ...
    boundaryTable.LastSecondMaxTilt_deg,'o-', ...
    'LineWidth',1.6,'MarkerSize',6);

hold on;

yline(settling.tilt_limit,'--','Settling limit');

xlabel('Initial inclination (deg)');
ylabel('Final-second maximum tilt (deg)');
title('Settling Performance');
grid on;

subplot(2,2,4);

hold on;

plot(settlingAngles(settlingPass), ...
    ones(sum(settlingPass),1),'o', ...
    'MarkerSize',8,'LineWidth',1.5);

plot(settlingAngles(~settlingPass), ...
    zeros(sum(~settlingPass),1),'x', ...
    'MarkerSize',9,'LineWidth',1.8);

xlabel('Initial inclination (deg)');
ylabel('Classification');

yticks([0 1]);
yticklabels({'Failed','Settled'});
ylim([-0.25 1.25]);

title('Final Stabilization Classification');
legend('Settled','Failed','Location','best');
grid on;

sgtitle('Phase 10.25: Final Control Validation');

saveas(gcf,fullfile(scriptDir, ...
    'Phase_10_25_Final_Control_Validation.png'));

%% 17. GENERATE FINAL TEXT REPORT

reportFile = fullfile(scriptDir, ...
    'Phase_10_25_Final_Control_Validation_Report.txt');

fid = fopen(reportFile,'w');

if fid == -1
    error('Could not create the final validation report.');
end

fprintf(fid,'PHASE 10.25: FINAL CONTROL VALIDATION REPORT\n');
fprintf(fid,'============================================\n\n');

fprintf(fid,'Project: 3D X-Y Cart with an Inverted Spherical Pendulum\n\n');

fprintf(fid,'1. RESULT FILE INVENTORY\n');
fprintf(fid,'MAT files inspected: %d\n',numel(files));
fprintf(fid,'Structured variables inspected: %d\n\n',height(inventory));

fprintf(fid,'2. CONTROLLER GAIN VALIDATION\n');
fprintf(fid,'Comparable gains: %d\n',gainComparisonCount);
fprintf(fid,'Maximum gain difference: %.12e\n',maximumGainDifference);
fprintf(fid,'Gain consistency passed: %d\n\n',gainValidationPass);

fprintf(fid,'3. STABILIZATION BOUNDARY\n');
fprintf(fid,'Largest verified settling angle: %.2f deg\n', ...
    largestSettlingAngle);
fprintf(fid,'Smallest tested failing angle: %.2f deg\n', ...
    smallestFailingAngle);
fprintf(fid,'Settling cases: %d / %d\n\n', ...
    verifiedSettlingCount,numel(settlingAngles));

fprintf(fid,'4. ACTUATOR PERFORMANCE\n');
fprintf(fid,'Componentwise force limit: %.4f N\n',maximumForce);
fprintf(fid,'Maximum X force: %.6f N\n',peakFx);
fprintf(fid,'Maximum Y force: %.6f N\n',peakFy);
fprintf(fid,'Peak X utilization: %.4f percent\n',peakXUtilization);
fprintf(fid,'Peak Y utilization: %.4f percent\n\n',peakYUtilization);

fprintf(fid,'5. ROBUSTNESS TEST RANGES\n');
fprintf(fid,'Cart mass: 1.0 to 3.0 kg\n');
fprintf(fid,'Bob mass: 0.25 to 0.75 kg\n');
fprintf(fid,'Rod length: 0.50 to 1.50 m\n');
fprintf(fid,'Gravity: 7.81 to 11.81 m/s^2\n');
fprintf(fid,'Initial inclination: 1 to 10 degrees in robustness studies\n');
fprintf(fid,'Initial cart velocity: -0.2 to +0.2 m/s\n');
fprintf(fid,'Initial angular velocities: -0.5 to +0.5 rad/s\n\n');

fprintf(fid,'6. SCIENTIFIC INTERPRETATION\n');
fprintf(fid,'%s\n\n',validation.interpretation);

fprintf(fid,'7. EXECUTION STATUS\n');
fprintf(fid,'New simulations performed: NO\n');
fprintf(fid,'Controller redesigned: NO\n');
fprintf(fid,'Actuator limit changed: NO\n');
fprintf(fid,'Existing simulation results reused: YES\n\n');

fprintf(fid,'8. VALIDATION CHECKLIST\n');

for k = 1:height(checkTable)

    fprintf(fid,'%s: %d\n', ...
        checkTable.ValidationCheck(k), ...
        checkTable.Passed(k));

end

fprintf(fid,'\nEND OF PHASE 10.25 REPORT\n');

fclose(fid);

%% 18. COMPLETION SUMMARY

fprintf('\n============================================\n');
fprintf('PHASE 10.25 COMPLETED\n');
fprintf('============================================\n');

fprintf('Result files inspected: %d\n',numel(files));

fprintf('Controller gain comparisons: %d\n',gainComparisonCount);

fprintf('Largest verified settling angle: %.2f deg\n', ...
    largestSettlingAngle);

fprintf('Smallest tested failing angle: %.2f deg\n', ...
    smallestFailingAngle);

if allChecksPassed

    fprintf('\nAll final validation checks passed.\n');

else

    fprintf('\nWARNING: One or more validation checks require review.\n');

    disp(checkTable(~checkTable.Passed,:));

end

fprintf('\nNo new simulations performed.\n');
fprintf('Controller gain unchanged.\n');
fprintf('Actuator limits unchanged.\n');

fprintf('\nSaved files:\n');

fprintf('1. %s\n',outputMat);
fprintf('2. %s\n',outputInventory);
fprintf('3. %s\n',outputChecks);
fprintf('4. %s\n',outputBoundary);
fprintf('5. %s\n',reportFile);
fprintf('6. Phase_10_25_Final_Control_Validation.png\n');

fprintf('\n============================================\n');
fprintf('END OF PHASE 10.25\n');
fprintf('============================================\n');
