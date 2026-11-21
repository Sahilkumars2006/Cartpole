
%% PHASE 9.20: FINAL DYNAMIC ANALYSIS CONSOLIDATION
% Project:
% Mathematical Modelling, Simulation and Analysis of a
% 3D X-Y Cart with an Inverted Spherical Pendulum
%
% Consolidates Phase 9.5 to Phase 9.19.
% No new physical simulations are performed.
% Existing simulation and model files remain unchanged.

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 9.20: FINAL DYNAMIC ANALYSIS CONSOLIDATION\n');
fprintf('============================================\n');

%% 1. PROJECT PATHS

analysisFolder = fileparts(mfilename('fullpath'));
projectRoot = fileparts(analysisFolder);

resultsFolder = fullfile(projectRoot,'Results','Phase_9');

if ~exist(resultsFolder,'dir')
    mkdir(resultsFolder);
end

fprintf('\nProject root:\n%s\n',projectRoot);
fprintf('\nResults folder:\n%s\n',resultsFolder);

%% 2. DEFINE PHASE 9 RESULT FILE INVENTORY

phaseNumbers = [
                "9.5"
                "9.6"
                "9.7"
                "9.8"
                "9.9"
                "9.10"
                "9.11"
                "9.12"
                "9.13"
                "9.14"
                "9.15"
                "9.16"
                "9.17"
                "9.18"
                "9.19"
                ];

phaseDescriptions = {
    'Parameter sensitivity analysis'
    'X-Y coupling and rotational symmetry'
    'Initial condition sensitivity'
    'Mechanical energy transfer analysis'
    'Uncontrolled stability analysis'
    'External force response analysis'
    'Mass sensitivity under external force'
    'Rod length sensitivity under external force'
    'Gravity sensitivity under external force'
    'External force magnitude and direction'
    'Initial azimuth and force interaction'
    'Combined parameter interaction'
    'Global statistical sensitivity'
    'Multivariable regression and interactions'
    'Regression validation and residual diagnostics'
};

resultFiles = {
    'phase_9_parameter_sensitivity.mat'
    'phase_9_6_xy_coupling_results.mat'
    'phase_9_7_initial_condition_results.mat'
    'phase_9_8_energy_transfer_results.mat'
    'phase_9_9_stability_results.mat'
    'phase_9_10_external_force_results.mat'
    'phase_9_11_mass_force_sensitivity_results.mat'
    'phase_9_12_rod_length_force_results.mat'
    'phase_9_13_gravity_force_results.mat'
    'phase_9_14_force_direction_results.mat'
    'phase_9_15_azimuth_force_results.mat'
    'phase_9_16_combined_parameter_results.mat'
    'phase_9_17_global_sensitivity_results.mat'
    'phase_9_18_regression_interaction_results.mat'
    'phase_9_19_regression_validation_results.mat'
};

nPhases = numel(resultFiles);

fileExists = false(nPhases,1);
fileSizeKB = zeros(nPhases,1);

fprintf('\n============================================\n');
fprintf('PHASE 9 RESULT FILE INVENTORY\n');
fprintf('============================================\n');

for i = 1:nPhases

    filePath = fullfile(resultsFolder,resultFiles{i});

    if isfile(filePath)

        fileInfo = dir(filePath);

        fileExists(i) = true;

        fileSizeKB(i) = fileInfo.bytes/1024;

        fprintf('Phase %s: FOUND\n', ...
            string(phaseNumbers(i)));

        fprintf('  %s\n',resultFiles{i});

    else

        fprintf('Phase %s: NOT FOUND\n', ...
            string(phaseNumbers(i)));

        fprintf('  Expected: %s\n',resultFiles{i});

    end
end

inventoryTable = table( ...
    phaseNumbers, ...
    string(phaseDescriptions), ...
    string(resultFiles), ...
    fileExists, ...
    fileSizeKB, ...
    'VariableNames',{ ...
    'Phase', ...
    'Analysis', ...
    'Result_File', ...
    'File_Found', ...
    'File_Size_KB'});

fprintf('\n--- INVENTORY SUMMARY ---\n');
disp(inventoryTable);

%% 3. LOAD PHASE 9.17 STATISTICAL RESULTS

fprintf('\n============================================\n');
fprintf('LOADING PHASE 9.17 RESULTS\n');
fprintf('============================================\n');

file917 = fullfile(resultsFolder, ...
    'phase_9_17_global_sensitivity_results.mat');

if ~isfile(file917)
    error('Phase 9.17 results are missing.');
end

D917 = load(file917);

required917 = {'etaSquared','correlationMatrix','rankingTable'};

for i = 1:numel(required917)

    if ~isfield(D917,required917{i})
        error('Missing Phase 9.17 variable: %s',required917{i});
    end

end

etaSquared = D917.etaSquared;
correlationMatrix = D917.correlationMatrix;
rankingTable = D917.rankingTable;

fprintf('Phase 9.17 results loaded successfully.\n');

%% 4. LOAD PHASE 9.18 REGRESSION RESULTS

fprintf('\n============================================\n');
fprintf('LOADING PHASE 9.18 RESULTS\n');
fprintf('============================================\n');

file918 = fullfile(resultsFolder, ...
    'phase_9_18_regression_interaction_results.mat');

if ~isfile(file918)
    error('Phase 9.18 results are missing.');
end

D918 = load(file918);

required918 = { ...
    'regressionSummary', ...
    'interactionRankingTable', ...
    'parameterContributionTable', ...
    'modelComparisonTable'};

for i = 1:numel(required918)

    if ~isfield(D918,required918{i})
        error('Missing Phase 9.18 variable: %s',required918{i});
    end

end

regressionSummary = D918.regressionSummary;
interactionRankingTable = D918.interactionRankingTable;
parameterContributionTable = D918.parameterContributionTable;
modelComparisonTable = D918.modelComparisonTable;

fprintf('Phase 9.18 results loaded successfully.\n');

%% 5. LOAD PHASE 9.19 VALIDATION RESULTS

fprintf('\n============================================\n');
fprintf('LOADING PHASE 9.19 RESULTS\n');
fprintf('============================================\n');

file919 = fullfile(resultsFolder, ...
    'phase_9_19_regression_validation_results.mat');

if ~isfile(file919)
    error('Phase 9.19 results are missing.');
end

D919 = load(file919);

if ~isfield(D919,'validationTable')
    error('Phase 9.19 validationTable is missing.');
end

validationTable = D919.validationTable;

fprintf('Phase 9.19 results loaded successfully.\n');

%% 6. CREATE CONSOLIDATED FINDINGS TABLE

fprintf('\n============================================\n');
fprintf('CONSOLIDATED DYNAMIC ANALYSIS FINDINGS\n');
fprintf('============================================\n');

findingPhase = [
    "9.5"
    "9.6"
    "9.7"
    "9.8"
    "9.9"
    "9.10"
    "9.11"
    "9.12"
    "9.13"
    "9.14"
    "9.15"
    "9.16"
    "9.17"
    "9.18"
    "9.19"
];

finding = [
    "Mass, bob mass, rod length and gravity sensitivity evaluated"
    "Horizontal rotational symmetry numerically verified"
    "Initial position and velocity sensitivity evaluated"
    "Mechanical energy conservation verified"
    "Upright uncontrolled instability demonstrated"
    "External force response and work-energy balance verified"
    "Cart and bob mass effects under applied force evaluated"
    "Rod length effects under applied force evaluated"
    "Gravity effects under applied force evaluated"
    "Force magnitude and direction effects evaluated"
    "Relative azimuth and force symmetry verified"
    "81-case combined parameter study completed"
    "Global parameter sensitivity rankings obtained"
    "Second-order regression and parameter interactions evaluated"
    "Leave-one-out regression validation completed"
];

findingStatus = repmat("COMPLETED",15,1);

consolidatedFindings = table( ...
    findingPhase, ...
    finding, ...
    findingStatus, ...
    'VariableNames',{ ...
    'Phase', ...
    'Main_Finding', ...
    'Status'});

disp(consolidatedFindings);

%% 7. DISPLAY GLOBAL SENSITIVITY RANKINGS

fprintf('\n============================================\n');
fprintf('GLOBAL SENSITIVITY RANKINGS\n');
fprintf('============================================\n');

disp(rankingTable);

fprintf('\nEta-squared sensitivity values:\n');

disp(array2table(etaSquared, ...
    'VariableNames',{ ...
    'Cart_Displacement', ...
    'Cart_Speed', ...
    'Pendulum_Inclination'}, ...
    'RowNames',{'Cart_Mass','Bob_Mass','Rod_Length','External_Force'}));

%% 8. DISPLAY REGRESSION PERFORMANCE

fprintf('\n============================================\n');
fprintf('REGRESSION MODEL PERFORMANCE\n');
fprintf('============================================\n');

disp(regressionSummary);

fprintf('\nLinear versus second-order model:\n');

disp(modelComparisonTable);

%% 9. DISPLAY INTERACTION RANKINGS

fprintf('\n============================================\n');
fprintf('PARAMETER INTERACTION RESULTS\n');
fprintf('============================================\n');

disp(interactionRankingTable);

fprintf('\nParameter group contributions:\n');

disp(parameterContributionTable);

%% 10. DISPLAY VALIDATION RESULTS

fprintf('\n============================================\n');
fprintf('REGRESSION VALIDATION RESULTS\n');
fprintf('============================================\n');

disp(validationTable);

%% 11. GENERATE GLOBAL SENSITIVITY FIGURE

figure('Name','Phase 9 Final Global Sensitivity');

bar(etaSquared);

set(gca, ...
    'XTick',1:4, ...
    'XTickLabel',{'Cart Mass','Bob Mass','Rod Length','External Force'});

ylabel('Eta-squared');

legend( ...
    {'Cart Displacement','Cart Speed','Pendulum Inclination'}, ...
    'Location','best');

title('Global Sensitivity of the 3D X-Y Pendulum System');

grid on;

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_20_Global_Sensitivity_Overview.png'));

%% 12. GENERATE LOOCV PERFORMANCE FIGURE

figure('Name','Phase 9 Final LOOCV Performance');

bar(validationTable.LOOCV_R2);

set(gca, ...
    'XTick',1:3, ...
    'XTickLabel',{ ...
    'Cart Displacement', ...
    'Cart Speed', ...
    'Pendulum Inclination'});

ylabel('LOOCV R-squared');

ylim([0 1]);

title('Regression Predictive Validation');

grid on;

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_20_LOOCV_Performance.png'));

%% 13. GENERATE REGRESSION ERROR FIGURE

figure('Name','Phase 9 Final Regression Errors');

bar([ ...
    validationTable.Training_RMSE, ...
    validationTable.LOOCV_RMSE]);

set(gca, ...
    'XTick',1:3, ...
    'XTickLabel',{ ...
    'Cart Displacement', ...
    'Cart Speed', ...
    'Pendulum Inclination'});

ylabel('RMSE');

legend({'Training RMSE','LOOCV RMSE'}, ...
    'Location','best');

title('Regression Training and Validation Errors');

grid on;

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_20_Regression_Error_Comparison.png'));

%% 14. GENERATE ENERGY AND CONSTRAINT SUMMARY

% These are the previously verified baseline validation results.
% They are reported here as historical findings, not recalculated.

energyDrift = 3.131742e-8;
rodConstraintError = 8.59e-11;
symmetryError = 1.488777e-7;
forceWorkEnergyResidual = 1.940696e-10;

validationNames = {
    'Baseline energy drift (J)'
    'Baseline rod constraint error (m)'
    'Rotational symmetry error'
    'External-force work-energy residual (J)'
};

validationValues = [
    energyDrift
    rodConstraintError
    symmetryError
    forceWorkEnergyResidual
];

physicalValidationTable = table( ...
    string(validationNames), ...
    validationValues, ...
    'VariableNames',{'Validation_Metric','Verified_Value'});

fprintf('\n============================================\n');
fprintf('PHYSICAL MODEL VALIDATION SUMMARY\n');
fprintf('============================================\n');

disp(physicalValidationTable);

%% 15. CHECK INVENTORY COMPLETENESS

numberFound = sum(fileExists);
numberExpected = numel(fileExists);

fprintf('\n============================================\n');
fprintf('RESULT FILE COMPLETENESS\n');
fprintf('============================================\n');

fprintf('Expected Phase 9 result files: %d\n',numberExpected);
fprintf('Files found: %d\n',numberFound);
fprintf('Files missing: %d\n',numberExpected-numberFound);

if numberFound == numberExpected
    fprintf('File inventory: COMPLETE\n');
else
    fprintf('File inventory: INCOMPLETE\n');
    fprintf('Review missing files before final report submission.\n');
end

%% 16. SAVE CONSOLIDATED MATLAB RESULTS

consolidatedMatFile = fullfile(resultsFolder, ...
    'phase_9_final_consolidated_results.mat');

save(consolidatedMatFile, ...
    'inventoryTable', ...
    'consolidatedFindings', ...
    'etaSquared', ...
    'correlationMatrix', ...
    'rankingTable', ...
    'regressionSummary', ...
    'interactionRankingTable', ...
    'parameterContributionTable', ...
    'modelComparisonTable', ...
    'validationTable', ...
    'physicalValidationTable');

%% 17. SAVE CONSOLIDATED CSV FILES

writetable(inventoryTable,fullfile(resultsFolder, ...
    'Phase_9_20_Result_File_Inventory.csv'));

writetable(consolidatedFindings,fullfile(resultsFolder, ...
    'Phase_9_20_Analysis_Findings.csv'));

writetable(physicalValidationTable,fullfile(resultsFolder, ...
    'Phase_9_20_Physical_Validation.csv'));

writetable(validationTable,fullfile(resultsFolder, ...
    'Phase_9_20_Regression_Validation.csv'));

%% 18. GENERATE TECHNICAL SUMMARY TEXT REPORT

reportFile = fullfile(resultsFolder, ...
    'Phase_9_Final_Dynamic_Analysis_Report.txt');

fid = fopen(reportFile,'w');

if fid == -1
    error('Could not create the final Phase 9 report.');
end

fprintf(fid,'====================================================\n');
fprintf(fid,'PHASE 9: FINAL DYNAMIC ANALYSIS REPORT\n');
fprintf(fid,'====================================================\n\n');

fprintf(fid,'PROJECT:\n');
fprintf(fid,['Mathematical Modelling, Simulation and Analysis of a ', ...
    '3D X-Y Cart with an Inverted Spherical Pendulum\n\n']);

fprintf(fid,'SYSTEM DESCRIPTION:\n');
fprintf(fid,['The system consists of a cart moving in horizontal ', ...
    'X and Y directions and an inverted spherical pendulum ', ...
    'whose bob moves in three-dimensional space.\n\n']);

fprintf(fid,'ANALYSIS COVERAGE:\n');

for i = 1:height(consolidatedFindings)

    fprintf(fid,'Phase %s: %s\n', ...
        consolidatedFindings.Phase(i), ...
        consolidatedFindings.Main_Finding(i));

end

fprintf(fid,'\nGLOBAL SENSITIVITY RESULTS:\n');

fprintf(fid,'Cart displacement eta-squared:\n');
fprintf(fid,'Cart mass: %.6f\n',etaSquared(1,1));
fprintf(fid,'Bob mass: %.6f\n',etaSquared(2,1));
fprintf(fid,'Rod length: %.6f\n',etaSquared(3,1));
fprintf(fid,'External force: %.6f\n',etaSquared(4,1));

fprintf(fid,'\nCart speed eta-squared:\n');
fprintf(fid,'Cart mass: %.6f\n',etaSquared(1,2));
fprintf(fid,'Bob mass: %.6f\n',etaSquared(2,2));
fprintf(fid,'Rod length: %.6f\n',etaSquared(3,2));
fprintf(fid,'External force: %.6f\n',etaSquared(4,2));

fprintf(fid,'\nPendulum inclination eta-squared:\n');
fprintf(fid,'Cart mass: %.6f\n',etaSquared(1,3));
fprintf(fid,'Bob mass: %.6f\n',etaSquared(2,3));
fprintf(fid,'Rod length: %.6f\n',etaSquared(3,3));
fprintf(fid,'External force: %.6f\n',etaSquared(4,3));

fprintf(fid,'\nREGRESSION MODEL PERFORMANCE:\n');

for i = 1:height(regressionSummary)

    fprintf(fid,'\n%s\n',regressionSummary.Response(i));

    fprintf(fid,'R-squared: %.8f\n', ...
        regressionSummary.R_Squared(i));

    fprintf(fid,'Adjusted R-squared: %.8f\n', ...
        regressionSummary.Adjusted_R_Squared(i));

    fprintf(fid,'RMSE: %.8e\n', ...
        regressionSummary.RMSE(i));

end

fprintf(fid,'\nLEAVE-ONE-OUT VALIDATION:\n');

for i = 1:height(validationTable)

    fprintf(fid,'\n%s\n',validationTable.Response(i));

    fprintf(fid,'LOOCV R-squared: %.8f\n', ...
        validationTable.LOOCV_R2(i));

    fprintf(fid,'LOOCV RMSE: %.8e\n', ...
        validationTable.LOOCV_RMSE(i));

    fprintf(fid,'LOOCV MAE: %.8e\n', ...
        validationTable.LOOCV_MAE(i));

end

fprintf(fid,'\nPHYSICAL VALIDATION:\n');

for i = 1:height(physicalValidationTable)

    fprintf(fid,'%s: %.10e\n', ...
        physicalValidationTable.Validation_Metric(i), ...
        physicalValidationTable.Verified_Value(i));

end

fprintf(fid,'\nRESULT FILE INVENTORY:\n');
fprintf(fid,'Expected files: %d\n',numberExpected);
fprintf(fid,'Files found: %d\n',numberFound);
fprintf(fid,'Files missing: %d\n',numberExpected-numberFound);

fprintf(fid,'\nIMPORTANT INTERPRETATION:\n');

fprintf(fid,['Regression validation is based on leave-one-out ', ...
    'cross-validation of the existing 81-case parameter grid. ', ...
    'It does not establish extrapolation accuracy outside ', ...
    'the tested parameter ranges.\n']);

fprintf(fid,['The inclination response has lower cross-validated ', ...
    'predictive performance than cart displacement and cart speed.\n']);

fprintf(fid,['The physical validation values in this report are ', ...
    'previously verified baseline results and are not newly ', ...
    'recalculated in Phase 9.20.\n']);

fprintf(fid,'\n====================================================\n');
fprintf(fid,'END OF PHASE 9 REPORT\n');
fprintf(fid,'====================================================\n');

fclose(fid);

%% 19. FINAL CONSOLIDATION SUMMARY

fprintf('\n============================================\n');
fprintf('PHASE 9.20 FINAL SUMMARY\n');
fprintf('============================================\n');

fprintf('Analysis stages consolidated: %d\n',nPhases);

fprintf('Existing result files found: %d/%d\n', ...
    numberFound,numberExpected);

fprintf('New ODE simulations performed: 0\n');

fprintf('\nConsolidated MATLAB file:\n%s\n',consolidatedMatFile);

fprintf('\nTechnical report:\n%s\n',reportFile);

fprintf('\nGenerated overview figures:\n');
fprintf('Phase_9_20_Global_Sensitivity_Overview.png\n');
fprintf('Phase_9_20_LOOCV_Performance.png\n');
fprintf('Phase_9_20_Regression_Error_Comparison.png\n');

fprintf('\n============================================\n');

if numberFound == numberExpected

    fprintf('PHASE 9.20 CONSOLIDATION COMPLETED\n');

else

    fprintf('PHASE 9.20 CONSOLIDATION COMPLETED WITH MISSING FILES\n');

end

fprintf('============================================\n');
