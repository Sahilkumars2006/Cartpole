
%% PHASE 9.19: REGRESSION VALIDATION AND RESIDUAL DIAGNOSTICS
% 3D X-Y Cart with Inverted Spherical Pendulum
%
% Uses 81 existing Phase 9.16 simulation cases.
% Validates the second-order regression model from Phase 9.18.
% No new ODE simulations are performed.

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 9.19: REGRESSION VALIDATION\n');
fprintf('============================================\n');

%% 1. PATHS

analysisFolder = fileparts(mfilename('fullpath'));
projectRoot = fileparts(analysisFolder);

resultsFolder = fullfile(projectRoot,'Results','Phase_9');

inputFile = fullfile(resultsFolder, ...
    'phase_9_18_regression_interaction_results.mat');

if ~isfile(inputFile)
    error('Phase 9.18 results file not found:\n%s',inputFile);
end

%% 2. LOAD DATA

fprintf('\nLoading Phase 9.18 regression results...\n');

D = load(inputFile);

requiredFields = { ...
    'Xcoded', ...
    'Y', ...
    'Xdesign', ...
    'termNames', ...
    'responseNames', ...
    'responseUnits'};

% Some Phase 9.18 files do not store responseNames or responseUnits.
% Define them here if necessary.

if ~isfield(D,'Xcoded') || ...
        ~isfield(D,'Y') || ...
        ~isfield(D,'Xdesign')

    error('Required regression data is missing.');
end

Xcoded = D.Xcoded;
Y = D.Y;
Xdesign = D.Xdesign;

if isfield(D,'termNames')
    termNames = D.termNames;
else
    error('Regression term names are missing.');
end

responseNames = { ...
    'Maximum Cart Displacement', ...
    'Maximum Cart Speed', ...
    'Maximum Pendulum Inclination'};

responseUnits = {'m','m/s','deg'};

n = size(Y,1);
nResponses = size(Y,2);
nTerms = size(Xdesign,2);

if n ~= 81 || nResponses ~= 3
    error('Unexpected dimensions in regression data.');
end

fprintf('Cases loaded: %d\n',n);
fprintf('Regression predictors: %d\n',nTerms);

%% 3. RECONSTRUCT THE REGRESSION DESIGN MATRIX

% The Phase 9.18 model contains an intercept,
% linear terms, quadratic terms and pairwise interactions.

fullDesign = [ones(n,1),Xdesign];

numberOfCoefficients = size(fullDesign,2);

if rank(fullDesign) ~= numberOfCoefficients
    error('Regression design matrix is rank deficient.');
end

fprintf('Regression design matrix rank: %d/%d\n', ...
    rank(fullDesign),numberOfCoefficients);

%% 4. TRAINING PREDICTIONS AND RESIDUALS

trainingPredictions = zeros(n,nResponses);
trainingResiduals = zeros(n,nResponses);

trainingR2 = zeros(nResponses,1);
trainingRMSE = zeros(nResponses,1);
trainingMAE = zeros(nResponses,1);

for j = 1:nResponses

    y = Y(:,j);

    beta = fullDesign \ y;

    yPredicted = fullDesign*beta;

    residual = y-yPredicted;

    trainingPredictions(:,j) = yPredicted;
    trainingResiduals(:,j) = residual;

    SSE = sum(residual.^2);
    SST = sum((y-mean(y)).^2);

    trainingR2(j) = 1-SSE/SST;

    trainingRMSE(j) = sqrt(mean(residual.^2));

    trainingMAE(j) = mean(abs(residual));

end

%% 5. LEAVE-ONE-OUT CROSS-VALIDATION

fprintf('\n============================================\n');
fprintf('LEAVE-ONE-OUT CROSS-VALIDATION\n');
fprintf('============================================\n');

% For each case:
% 1. Remove that case.
% 2. Fit the regression model to the remaining 80 cases.
% 3. Predict the omitted case.
%
% This avoids using the omitted response value in its own model fit.

loocvPredictions = zeros(n,nResponses);

for i = 1:n

    trainingMask = true(n,1);

    trainingMask(i) = false;

    Xtrain = fullDesign(trainingMask,:);
    Xtest = fullDesign(i,:);

    for j = 1:nResponses

        ytrain = Y(trainingMask,j);

        betaLOO = Xtrain \ ytrain;

        loocvPredictions(i,j) = Xtest*betaLOO;

    end

end

loocvResiduals = Y-loocvPredictions;

loocvR2 = zeros(nResponses,1);
loocvRMSE = zeros(nResponses,1);
loocvMAE = zeros(nResponses,1);
loocvMaxError = zeros(nResponses,1);

for j = 1:nResponses

    y = Y(:,j);

    errorVector = loocvResiduals(:,j);

    PRESS = sum(errorVector.^2);

    SST = sum((y-mean(y)).^2);

    loocvR2(j) = 1-PRESS/SST;

    loocvRMSE(j) = sqrt(mean(errorVector.^2));

    loocvMAE(j) = mean(abs(errorVector));

    loocvMaxError(j) = max(abs(errorVector));

end

fprintf('\nCross-validation results:\n');

for j = 1:nResponses

    fprintf('\n%s:\n',responseNames{j});

    fprintf('Training R-squared: %.8f\n',trainingR2(j));

    fprintf('LOOCV R-squared:    %.8f\n',loocvR2(j));

    fprintf('Training RMSE:      %.8e %s\n', ...
        trainingRMSE(j),responseUnits{j});

    fprintf('LOOCV RMSE:         %.8e %s\n', ...
        loocvRMSE(j),responseUnits{j});

    fprintf('LOOCV MAE:          %.8e %s\n', ...
        loocvMAE(j),responseUnits{j});

    fprintf('Maximum LOOCV error: %.8e %s\n', ...
        loocvMaxError(j),responseUnits{j});

end

%% 6. TRAINING VS CROSS-VALIDATION SUMMARY

validationTable = table( ...
    string(responseNames(:)), ...
    trainingR2, ...
    loocvR2, ...
    trainingRMSE, ...
    loocvRMSE, ...
    trainingMAE, ...
    loocvMAE, ...
    loocvMaxError, ...
    'VariableNames',{ ...
    'Response', ...
    'Training_R2', ...
    'LOOCV_R2', ...
    'Training_RMSE', ...
    'LOOCV_RMSE', ...
    'Training_MAE', ...
    'LOOCV_MAE', ...
    'Maximum_LOOCV_Error'});

fprintf('\n--- VALIDATION SUMMARY TABLE ---\n');

disp(validationTable);

%% 7. IDENTIFY WORST-PREDICTED CASES

fprintf('\n============================================\n');
fprintf('WORST-PREDICTED CASES\n');
fprintf('============================================\n');

worstCaseTables = cell(nResponses,1);

for j = 1:nResponses

    [sortedErrors,order] = ...
        sort(abs(loocvResiduals(:,j)),'descend');

    numberToShow = min(5,n);

    caseNumber = order(1:numberToShow);

    actualValues = Y(caseNumber,j);

    predictedValues = loocvPredictions(caseNumber,j);

    signedErrors = loocvResiduals(caseNumber,j);

    worstCaseTables{j} = table( ...
        caseNumber, ...
        actualValues, ...
        predictedValues, ...
        signedErrors, ...
        sortedErrors(1:numberToShow), ...
        'VariableNames',{ ...
        'Case', ...
        'Actual', ...
        'LOOCV_Predicted', ...
        'Signed_Error', ...
        'Absolute_Error'});

    fprintf('\nResponse: %s\n',responseNames{j});

    disp(worstCaseTables{j});

end

%% 8. RESIDUAL DISTRIBUTION STATISTICS

fprintf('\n============================================\n');
fprintf('RESIDUAL DISTRIBUTION ANALYSIS\n');
fprintf('============================================\n');

residualMean = mean(trainingResiduals,1)';
residualStd = std(trainingResiduals,0,1)';
residualMin = min(trainingResiduals,[],1)';
residualMax = max(trainingResiduals,[],1)';

residualStatisticsTable = table( ...
    string(responseNames(:)), ...
    residualMean, ...
    residualStd, ...
    residualMin, ...
    residualMax, ...
    'VariableNames',{ ...
    'Response', ...
    'Residual_Mean', ...
    'Residual_Standard_Deviation', ...
    'Minimum_Residual', ...
    'Maximum_Residual'});

disp(residualStatisticsTable);

%% 9. RESIDUAL VS PREDICTED PLOTS

figure('Name','Phase 9.19 Residual vs Predicted');

for j = 1:nResponses

    subplot(1,3,j);

    scatter(trainingPredictions(:,j), ...
        trainingResiduals(:,j),35,'filled');

    hold on;

    yline(0,'--');

    xlabel(['Predicted ',responseUnits{j}]);
    ylabel(['Residual ',responseUnits{j}]);

    title(responseNames{j});

    grid on;

end

sgtitle('Training Residual Diagnostics');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_19_Residual_vs_Predicted.png'));

%% 10. LOOCV PREDICTED VS ACTUAL PLOTS

figure('Name','Phase 9.19 LOOCV Predicted vs Actual');

for j = 1:nResponses

    subplot(1,3,j);

    scatter(Y(:,j),loocvPredictions(:,j),35,'filled');

    hold on;

    minimumValue = min(Y(:,j));
    maximumValue = max(Y(:,j));

    plot([minimumValue maximumValue], ...
        [minimumValue maximumValue],'--');

    xlabel(['Actual ',responseUnits{j}]);
    ylabel(['LOOCV predicted ',responseUnits{j}]);

    title(responseNames{j});

    grid on;

end

sgtitle('Leave-One-Out Cross-Validation Predictions');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_19_LOOCV_Predicted_vs_Actual.png'));

%% 11. LOOCV RESIDUAL PLOTS

figure('Name','Phase 9.19 LOOCV Residuals');

for j = 1:nResponses

    subplot(1,3,j);

    scatter(loocvPredictions(:,j), ...
        loocvResiduals(:,j),35,'filled');

    hold on;

    yline(0,'--');

    xlabel(['LOOCV predicted ',responseUnits{j}]);
    ylabel(['LOOCV residual ',responseUnits{j}]);

    title(responseNames{j});

    grid on;

end

sgtitle('Leave-One-Out Cross-Validation Residuals');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_19_LOOCV_Residuals.png'));

%% 12. LOOCV ERROR DISTRIBUTION

figure('Name','Phase 9.19 LOOCV Error Distribution');

for j = 1:nResponses

    subplot(1,3,j);

    histogram(loocvResiduals(:,j));

    xlabel(['Prediction error (',responseUnits{j},')']);
    ylabel('Frequency');

    title(responseNames{j});

    grid on;

end

sgtitle('LOOCV Prediction Error Distributions');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_19_LOOCV_Error_Distribution.png'));

%% 13. PREDICTION ERROR BY CASE

figure('Name','Phase 9.19 Prediction Error by Case');

for j = 1:nResponses

    subplot(1,3,j);

    plot(1:n,loocvResiduals(:,j),'-o');

    hold on;

    yline(0,'--');

    xlabel('Simulation case');
    ylabel(['Prediction error (',responseUnits{j},')']);

    title(responseNames{j});

    grid on;

end

sgtitle('LOOCV Prediction Error Across Simulation Cases');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_19_Error_by_Case.png'));

%% 14. VALIDATION INTERPRETATION

fprintf('\n============================================\n');
fprintf('VALIDATION INTERPRETATION\n');
fprintf('============================================\n');

for j = 1:nResponses

    fprintf('\n%s:\n',responseNames{j});

    if loocvR2(j) >= 0.90
        fprintf('LOOCV R-squared: strong predictive performance.\n');

    elseif loocvR2(j) >= 0.70
        fprintf('LOOCV R-squared: moderate predictive performance.\n');

    elseif loocvR2(j) >= 0
        fprintf('LOOCV R-squared: limited predictive performance.\n');

    else
        fprintf('LOOCV R-squared: weaker than the mean-only baseline.\n');

    end

    if loocvRMSE(j) > trainingRMSE(j)*1.5
        fprintf('Warning: substantial cross-validation error increase.\n');
    else
        fprintf('Cross-validation error increase is within 50%% of training RMSE.\n');
    end

end

%% 15. SAVE VALIDATION RESULTS

outputFile = fullfile(resultsFolder, ...
    'phase_9_19_regression_validation_results.mat');

save(outputFile, ...
    'trainingPredictions', ...
    'trainingResiduals', ...
    'loocvPredictions', ...
    'loocvResiduals', ...
    'trainingR2', ...
    'trainingRMSE', ...
    'trainingMAE', ...
    'loocvR2', ...
    'loocvRMSE', ...
    'loocvMAE', ...
    'loocvMaxError', ...
    'validationTable', ...
    'worstCaseTables', ...
    'residualStatisticsTable');

writetable(validationTable,fullfile(resultsFolder, ...
    'Phase_9_19_Validation_Summary.csv'));

writetable(residualStatisticsTable,fullfile(resultsFolder, ...
    'Phase_9_19_Residual_Statistics.csv'));

for j = 1:nResponses

    outputTable = table( ...
        (1:n)', ...
        Y(:,j), ...
        trainingPredictions(:,j), ...
        trainingResiduals(:,j), ...
        loocvPredictions(:,j), ...
        loocvResiduals(:,j), ...
        'VariableNames',{ ...
        'Simulation_Case', ...
        'Actual_Response', ...
        'Training_Prediction', ...
        'Training_Residual', ...
        'LOOCV_Prediction', ...
        'LOOCV_Residual'});

    writetable(outputTable,fullfile(resultsFolder, ...
        sprintf('Phase_9_19_Response_%d_Predictions.csv',j)));

end

%% 16. FINAL SUMMARY

fprintf('\n============================================\n');
fprintf('PHASE 9.19 FINAL SUMMARY\n');
fprintf('============================================\n');

fprintf('Simulation cases analyzed: %d\n',n);

fprintf('New ODE simulations performed: 0\n');

fprintf('Cross-validation method: Leave-One-Out\n');

fprintf('Regression predictors: %d\n',nTerms);

fprintf('Regression coefficients including intercept: %d\n', ...
    numberOfCoefficients);

fprintf('\nCross-validated R-squared:\n');

for j = 1:nResponses

    fprintf('%s: %.8f\n', ...
        responseNames{j},loocvR2(j));

end

fprintf('\nValidation results saved:\n%s\n',outputFile);

fprintf('\n============================================\n');
fprintf('PHASE 9.19 ANALYSIS COMPLETED\n');
fprintf('============================================\n');
