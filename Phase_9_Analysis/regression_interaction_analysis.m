
%% PHASE 9.18: MULTIVARIABLE REGRESSION AND INTERACTION ANALYSIS
% 3D X-Y Cart with Inverted Spherical Pendulum
%
% Uses the 81 simulation cases from Phase 9.16.
% No new ODE simulations are performed.
%
% Second-order response surface:
% - Linear terms
% - Quadratic terms
% - Pairwise interaction terms

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 9.18: REGRESSION AND INTERACTION ANALYSIS\n');
fprintf('============================================\n');

%% 1. PATHS AND INPUT DATA

analysisFolder = fileparts(mfilename('fullpath'));
projectRoot = fileparts(analysisFolder);

resultsFolder = fullfile(projectRoot,'Results','Phase_9');

inputFile = fullfile(resultsFolder, ...
    'phase_9_16_combined_parameter_results.mat');

if ~isfile(inputFile)
    error('Phase 9.16 results file not found:\n%s',inputFile);
end

loadedData = load(inputFile,'summaryTable');

if ~isfield(loadedData,'summaryTable')
    error('summaryTable is missing from the Phase 9.16 file.');
end

T = loadedData.summaryTable;

if height(T) ~= 81
    error('Expected 81 cases. Found %d.',height(T));
end

fprintf('\nLoaded %d simulation cases.\n',height(T));

%% 2. DEFINE PARAMETERS AND RESPONSES

% Original physical parameter values

Xphysical = [ ...
    T.Cart_Mass_kg, ...
    T.Bob_Mass_kg, ...
    T.Rod_Length_m, ...
    T.External_Force_N];

Y = [ ...
    T.Max_Cart_Displacement_m, ...
    T.Max_Cart_Speed_mps, ...
    T.Max_Inclination_deg];

parameterNames = {'Cart Mass','Bob Mass','Rod Length','External Force'};

parameterSymbols = {'M','m','L','Fx'};

responseNames = { ...
    'Maximum Cart Displacement', ...
    'Maximum Cart Speed', ...
    'Maximum Pendulum Inclination'};

responseUnits = {'m','m/s','deg'};

if any(~isfinite(Xphysical(:))) || any(~isfinite(Y(:)))
    error('Input data contains NaN or Inf values.');
end

n = size(Xphysical,1);
nFactors = size(Xphysical,2);
nResponses = size(Y,2);

%% 3. CODE FACTORS TO -1, 0, +1

% The three tested levels of each parameter are equally spaced.
% Coding makes the regression numerically better conditioned.

Xcoded = zeros(size(Xphysical));

for i = 1:nFactors

    levels = unique(Xphysical(:,i));

    if numel(levels) ~= 3
        error('Parameter %s does not have exactly 3 levels.', ...
            parameterNames{i});
    end

    Xcoded(:,i) = (Xphysical(:,i)-mean(levels)) / ...
        ((max(levels)-min(levels))/2);

end

fprintf('\nCoded parameter levels:\n');

for i = 1:nFactors
    fprintf('%s: ',parameterNames{i});
    fprintf('%.2f ',unique(Xcoded(:,i)));
    fprintf('\n');
end

%% 4. BUILD SECOND-ORDER REGRESSION DESIGN MATRIX

% Design matrix includes:
% 4 linear terms
% 4 quadratic terms
% 6 pairwise interactions
%
% Total = 14 predictor terms + intercept.

nInteractions = nFactors*(nFactors-1)/2;
nTerms = 2*nFactors+nInteractions;

Xdesign = zeros(n,nTerms);

termNames = cell(nTerms,1);

termFactor1 = zeros(nTerms,1);
termFactor2 = zeros(nTerms,1);

termType = cell(nTerms,1);

column = 0;

% Linear terms

for i = 1:nFactors

    column = column+1;

    Xdesign(:,column) = Xcoded(:,i);

    termNames{column} = parameterSymbols{i};

    termFactor1(column) = i;
    termFactor2(column) = 0;

    termType{column} = 'Linear';

end

% Quadratic terms

for i = 1:nFactors

    column = column+1;

    Xdesign(:,column) = Xcoded(:,i).^2;

    termNames{column} = [parameterSymbols{i},'^2'];

    termFactor1(column) = i;
    termFactor2(column) = 0;

    termType{column} = 'Quadratic';

end

% Pairwise interactions

for i = 1:nFactors

    for j = i+1:nFactors

        column = column+1;

        Xdesign(:,column) = Xcoded(:,i).*Xcoded(:,j);

        termNames{column} = ...
            [parameterSymbols{i},' x ',parameterSymbols{j}];

        termFactor1(column) = i;
        termFactor2(column) = j;

        termType{column} = 'Interaction';

    end
end

if column ~= nTerms
    error('Regression design matrix construction failed.');
end

fprintf('\nRegression design matrix created.\n');
fprintf('Simulation cases: %d\n',n);
fprintf('Predictor terms: %d\n',nTerms);
fprintf('Total regression coefficients: %d\n',nTerms+1);

%% 5. CHECK DESIGN MATRIX RANK

fullDesign = [ones(n,1),Xdesign];

designRank = rank(fullDesign);

fprintf('\nDesign matrix rank: %d of %d\n', ...
    designRank,size(fullDesign,2));

if designRank < size(fullDesign,2)
    error('Regression design matrix is rank deficient.');
end

%% 6. FIT SECOND-ORDER MODELS

fprintf('\n============================================\n');
fprintf('SECOND-ORDER REGRESSION RESULTS\n');
fprintf('============================================\n');

regressionModels = cell(nResponses,1);

regressionSummary = table();
coefficientTable = table();

standardizedCoefficients = zeros(nTerms,nResponses);

predictedResponses = zeros(n,nResponses);
residualResponses = zeros(n,nResponses);

R2 = zeros(nResponses,1);
adjustedR2 = zeros(nResponses,1);
RMSE = zeros(nResponses,1);
residualStandardError = zeros(nResponses,1);

for j = 1:nResponses

    y = Y(:,j);

    % Fit the model with intercept

    beta = fullDesign \ y;

    yPredicted = fullDesign*beta;

    residual = y-yPredicted;

    SSE = sum(residual.^2);

    SST = sum((y-mean(y)).^2);

    if SST > 0
        R2(j) = 1-SSE/SST;
    else
        R2(j) = NaN;
    end

    numberOfPredictors = nTerms;

    adjustedR2(j) = 1-(1-R2(j))* ...
        (n-1)/(n-numberOfPredictors-1);

    RMSE(j) = sqrt(SSE/n);

    residualStandardError(j) = ...
        sqrt(SSE/(n-numberOfPredictors-1));

    % Standardize predictors and response.
    % Standardized coefficients allow comparison of term magnitudes.

    yStd = std(y,0);

    if yStd <= eps
        error('Response %s has zero standard deviation.',responseNames{j});
    end

    Xstd = zeros(n,nTerms);

    for k = 1:nTerms

        termStd = std(Xdesign(:,k),0);

        if termStd <= eps
            error('Regression term %s has zero variation.',termNames{k});
        end

        Xstd(:,k) = ...
            (Xdesign(:,k)-mean(Xdesign(:,k)))/termStd;

    end

    yStandardized = (y-mean(y))/yStd;

    betaStandardized = Xstd \ yStandardized;

    standardizedCoefficients(:,j) = betaStandardized;

    predictedResponses(:,j) = yPredicted;
    residualResponses(:,j) = residual;

    regressionModels{j} = struct( ...
        'coefficients',beta, ...
        'standardizedCoefficients',betaStandardized, ...
        'predicted',yPredicted, ...
        'residuals',residual, ...
        'SSE',SSE);

    fprintf('\nResponse: %s (%s)\n', ...
        responseNames{j},responseUnits{j});

    fprintf('R-squared:             %.8f\n',R2(j));
    fprintf('Adjusted R-squared:    %.8f\n',adjustedR2(j));
    fprintf('RMSE:                  %.8e\n',RMSE(j));
    fprintf('Residual standard err: %.8e\n', ...
        residualStandardError(j));

    fprintf('\nStandardized regression coefficients:\n');

    for k = 1:nTerms

        fprintf('%-12s : %+12.6f\n', ...
            termNames{k},betaStandardized(k));

    end

    fprintf('Intercept (original units): %.8f\n',beta(1));

end

%% 7. REGRESSION SUMMARY TABLE

regressionSummary = table( ...
    string(responseNames(:)), ...
    R2, ...
    adjustedR2, ...
    RMSE, ...
    residualStandardError, ...
    'VariableNames',{ ...
    'Response', ...
    'R_Squared', ...
    'Adjusted_R_Squared', ...
    'RMSE', ...
    'Residual_Standard_Error'});

fprintf('\n--- MODEL QUALITY SUMMARY ---\n');
disp(regressionSummary);

%% 8. STANDARDIZED COEFFICIENT TABLE

coefficientTable = table( ...
    string(termNames), ...
    string(termType), ...
    standardizedCoefficients(:,1), ...
    standardizedCoefficients(:,2), ...
    standardizedCoefficients(:,3), ...
    'VariableNames',{ ...
    'Regression_Term', ...
    'Term_Type', ...
    'Cart_Displacement_Beta', ...
    'Cart_Speed_Beta', ...
    'Inclination_Beta'});

fprintf('\n--- STANDARDIZED COEFFICIENTS ---\n');
disp(coefficientTable);

%% 9. INDIVIDUAL TERM PARTIAL R-SQUARED

% For each term, remove that term from the full model.
% Partial R-squared measures the additional fit associated
% with that term, conditional on all the other model terms.

partialR2Terms = zeros(nTerms,nResponses);

for j = 1:nResponses

    y = Y(:,j);

    fullBeta = fullDesign \ y;

    fullResidual = y-fullDesign*fullBeta;

    fullSSE = sum(fullResidual.^2);

    for k = 1:nTerms

        retainedColumns = setdiff(1:nTerms,k);

        reducedDesign = [ones(n,1),Xdesign(:,retainedColumns)];

        reducedBeta = reducedDesign \ y;

        reducedResidual = y-reducedDesign*reducedBeta;

        reducedSSE = sum(reducedResidual.^2);

        partialR2Terms(k,j) = ...
            max(0,(reducedSSE-fullSSE)/reducedSSE);

    end
end

partialTermTable = table( ...
    string(termNames), ...
    string(termType), ...
    partialR2Terms(:,1), ...
    partialR2Terms(:,2), ...
    partialR2Terms(:,3), ...
    'VariableNames',{ ...
    'Term', ...
    'Term_Type', ...
    'Cart_Displacement_Partial_R2', ...
    'Cart_Speed_Partial_R2', ...
    'Inclination_Partial_R2'});

fprintf('\n--- INDIVIDUAL TERM PARTIAL R-SQUARED ---\n');
disp(partialTermTable);

%% 10. PARAMETER GROUP CONTRIBUTION

% Remove all terms involving a particular physical parameter.
% This measures the combined conditional contribution of its
% linear, quadratic and pairwise interaction terms.

parameterPartialR2 = zeros(nFactors,nResponses);

for j = 1:nResponses

    y = Y(:,j);

    fullBeta = fullDesign \ y;

    fullResidual = y-fullDesign*fullBeta;

    fullSSE = sum(fullResidual.^2);

    for i = 1:nFactors

        involvedTerms = ...
            termFactor1 == i | termFactor2 == i;

        retainedTerms = find(~involvedTerms);

        reducedDesign = ...
            [ones(n,1),Xdesign(:,retainedTerms)];

        reducedBeta = reducedDesign \ y;

        reducedResidual = y-reducedDesign*reducedBeta;

        reducedSSE = sum(reducedResidual.^2);

        parameterPartialR2(i,j) = ...
            max(0,(reducedSSE-fullSSE)/reducedSSE);

    end
end

parameterContributionTable = table( ...
    string(parameterNames(:)), ...
    parameterPartialR2(:,1), ...
    parameterPartialR2(:,2), ...
    parameterPartialR2(:,3), ...
    'VariableNames',{ ...
    'Parameter', ...
    'Cart_Displacement_Partial_R2', ...
    'Cart_Speed_Partial_R2', ...
    'Inclination_Partial_R2'});

fprintf('\n--- PARAMETER GROUP CONTRIBUTION ---\n');
disp(parameterContributionTable);

%% 11. RANK PAIRWISE INTERACTIONS

interactionIndices = find(strcmp(termType,'Interaction'));

interactionNames = termNames(interactionIndices);

interactionScores = partialR2Terms(interactionIndices,:);

interactionRankingTable = table( ...
    string(interactionNames), ...
    interactionScores(:,1), ...
    interactionScores(:,2), ...
    interactionScores(:,3), ...
    'VariableNames',{ ...
    'Interaction', ...
    'Cart_Displacement_Partial_R2', ...
    'Cart_Speed_Partial_R2', ...
    'Inclination_Partial_R2'});

fprintf('\n============================================\n');
fprintf('PAIRWISE INTERACTION RANKING\n');
fprintf('============================================\n');

for j = 1:nResponses

    [sortedScores,order] = ...
        sort(interactionScores(:,j),'descend');

    fprintf('\nResponse: %s\n',responseNames{j});

    for k = 1:numel(order)

        fprintf('%d. %-12s Partial R2 = %.8f\n', ...
            k,interactionNames{order(k)},sortedScores(k));

    end
end

%% 12. LINEAR-ONLY MODEL COMPARISON

% Compare the second-order model with a simpler linear model.

linearDesign = [ones(n,1),Xcoded];

linearR2 = zeros(nResponses,1);
linearRMSE = zeros(nResponses,1);

for j = 1:nResponses

    y = Y(:,j);

    linearBeta = linearDesign \ y;

    linearPrediction = linearDesign*linearBeta;

    linearResidual = y-linearPrediction;

    SSE = sum(linearResidual.^2);

    SST = sum((y-mean(y)).^2);

    linearR2(j) = 1-SSE/SST;

    linearRMSE(j) = sqrt(SSE/n);

end

modelComparisonTable = table( ...
    string(responseNames(:)), ...
    linearR2, ...
    R2, ...
    linearRMSE, ...
    RMSE, ...
    'VariableNames',{ ...
    'Response', ...
    'Linear_R2', ...
    'Second_Order_R2', ...
    'Linear_RMSE', ...
    'Second_Order_RMSE'});

fprintf('\n============================================\n');
fprintf('LINEAR VS SECOND-ORDER MODEL COMPARISON\n');
fprintf('============================================\n');

disp(modelComparisonTable);

%% 13. PREDICTED VS ACTUAL RESPONSE PLOTS

figure('Name','Regression Predicted vs Actual');

for j = 1:nResponses

    subplot(1,3,j);

    plot(Y(:,j),predictedResponses(:,j),'o');

    hold on;

    minimumValue = min(Y(:,j));
    maximumValue = max(Y(:,j));

    plot([minimumValue maximumValue], ...
        [minimumValue maximumValue],'--');

    xlabel(['Actual ',responseUnits{j}]);
    ylabel(['Predicted ',responseUnits{j}]);

    title(responseNames{j});

    grid on;
    axis tight;

end

sgtitle('Second-Order Regression: Predicted vs Actual');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_18_Regression_Predicted_vs_Actual.png'));

%% 14. RESIDUAL ANALYSIS

figure('Name','Regression Residual Analysis');

for j = 1:nResponses

    subplot(1,3,j);

    plot(predictedResponses(:,j), ...
        residualResponses(:,j),'o');

    hold on;

    yline(0,'--');

    xlabel(['Predicted ',responseUnits{j}]);
    ylabel(['Residual ',responseUnits{j}]);

    title(responseNames{j});

    grid on;

end

sgtitle('Second-Order Regression Residuals');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_18_Regression_Residuals.png'));

%% 15. STANDARDIZED COEFFICIENT PLOT

figure('Name','Standardized Regression Coefficients');

bar(standardizedCoefficients);

set(gca, ...
    'XTick',1:nTerms, ...
    'XTickLabel',termNames);

xtickangle(45);

ylabel('Standardized coefficient');

legend(responseNames,'Location','best');

title('Standardized Regression Coefficients');

grid on;

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_18_Standardized_Coefficients.png'));

%% 16. INTERACTION CONTRIBUTION PLOT

figure('Name','Pairwise Interaction Contributions');

bar(interactionScores);

set(gca, ...
    'XTick',1:numel(interactionNames), ...
    'XTickLabel',interactionNames);

xtickangle(30);

ylabel('Partial R-squared');

legend(responseNames,'Location','best');

title('Conditional Pairwise Interaction Contributions');

grid on;

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_18_Interaction_Contributions.png'));

%% 17. SAVE ALL RESULTS

outputFile = fullfile(resultsFolder, ...
    'phase_9_18_regression_interaction_results.mat');

save(outputFile, ...
    'Xphysical', ...
    'Xcoded', ...
    'Y', ...
    'Xdesign', ...
    'termNames', ...
    'termType', ...
    'regressionModels', ...
    'regressionSummary', ...
    'coefficientTable', ...
    'standardizedCoefficients', ...
    'partialR2Terms', ...
    'partialTermTable', ...
    'parameterPartialR2', ...
    'parameterContributionTable', ...
    'interactionRankingTable', ...
    'modelComparisonTable', ...
    'predictedResponses', ...
    'residualResponses');

writetable(regressionSummary,fullfile(resultsFolder, ...
    'Phase_9_18_Regression_Model_Summary.csv'));

writetable(coefficientTable,fullfile(resultsFolder, ...
    'Phase_9_18_Standardized_Coefficients.csv'));

writetable(partialTermTable,fullfile(resultsFolder, ...
    'Phase_9_18_Term_Partial_R2.csv'));

writetable(parameterContributionTable,fullfile(resultsFolder, ...
    'Phase_9_18_Parameter_Contribution.csv'));

writetable(interactionRankingTable,fullfile(resultsFolder, ...
    'Phase_9_18_Interaction_Ranking.csv'));

writetable(modelComparisonTable,fullfile(resultsFolder, ...
    'Phase_9_18_Model_Comparison.csv'));

%% 18. FINAL VALIDATION AND SUMMARY

fprintf('\n============================================\n');
fprintf('PHASE 9.18 FINAL SUMMARY\n');
fprintf('============================================\n');

fprintf('Simulation cases analyzed: %d\n',n);
fprintf('New ODE simulations performed: 0\n');

fprintf('\nRegression model:\n');
fprintf('Linear terms: 4\n');
fprintf('Quadratic terms: 4\n');
fprintf('Pairwise interaction terms: 6\n');
fprintf('Total predictor terms: %d\n',nTerms);

fprintf('\nModel fit results:\n');

for j = 1:nResponses

    fprintf('\n%s:\n',responseNames{j});

    fprintf('R-squared: %.8f\n',R2(j));

    fprintf('Adjusted R-squared: %.8f\n',adjustedR2(j));

    fprintf('RMSE: %.8e\n',RMSE(j));

    fprintf('Linear model R-squared: %.8f\n',linearR2(j));

    fprintf('Second-order model R-squared: %.8f\n',R2(j));

end

fprintf('\nMaximum absolute regression residuals:\n');

for j = 1:nResponses

    fprintf('%s: %.8e %s\n', ...
        responseNames{j}, ...
        max(abs(residualResponses(:,j))), ...
        responseUnits{j});

end

fprintf('\nSaved regression results:\n%s\n',outputFile);

fprintf('\n============================================\n');
fprintf('PHASE 9.18 ANALYSIS COMPLETED\n');
fprintf('============================================\n');
