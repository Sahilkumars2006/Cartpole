
%% PHASE 9.17: STATISTICAL AND GLOBAL SENSITIVITY ANALYSIS
% 3D X-Y Cart with Inverted Spherical Pendulum
%
% Uses the 81 simulations generated in Phase 9.16.
% No new ODE simulations are required.

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 9.17: GLOBAL SENSITIVITY ANALYSIS\n');
fprintf('============================================\n');

%% 1. PROJECT PATHS

analysisFolder = fileparts(mfilename('fullpath'));
projectRoot = fileparts(analysisFolder);

resultsFolder = fullfile(projectRoot, ...
    'Results', 'Phase_9');

inputFile = fullfile(resultsFolder, ...
    'phase_9_16_combined_parameter_results.mat');

if ~isfile(inputFile)
    error('Phase 9.16 results file was not found:\n%s',inputFile);
end

if ~exist(resultsFolder,'dir')
    mkdir(resultsFolder);
end

%% 2. LOAD PHASE 9.16 RESULTS

fprintf('\nLoading Phase 9.16 results...\n');

loadedData = load(inputFile,'summaryTable');

if ~isfield(loadedData,'summaryTable')
    error('The summaryTable variable is missing from the results file.');
end

T = loadedData.summaryTable;

if height(T) ~= 81
    error('Expected 81 simulation cases, but found %d.',height(T));
end

fprintf('Loaded %d simulation cases successfully.\n',height(T));

%% 3. DEFINE INPUT PARAMETERS AND OUTPUT RESPONSES

parameterNames = { ...
    'Cart Mass (kg)', ...
    'Bob Mass (kg)', ...
    'Rod Length (m)', ...
    'External Force (N)'};

parameterData = [ ...
    T.Cart_Mass_kg, ...
    T.Bob_Mass_kg, ...
    T.Rod_Length_m, ...
    T.External_Force_N];

responseNames = { ...
    'Maximum Cart Displacement (m)', ...
    'Maximum Cart Speed (m/s)', ...
    'Maximum Pendulum Inclination (deg)'};

responseData = [ ...
    T.Max_Cart_Displacement_m, ...
    T.Max_Cart_Speed_mps, ...
    T.Max_Inclination_deg];

parameterShortNames = {'M','m','L','Fx'};

responseShortNames = { ...
    'Cart_Displacement', ...
    'Cart_Speed', ...
    'Pendulum_Inclination'};

nParameters = size(parameterData,2);
nResponses = size(responseData,2);

%% 4. CHECK DATA VALIDITY

if any(~isfinite(parameterData(:))) || ...
        any(~isfinite(responseData(:)))

    error('The input or response data contains NaN or Inf values.');
end

fprintf('\nData validity check: PASSED\n');

fprintf('Number of input parameters: %d\n',nParameters);
fprintf('Number of response variables: %d\n',nResponses);

%% 5. PEARSON CORRELATION ANALYSIS

fprintf('\n============================================\n');
fprintf('PEARSON CORRELATION ANALYSIS\n');
fprintf('============================================\n');

correlationMatrix = zeros(nParameters,nResponses);

for i = 1:nParameters

    for j = 1:nResponses

        C = corrcoef(parameterData(:,i),responseData(:,j));

        correlationMatrix(i,j) = C(1,2);

    end
end

correlationTable = table( ...
    string(parameterNames(:)), ...
    correlationMatrix(:,1), ...
    correlationMatrix(:,2), ...
    correlationMatrix(:,3), ...
    'VariableNames',{ ...
    'Parameter', ...
    'Cart_Displacement_Correlation', ...
    'Cart_Speed_Correlation', ...
    'Inclination_Correlation'});

disp(correlationTable);

%% 6. ETA-SQUARED GLOBAL SCREENING SENSITIVITY

% Eta-squared measures the fraction of response variation
% associated with the different levels of each parameter.
%
% Values closer to 1 indicate a stronger main effect.
%
% This is a screening measure, not a Sobol index.

fprintf('\n============================================\n');
fprintf('ETA-SQUARED SENSITIVITY ANALYSIS\n');
fprintf('============================================\n');

etaSquared = zeros(nParameters,nResponses);

for i = 1:nParameters

    factorValues = unique(parameterData(:,i));

    for j = 1:nResponses

        y = responseData(:,j);

        overallMean = mean(y);

        totalSS = sum((y-overallMean).^2);

        factorSS = 0;

        for k = 1:numel(factorValues)

            mask = parameterData(:,i) == factorValues(k);

            groupMean = mean(y(mask));

            groupCount = sum(mask);

            factorSS = factorSS + ...
                groupCount*(groupMean-overallMean)^2;

        end

        if totalSS > 0
            etaSquared(i,j) = factorSS/totalSS;
        else
            etaSquared(i,j) = 0;
        end

    end
end

etaTable = table( ...
    string(parameterNames(:)), ...
    etaSquared(:,1), ...
    etaSquared(:,2), ...
    etaSquared(:,3), ...
    'VariableNames',{ ...
    'Parameter', ...
    'Cart_Displacement_Eta2', ...
    'Cart_Speed_Eta2', ...
    'Inclination_Eta2'});

disp(etaTable);

%% 7. RANK PARAMETER SENSITIVITY

fprintf('\n============================================\n');
fprintf('PARAMETER SENSITIVITY RANKING\n');
fprintf('============================================\n');

sensitivityRank = zeros(nParameters,nResponses);

for j = 1:nResponses

    [sortedValues,order] = sort( ...
        etaSquared(:,j),'descend');

    sensitivityRank(order,j) = 1:nParameters;

    fprintf('\nResponse: %s\n',responseNames{j});

    for k = 1:nParameters

        fprintf('%d. %-25s Eta-squared = %.6f\n', ...
            k,parameterNames{order(k)},sortedValues(k));

    end
end

rankingTable = table( ...
    string(parameterNames(:)), ...
    sensitivityRank(:,1), ...
    sensitivityRank(:,2), ...
    sensitivityRank(:,3), ...
    'VariableNames',{ ...
    'Parameter', ...
    'Cart_Displacement_Rank', ...
    'Cart_Speed_Rank', ...
    'Inclination_Rank'});

fprintf('\n--- RANKING SUMMARY ---\n');

disp(rankingTable);

%% 8. MAIN-EFFECT ANALYSIS

fprintf('\n============================================\n');
fprintf('MAIN-EFFECT ANALYSIS\n');
fprintf('============================================\n');

mainEffectMeans = cell(nParameters,nResponses);
mainEffectLevels = cell(nParameters,1);

for i = 1:nParameters

    levels = unique(parameterData(:,i));

    mainEffectLevels{i} = levels;

    fprintf('\nParameter: %s\n',parameterNames{i});

    for j = 1:nResponses

        means = zeros(numel(levels),1);

        for k = 1:numel(levels)

            mask = parameterData(:,i) == levels(k);

            means(k) = mean(responseData(mask,j));

        end

        mainEffectMeans{i,j} = means;

        fprintf('\nResponse: %s\n',responseNames{j});

        for k = 1:numel(levels)

            fprintf('Parameter = %.4f, Mean response = %.6f\n', ...
                levels(k),means(k));

        end
    end
end

%% 9. MAIN-EFFECT PLOTS FOR CART DISPLACEMENT

figure('Name','Global Sensitivity - Cart Displacement');

for i = 1:nParameters

    subplot(2,2,i);

    plot(mainEffectLevels{i}, ...
        mainEffectMeans{i,1},'-o','LineWidth',1.5);

    xlabel(parameterNames{i});
    ylabel(responseNames{1});

    title(['Effect of ',parameterShortNames{i}]);

    grid on;

end

sgtitle('Main Effects on Maximum Cart Displacement');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_17_Main_Effects_Cart_Displacement.png'));

%% 10. MAIN-EFFECT PLOTS FOR CART SPEED

figure('Name','Global Sensitivity - Cart Speed');

for i = 1:nParameters

    subplot(2,2,i);

    plot(mainEffectLevels{i}, ...
        mainEffectMeans{i,2},'-o','LineWidth',1.5);

    xlabel(parameterNames{i});
    ylabel(responseNames{2});

    title(['Effect of ',parameterShortNames{i}]);

    grid on;

end

sgtitle('Main Effects on Maximum Cart Speed');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_17_Main_Effects_Cart_Speed.png'));

%% 11. MAIN-EFFECT PLOTS FOR PENDULUM INCLINATION

figure('Name','Global Sensitivity - Pendulum Inclination');

for i = 1:nParameters

    subplot(2,2,i);

    plot(mainEffectLevels{i}, ...
        mainEffectMeans{i,3},'-o','LineWidth',1.5);

    xlabel(parameterNames{i});
    ylabel(responseNames{3});

    title(['Effect of ',parameterShortNames{i}]);

    grid on;

end

sgtitle('Main Effects on Maximum Pendulum Inclination');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_17_Main_Effects_Inclination.png'));

%% 12. ETA-SQUARED SENSITIVITY BAR CHARTS

figure('Name','Eta Squared Sensitivity');

for j = 1:nResponses

    subplot(1,3,j);

    bar(etaSquared(:,j));

    set(gca, ...
        'XTick',1:nParameters, ...
        'XTickLabel',parameterShortNames);

    ylabel('Eta-squared');

    title(responseShortNames{j});

    ylim([0 1]);

    grid on;

end

sgtitle('Global Parameter Sensitivity Ranking');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_17_Eta_Squared_Sensitivity.png'));

%% 13. PEARSON CORRELATION HEATMAP

figure('Name','Pearson Correlation Matrix');

imagesc(correlationMatrix);

colorbar;

clim([-1 1]);

set(gca, ...
    'XTick',1:nResponses, ...
    'XTickLabel',responseShortNames, ...
    'YTick',1:nParameters, ...
    'YTickLabel',parameterShortNames);

xlabel('Response variable');
ylabel('Input parameter');

title('Pearson Correlation Matrix');

for i = 1:nParameters

    for j = 1:nResponses

        text(j,i,sprintf('%.3f',correlationMatrix(i,j)), ...
            'HorizontalAlignment','center', ...
            'FontWeight','bold');

    end
end

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_17_Pearson_Correlation_Heatmap.png'));

%% 14. SAVE ANALYSIS RESULTS

save(fullfile(resultsFolder, ...
    'phase_9_17_global_sensitivity_results.mat'), ...
    'correlationMatrix', ...
    'correlationTable', ...
    'etaSquared', ...
    'etaTable', ...
    'sensitivityRank', ...
    'rankingTable', ...
    'mainEffectMeans', ...
    'mainEffectLevels', ...
    'parameterData', ...
    'responseData', ...
    'parameterNames', ...
    'responseNames');

writetable(correlationTable,fullfile(resultsFolder, ...
    'Phase_9_17_Pearson_Correlation.csv'));

writetable(etaTable,fullfile(resultsFolder, ...
    'Phase_9_17_Eta_Squared_Sensitivity.csv'));

writetable(rankingTable,fullfile(resultsFolder, ...
    'Phase_9_17_Sensitivity_Ranking.csv'));

%% 15. FINAL SUMMARY

fprintf('\n============================================\n');
fprintf('PHASE 9.17 FINAL SUMMARY\n');
fprintf('============================================\n');

fprintf('Simulation cases analyzed: %d\n',height(T));

fprintf('New ODE simulations performed: 0\n');

fprintf('\nMaximum eta-squared values:\n');

for j = 1:nResponses

    [maximumEta,index] = max(etaSquared(:,j));

    fprintf('%s:\n',responseNames{j});

    fprintf('Most influential parameter: %s\n', ...
        parameterNames{index});

    fprintf('Eta-squared: %.6f\n',maximumEta);

end

fprintf('\n============================================\n');
fprintf('PHASE 9.17 ANALYSIS COMPLETED\n');
fprintf('============================================\n');

fprintf('\nSaved files in:\n%s\n',resultsFolder);

fprintf('\nMAT-files and CSV summaries saved successfully.\n');

fprintf('\n============================================\n');
fprintf('PHASE 9.17 SCRIPT FINISHED\n');
fprintf('============================================\n');
