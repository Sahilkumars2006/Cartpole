
%% ============================================================
% PHASE 7.10I: SOLVER SENSITIVITY ANALYSIS
% Baseline Simscape vs Tight-Solver Simscape vs MATLAB
% ============================================================

clc;
close all;

fprintf('\n============================================\n');
fprintf(' PHASE 7.10I: SOLVER SENSITIVITY ANALYSIS\n');
fprintf('============================================\n');

%% 1. FILE LOCATIONS

projectRoot = fileparts(fileparts(mfilename('fullpath')));

baselineFile = fullfile(projectRoot, ...
    'Results','Simscape', ...
    'uncontrolled_simscape_baseline.mat');

tightFile = fullfile(projectRoot, ...
    'Results','Simscape', ...
    'uncontrolled_simscape_tight_solver_results.mat');

matlabFile = fullfile(projectRoot, ...
    'Results','MATLAB', ...
    'cartesian_simulation_results.mat');

if ~isfile(baselineFile)
    error('Baseline Simscape results not found.');
end

if ~isfile(tightFile)
    error('Tight-solver results not found.');
end

if ~isfile(matlabFile)
    error('MATLAB simulation results not found.');
end

%% 2. LOAD RESULTS

A = load(baselineFile,'results');
B = load(tightFile,'results');
C = load(matlabFile,'results');

baseline = A.results;
tight = B.results;
matlab = C.results;

fprintf('\nAll three result files loaded successfully.\n');

%% 3. EXTRACT RELATIVE POSITION VECTORS

% Simscape results already contain bob position
% relative to the cart pivot.

rBaseline = baseline.relative_position;
rTight = tight.relative_position;

% MATLAB Cartesian model:
% Bob position minus cart pivot position.

bobMAT = matlab.bob_position;
cartMAT = matlab.cart_position;

if size(cartMAT,2) == 2
    cartMAT = [cartMAT, zeros(size(cartMAT,1),1)];
end

rMAT = bobMAT - cartMAT;

% Verify dimensions.

if size(rBaseline,2) ~= 3 || ...
   size(rTight,2) ~= 3 || ...
   size(rMAT,2) ~= 3

    error('Relative position data must have three columns.');
end

%% 4. COMMON TIME VECTOR

commonStart = max([ ...
    baseline.time(1), ...
    tight.time(1), ...
    matlab.time(1)]);

commonEnd = min([ ...
    baseline.time(end), ...
    tight.time(end), ...
    matlab.time(end)]);

t = linspace(commonStart,commonEnd,1501).';

% Interpolate all trajectories onto the same time vector.

rBaselineI = interp1( ...
    baseline.time,rBaseline,t,'linear');

rTightI = interp1( ...
    tight.time,rTight,t,'linear');

rMATI = interp1( ...
    matlab.time,rMAT,t,'linear');

fprintf('\nCommon comparison duration: %.4f s\n', ...
    commonEnd-commonStart);

fprintf('Comparison samples: %d\n',numel(t));

%% 5. POSITION ERRORS

errorBaselineTight = rBaselineI-rTightI;
errorMATBaseline = rMATI-rBaselineI;
errorMATTight = rMATI-rTightI;

normBaselineTight = vecnorm(errorBaselineTight,2,2);
normMATBaseline = vecnorm(errorMATBaseline,2,2);
normMATTight = vecnorm(errorMATTight,2,2);

rmsBaselineTight = sqrt(mean(normBaselineTight.^2));
rmsMATBaseline = sqrt(mean(normMATBaseline.^2));
rmsMATTight = sqrt(mean(normMATTight.^2));

maxBaselineTight = max(normBaselineTight);
maxMATBaseline = max(normMATBaseline);
maxMATTight = max(normMATTight);

%% 6. DIRECTION ERRORS

% Normalize relative position vectors.

uBaseline = rBaselineI ./ vecnorm(rBaselineI,2,2);
uTight = rTightI ./ vecnorm(rTightI,2,2);
uMAT = rMATI ./ vecnorm(rMATI,2,2);

dotBaselineTight = sum(uBaseline.*uTight,2);
dotMATBaseline = sum(uMAT.*uBaseline,2);
dotMATTight = sum(uMAT.*uTight,2);

angleBaselineTight = rad2deg(acos( ...
    max(-1,min(1,dotBaselineTight))));

angleMATBaseline = rad2deg(acos( ...
    max(-1,min(1,dotMATBaseline))));

angleMATTight = rad2deg(acos( ...
    max(-1,min(1,dotMATTight))));

%% 7. PRINT RESULTS

fprintf('\n============================================\n');
fprintf(' POSITION COMPARISON\n');
fprintf('============================================\n');

fprintf('\nBaseline vs Tight Simscape:\n');
fprintf('RMS error: %.8f m\n',rmsBaselineTight);
fprintf('Maximum error: %.8f m\n',maxBaselineTight);

fprintf('\nMATLAB vs Baseline Simscape:\n');
fprintf('RMS error: %.8f m\n',rmsMATBaseline);
fprintf('Maximum error: %.8f m\n',maxMATBaseline);

fprintf('\nMATLAB vs Tight Simscape:\n');
fprintf('RMS error: %.8f m\n',rmsMATTight);
fprintf('Maximum error: %.8f m\n',maxMATTight);

fprintf('\n============================================\n');
fprintf(' DIRECTION COMPARISON\n');
fprintf('============================================\n');

fprintf('\nBaseline vs Tight Simscape:\n');
fprintf('RMS angle error: %.6f degrees\n', ...
    sqrt(mean(angleBaselineTight.^2)));

fprintf('\nMATLAB vs Baseline Simscape:\n');
fprintf('RMS angle error: %.6f degrees\n', ...
    sqrt(mean(angleMATBaseline.^2)));

fprintf('\nMATLAB vs Tight Simscape:\n');
fprintf('RMS angle error: %.6f degrees\n', ...
    sqrt(mean(angleMATTight.^2)));

%% 8. TIME-WINDOW SENSITIVITY

timeWindows = [1 2 3 5 7.5 10 12.5 15];

fprintf('\n============================================\n');
fprintf(' TIME-WINDOW RMS POSITION ERROR\n');
fprintf('============================================\n');

fprintf('\nTime (s) | Baseline-Tight | MATLAB-Baseline | MATLAB-Tight\n');

for k = 1:numel(timeWindows)

    idx = t <= timeWindows(k);

    if ~any(idx)
        continue;
    end

    e1 = sqrt(mean(normBaselineTight(idx).^2));
    e2 = sqrt(mean(normMATBaseline(idx).^2));
    e3 = sqrt(mean(normMATTight(idx).^2));

    fprintf('%8.2f | %14.6f | %15.6f | %12.6f\n', ...
        timeWindows(k),e1,e2,e3);

end

%% 9. PLOT RELATIVE POSITION COMPARISON

figure('Name','Solver Sensitivity - Relative Position', ...
    'Color','w','Position',[100 100 1250 850]);

labels = {'Relative X (m)','Relative Y (m)','Relative Z (m)'};

for k = 1:3

    subplot(3,1,k);

    plot(t,rMATI(:,k),'k','LineWidth',1.5);
    hold on;

    plot(t,rBaselineI(:,k),'b--','LineWidth',1.3);
    plot(t,rTightI(:,k),'r-.','LineWidth',1.3);

    grid on;

    xlabel('Time (s)');
    ylabel(labels{k});

    if k == 1
        title('MATLAB and Simscape Relative Position');
        legend('MATLAB','Baseline Simscape', ...
            'Tight Simscape','Location','best');
    end

end

%% 10. PLOT POSITION ERROR

figure('Name','Solver Sensitivity - Position Error', ...
    'Color','w','Position',[150 150 1200 650]);

plot(t,normBaselineTight,'k','LineWidth',1.5);
hold on;

plot(t,normMATBaseline,'b','LineWidth',1.3);
plot(t,normMATTight,'r','LineWidth',1.3);

grid on;

xlabel('Time (s)');
ylabel('3D relative position error (m)');

title('Trajectory Difference');

legend('Baseline vs Tight Simscape', ...
    'MATLAB vs Baseline Simscape', ...
    'MATLAB vs Tight Simscape', ...
    'Location','best');

%% 11. PLOT DIRECTION ERROR

figure('Name','Solver Sensitivity - Direction Error', ...
    'Color','w','Position',[200 200 1200 600]);

plot(t,angleBaselineTight,'k','LineWidth',1.5);
hold on;

plot(t,angleMATBaseline,'b','LineWidth',1.3);
plot(t,angleMATTight,'r','LineWidth',1.3);

grid on;

xlabel('Time (s)');
ylabel('Direction error (degrees)');

title('Pendulum Direction Comparison');

legend('Baseline vs Tight Simscape', ...
    'MATLAB vs Baseline Simscape', ...
    'MATLAB vs Tight Simscape', ...
    'Location','best');

%% 12. SAVE ANALYSIS

analysis.time = t;

analysis.baseline_tight_position_error = normBaselineTight;
analysis.matlab_baseline_position_error = normMATBaseline;
analysis.matlab_tight_position_error = normMATTight;

analysis.baseline_tight_direction_error = angleBaselineTight;
analysis.matlab_baseline_direction_error = angleMATBaseline;
analysis.matlab_tight_direction_error = angleMATTight;

analysis.rms_baseline_tight = rmsBaselineTight;
analysis.rms_matlab_baseline = rmsMATBaseline;
analysis.rms_matlab_tight = rmsMATTight;

analysis.max_baseline_tight = maxBaselineTight;
analysis.max_matlab_baseline = maxMATBaseline;
analysis.max_matlab_tight = maxMATTight;

outputFile = fullfile(projectRoot, ...
    'Results','Comparisons', ...
    'solver_sensitivity_analysis.mat');

outputFolder = fileparts(outputFile);

if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end

save(outputFile,'analysis');

fprintf('\n============================================\n');
fprintf(' PHASE 7.10I ANALYSIS COMPLETED\n');
fprintf('============================================\n');

fprintf('\nAnalysis saved to:\n%s\n',outputFile);
