
%% ============================================
% PHASE 7.10G: RELATIVE VECTOR COMPARISON
% =============================================

clc;
close all;

fprintf('\n============================================\n');
fprintf(' PHASE 7.10G: RELATIVE VECTOR COMPARISON\n');
fprintf('============================================\n');

%% 1. Load results

M = load(fullfile('..','Results','MATLAB', ...
    'cartesian_simulation_results.mat'));

S = load(fullfile('..','Results','Simscape', ...
    'uncontrolled_simscape_results.mat'));

mat = M.results;
sim = S.results;

%% 2. Calculate MATLAB bob position relative to cart

cartM3 = [mat.cart_position, ...
    zeros(size(mat.cart_position,1),1)];

relativeM = mat.bob_position - cartM3;

% Simscape relative_position is already expressed in world coordinates.
relativeS = sim.relative_position;

%% 3. Common time vector

tStart = max(mat.time(1),sim.time(1));
tEnd = min(mat.time(end),sim.time(end));

t = linspace(tStart,tEnd,1501)';

relativeMi = interp1(mat.time,relativeM,t,'linear');
relativeSi = interp1(sim.time,relativeS,t,'linear');

%% 4. Relative vector errors

vectorError = relativeMi-relativeSi;

vectorErrorNorm = sqrt(sum(vectorError.^2,2));

% Normalize vectors to compare pendulum directions.
normM = sqrt(sum(relativeMi.^2,2));
normS = sqrt(sum(relativeSi.^2,2));

unitM = relativeMi./normM;
unitS = relativeSi./normS;

dotProduct = sum(unitM.*unitS,2);
dotProduct = max(-1,min(1,dotProduct));

directionErrorDeg = rad2deg(acos(dotProduct));

fprintf('\n--- RELATIVE POSITION ERRORS ---\n');

fprintf('X RMS error: %.8f m\n', ...
    sqrt(mean(vectorError(:,1).^2)));

fprintf('Y RMS error: %.8f m\n', ...
    sqrt(mean(vectorError(:,2).^2)));

fprintf('Z RMS error: %.8f m\n', ...
    sqrt(mean(vectorError(:,3).^2)));

fprintf('3D relative position RMS error: %.8f m\n', ...
    sqrt(mean(vectorErrorNorm.^2)));

fprintf('Pendulum direction RMS error: %.6f deg\n', ...
    sqrt(mean(directionErrorDeg.^2)));

fprintf('Maximum direction error: %.6f deg\n', ...
    max(directionErrorDeg));

%% 5. Selected time comparison

checkTimes = [0 1 2 3 5 7.5 10 12.5 15]';
checkTimes = checkTimes(checkTimes >= tStart & ...
    checkTimes <= tEnd);

idx = zeros(size(checkTimes));

for k = 1:numel(checkTimes)
    [~,idx(k)] = min(abs(t-checkTimes(k)));
end

T = table( ...
    t(idx), ...
    directionErrorDeg(idx), ...
    vectorErrorNorm(idx), ...
    relativeMi(idx,1), ...
    relativeSi(idx,1), ...
    relativeMi(idx,2), ...
    relativeSi(idx,2), ...
    relativeMi(idx,3), ...
    relativeSi(idx,3), ...
    'VariableNames', { ...
    'Time_s', ...
    'DirectionError_deg', ...
    'PositionError_m', ...
    'MATLAB_RelX', ...
    'Simscape_RelX', ...
    'MATLAB_RelY', ...
    'Simscape_RelY', ...
    'MATLAB_RelZ', ...
    'Simscape_RelZ'});

fprintf('\n--- SELECTED TIME COMPARISON ---\n');
disp(T);

%% 6. Plot relative position components

figure('Name','Relative Position Comparison', ...
    'Color','w');

componentNames = {'X','Y','Z'};

for k = 1:3

    subplot(3,1,k);

    plot(mat.time,relativeM(:,k), ...
        'b-','LineWidth',1.5);
    hold on;

    plot(sim.time,relativeS(:,k), ...
        'r--','LineWidth',1.5);

    grid on;
    ylabel(['Relative ' componentNames{k} ' (m)']);

    if k == 1
        title('Pendulum Relative Position: MATLAB vs Simscape');
        legend('MATLAB','Simscape','Location','best');
    end

    if k == 3
        xlabel('Time (s)');
    end
end

%% 7. Plot direction error

figure('Name','Pendulum Direction Error', ...
    'Color','w');

plot(t,directionErrorDeg,'LineWidth',1.5);

grid on;
xlabel('Time (s)');
ylabel('Direction Error (degrees)');
title('Angular Difference Between Pendulum Direction Vectors');

%% 8. Save results

comparison = struct();

comparison.time = t;
comparison.relative_matlab = relativeMi;
comparison.relative_simscape = relativeSi;
comparison.vector_error = vectorError;
comparison.vector_error_norm = vectorErrorNorm;
comparison.direction_error_deg = directionErrorDeg;
comparison.direction_error_RMS_deg = ...
    sqrt(mean(directionErrorDeg.^2));

save(fullfile('..','Results','Comparisons', ...
    'relative_vector_comparison.mat'), ...
    'comparison');

fprintf('\n============================================\n');
fprintf(' PHASE 7.10G COMPLETED\n');
fprintf('============================================\n');
