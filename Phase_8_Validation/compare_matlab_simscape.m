
%% ============================================================
% PHASE 7.10B: MATLAB VS SIMSCAPE DYNAMIC VALIDATION
% Inverted Spherical Pendulum on an X-Y Cart
% =============================================================

clc;
close all;

fprintf('\n============================================\n');
fprintf(' PHASE 7.10B: DYNAMIC MODEL COMPARISON\n');
fprintf('============================================\n');

%% 1. PROJECT PATHS

projectRoot = ...
    '/home/sahil/Documents/Multibody_CartPole/XY_Spherical_Pendulum';

matlabFile = fullfile(projectRoot, ...
    'Results','MATLAB','cartesian_simulation_results.mat');

simscapeFile = fullfile(projectRoot, ...
    'Results','Simscape','uncontrolled_simscape_results.mat');

outputFolder = fullfile(projectRoot, ...
    'Results','Comparisons');

if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end

%% 2. LOAD DATA

if ~isfile(matlabFile)
    error('MATLAB simulation results not found.');
end

if ~isfile(simscapeFile)
    error('Simscape simulation results not found.');
end

matlabData = load(matlabFile);
simscapeData = load(simscapeFile);

matlab = matlabData.results;
simscape = simscapeData.results;

%% 3. CHECK DATA DIMENSIONS

fprintf('\n--- DATA DIMENSIONS ---\n');

fprintf('MATLAB time samples: %d\n',numel(matlab.time));
fprintf('Simscape time samples: %d\n',numel(simscape.time));

fprintf('MATLAB bob position: %s\n', ...
    mat2str(size(matlab.bob_position)));

fprintf('Simscape bob position: %s\n', ...
    mat2str(size(simscape.bob_position)));

fprintf('MATLAB cart position: %s\n', ...
    mat2str(size(matlab.cart_position)));

fprintf('Simscape cart position: %s\n', ...
    mat2str(size(simscape.cart_position)));

%% 4. PREPARE TIME VECTORS

tM = matlab.time(:);
tS = simscape.time(:);

% Restrict comparison to common time interval.
tStart = max(tM(1),tS(1));
tEnd = min(tM(end),tS(end));

if tEnd <= tStart
    error('The simulation time intervals do not overlap.');
end

% Use Simscape time samples within common interval.
valid = tS >= tStart & tS <= tEnd;
t = tS(valid);

%% 5. PREPARE BOB POSITIONS

bobM = matlab.bob_position;
bobS = simscape.bob_position;

if size(bobM,2) ~= 3 || size(bobS,2) ~= 3
    error('Bob position data must contain three columns: X, Y, Z.');
end

% Interpolate MATLAB positions onto Simscape time.
bobM_interp = interp1(tM,bobM,t,'linear');
bobS_compare = bobS(valid,:);

%% 6. PREPARE CART POSITIONS

cartM = matlab.cart_position;
cartS = simscape.cart_position;

if size(cartM,2) < 2 || size(cartS,2) < 2
    error('Cart position data must contain X and Y columns.');
end

cartM_interp = interp1(tM,cartM(:,1:2),t,'linear');
cartS_compare = cartS(valid,1:2);

%% 7. CALCULATE POSITION ERRORS

bobError = bobS_compare - bobM_interp;
cartError = cartS_compare - cartM_interp;

bobErrorNorm = sqrt(sum(bobError.^2,2));
cartErrorNorm = sqrt(sum(cartError.^2,2));

bobRMSError = sqrt(mean(bobErrorNorm.^2));
cartRMSError = sqrt(mean(cartErrorNorm.^2));

bobMaxError = max(bobErrorNorm);
cartMaxError = max(cartErrorNorm);

%% 8. CALCULATE INCLINATION

% MATLAB inclination from the bob's relative position.
relativeM = bobM_interp - ...
    [cartM_interp, zeros(numel(t),1)];

thetaM = atan2( ...
    sqrt(relativeM(:,1).^2 + relativeM(:,2).^2), ...
    relativeM(:,3));

% Simscape inclination already calculated from relative position.
thetaS = simscape.inclination(valid);

thetaM_deg = rad2deg(thetaM);
thetaS_deg = rad2deg(thetaS);

inclinationError = thetaS_deg - thetaM_deg;

inclinationRMSError = sqrt(mean(inclinationError.^2));
inclinationMaxError = max(abs(inclinationError));

%% 9. PRINT NUMERICAL COMPARISON

fprintf('\n============================================\n');
fprintf(' DYNAMIC COMPARISON RESULTS\n');
fprintf('============================================\n');

fprintf('\nCommon comparison duration: %.4f s\n', ...
    t(end)-t(1));

fprintf('\n--- FINAL INCLINATION ---\n');

fprintf('MATLAB:   %.6f degrees\n',thetaM_deg(end));
fprintf('Simscape: %.6f degrees\n',thetaS_deg(end));

fprintf('\n--- INCLINATION ERROR ---\n');

fprintf('RMS error: %.6f degrees\n',inclinationRMSError);
fprintf('Maximum error: %.6f degrees\n',inclinationMaxError);

fprintf('\n--- BOB POSITION ERROR ---\n');

fprintf('RMS error: %.8f m\n',bobRMSError);
fprintf('Maximum error: %.8f m\n',bobMaxError);

fprintf('\n--- CART POSITION ERROR ---\n');

fprintf('RMS error: %.8f m\n',cartRMSError);
fprintf('Maximum error: %.8f m\n',cartMaxError);

fprintf('\n--- FINAL CART POSITIONS ---\n');

fprintf('MATLAB X:   %.8f m\n',cartM_interp(end,1));
fprintf('Simscape X: %.8f m\n',cartS_compare(end,1));

fprintf('MATLAB Y:   %.8f m\n',cartM_interp(end,2));
fprintf('Simscape Y: %.8f m\n',cartS_compare(end,2));

%% 10. PLOT INCLINATION COMPARISON

figure('Name','Inclination Comparison', ...
    'Color','w','Position',[100 100 1000 600]);

plot(t,thetaM_deg,'LineWidth',1.8);
hold on;
plot(t,thetaS_deg,'--','LineWidth',1.8);

grid on;
xlabel('Time (s)');
ylabel('Inclination (degrees)');
title('Uncontrolled Pendulum Inclination');

legend('MATLAB Cartesian Model', ...
       'Simscape Multibody','Location','best');

%% 11. PLOT BOB POSITION COMPARISON

figure('Name','Bob Position Comparison', ...
    'Color','w','Position',[100 100 1100 750]);

labels = {'X position','Y position','Z position'};

for k = 1:3

    subplot(3,1,k);

    plot(t,bobM_interp(:,k),'LineWidth',1.7);
    hold on;

    plot(t,bobS_compare(:,k),'--','LineWidth',1.7);

    grid on;
    ylabel('Position (m)');
    title(labels{k});

    if k == 1
        legend('MATLAB','Simscape','Location','best');
    end

end

xlabel('Time (s)');

%% 12. PLOT CART POSITION COMPARISON

figure('Name','Cart Position Comparison', ...
    'Color','w','Position',[100 100 1100 650]);

subplot(2,1,1);

plot(t,cartM_interp(:,1),'LineWidth',1.7);
hold on;
plot(t,cartS_compare(:,1),'--','LineWidth',1.7);

grid on;
ylabel('X displacement (m)');
title('Cart X Position');
legend('MATLAB','Simscape','Location','best');

subplot(2,1,2);

plot(t,cartM_interp(:,2),'LineWidth',1.7);
hold on;
plot(t,cartS_compare(:,2),'--','LineWidth',1.7);

grid on;
xlabel('Time (s)');
ylabel('Y displacement (m)');
title('Cart Y Position');
legend('MATLAB','Simscape','Location','best');

%% 13. PLOT POSITION ERROR

figure('Name','Dynamic Position Errors', ...
    'Color','w','Position',[100 100 1000 600]);

subplot(2,1,1);

plot(t,bobErrorNorm,'LineWidth',1.7);

grid on;
ylabel('Error (m)');
title('Bob Position Error');

subplot(2,1,2);

plot(t,cartErrorNorm,'LineWidth',1.7);

grid on;
xlabel('Time (s)');
ylabel('Error (m)');
title('Cart Position Error');

%% 14. SAVE COMPARISON RESULTS

comparison.time = t;

comparison.matlab.bob_position = bobM_interp;
comparison.simscape.bob_position = bobS_compare;

comparison.matlab.cart_position = cartM_interp;
comparison.simscape.cart_position = cartS_compare;

comparison.matlab.inclination_deg = thetaM_deg;
comparison.simscape.inclination_deg = thetaS_deg;

comparison.bob_position_error = bobError;
comparison.bob_position_error_norm = bobErrorNorm;

comparison.cart_position_error = cartError;
comparison.cart_position_error_norm = cartErrorNorm;

comparison.inclination_error_deg = inclinationError;

comparison.metrics.bob_position_rms = bobRMSError;
comparison.metrics.bob_position_max = bobMaxError;

comparison.metrics.cart_position_rms = cartRMSError;
comparison.metrics.cart_position_max = cartMaxError;

comparison.metrics.inclination_rms_deg = inclinationRMSError;
comparison.metrics.inclination_max_deg = inclinationMaxError;

save(fullfile(outputFolder, ...
    'matlab_simscape_comparison.mat'),'comparison');

fprintf('\nComparison results saved:\n');
fprintf('Results/Comparisons/matlab_simscape_comparison.mat\n');

fprintf('\n============================================\n');
fprintf(' PHASE 7.10B COMPLETED\n');
fprintf('============================================\n');
