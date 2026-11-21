
%% ============================================
% PHASE 7.10E: TIME-ALIGNED DYNAMICS COMPARISON
% =============================================

clc;
close all;

fprintf('\n============================================\n');
fprintf(' PHASE 7.10E: DYNAMICS COMPARISON\n');
fprintf('============================================\n');

%% 1. Load simulation results

M = load(fullfile('..','Results','MATLAB', ...
    'cartesian_simulation_results.mat'));

S = load(fullfile('..','Results','Simscape', ...
    'uncontrolled_simscape_results.mat'));

mat = M.results;
sim = S.results;

%% 2. Extract signals

tM = mat.time(:);
tS = sim.time(:);

thetaM = rad2deg(mat.theta(:));
phiM = rad2deg(mat.phi(:));

thetaS = sim.inclination_deg(:);
phiS = sim.azimuth_deg(:);

cartM = mat.cart_position;
cartS = sim.cart_position;

bobM = mat.bob_position;
bobS = sim.bob_position;

%% 3. Establish common time vector

tStart = max(tM(1),tS(1));
tEnd = min(tM(end),tS(end));

tCommon = linspace(tStart,tEnd,1001)';

fprintf('\nCommon simulation duration: %.3f s\n', ...
    tEnd-tStart);

%% 4. Interpolate all signals

thetaMi = interp1(tM,thetaM,tCommon,'linear');
thetaSi = interp1(tS,thetaS,tCommon,'linear');

phiMi = interp1(tM,phiM,tCommon,'linear');
phiSi = interp1(tS,phiS,tCommon,'linear');

cartMi = interp1(tM,cartM,tCommon,'linear');
cartSi = interp1(tS,cartS,tCommon,'linear');

bobMi = interp1(tM,bobM,tCommon,'linear');
bobSi = interp1(tS,bobS,tCommon,'linear');

%% 5. Calculate errors

inclinationError = thetaMi-thetaSi;

% Wrap azimuth difference into [-180,180] degrees.
azimuthError = mod(phiMi-phiSi+180,360)-180;

cartError = cartMi-cartSi;
bobError = bobMi-bobSi;

fprintf('\n--- FULL-TRAJECTORY RMS ERRORS ---\n');

fprintf('Inclination RMS error: %.6f deg\n', ...
    sqrt(mean(inclinationError.^2)));

fprintf('Azimuth RMS error: %.6f deg\n', ...
    sqrt(mean(azimuthError.^2)));

fprintf('Cart X RMS error: %.8f m\n', ...
    sqrt(mean(cartError(:,1).^2)));

fprintf('Cart Y RMS error: %.8f m\n', ...
    sqrt(mean(cartError(:,2).^2)));

fprintf('Bob position RMS error: %.8f m\n', ...
    sqrt(mean(sum(bobError.^2,2))));

%% 6. Compare at selected time instants

checkTimes = [0 1 2 3 5 7.5 10 12.5 15]';

checkTimes = checkTimes(checkTimes >= tStart & ...
                        checkTimes <= tEnd);

thetaMcheck = interp1(tM,thetaM,checkTimes);
thetaScheck = interp1(tS,thetaS,checkTimes);

phiMcheck = interp1(tM,phiM,checkTimes);
phiScheck = interp1(tS,phiS,checkTimes);

cartMcheck = interp1(tM,cartM,checkTimes);
cartScheck = interp1(tS,cartS,checkTimes);

comparisonTable = table( ...
    checkTimes, ...
    thetaMcheck, ...
    thetaScheck, ...
    phiMcheck, ...
    phiScheck, ...
    cartMcheck(:,1), ...
    cartScheck(:,1), ...
    cartMcheck(:,2), ...
    cartScheck(:,2), ...
    'VariableNames', { ...
    'Time_s', ...
    'MATLAB_Theta_deg', ...
    'Simscape_Theta_deg', ...
    'MATLAB_Phi_deg', ...
    'Simscape_Phi_deg', ...
    'MATLAB_X_m', ...
    'Simscape_X_m', ...
    'MATLAB_Y_m', ...
    'Simscape_Y_m'});

fprintf('\n--- SELECTED TIME COMPARISON ---\n');
disp(comparisonTable);

%% 7. Plot inclination

figure('Name','Inclination Comparison', ...
    'Color','w');

plot(tM,thetaM,'b-','LineWidth',1.5);
hold on;
plot(tS,thetaS,'r--','LineWidth',1.5);

grid on;
xlabel('Time (s)');
ylabel('Inclination (degrees)');
title('MATLAB vs Simscape: Pendulum Inclination');
legend('MATLAB','Simscape','Location','best');

%% 8. Plot cart positions

figure('Name','Cart Position Comparison', ...
    'Color','w');

subplot(2,1,1);

plot(tM,cartM(:,1),'b-','LineWidth',1.5);
hold on;
plot(tS,cartS(:,1),'r--','LineWidth',1.5);

grid on;
ylabel('X Position (m)');
title('Cart X Displacement');
legend('MATLAB','Simscape','Location','best');

subplot(2,1,2);

plot(tM,cartM(:,2),'b-','LineWidth',1.5);
hold on;
plot(tS,cartS(:,2),'r--','LineWidth',1.5);

grid on;
xlabel('Time (s)');
ylabel('Y Position (m)');
title('Cart Y Displacement');
legend('MATLAB','Simscape','Location','best');

%% 9. Plot bob position comparison

figure('Name','Bob Position Comparison', ...
    'Color','w');

axisNames = {'X','Y','Z'};

for k = 1:3

    subplot(3,1,k);

    plot(tM,bobM(:,k),'b-','LineWidth',1.5);
    hold on;
    plot(tS,bobS(:,k),'r--','LineWidth',1.5);

    grid on;
    ylabel([axisNames{k} ' (m)']);

    if k == 1
        title('Bob World Position');
        legend('MATLAB','Simscape','Location','best');
    end

    if k == 3
        xlabel('Time (s)');
    end
end

%% 10. Save comparison results

comparison = struct();

comparison.time = tCommon;
comparison.inclination_error = inclinationError;
comparison.azimuth_error = azimuthError;
comparison.cart_error = cartError;
comparison.bob_error = bobError;
comparison.checkpoint_table = comparisonTable;

comparison.inclination_RMS_deg = ...
    sqrt(mean(inclinationError.^2));

comparison.azimuth_RMS_deg = ...
    sqrt(mean(azimuthError.^2));

comparison.cart_X_RMS_m = ...
    sqrt(mean(cartError(:,1).^2));

comparison.cart_Y_RMS_m = ...
    sqrt(mean(cartError(:,2).^2));

comparison.bob_position_RMS_m = ...
    sqrt(mean(sum(bobError.^2,2)));

save(fullfile('..','Results','Comparisons', ...
    'time_aligned_dynamics_comparison.mat'), ...
    'comparison');

fprintf('\n============================================\n');
fprintf(' PHASE 7.10E COMPLETED\n');
fprintf('============================================\n');
