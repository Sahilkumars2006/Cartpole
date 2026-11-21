
%% ============================================================
% PHASE 7.10J: CART AND PENDULUM ERROR DECOMPOSITION
% ============================================================

clc;
close all;

projectRoot = fileparts(fileparts(mfilename('fullpath')));

load(fullfile(projectRoot,'Results','Simscape', ...
    'uncontrolled_simscape_baseline.mat'),'results');
baseline = results;

load(fullfile(projectRoot,'Results','Simscape', ...
    'uncontrolled_simscape_tight_solver_results.mat'),'results');
tight = results;

load(fullfile(projectRoot,'Results','MATLAB', ...
    'cartesian_simulation_results.mat'),'results');
matlab = results;

%% Common time vector

t0 = max([baseline.time(1),tight.time(1),matlab.time(1)]);
tf = min([baseline.time(end),tight.time(end),matlab.time(end)]);

t = linspace(t0,tf,1501).';

%% Interpolate cart positions

cartBase = interp1(baseline.time,baseline.cart_position,t);
cartTight = interp1(tight.time,tight.cart_position,t);
cartMAT = interp1(matlab.time,matlab.cart_position,t);

%% Interpolate relative pendulum positions

relBase = interp1(baseline.time,baseline.relative_position,t);
relTight = interp1(tight.time,tight.relative_position,t);

cartMAT3 = [cartMAT,zeros(size(cartMAT,1),1)];
bobMAT = interp1(matlab.time,matlab.bob_position,t);

relMAT = bobMAT-cartMAT3;

%% Error calculations

cartErrorBase = cartMAT-cartBase;
cartErrorTight = cartMAT-cartTight;

relErrorBase = relMAT-relBase;
relErrorTight = relMAT-relTight;

cartRMSBase = sqrt(mean(sum(cartErrorBase.^2,2)));
cartRMSTight = sqrt(mean(sum(cartErrorTight.^2,2)));

relRMSBase = sqrt(mean(sum(relErrorBase.^2,2)));
relRMSTight = sqrt(mean(sum(relErrorTight.^2,2)));

fprintf('\n============================================\n');
fprintf(' CART AND PENDULUM ERROR DECOMPOSITION\n');
fprintf('============================================\n');

fprintf('\nMATLAB vs Baseline Simscape:\n');
fprintf('Cart position RMS error: %.8f m\n',cartRMSBase);
fprintf('Pendulum relative position RMS error: %.8f m\n',relRMSBase);

fprintf('\nMATLAB vs Tight Simscape:\n');
fprintf('Cart position RMS error: %.8f m\n',cartRMSTight);
fprintf('Pendulum relative position RMS error: %.8f m\n',relRMSTight);

%% Component-wise cart errors

fprintf('\nCart component RMS errors:\n');

fprintf('Baseline X: %.8f m\n', ...
    sqrt(mean(cartErrorBase(:,1).^2)));

fprintf('Baseline Y: %.8f m\n', ...
    sqrt(mean(cartErrorBase(:,2).^2)));

fprintf('Tight X: %.8f m\n', ...
    sqrt(mean(cartErrorTight(:,1).^2)));

fprintf('Tight Y: %.8f m\n', ...
    sqrt(mean(cartErrorTight(:,2).^2)));

%% Plot cart position

figure('Color','w','Name','Cart Position Comparison');

subplot(2,1,1);
plot(t,cartMAT(:,1),'k','LineWidth',1.5);
hold on;
plot(t,cartBase(:,1),'b--','LineWidth',1.2);
plot(t,cartTight(:,1),'r-.','LineWidth',1.2);
grid on;
ylabel('X position (m)');
title('Cart X Position');
legend('MATLAB','Baseline Simscape','Tight Simscape');

subplot(2,1,2);
plot(t,cartMAT(:,2),'k','LineWidth',1.5);
hold on;
plot(t,cartBase(:,2),'b--','LineWidth',1.2);
plot(t,cartTight(:,2),'r-.','LineWidth',1.2);
grid on;
xlabel('Time (s)');
ylabel('Y position (m)');
title('Cart Y Position');

%% Plot pendulum relative position error

figure('Color','w','Name','Pendulum Position Error');

plot(t,vecnorm(relErrorBase,2,2),'b--','LineWidth',1.3);
hold on;
plot(t,vecnorm(relErrorTight,2,2),'r-.','LineWidth',1.3);
grid on;

xlabel('Time (s)');
ylabel('Relative position error (m)');
title('Pendulum Position Error');
legend('Baseline Simscape','Tight Simscape');

fprintf('\nPHASE 7.10J COMPLETED\n');
