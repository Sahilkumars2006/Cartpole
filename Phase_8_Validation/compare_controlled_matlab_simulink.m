
%% ============================================================
% PHASE 8: CONTROLLED MATLAB-SIMULINK VALIDATION
% =============================================================

clc;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 8: CONTROLLED MODEL COMPARISON\n');
fprintf('============================================\n');

%% 1. LOAD RESULTS

projectRoot = ...
    '/home/sahil/Documents/Multibody_CartPole/XY_Spherical_Pendulum';

matlabFile = fullfile(projectRoot, ...
    'Results/MATLAB/controlled_simulation_results.mat');

simulinkFile = fullfile(projectRoot, ...
    'Results/Simulink/simulink_simulation_results.mat');

M = load(matlabFile);
S = load(simulinkFile);

RM = M.results;
RS = S.simulink_results;

%% 2. TIME ALIGNMENT

tM = RM.time(:);
tS = RS.time(:);

if abs(tM(1)-tS(1)) > 1e-10 || ...
   abs(tM(end)-tS(end)) > 1e-8

    error('Simulation time intervals do not match.');
end

simulinkStates = interp1(tS,RS.states,tM,'pchip');

simulinkForces = interp1(tS,RS.control_force,tM,'pchip');

matlabStates = RM.states;
matlabForces = RM.control_force;

%% 3. STATE ERROR

stateError = matlabStates - simulinkStates;

stateNames = {
    'Cart X position'
    'Cart Y position'
    'Bob X position'
    'Bob Y position'
    'Bob Z position'
    'Cart X velocity'
    'Cart Y velocity'
    'Bob X velocity'
    'Bob Y velocity'
    'Bob Z velocity'
};

stateRMS = sqrt(mean(stateError.^2,1));
stateMax = max(abs(stateError),[],1);

fprintf('\n============================================\n');
fprintf('STATE-BY-STATE COMPARISON\n');
fprintf('============================================\n');

for k = 1:10

    fprintf('\n%s\n',stateNames{k});
    fprintf('RMS error:     %.10e\n',stateRMS(k));
    fprintf('Maximum error: %.10e\n',stateMax(k));

end

%% 4. CONTROL FORCE COMPARISON

forceError = matlabForces - simulinkForces;

forceNames = {'Fx','Fy'};

fprintf('\n============================================\n');
fprintf('CONTROL FORCE COMPARISON\n');
fprintf('============================================\n');

for k = 1:2

    fprintf('\n%s\n',forceNames{k});
    fprintf('MATLAB maximum:   %.10f N\n', ...
        max(abs(matlabForces(:,k))));

    fprintf('Simulink maximum: %.10f N\n', ...
        max(abs(simulinkForces(:,k))));

    fprintf('RMS error:        %.10e N\n', ...
        rms(forceError(:,k)));

    fprintf('Maximum error:    %.10e N\n', ...
        max(abs(forceError(:,k))));

end

%% 5. GLOBAL VALIDATION METRICS

globalStateRMS = sqrt(mean(stateError(:).^2));
globalStateMax = max(abs(stateError(:)));

globalForceRMS = sqrt(mean(forceError(:).^2));
globalForceMax = max(abs(forceError(:)));

fprintf('\n============================================\n');
fprintf('GLOBAL VALIDATION METRICS\n');
fprintf('============================================\n');

fprintf('Global state RMS error: %.10e\n',globalStateRMS);
fprintf('Global state maximum error: %.10e\n',globalStateMax);

fprintf('Global force RMS error: %.10e N\n',globalForceRMS);
fprintf('Global force maximum error: %.10e N\n',globalForceMax);

%% 6. FINAL PERFORMANCE

fprintf('\n============================================\n');
fprintf('CONTROL PERFORMANCE\n');
fprintf('============================================\n');

fprintf('MATLAB final inclination: %.10e deg\n', ...
    rad2deg(RM.theta(end)));

fprintf('Simulink final inclination: %.10e deg\n', ...
    RS.final_inclination);

fprintf('MATLAB maximum rod error: %.10e m\n', ...
    RM.max_rod_error);

fprintf('Simulink maximum rod error: %.10e m\n', ...
    RS.maximum_rod_error);

fprintf('MATLAB maximum energy error: %.10e J\n', ...
    RM.max_energy_error);

%% 7. PLOT STATE COMPARISON

figure('Name','Controlled State Comparison');

for k = 1:10

    subplot(5,2,k);

    plot(tM,matlabStates(:,k),'LineWidth',1.3);
    hold on;

    plot(tM,simulinkStates(:,k),'--','LineWidth',1.3);

    grid on;
    title(stateNames{k});
    xlabel('Time (s)');

    if k == 1
        legend('MATLAB','Simulink');
    end
end

%% 8. PLOT STATE ERRORS

figure('Name','Controlled State Errors');

for k = 1:10

    subplot(5,2,k);

    plot(tM,stateError(:,k),'LineWidth',1.2);

    grid on;
    title([stateNames{k},' error']);
    xlabel('Time (s)');
end

%% 9. PLOT CONTROL FORCE COMPARISON

figure('Name','LQR Control Force Comparison');

subplot(2,1,1);

plot(tM,matlabForces(:,1),'LineWidth',1.4);
hold on;
plot(tM,simulinkForces(:,1),'--','LineWidth',1.4);

grid on;
ylabel('F_x (N)');
legend('MATLAB','Simulink');

subplot(2,1,2);

plot(tM,matlabForces(:,2),'LineWidth',1.4);
hold on;
plot(tM,simulinkForces(:,2),'--','LineWidth',1.4);

grid on;
xlabel('Time (s)');
ylabel('F_y (N)');
legend('MATLAB','Simulink');

%% 10. SAVE VALIDATION DATA

validation.time = tM;

validation.matlab_states = matlabStates;
validation.simulink_states = simulinkStates;
validation.state_error = stateError;

validation.state_rms_error = stateRMS;
validation.state_max_error = stateMax;

validation.matlab_control_force = matlabForces;
validation.simulink_control_force = simulinkForces;
validation.control_force_error = forceError;

validation.global_state_rms = globalStateRMS;
validation.global_state_max = globalStateMax;

validation.global_force_rms = globalForceRMS;
validation.global_force_max = globalForceMax;

validation.matlab_final_inclination = rad2deg(RM.theta(end));
validation.simulink_final_inclination = RS.final_inclination;

outputFolder = fullfile(projectRoot,'Results','Comparisons');

if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end

save(fullfile(outputFolder, ...
    'phase_8_controlled_validation.mat'),'validation');

fprintf('\n============================================\n');
fprintf('PHASE 8 CONTROLLED COMPARISON COMPLETED\n');
fprintf('============================================\n');

fprintf('Results saved successfully.\n');
fprintf('Location: %s\n',outputFolder);
