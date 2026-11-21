
%% ============================================================
% PHASE 7.10L: ENERGY CONSERVATION COMPARISON
% MATLAB Cartesian vs Tight-Solver Simscape
% ============================================================

clc;
close all;

fprintf('\n============================================\n');
fprintf(' PHASE 7.10L: ENERGY COMPARISON\n');
fprintf('============================================\n');

%% 1. LOAD RESULTS

projectRoot = fileparts(fileparts(mfilename('fullpath')));

A = load(fullfile(projectRoot, ...
    'Results','MATLAB','cartesian_simulation_results.mat'));

B = load(fullfile(projectRoot, ...
    'Results','Simscape', ...
    'uncontrolled_simscape_tight_solver_results.mat'));

matlab = A.results;
simscape = B.results;

%% 2. PARAMETERS

M = matlab.parameters.M;
m = matlab.parameters.m;
L = matlab.parameters.L;
g = matlab.parameters.g;

mr = 0.001;

fprintf('\nParameters:\n');
fprintf('Cart mass: %.4f kg\n',M);
fprintf('Bob mass: %.4f kg\n',m);
fprintf('Rod mass: %.4f kg\n',mr);
fprintf('Rod length: %.4f m\n',L);

%% 3. COMMON TIME VECTOR

t0 = max(matlab.time(1),simscape.time(1));
tf = min(matlab.time(end),simscape.time(end));

t = linspace(t0,tf,1501).';

%% 4. MATLAB ENERGY

Tmat = interp1(matlab.time, ...
    matlab.kinetic_energy,t,'linear');

Vmat = interp1(matlab.time, ...
    matlab.potential_energy,t,'linear');

Emat = interp1(matlab.time, ...
    matlab.total_energy,t,'linear');

%% 5. INTERPOLATE SIMSCAPE MEASUREMENTS

cartPosition = interp1(simscape.time, ...
    simscape.cart_position,t,'linear');

cartVelocity = interp1(simscape.time, ...
    simscape.cart_velocity,t,'linear');

bobPosition = interp1(simscape.time, ...
    simscape.bob_position,t,'linear');

% Bob velocity from numerical differentiation.
bobVelocity = zeros(size(bobPosition));

for k = 1:3
    bobVelocity(:,k) = gradient(bobPosition(:,k),t);
end

%% 6. SIMSCAPE KINETIC ENERGY

% Cart translational kinetic energy.
Tcart = 0.5*M*sum(cartVelocity.^2,2);

% Bob translational kinetic energy.
Tbob = 0.5*m*sum(bobVelocity.^2,2);

% Rod centre position: halfway between pivot and bob.
rodCentreVelocity = 0.5*(bobVelocity + ...
    [cartVelocity,zeros(size(t))]);

Trod = 0.5*mr*sum(rodCentreVelocity.^2,2);

Tsim = Tcart + Tbob + Trod;

%% 7. SIMSCAPE POTENTIAL ENERGY

% Bob gravitational potential energy.
Vbob = m*g*bobPosition(:,3);

% Rod centre gravitational potential energy.
Vrod = mr*g*(0.5*bobPosition(:,3));

Vsim = Vbob + Vrod;

Esim = Tsim + Vsim;

%% 8. ENERGY CONSERVATION ERRORS

% Energy change relative to initial energy.
driftMAT = Emat-Emat(1);
driftSIM = Esim-Esim(1);

rmsMAT = sqrt(mean(driftMAT.^2));
rmsSIM = sqrt(mean(driftSIM.^2));

maxMAT = max(abs(driftMAT));
maxSIM = max(abs(driftSIM));

fprintf('\n============================================\n');
fprintf(' ENERGY CONSERVATION RESULTS\n');
fprintf('============================================\n');

fprintf('\nMATLAB:\n');
fprintf('Initial energy: %.10f J\n',Emat(1));
fprintf('Final energy: %.10f J\n',Emat(end));
fprintf('RMS energy drift: %.10e J\n',rmsMAT);
fprintf('Maximum energy drift: %.10e J\n',maxMAT);

fprintf('\nTight Simscape (translational energy):\n');
fprintf('Initial energy: %.10f J\n',Esim(1));
fprintf('Final energy: %.10f J\n',Esim(end));
fprintf('RMS energy drift: %.10e J\n',rmsSIM);
fprintf('Maximum energy drift: %.10e J\n',maxSIM);

%% 9. ENERGY COMPARISON

energyDifference = Esim-Emat;

rmsEnergyDifference = sqrt(mean(energyDifference.^2));

fprintf('\n============================================\n');
fprintf(' ENERGY MODEL DIFFERENCE\n');
fprintf('============================================\n');

fprintf('RMS energy difference: %.10e J\n', ...
    rmsEnergyDifference);

fprintf('Maximum energy difference: %.10e J\n', ...
    max(abs(energyDifference)));

%% 10. PLOT ENERGY

figure('Name','Mechanical Energy Comparison', ...
    'Color','w','Position',[100 100 1200 750]);

subplot(2,1,1);

plot(t,Emat,'k','LineWidth',1.5);
hold on;
plot(t,Esim,'r--','LineWidth',1.4);

grid on;
xlabel('Time (s)');
ylabel('Mechanical energy (J)');
title('Total Mechanical Energy');

legend('MATLAB','Tight Simscape', ...
    'Location','best');

subplot(2,1,2);

plot(t,driftMAT,'k','LineWidth',1.5);
hold on;
plot(t,driftSIM,'r--','LineWidth',1.4);

grid on;
xlabel('Time (s)');
ylabel('Energy drift (J)');
title('Energy Conservation');

legend('MATLAB','Tight Simscape', ...
    'Location','best');

%% 11. SAVE RESULTS

energyAnalysis.time = t;

energyAnalysis.matlab_kinetic = Tmat;
energyAnalysis.matlab_potential = Vmat;
energyAnalysis.matlab_total = Emat;

energyAnalysis.simscape_kinetic = Tsim;
energyAnalysis.simscape_potential = Vsim;
energyAnalysis.simscape_total = Esim;

energyAnalysis.matlab_energy_drift = driftMAT;
energyAnalysis.simscape_energy_drift = driftSIM;

energyAnalysis.rms_matlab_drift = rmsMAT;
energyAnalysis.rms_simscape_drift = rmsSIM;

energyAnalysis.rms_energy_difference = ...
    rmsEnergyDifference;

outputFolder = fullfile(projectRoot, ...
    'Results','Comparisons');

if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end

save(fullfile(outputFolder, ...
    'energy_conservation_analysis.mat'), ...
    'energyAnalysis');

fprintf('\nPHASE 7.10L COMPLETED\n');
fprintf('Energy results saved successfully.\n');
fprintf('\nNOTE: Simscape rotational kinetic energy is not included.\n');
