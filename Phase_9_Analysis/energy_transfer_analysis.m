
%% PHASE 9.8: MECHANICAL ENERGY DISTRIBUTION AND TRANSFER
%
% Project:
% 3D X-Y Cart with an Inverted Spherical Pendulum
%
% Energy components:
% 1. Cart translational kinetic energy
% 2. Bob translational kinetic energy
% 3. Gravitational potential energy
% 4. Total mechanical energy
%
% The rod is treated as massless, consistent with
% the Cartesian equations of motion.

clc;
close all;

fprintf('\n');
fprintf('============================================\n');
fprintf('PHASE 9.8: ENERGY TRANSFER ANALYSIS\n');
fprintf('============================================\n');

%% 1. PROJECT PATHS

scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);

phase4Dir = fullfile(projectDir, ...
    'Phase_4_MATLAB_Simulation');

resultsDir = fullfile(projectDir, ...
    'Results', 'Phase_9');

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

addpath(phase4Dir);

%% 2. LOAD BASELINE PARAMETERS

p = system_parameters();

fprintf('\n--- BASELINE PARAMETERS ---\n');
fprintf('Cart mass: %.4f kg\n', p.M);
fprintf('Bob mass: %.4f kg\n', p.m);
fprintf('Rod length: %.4f m\n', p.L);
fprintf('Gravity: %.4f m/s^2\n', p.g);
fprintf('Initial inclination: %.4f deg\n', ...
    rad2deg(p.theta0));
fprintf('Initial azimuth: %.4f deg\n', ...
    rad2deg(p.phi0));

%% 3. SIMULATION SETTINGS

time = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol', p.relative_tolerance, ...
    'AbsTol', p.absolute_tolerance, ...
    'MaxStep', p.sample_time);

%% 4. CONSTRUCT INITIAL CARTESIAN STATE

theta = p.theta0;
phi = p.phi0;

x0 = p.x0;
y0 = p.y0;

% Initial bob position
X0 = x0 + p.L*sin(theta)*cos(phi);
Y0 = y0 + p.L*sin(theta)*sin(phi);
Z0 = p.L*cos(theta);

% Position derivatives with respect to angular coordinates
rTheta = p.L * [
    cos(theta)*cos(phi);
    cos(theta)*sin(phi);
    -sin(theta)
];

rPhi = p.L * [
    -sin(theta)*sin(phi);
     sin(theta)*cos(phi);
     0
];

% Initial bob absolute velocity
bobVelocity = [
    p.dx0;
    p.dy0;
    0
] + p.dtheta0*rTheta + p.dphi0*rPhi;

% Cartesian state:
% [x; y; X; Y; Z; dx; dy; dX; dY; dZ]

z0 = [
    x0;
    y0;
    X0;
    Y0;
    Z0;
    p.dx0;
    p.dy0;
    bobVelocity
];

%% 5. RUN SIMULATION

fprintf('\nRunning uncontrolled Cartesian simulation...\n');

[time, states] = ode45( ...
    @(t,z) cartesian_equations_of_motion(t,z,p), ...
    time, z0, options);

fprintf('Simulation completed.\n');
fprintf('Number of time samples: %d\n', length(time));

%% 6. CALCULATE ENERGY COMPONENTS

cartVelocitySquared = ...
    states(:,6).^2 + states(:,7).^2;

bobVelocitySquared = ...
    states(:,8).^2 + ...
    states(:,9).^2 + ...
    states(:,10).^2;

% Cart translational kinetic energy
cartKE = 0.5*p.M*cartVelocitySquared;

% Bob translational kinetic energy
bobKE = 0.5*p.m*bobVelocitySquared;

% Gravitational potential energy
% Zero potential is defined at Z = 0.
potentialEnergy = p.m*p.g*states(:,5);

% Total mechanical energy
totalEnergy = cartKE + bobKE + potentialEnergy;

% Energy changes relative to initial values
deltaCartKE = cartKE - cartKE(1);
deltaBobKE = bobKE - bobKE(1);
deltaPotential = potentialEnergy - potentialEnergy(1);
deltaTotalEnergy = totalEnergy - totalEnergy(1);

%% 7. ENERGY CONSERVATION METRICS

initialEnergy = totalEnergy(1);
finalEnergy = totalEnergy(end);

energyDrift = totalEnergy - initialEnergy;

maximumAbsoluteDrift = max(abs(energyDrift));

rmsEnergyDrift = sqrt(mean(energyDrift.^2));

relativeMaximumDrift = ...
    maximumAbsoluteDrift/max(abs(initialEnergy), eps)*100;

%% 8. ENERGY COMPONENT STATISTICS

maxCartKE = max(cartKE);
minCartKE = min(cartKE);

maxBobKE = max(bobKE);
minBobKE = min(bobKE);

maxPotential = max(potentialEnergy);
minPotential = min(potentialEnergy);

maxCartKEChange = max(abs(deltaCartKE));
maxBobKEChange = max(abs(deltaBobKE));
maxPotentialChange = max(abs(deltaPotential));

%% 9. PRINT ENERGY RESULTS

fprintf('\n');
fprintf('============================================\n');
fprintf('INITIAL ENERGY DISTRIBUTION\n');
fprintf('============================================\n');

fprintf('Cart kinetic energy: %.10f J\n', cartKE(1));
fprintf('Bob kinetic energy: %.10f J\n', bobKE(1));
fprintf('Potential energy: %.10f J\n', potentialEnergy(1));

fprintf('--------------------------------------------\n');
fprintf('Initial total energy: %.10f J\n', initialEnergy);

fprintf('\n');
fprintf('============================================\n');
fprintf('FINAL ENERGY DISTRIBUTION\n');
fprintf('============================================\n');

fprintf('Cart kinetic energy: %.10f J\n', cartKE(end));
fprintf('Bob kinetic energy: %.10f J\n', bobKE(end));
fprintf('Potential energy: %.10f J\n', potentialEnergy(end));

fprintf('--------------------------------------------\n');
fprintf('Final total energy: %.10f J\n', finalEnergy);

fprintf('\n');
fprintf('============================================\n');
fprintf('ENERGY CONSERVATION RESULTS\n');
fprintf('============================================\n');

fprintf('Initial energy: %.12f J\n', initialEnergy);
fprintf('Final energy:   %.12f J\n', finalEnergy);

fprintf('Maximum absolute energy drift: %.6e J\n', ...
    maximumAbsoluteDrift);

fprintf('RMS energy drift: %.6e J\n', rmsEnergyDrift);

fprintf('Relative maximum energy drift: %.8f %%\n', ...
    relativeMaximumDrift);

%% 10. ENERGY TRANSFER SUMMARY

fprintf('\n');
fprintf('============================================\n');
fprintf('ENERGY TRANSFER SUMMARY\n');
fprintf('============================================\n');

fprintf('Maximum cart KE change: %.10f J\n', ...
    maxCartKEChange);

fprintf('Maximum bob KE change: %.10f J\n', ...
    maxBobKEChange);

fprintf('Maximum potential energy change: %.10f J\n', ...
    maxPotentialChange);

fprintf('\nEnergy component ranges:\n');

fprintf('Cart KE: %.10f to %.10f J\n', ...
    minCartKE, maxCartKE);

fprintf('Bob KE: %.10f to %.10f J\n', ...
    minBobKE, maxBobKE);

fprintf('Potential energy: %.10f to %.10f J\n', ...
    minPotential, maxPotential);

%% 11. ENERGY CONSERVATION VALIDATION

fprintf('\n');
fprintf('============================================\n');
fprintf('ENERGY VALIDATION\n');
fprintf('============================================\n');

if maximumAbsoluteDrift < 1e-5
    fprintf('Energy conservation: PASSED\n');
else
    fprintf('Energy conservation: REVIEW REQUIRED\n');
end

if maximumAbsoluteDrift < 1e-7
    fprintf('Energy accuracy: EXCELLENT\n');
elseif maximumAbsoluteDrift < 1e-5
    fprintf('Energy accuracy: ACCEPTABLE\n');
else
    fprintf('Energy accuracy: NEEDS INVESTIGATION\n');
end

%% 12. FIGURE 1: ABSOLUTE ENERGY COMPONENTS

figure('Name', 'Phase 9.8 - Energy Components', ...
    'Color', 'w');

plot(time, cartKE, 'LineWidth', 1.5, ...
    'DisplayName', 'Cart kinetic energy');

hold on;

plot(time, bobKE, 'LineWidth', 1.5, ...
    'DisplayName', 'Bob kinetic energy');

plot(time, potentialEnergy, 'LineWidth', 1.5, ...
    'DisplayName', 'Gravitational potential energy');

plot(time, totalEnergy, 'k--', ...
    'LineWidth', 1.8, ...
    'DisplayName', 'Total mechanical energy');

grid on;

xlabel('Time (s)');
ylabel('Energy (J)');

title('Mechanical Energy Components');

legend('Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_Energy_Components.png'));

%% 13. FIGURE 2: ENERGY CHANGES

figure('Name', 'Phase 9.8 - Energy Changes', ...
    'Color', 'w');

plot(time, deltaCartKE, ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Change in cart KE');

hold on;

plot(time, deltaBobKE, ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Change in bob KE');

plot(time, deltaPotential, ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Change in potential energy');

plot(time, deltaTotalEnergy, 'k--', ...
    'LineWidth', 1.8, ...
    'DisplayName', 'Change in total energy');

grid on;

xlabel('Time (s)');
ylabel('Energy change (J)');

title('Energy Transfer Relative to Initial State');

legend('Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_Energy_Transfer.png'));

%% 14. FIGURE 3: TOTAL ENERGY DRIFT

figure('Name', 'Phase 9.8 - Energy Drift', ...
    'Color', 'w');

plot(time, energyDrift, ...
    'LineWidth', 1.5);

grid on;

xlabel('Time (s)');
ylabel('Total energy drift (J)');

title('Total Mechanical Energy Conservation');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_Energy_Drift.png'));

%% 15. FIGURE 4: ENERGY COMPONENT RANGES

figure('Name', 'Phase 9.8 - Energy Range', ...
    'Color', 'w');

energyRanges = [
    minCartKE maxCartKE;
    minBobKE maxBobKE;
    minPotential maxPotential
];

bar(energyRanges, 'grouped');

grid on;

xlabel('Energy Component');
ylabel('Energy (J)');

title('Minimum and Maximum Energy Values');

xticklabels({
    'Cart KE'
    'Bob KE'
    'Potential Energy'
});

legend('Minimum', 'Maximum', ...
    'Location', 'best');

saveas(gcf, fullfile(resultsDir, ...
    'Phase_9_Energy_Range.png'));

%% 16. SAVE RESULTS

phase9_8_results.time = time;
phase9_8_results.states = states;

phase9_8_results.cartKE = cartKE;
phase9_8_results.bobKE = bobKE;
phase9_8_results.potentialEnergy = potentialEnergy;
phase9_8_results.totalEnergy = totalEnergy;

phase9_8_results.deltaCartKE = deltaCartKE;
phase9_8_results.deltaBobKE = deltaBobKE;
phase9_8_results.deltaPotential = deltaPotential;
phase9_8_results.deltaTotalEnergy = deltaTotalEnergy;

phase9_8_results.initialEnergy = initialEnergy;
phase9_8_results.finalEnergy = finalEnergy;
phase9_8_results.maximumAbsoluteDrift = maximumAbsoluteDrift;
phase9_8_results.rmsEnergyDrift = rmsEnergyDrift;
phase9_8_results.relativeMaximumDrift = relativeMaximumDrift;

phase9_8_results.parameters = p;

save(fullfile(resultsDir, ...
    'phase_9_8_energy_transfer_results.mat'), ...
    'phase9_8_results', '-v7.3');

%% 17. COMPLETION SUMMARY

fprintf('\n');
fprintf('============================================\n');
fprintf('PHASE 9.8 COMPLETION SUMMARY\n');
fprintf('============================================\n');

fprintf('Simulation duration: %.2f seconds\n', ...
    p.t_end-p.t_start);

fprintf('Number of samples: %d\n', length(time));

fprintf('Initial total energy: %.10f J\n', initialEnergy);

fprintf('Final total energy: %.10f J\n', finalEnergy);

fprintf('Maximum energy drift: %.6e J\n', ...
    maximumAbsoluteDrift);

fprintf('\nResults directory:\n%s\n', resultsDir);

fprintf('\nSaved file:\n');
fprintf('phase_9_8_energy_transfer_results.mat\n');

fprintf('\nGenerated figures:\n');
fprintf('1. Phase_9_Energy_Components.png\n');
fprintf('2. Phase_9_Energy_Transfer.png\n');
fprintf('3. Phase_9_Energy_Drift.png\n');
fprintf('4. Phase_9_Energy_Range.png\n');

fprintf('\n============================================\n');
fprintf('PHASE 9.8 ANALYSIS COMPLETED\n');
fprintf('============================================\n');
