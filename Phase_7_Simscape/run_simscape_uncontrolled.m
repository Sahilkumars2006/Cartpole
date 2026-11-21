
%% ============================================================
% PHASE 7.9B: UNCONTROLLED SIMSCAPE DYNAMICS
% Inverted Spherical Pendulum on an X-Y Cart
% =============================================================

clc;
close all;

fprintf('\n============================================\n');
fprintf(' PHASE 7.9B: UNCONTROLLED SIMSCAPE SIMULATION\n');
fprintf('============================================\n');

%% 1. MODEL CONFIGURATION

model = 'sm_inverted_spherical_pendulum';

projectRoot = fileparts(fileparts(mfilename('fullpath')));

modelPath = fullfile(projectRoot, ...
    'Phase_7_Simscape', [model '.slx']);

resultsFolder = fullfile(projectRoot, ...
    'Results', 'Simscape');

if ~exist(resultsFolder,'dir')
    mkdir(resultsFolder);
end

if ~bdIsLoaded(model)
    load_system(modelPath);
end

M = 2.0;
m = 0.5;
L = 1.0;
g = 9.81;

simulationTime = 15;

%% 2. SET CART FORCES TO ZERO

forceXBlock = find_system(model, ...
    'SearchDepth',1,'Name','Force_X');

forceYBlock = find_system(model, ...
    'SearchDepth',1,'Name','Force_Y');

if isempty(forceXBlock) || isempty(forceYBlock)
    error(['Could not find Force_X or Force_Y. ' ...
        'Check the force source block names.']);
end

set_param(forceXBlock{1},'Value','0');
set_param(forceYBlock{1},'Value','0');

fprintf('\nCart forces:\n');
fprintf('Fx = 0 N\n');
fprintf('Fy = 0 N\n');

%% 3. SET SIMULATION TIME

set_param(model,'StopTime',num2str(simulationTime));

fprintf('\nSimulation duration: %.2f seconds\n', ...
    simulationTime);

%% 4. RUN SIMULATION

fprintf('\nCompiling and running Simscape model...\n');

try
    simIn = Simulink.SimulationInput(model);

    simIn = simIn.setModelParameter( ...
        'StopTime',num2str(simulationTime));

    simOut = sim(simIn);

    fprintf('SIMULATION: SUCCESS\n');

catch ME
    fprintf('\nSIMULATION FAILED\n');
    fprintf('%s\n', ...
        getReport(ME,'extended','hyperlinks','off'));
    rethrow(ME);
end

%% 5. EXTRACT MEASUREMENTS

fprintf('\nExtracting measurement signals...\n');

[tBob,bobXYZ] = readLoggedSignal(simOut,'bob_xyz');

[tX,xPosition] = readLoggedSignal(simOut,'x_position');
[tY,yPosition] = readLoggedSignal(simOut,'y_position');

[tXV,xVelocity] = readLoggedSignal(simOut,'x_velocity');
[tYV,yVelocity] = readLoggedSignal(simOut,'y_velocity');

% Ensure bob XYZ has three columns.
if size(bobXYZ,2) ~= 3
    error('Bob position must contain three coordinate columns.');
end

% Interpolate cart measurements onto bob measurement time.
xPosition = interp1(tX,xPosition,tBob,'linear','extrap');
yPosition = interp1(tY,yPosition,tBob,'linear','extrap');

xVelocity = interp1(tXV,xVelocity,tBob,'linear','extrap');
yVelocity = interp1(tYV,yVelocity,tBob,'linear','extrap');

t = tBob;

%% 6. CALCULATE PENDULUM GEOMETRY

% Bob position relative to cart pivot.
relativeX = bobXYZ(:,1) - xPosition;
relativeY = bobXYZ(:,2) - yPosition;
relativeZ = bobXYZ(:,3);

% Actual rod length.
rodLength = sqrt( ...
    relativeX.^2 + relativeY.^2 + relativeZ.^2);

% Inclination from upward vertical.
inclination = atan2( ...
    sqrt(relativeX.^2 + relativeY.^2), ...
    relativeZ);

inclinationDeg = rad2deg(inclination);

% Azimuth measured from +X toward +Y.
azimuth = atan2(relativeY,relativeX);
azimuthDeg = rad2deg(unwrap(azimuth));

%% 7. CALCULATE INITIAL AND FINAL CONDITIONS

initialInclination = inclinationDeg(1);
finalInclination = inclinationDeg(end);

initialRodLength = rodLength(1);
maximumRodError = max(abs(rodLength-L));

fprintf('\n============================================\n');
fprintf(' UNCONTROLLED SIMSCAPE RESULTS\n');
fprintf('============================================\n');

fprintf('\nSimulation duration: %.4f s\n',t(end));
fprintf('Number of samples: %d\n',numel(t));

fprintf('\nInitial inclination: %.6f degrees\n', ...
    initialInclination);

fprintf('Final inclination: %.6f degrees\n', ...
    finalInclination);

fprintf('\nInitial azimuth: %.6f degrees\n', ...
    azimuthDeg(1));

fprintf('Final azimuth: %.6f degrees\n', ...
    azimuthDeg(end));

fprintf('\nInitial rod length: %.12f m\n', ...
    initialRodLength);

fprintf('Maximum rod length error: %.6e m\n', ...
    maximumRodError);

fprintf('\nFinal cart X position: %.8f m\n', ...
    xPosition(end));

fprintf('Final cart Y position: %.8f m\n', ...
    yPosition(end));

%% 8. PLOT RESULTS

figure('Name','Uncontrolled Simscape Dynamics', ...
    'Color','w','Position',[100 100 1200 800]);

subplot(2,2,1);

plot(t,inclinationDeg,'LineWidth',1.8);

grid on;
xlabel('Time (s)');
ylabel('Inclination (degrees)');
title('Pendulum Inclination');

subplot(2,2,2);

plot(t,azimuthDeg,'LineWidth',1.8);

grid on;
xlabel('Time (s)');
ylabel('Azimuth (degrees)');
title('Pendulum Azimuth');

subplot(2,2,3);

plot(t,xPosition,'LineWidth',1.8);
hold on;
plot(t,yPosition,'LineWidth',1.8);

grid on;
xlabel('Time (s)');
ylabel('Cart displacement (m)');
title('Cart Position');

legend('X position','Y position','Location','best');

subplot(2,2,4);

plot(t,rodLength,'LineWidth',1.8);
hold on;
yline(L,'--');

grid on;
xlabel('Time (s)');
ylabel('Rod length (m)');
title('Rod Length Constraint');

%% 9. SAVE RESULTS

results.time = t;

results.bob_position = bobXYZ;

results.cart_position = [xPosition,yPosition];
results.cart_velocity = [xVelocity,yVelocity];

results.relative_position = ...
    [relativeX,relativeY,relativeZ];

results.inclination = inclination;
results.inclination_deg = inclinationDeg;

results.azimuth = azimuth;
results.azimuth_deg = azimuthDeg;

results.rod_length = rodLength;

results.parameters.M = M;
results.parameters.m = m;
results.parameters.L = L;
results.parameters.g = g;

results.parameters.rod_mass = 0.001;

results.initial_inclination_deg = initialInclination;
results.final_inclination_deg = finalInclination;

results.maximum_rod_length_error = maximumRodError;

save(fullfile(resultsFolder, ...
    'uncontrolled_simscape_results.mat'),'results');

fprintf('\nResults saved successfully:\n');
fprintf('Results/Simscape/uncontrolled_simscape_results.mat\n');

%% 10. VALIDATION SUMMARY

fprintf('\n============================================\n');
fprintf(' PHASE 7.9B VALIDATION SUMMARY\n');
fprintf('============================================\n');

if abs(t(end)-simulationTime) < 1e-3
    fprintf('Simulation duration: PASS\n');
else
    fprintf('Simulation duration: CHECK\n');
end

if maximumRodError < 1e-5
    fprintf('Rod length constraint: PASS\n');
else
    fprintf('Rod length constraint: CHECK\n');
end

fprintf('\nPHASE 7.9B COMPLETED\n');

%% ============================================================
% LOCAL FUNCTION: READ LOGGED SIGNAL
% =============================================================

function [t,data] = readLoggedSignal(simOut,variableName)

    signal = simOut.get(variableName);

    if isempty(signal)
        error('Workspace signal "%s" was not found.', ...
            variableName);
    end

    if isa(signal,'timeseries')

        t = signal.Time(:);
        data = squeeze(signal.Data);

    elseif isstruct(signal) && ...
            isfield(signal,'time') && ...
            isfield(signal,'signals')

        t = signal.time(:);
        data = squeeze(signal.signals.values);

    else

        error('Unsupported data format for "%s".', ...
            variableName);

    end

    % Convert row-oriented data into time-by-channel format.
    if isvector(data) && numel(data)==numel(t)
        data = data(:);
    elseif size(data,1) ~= numel(t) && ...
            size(data,2) == numel(t)
        data = data.';
    end

    if size(data,1) ~= numel(t)
        error('Time and data dimensions do not match for "%s".', ...
            variableName);
    end

end
