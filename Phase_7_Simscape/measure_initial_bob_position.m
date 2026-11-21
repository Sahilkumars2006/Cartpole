
%% PHASE 7.8B: MEASURE INITIAL BOB POSITION
%
% Connects Transform Sensor XYZ output to workspace.
% Runs a short zero-force simulation.
% Compares measured position with analytical geometry.

clear;
clc;

model = 'sm_inverted_spherical_pendulum';

fprintf('\n============================================\n');
fprintf(' PHASE 7.8B: INITIAL POSITION VALIDATION\n');
fprintf('============================================\n');

%% Load model

if ~bdIsLoaded(model)
    load_system(model);
end

%% 1. SET ZERO CART FORCES

set_param([model '/Force_X'],'Value','0');
set_param([model '/Force_Y'],'Value','0');

fprintf('\nCart forces set to zero.\n');

%% 2. ADD PS-SIMULINK CONVERTER

converter = [model '/Bob_XYZ_Converter'];

if getSimulinkBlockHandle(converter) == -1

    add_block( ...
        'nesl_utility/PS-Simulink Converter', ...
        converter, ...
        'Position',[1930 410 2020 460]);

end

set_param(converter,'Unit','m');

%% 3. ADD TO WORKSPACE BLOCK

workspaceBlock = [model '/Bob_XYZ_Workspace'];

if getSimulinkBlockHandle(workspaceBlock) == -1

    add_block( ...
        'simulink/Sinks/To Workspace', ...
        workspaceBlock, ...
        'Position',[2120 410 2230 460]);

end

set_param(workspaceBlock, ...
    'VariableName','bob_xyz', ...
    'SaveFormat','Timeseries', ...
    'MaxDataPoints','inf');


%% 4. CONNECT SENSOR OUTPUT

sensor = [model '/Bob_Position_Sensor'];
converter = [model '/Bob_XYZ_Converter'];
workspaceBlock = [model '/Bob_XYZ_Workspace'];

sensorPorts = get_param(sensor,'PortHandles');
converterPorts = get_param(converter,'PortHandles');
workspacePorts = get_param(workspaceBlock,'PortHandles');

% Connect sensor output only if not already connected.
sensorLine = get_param(sensorPorts.RConn(2),'Line');

if sensorLine == -1

    add_line(model, ...
        sensorPorts.RConn(2), ...
        converterPorts.LConn(1), ...
        'autorouting','on');

    fprintf('Sensor-to-converter connection created.\n');

else

    fprintf('Sensor-to-converter connection already exists.\n');

end

% Connect converter output only if not already connected.
converterLine = get_param(converterPorts.Outport(1),'Line');

if converterLine == -1

    add_line(model, ...
        converterPorts.Outport(1), ...
        workspacePorts.Inport(1), ...
        'autorouting','on');

    fprintf('Converter-to-workspace connection created.\n');

else

    fprintf('Converter-to-workspace connection already exists.\n');

end

fprintf('XYZ measurement logging connection verified.\n');


%% 5. COMPILE MODEL

set_param(model,'StopTime','0.01');
set_param(model,'ReturnWorkspaceOutputs','on');

fprintf('\nCompiling model...\n');

set_param(model,'SimulationCommand','update');

fprintf('MODEL UPDATE: SUCCESS\n');

%% 6. RUN SHORT SIMULATION

fprintf('\nRunning initial-position simulation...\n');

simOut = sim(model,'StopTime','0.01');

fprintf('SIMULATION: SUCCESS\n');

%% 7. RETRIEVE MEASUREMENTS

bobXYZ = simOut.get('bob_xyz');

if isempty(bobXYZ)
    error('No bob_xyz data returned from simulation.');
end

time = bobXYZ.Time;
positionData = squeeze(bobXYZ.Data);

% Normalize data to N-by-3.
if size(positionData,2) ~= 3 && size(positionData,1) == 3
    positionData = positionData.';
end

if size(positionData,2) ~= 3
    error('Expected three XYZ measurement components. Actual size: %s', ...
        mat2str(size(positionData)));
end

initialPosition = positionData(1,:).';
finalPosition = positionData(end,:).';

%% 8. ANALYTICAL REFERENCE

L = 1.0;
theta0 = deg2rad(5);
phi0 = deg2rad(30);

expectedPosition = [
    L*sin(theta0)*cos(phi0);
    L*sin(theta0)*sin(phi0);
    L*cos(theta0)
];

%% 9. CALCULATE ERRORS

positionError = initialPosition - expectedPosition;

errorNorm = norm(positionError);

measuredLength = norm(initialPosition);

lengthError = abs(measuredLength - L);

%% 10. DISPLAY RESULTS

fprintf('\n============================================\n');
fprintf(' INITIAL BOB POSITION COMPARISON\n');
fprintf('============================================\n');

fprintf('\nCoordinate       Expected       Simscape\n');

fprintf('X (m)          %12.8f   %12.8f\n', ...
    expectedPosition(1),initialPosition(1));

fprintf('Y (m)          %12.8f   %12.8f\n', ...
    expectedPosition(2),initialPosition(2));

fprintf('Z (m)          %12.8f   %12.8f\n', ...
    expectedPosition(3),initialPosition(3));

fprintf('\nPosition error norm: %.8e m\n',errorNorm);

fprintf('Measured rod length: %.10f m\n',measuredLength);

fprintf('Rod length error: %.8e m\n',lengthError);

fprintf('\nFinal position after 0.01 s:\n');
fprintf('X = %.8f m\n',finalPosition(1));
fprintf('Y = %.8f m\n',finalPosition(2));
fprintf('Z = %.8f m\n',finalPosition(3));

%% 11. VALIDATION STATUS

positionTolerance = 1e-4;
lengthTolerance = 1e-4;

fprintf('\n============================================\n');
fprintf(' GEOMETRY VALIDATION\n');
fprintf('============================================\n');

if errorNorm < positionTolerance
    fprintf('Initial position: PASS\n');
else
    fprintf('Initial position: CHECK REQUIRED\n');
end

if lengthError < lengthTolerance
    fprintf('Rod length: PASS\n');
else
    fprintf('Rod length: CHECK REQUIRED\n');
end

%% 12. SAVE RESULTS

results.time = time;
results.position = positionData;
results.initialPosition = initialPosition;
results.finalPosition = finalPosition;
results.expectedPosition = expectedPosition;
results.positionError = positionError;
results.positionErrorNorm = errorNorm;
results.measuredLength = measuredLength;
results.lengthError = lengthError;

results.parameters.rodLength = L;
results.parameters.initialTilt = 5;
results.parameters.initialAzimuth = 30;

resultsFile = fullfile(pwd, ...
    '..','Results','Simscape', ...
    'initial_geometry_validation.mat');

resultsFolder = fileparts(resultsFile);

if ~exist(resultsFolder,'dir')
    mkdir(resultsFolder);
end

save(resultsFile,'results');

save_system(model);

fprintf('\nResults saved:\n%s\n',resultsFile);

fprintf('\n============================================\n');
fprintf(' PHASE 7.8B COMPLETED\n');
fprintf('============================================\n');
