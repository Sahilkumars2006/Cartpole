
%% PHASE 7.8G: MEASURE MAIN CART ROTATION

clear;
clc;

model = 'sm_inverted_spherical_pendulum';

if ~bdIsLoaded(model)
    load_system(model);
end

fprintf('\n============================================\n');
fprintf(' PHASE 7.8G: MAIN CART ROTATION MEASUREMENT\n');
fprintf('============================================\n');

%% Configure sensor

sensor = [model '/World_To_Cart_Sensor'];

set_param(sensor, ...
    'SenseXYZ','on', ...
    'SenseR','on');

%% Add PS-Simulink Converter for rotation output

converter = [model '/Cart_Rotation_Converter'];

if getSimulinkBlockHandle(converter) == -1
    add_block( ...
        'nesl_utility/PS-Simulink Converter', ...
        converter, ...
        'Position',[1940 830 2040 880]);
end

set_param(converter,'Unit','1');

%% Add To Workspace block

sink = [model '/Cart_Rotation_Workspace'];

if getSimulinkBlockHandle(sink) == -1
    add_block( ...
        'simulink/Sinks/To Workspace', ...
        sink, ...
        'Position',[2140 830 2260 880]);
end

set_param(sink, ...
    'VariableName','cart_rotation_data', ...
    'SaveFormat','Timeseries');

%% Connect rotation output

sensorPorts = get_param(sensor,'PortHandles');
converterPorts = get_param(converter,'PortHandles');
sinkPorts = get_param(sink,'PortHandles');

add_line(model, ...
    sensorPorts.RConn(3), ...
    converterPorts.LConn(1), ...
    'autorouting','on');

add_line(model, ...
    converterPorts.Outport(1), ...
    sinkPorts.Inport(1), ...
    'autorouting','on');

%% Simulate

set_param(model, ...
    'StopTime','0.001', ...
    'ReturnWorkspaceOutputs','on');

fprintf('\nCompiling model...\n');

set_param(model,'SimulationCommand','update');

fprintf('MODEL UPDATE: SUCCESS\n');

fprintf('\nRunning simulation...\n');

simOut = sim(model,'StopTime','0.001');

fprintf('SIMULATION: SUCCESS\n');

%% Retrieve rotation data

rotationSignal = simOut.get('cart_rotation_data');

if isempty(rotationSignal)
    error('Rotation measurement data is empty.');
end

rotationData = squeeze(rotationSignal.Data);

fprintf('\nRaw rotation data dimensions: %s\n', ...
    mat2str(size(rotationData)));

fprintf('\nRaw first sample:\n');
disp(rotationData(1,:));

%% Reshape rotation matrix

if numel(rotationData(1,:)) ~= 9
    error(['Expected 9 rotation matrix elements, ' ...
           'but received %d.'], ...
           numel(rotationData(1,:)));
end

R = reshape(rotationData(1,:),3,3);

fprintf('\n============================================\n');
fprintf(' MEASURED WORLD-TO-CART ROTATION MATRIX\n');
fprintf('============================================\n');

disp(R);

%% Compare with identity

R_identity = eye(3);

rotationError = norm(R-R_identity,'fro');

fprintf('Rotation matrix orthogonality error: %.8e\n', ...
    norm(R'*R-eye(3),'fro'));

fprintf('Determinant: %.8f\n',det(R));

fprintf('Difference from identity: %.8e\n', ...
    rotationError);

%% Save results

results.rotationMatrix = R;
results.rotationError = rotationError;
results.orthogonalityError = norm(R'*R-eye(3),'fro');
results.determinant = det(R);

save(fullfile(pwd,'..','Results','Simscape', ...
    'main_cart_rotation_results.mat'),'results');

save_system(model);

fprintf('\nPHASE 7.8G COMPLETED\n');
