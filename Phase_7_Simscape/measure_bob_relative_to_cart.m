
%% PHASE 7.8C: MEASURE BOB RELATIVE TO MAIN CART

clear;
clc;

model = 'sm_inverted_spherical_pendulum';

if ~bdIsLoaded(model)
    load_system(model);
end

fprintf('\n============================================\n');
fprintf(' PHASE 7.8C: RELATIVE FRAME MEASUREMENT\n');
fprintf('============================================\n');

%% Add second Transform Sensor

sensor = [model '/Cart_To_Bob_Sensor'];

if getSimulinkBlockHandle(sensor) == -1

    add_block( ...
        'sm_lib/Frames and Transforms/Transform Sensor', ...
        sensor, ...
        'Position',[1740 600 1840 690]);

end

set_param(sensor,'SenseXYZ','on');

%% Add PS-Simulink Converter

converter = [model '/Cart_To_Bob_Converter'];

if getSimulinkBlockHandle(converter) == -1

    add_block( ...
        'nesl_utility/PS-Simulink Converter', ...
        converter, ...
        'Position',[1930 610 2020 660]);

end

set_param(converter,'Unit','m');

%% Add To Workspace

sink = [model '/Cart_To_Bob_Workspace'];

if getSimulinkBlockHandle(sink) == -1

    add_block( ...
        'simulink/Sinks/To Workspace', ...
        sink, ...
        'Position',[2120 610 2230 660]);

end

set_param(sink, ...
    'VariableName','cart_to_bob_xyz', ...
    'SaveFormat','Timeseries');

%% Get frame ports

cartPorts = get_param( ...
    [model '/Main_Cart'],'PortHandles');

bobPorts = get_param( ...
    [model '/Pendulum_Bob'],'PortHandles');

sensorPorts = get_param(sensor,'PortHandles');

converterPorts = get_param(converter,'PortHandles');

sinkPorts = get_param(sink,'PortHandles');

%% Connect Main Cart to sensor base

add_line(model, ...
    cartPorts.RConn(1), ...
    sensorPorts.LConn(1), ...
    'autorouting','on');

%% Connect Bob to sensor follower

add_line(model, ...
    bobPorts.RConn(1), ...
    sensorPorts.RConn(1), ...
    'autorouting','on');

%% Connect sensor XYZ output

add_line(model, ...
    sensorPorts.RConn(2), ...
    converterPorts.LConn(1), ...
    'autorouting','on');

add_line(model, ...
    converterPorts.Outport(1), ...
    sinkPorts.Inport(1), ...
    'autorouting','on');

%% Compile and simulate

set_param(model,'StopTime','0.01');
set_param(model,'ReturnWorkspaceOutputs','on');

fprintf('\nCompiling model...\n');

set_param(model,'SimulationCommand','update');

fprintf('MODEL UPDATE: SUCCESS\n');

fprintf('\nRunning simulation...\n');

simOut = sim(model,'StopTime','0.01');

fprintf('SIMULATION: SUCCESS\n');

%% Retrieve measurement

measurement = simOut.get('cart_to_bob_xyz');

if isempty(measurement)
    error('No cart_to_bob_xyz data returned.');
end

data = squeeze(measurement.Data);

if size(data,2) ~= 3 && size(data,1) == 3
    data = data.';
end

if size(data,2) ~= 3
    error('Unexpected measurement dimensions: %s',mat2str(size(data)));
end

initialPosition = data(1,:).';

%% Expected geometry

L = 1;
theta = deg2rad(5);
phi = deg2rad(30);

expected = [
    L*sin(theta)*cos(phi);
    L*sin(theta)*sin(phi);
    L*cos(theta)
];

%% Print results

fprintf('\n============================================\n');
fprintf(' CART-RELATIVE BOB POSITION\n');
fprintf('============================================\n');

fprintf('\nExpected position:\n');
disp(expected);

fprintf('Measured position:\n');
disp(initialPosition);

fprintf('Position error norm: %.8e m\n', ...
    norm(initialPosition-expected));

fprintf('Measured length: %.10f m\n',norm(initialPosition));

%% Save

results.expected = expected;
results.measured = initialPosition;
results.error = initialPosition-expected;
results.errorNorm = norm(initialPosition-expected);

save_system(model);

save(fullfile(pwd,'..','Results','Simscape', ...
    'cart_relative_geometry_validation.mat'),'results');

fprintf('\nPHASE 7.8C COMPLETED\n');
