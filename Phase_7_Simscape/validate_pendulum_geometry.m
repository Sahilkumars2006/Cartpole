
%% PHASE 7.8: PENDULUM GEOMETRY SENSOR SETUP
%
% Adds a Transform Sensor between World and Bob.
% Measures the bob's actual XYZ position.
% The existing cart and pendulum assembly are preserved.

clear;
clc;

model = 'sm_inverted_spherical_pendulum';

fprintf('\n============================================\n');
fprintf(' PHASE 7.8: GEOMETRY SENSOR SETUP\n');
fprintf('============================================\n');

%% Load model

if ~bdIsLoaded(model)
    load_system(model);
end

%% Add Transform Sensor

sensorPath = [model '/Bob_Position_Sensor'];

if getSimulinkBlockHandle(sensorPath) == -1

    add_block( ...
        'sm_lib/Frames and Transforms/Transform Sensor', ...
        sensorPath, ...
        'Position',[1740 400 1840 490]);

end

%% Enable XYZ sensing

set_param(sensorPath,'SenseXYZ','on');

%% Obtain frame ports

worldPorts = get_param([model '/World'],'PortHandles');

bobPorts = get_param([model '/Pendulum_Bob'],'PortHandles');

sensorPorts = get_param(sensorPath,'PortHandles');

%% Connect World Frame to sensor base

add_line(model, ...
    worldPorts.RConn(1), ...
    sensorPorts.LConn(1), ...
    'autorouting','on');

%% Connect Bob frame to sensor follower

add_line(model, ...
    bobPorts.RConn(1), ...
    sensorPorts.RConn(1), ...
    'autorouting','on');

fprintf('\nTransform Sensor connected.\n');

%% Compile model

fprintf('\nCompiling model...\n');

set_param(model,'SimulationCommand','update');

fprintf('MODEL UPDATE: SUCCESS\n');

%% Inspect sensor ports

sensorPorts = get_param(sensorPath,'PortHandles');

fprintf('\n============================================\n');
fprintf(' TRANSFORM SENSOR PORT INFORMATION\n');
fprintf('============================================\n');

portTypes = {'LConn','RConn','Inport','Outport'};

for k = 1:numel(portTypes)

    name = portTypes{k};

    if isfield(sensorPorts,name)

        handles = sensorPorts.(name);

        fprintf('\n%s count: %d\n',name,numel(handles));

        for j = 1:numel(handles)
            fprintf('  Port %d: %.0f\n',j,handles(j));
        end

    end

end

%% Save

save_system(model);

fprintf('\n============================================\n');
fprintf(' GEOMETRY SENSOR SETUP COMPLETED\n');
fprintf('============================================\n');
