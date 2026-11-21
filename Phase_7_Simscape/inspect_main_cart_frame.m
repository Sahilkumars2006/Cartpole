
%% PHASE 7.8E: INSPECT MAIN CART FRAME

clear;
clc;

model = 'sm_inverted_spherical_pendulum';

if ~bdIsLoaded(model)
    load_system(model);
end

fprintf('\n============================================\n');
fprintf(' PHASE 7.8E: MAIN CART FRAME INSPECTION\n');
fprintf('============================================\n');

%% Create sensor

sensor = [model '/World_To_Cart_Sensor'];

if getSimulinkBlockHandle(sensor) == -1
    add_block( ...
        'sm_lib/Frames and Transforms/Transform Sensor', ...
        sensor, ...
        'Position',[1740 800 1840 900]);
end

%% Enable XYZ and rotation matrix sensing

set_param(sensor,'SenseXYZ','on');

% Check whether this MATLAB release supports this parameter
params = get_param(sensor,'DialogParameters');

if isfield(params,'SenseRotationMatrix')

    set_param(sensor,'SenseRotationMatrix','on');
    fprintf('Rotation matrix sensing enabled.\n');

else

    fprintf(['SenseRotationMatrix parameter not found. ' ...
             'Available sensing parameters will be inspected.\n']);

end

%% Connect frames only if not already connected

sensorPorts = get_param(sensor,'PortHandles');
worldPorts = get_param([model '/World'],'PortHandles');
cartPorts = get_param([model '/Main_Cart'],'PortHandles');

% World -> Sensor base
if isempty(get_param(sensor,'PortConnectivity')) || ...
   isempty(get_param(sensor,'LineHandles'))
    % Connections are added below if needed.
end

add_line(model, ...
    worldPorts.RConn(1), ...
    sensorPorts.LConn(1), ...
    'autorouting','on');

add_line(model, ...
    cartPorts.RConn(1), ...
    sensorPorts.RConn(1), ...
    'autorouting','on');

%% Compile

fprintf('\nCompiling model...\n');

set_param(model,'SimulationCommand','update');

fprintf('MODEL UPDATE: SUCCESS\n');

%% Print settings

fprintf('\nTRANSFORM SENSOR SETTINGS\n');

params = get_param(sensor,'DialogParameters');
names = fieldnames(params);

for k = 1:numel(names)

    if contains(names{k},'Sense','IgnoreCase',true)

        try
            fprintf('%s: %s\n', ...
                names{k},get_param(sensor,names{k}));
        catch
        end

    end
end

%% Print port information

sensorPorts = get_param(sensor,'PortHandles');

fprintf('\nSENSOR PORT INFORMATION\n');
fprintf('LConn count: %d\n',numel(sensorPorts.LConn));
fprintf('RConn count: %d\n',numel(sensorPorts.RConn));

ports = get_param(sensor,'PortConnectivity');

for k = 1:numel(ports)

    fprintf('\nPort %d\n',k);
    fprintf('Type: %s\n',ports(k).Type);
    fprintf('Position: %s\n',mat2str(ports(k).Position));

    if ~isempty(ports(k).SrcBlock)
        fprintf('Source: %s\n', ...
            getfullname(ports(k).SrcBlock));
    end

    if ~isempty(ports(k).DstBlock)

        for j = 1:numel(ports(k).DstBlock)
            fprintf('Destination: %s\n', ...
                getfullname(ports(k).DstBlock(j)));
        end

    end
end

fprintf('\n============================================\n');
fprintf(' MAIN CART FRAME INSPECTION COMPLETED\n');
fprintf('============================================\n');

save_system(model);
