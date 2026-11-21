
%% PHASE 7.8A: TRANSFORM SENSOR PORT IDENTIFICATION

clear;
clc;

model = 'sm_inverted_spherical_pendulum';
sensor = [model '/Bob_Position_Sensor'];

if ~bdIsLoaded(model)
    load_system(model);
end

fprintf('\n============================================\n');
fprintf(' TRANSFORM SENSOR PORT INSPECTION\n');
fprintf('============================================\n');

%% Dialog parameters

params = get_param(sensor,'DialogParameters');

fprintf('\n--- SENSOR SETTINGS ---\n');

names = fieldnames(params);

for k = 1:numel(names)

    name = names{k};

    if contains(name,'Sense') || contains(name,'Unit')

        fprintf('%s: ',name);

        try
            disp(get_param(sensor,name));
        catch
            fprintf('Unavailable\n');
        end

    end
end

%% Port handles

ports = get_param(sensor,'PortHandles');

fprintf('\n--- PORT HANDLE INFORMATION ---\n');

types = {'LConn','RConn','Inport','Outport'};

for k = 1:numel(types)

    name = types{k};

    if ~isfield(ports,name)
        continue;
    end

    handles = ports.(name);

    fprintf('\n%s ports: %d\n',name,numel(handles));

    for j = 1:numel(handles)

        h = handles(j);

        fprintf('\nPort %d, handle %.0f\n',j,h);

        try
            lineHandle = get_param(h,'Line');
            fprintf('Connected line: %.0f\n',lineHandle);
        catch
        end

    end
end

%% Port connectivity

fprintf('\n--- PORT CONNECTIVITY ---\n');

try

    connectivity = get_param(sensor,'PortConnectivity');

    for k = 1:numel(connectivity)

        fprintf('\nPort %d\n',k);

        if isfield(connectivity(k),'Type')
            fprintf('Type: %s\n',connectivity(k).Type);
        end

        if isfield(connectivity(k),'Position')
            fprintf('Position: ');
            disp(connectivity(k).Position);
        end

        if isfield(connectivity(k),'SrcBlock')
            fprintf('Source block: ');
            disp(connectivity(k).SrcBlock);
        end

        if isfield(connectivity(k),'DstBlock')
            fprintf('Destination block: ');
            disp(connectivity(k).DstBlock);
        end

    end

catch ME
    fprintf('Connectivity inspection failed: %s\n',ME.message);
end

fprintf('\n============================================\n');
fprintf(' SENSOR PORT INSPECTION COMPLETED\n');
fprintf('============================================\n');
