
%% PHASE 7.8D: INSPECT PENDULUM ORIENTATION

clear;
clc;

model = 'sm_inverted_spherical_pendulum';

if ~bdIsLoaded(model)
    load_system(model);
end

fprintf('\n============================================\n');
fprintf(' PHASE 7.8D: ORIENTATION DIAGNOSTICS\n');
fprintf('============================================\n');

%% Blocks to inspect

blocks = {
    'Spherical_Joint'
    'Rod_Transform'
    'Bob_Transform'
    'X_Axis_Orientation'
    'Y_Axis_Orientation'
    };

for k = 1:numel(blocks)

    block = [model '/' blocks{k}];

    fprintf('\n--------------------------------------------\n');
    fprintf('BLOCK: %s\n',blocks{k});
    fprintf('--------------------------------------------\n');

    if getSimulinkBlockHandle(block) == -1
        fprintf('BLOCK NOT FOUND\n');
        continue;
    end

    params = get_param(block,'DialogParameters');
    names = fieldnames(params);

    for j = 1:numel(names)

        name = names{j};

        if contains(name,'Rotation','IgnoreCase',true) || ...
                contains(name,'Translation','IgnoreCase',true) || ...
                contains(name,'PositionTarget','IgnoreCase',true) || ...
                contains(name,'Frame','IgnoreCase',true)

            try
                value = get_param(block,name);

                fprintf('%s:\n',name);
                disp(value);

            catch
                fprintf('%s: Could not read\n',name);
            end

        end
    end
end

%% Inspect spherical joint connections

fprintf('\n============================================\n');
fprintf(' SPHERICAL JOINT CONNECTIONS\n');
fprintf('============================================\n');

joint = [model '/Spherical_Joint'];

ports = get_param(joint,'PortConnectivity');

for k = 1:numel(ports)

    fprintf('\nPort %d\n',k);
    fprintf('Type: %s\n',ports(k).Type);
    fprintf('Position: %s\n',mat2str(ports(k).Position));

    if ~isempty(ports(k).SrcBlock)
        fprintf('Source block: %s\n', ...
            getfullname(ports(k).SrcBlock));
    end

    if ~isempty(ports(k).DstBlock)
        fprintf('Destination block(s):\n');

        for j = 1:numel(ports(k).DstBlock)
            fprintf('  %s\n', ...
                getfullname(ports(k).DstBlock(j)));
        end
    end
end

fprintf('\n============================================\n');
fprintf(' ORIENTATION INSPECTION COMPLETED\n');
fprintf('============================================\n');
