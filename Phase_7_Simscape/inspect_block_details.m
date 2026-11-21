
%% PHASE 7.2: INSPECT SIMSCAPE BLOCK DETAILS

clear;
clc;

load_system('sm_lib');

blocks = { ...
    'sm_lib/Body Elements/Brick Solid', ...
    'sm_lib/Body Elements/Cylindrical Solid', ...
    'sm_lib/Body Elements/Spherical Solid', ...
    'sm_lib/Joints/Prismatic Joint', ...
    'sm_lib/Joints/Spherical Joint', ...
    'sm_lib/Frames and Transforms/Rigid Transform', ...
    'sm_lib/Frames and Transforms/Transform Sensor', ...
    'sm_lib/Frames and Transforms/World Frame', ...
    'sm_lib/Utilities/Mechanism Configuration'};

for k = 1:numel(blocks)

    blk = blocks{k};

    fprintf('\n============================================\n');
    fprintf('BLOCK: %s\n',blk);
    fprintf('============================================\n');

    %% MASK PARAMETERS

    try
        params = get_param(blk,'DialogParameters');
        names = fieldnames(params);

        fprintf('\n--- DIALOG PARAMETERS ---\n');

        if isempty(names)
            fprintf('No dialog parameters found.\n');
        else
            for j = 1:numel(names)
                fprintf('%s\n',names{j});
            end
        end
    catch ME
        fprintf('Parameter inspection error: %s\n',ME.message);
    end

    %% PORT INFORMATION

    try
        ports = get_param(blk,'PortHandles');

        fprintf('\n--- PORT COUNTS ---\n');

        portTypes = fieldnames(ports);

        for j = 1:numel(portTypes)
            handles = ports.(portTypes{j});

            if ~isempty(handles)
                fprintf('%s: %d\n', ...
                    portTypes{j},numel(handles));
            end
        end
    catch ME
        fprintf('Port inspection error: %s\n',ME.message);
    end

end

fprintf('\n============================================\n');
fprintf(' BLOCK DETAIL INSPECTION COMPLETED\n');
fprintf('============================================\n');
