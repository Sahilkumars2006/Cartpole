
%% PHASE 7.4: FORCE AND CONVERTER CONFIGURATION

clear;
clc;

load_system('sm_lib');
load_system('nesl_utility');

blocks = { ...
    'sm_lib/Joints/Prismatic Joint', ...
    'sm_lib/Forces and Torques/External Force and Torque', ...
    'nesl_utility/Simulink-PS Converter', ...
    'nesl_utility/PS-Simulink Converter'};

parametersToInspect = { ...
    {'JointMode','TorqueActuationMode','MotionActuationMode'}, ...
    {}, ...
    {}, ...
    {}};

for k = 1:numel(blocks)

    blk = blocks{k};

    fprintf('\n============================================\n');
    fprintf('BLOCK: %s\n',blk);
    fprintf('============================================\n');

    %% MASK PARAMETERS

    try
        params = get_param(blk,'DialogParameters');
        names = fieldnames(params);

        for j = 1:numel(names)
            fprintf('\n%s\n',names{j});
            disp(params.(names{j}));
        end
    catch ME
        fprintf('Dialog inspection: %s\n',ME.message);
    end

    %% CURRENT VALUES OF IMPORTANT JOINT SETTINGS

    for j = 1:numel(parametersToInspect{k})

        name = parametersToInspect{k}{j};

        try
            fprintf('Current %s = %s\n', ...
                name,string(get_param(blk,name)));
        catch ME
            fprintf('Could not read %s: %s\n',name,ME.message);
        end
    end

    %% PORT COUNTS

    try
        ports = get_param(blk,'PortHandles');

        fprintf('\n--- PORT COUNTS ---\n');

        names = fieldnames(ports);

        for j = 1:numel(names)
            handles = ports.(names{j});

            if ~isempty(handles)
                fprintf('%s: %d\n',names{j},numel(handles));
            end
        end
    catch ME
        fprintf('Port inspection: %s\n',ME.message);
    end
end

fprintf('\n============================================\n');
fprintf(' FORCE CONFIGURATION INSPECTION COMPLETED\n');
fprintf('============================================\n');

