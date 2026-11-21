
%% PHASE 7: FINAL SIMSCAPE SETTINGS INSPECTION

clear;
clc;

load_system('sm_lib');
load_system('nesl_utility');

%% 1. INSPECT ENUM OPTIONS

blocks = { ...
    'sm_lib/Joints/Prismatic Joint', ...
    'sm_lib/Joints/Spherical Joint', ...
    'sm_lib/Forces and Torques/External Force and Torque'};

fprintf('\n============================================\n');
fprintf(' FINAL SIMSCAPE SETTINGS\n');
fprintf('============================================\n');

for k = 1:numel(blocks)

    blk = blocks{k};
    fprintf('\n--- %s ---\n',blk);

    params = get_param(blk,'DialogParameters');

    if contains(blk,'Prismatic Joint')
        names = {'JointMode','TorqueActuationMode', ...
            'MotionActuationMode'};
    elseif contains(blk,'Spherical Joint')
        names = {'JointMode','PositionTargetRotationMethod', ...
            'TorqueActuationMode','ActuationFrame'};
    else
        names = {'ForceResolutionFrame','EnableForceX', ...
            'EnableForceY','EnableForceZ','EnableForce'};
    end

    for j = 1:numel(names)

        name = names{j};

        if isfield(params,name)
            p = params.(name);

            fprintf('\n%s\n',name);

            if isfield(p,'Enum')
                disp(p.Enum);
            else
                disp(p);
            end
        end
    end

    %% Port handles

    ports = get_param(blk,'PortHandles');

    fprintf('\nPort counts:\n');

    fields = fieldnames(ports);

    for j = 1:numel(fields)
        h = ports.(fields{j});

        if ~isempty(h)
            fprintf('%s: %d\n',fields{j},numel(h));
        end
    end
end

%% 2. INSPECT CONVERTER DEFAULTS

fprintf('\n--- SIMULINK-PS CONVERTER ---\n');

converter = 'nesl_utility/Simulink-PS Converter';

disp(get_param(converter,'Unit'));

fprintf('\n============================================\n');
fprintf(' FINAL INSPECTION COMPLETED\n');
fprintf('============================================\n');
