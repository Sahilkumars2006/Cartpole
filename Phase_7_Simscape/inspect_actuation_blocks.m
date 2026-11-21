
%% PHASE 7.3: ACTUATION BLOCK DISCOVERY

clear;
clc;

load_system('sm_lib');

fprintf('\n============================================\n');
fprintf(' ACTUATION BLOCK DISCOVERY\n');
fprintf('============================================\n');

%% 1. SEARCH FOR ACTUATOR AND CONVERTER BLOCKS

libraries = {'sm_lib','nesl_utility','simulink'};

keywords = { ...
    '.*Force.*', ...
    '.*Torque.*', ...
    '.*Simulink.*PS.*', ...
    '.*PS.*Simulink.*'};

for a = 1:numel(libraries)

    try
        load_system(libraries{a});
    catch
        continue;
    end

    for b = 1:numel(keywords)

        matches = find_system(libraries{a}, ...
            'LookUnderMasks','all', ...
            'FollowLinks','on', ...
            'RegExp','on', ...
            'Name',keywords{b});

        for k = 1:numel(matches)
            fprintf('%s\n',matches{k});
        end
    end
end

%% 2. INSPECT PRISMATIC JOINT ACTUATION ENUMS

blk = 'sm_lib/Joints/Prismatic Joint';
params = get_param(blk,'DialogParameters');

names = { ...
    'JointMode', ...
    'TorqueActuationMode', ...
    'MotionActuationMode', ...
    'SensePosition', ...
    'SenseVelocity'};

fprintf('\n--- PRISMATIC JOINT SETTINGS ---\n');

for k = 1:numel(names)

    name = names{k};

    if isfield(params,name)

        p = params.(name);

        fprintf('\n%s\n',name);

        fields = fieldnames(p);

        for j = 1:numel(fields)
            value = p.(fields{j});

            if ischar(value) || isstring(value)
                fprintf('  %s: %s\n',fields{j},string(value));
            elseif isnumeric(value) || islogical(value)
                fprintf('  %s: %s\n',fields{j},mat2str(value));
            end
        end
    end
end

%% 3. INSPECT SPHERICAL JOINT SETTINGS

blk = 'sm_lib/Joints/Spherical Joint';
params = get_param(blk,'DialogParameters');

names = { ...
    'JointMode', ...
    'PositionTargetSpecify', ...
    'PositionTargetRotationMethod', ...
    'ActuateTorqueX', ...
    'ActuateTorqueY', ...
    'ActuateTorqueZ', ...
    'SensePosition', ...
    'SenseVelocity'};

fprintf('\n--- SPHERICAL JOINT SETTINGS ---\n');

for k = 1:numel(names)

    name = names{k};

    if isfield(params,name)

        p = params.(name);

        fprintf('\n%s\n',name);

        fields = fieldnames(p);

        for j = 1:numel(fields)
            value = p.(fields{j});

            if ischar(value) || isstring(value)
                fprintf('  %s: %s\n',fields{j},string(value));
            elseif isnumeric(value) || islogical(value)
                fprintf('  %s: %s\n',fields{j},mat2str(value));
            end
        end
    end
end

fprintf('\n============================================\n');
fprintf(' ACTUATION INSPECTION COMPLETED\n');
fprintf('============================================\n');
