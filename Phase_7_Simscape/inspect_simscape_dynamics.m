
%% ============================================================
% PHASE 7.10C: SIMSCAPE DYNAMICS DIAGNOSTICS
% Inspect body masses and joint configuration
% ============================================================

clc;

model = 'sm_inverted_spherical_pendulum';

if ~bdIsLoaded(model)
    load_system(model);
end

fprintf('\n============================================\n');
fprintf(' PHASE 7.10C: DYNAMICS DIAGNOSTICS\n');
fprintf('============================================\n');

%% 1. INSPECT BODY MASS PROPERTIES

fprintf('\n============================================\n');
fprintf(' BODY MASS PROPERTIES\n');
fprintf('============================================\n');

allBlocks = find_system(model, ...
    'LookUnderMasks','all', ...
    'FollowLinks','on', ...
    'Type','Block');

for k = 1:numel(allBlocks)

    block = allBlocks{k};

    try
        params = get_param(block,'DialogParameters');
    catch
        continue;
    end

    if isempty(params)
        continue;
    end

    paramNames = fieldnames(params);

    relevant = {};

    for j = 1:numel(paramNames)

        p = paramNames{j};

        if contains(lower(p),'mass') || ...
           contains(lower(p),'density') || ...
           contains(lower(p),'inertia') || ...
           contains(lower(p),'basedontype')

            relevant{end+1} = p; %#ok<SAGROW>

        end
    end

    if ~isempty(relevant)

        fprintf('\nBlock: %s\n',block);

        try
            fprintf('Mask type: %s\n', ...
                get_param(block,'MaskType'));
        catch
        end

        for j = 1:numel(relevant)

            p = relevant{j};

            try
                value = get_param(block,p);

                if ischar(value) || isstring(value)
                    fprintf('%s = %s\n',p,char(value));
                else
                    disp(value);
                end
            catch
            end

        end
    end
end

%% 2. INSPECT JOINT CONFIGURATION

fprintf('\n============================================\n');
fprintf(' JOINT CONFIGURATION\n');
fprintf('============================================\n');

jointNames = {'Spherical_Joint','Prismatic_X','Prismatic_Y'};

for k = 1:numel(jointNames)

    matches = find_system(model, ...
        'LookUnderMasks','all', ...
        'FollowLinks','on', ...
        'Name',jointNames{k});

    for j = 1:numel(matches)

        block = matches{j};

        fprintf('\nBlock: %s\n',block);

        try
            params = get_param(block,'DialogParameters');
            names = fieldnames(params);

            for n = 1:numel(names)

                p = names{n};

                if contains(lower(p),'mode') || ...
                   contains(lower(p),'positiontarget') || ...
                   contains(lower(p),'actuation') || ...
                   contains(lower(p),'motion') || ...
                   contains(lower(p),'sense')

                    try
                        fprintf('%s = %s\n', ...
                            p,string(get_param(block,p)));
                    catch
                    end

                end
            end
        catch
        end
    end
end

%% 3. INSPECT MECHANISM GRAVITY

fprintf('\n============================================\n');
fprintf(' MECHANISM GRAVITY\n');
fprintf('============================================\n');

mechanismBlocks = find_system(model, ...
    'LookUnderMasks','all', ...
    'FollowLinks','on', ...
    'Name','Mechanism');

for k = 1:numel(mechanismBlocks)

    block = mechanismBlocks{k};

    try
        fprintf('GravityVector = %s\n', ...
            get_param(block,'GravityVector'));
    catch
        fprintf('Could not read GravityVector.\n');
    end
end

fprintf('\n============================================\n');
fprintf(' PHASE 7.10C INSPECTION COMPLETED\n');
fprintf('============================================\n');
