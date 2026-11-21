
%% PHASE 7: SIMSCAPE BLOCK DISCOVERY
% Identify body, joint, and sensor blocks

clear;
clc;

load_system('sm_lib');

fprintf('\n============================================\n');
fprintf(' SIMSCAPE MULTIBODY BLOCK DISCOVERY\n');
fprintf('============================================\n');

%% SEARCH FOR BODY AND SOLID BLOCKS

patterns = { ...
    '.*Solid.*', ...
    '.*Body.*', ...
    '.*Inertia.*', ...
    '.*Joint.*Sensor.*', ...
    '.*Transform.*Sensor.*'};

labels = { ...
    'SOLID BLOCKS', ...
    'BODY BLOCKS', ...
    'INERTIA BLOCKS', ...
    'JOINT SENSOR BLOCKS', ...
    'TRANSFORM SENSOR BLOCKS'};

for k = 1:numel(patterns)

    fprintf('\n--- %s ---\n',labels{k});

    matches = find_system('sm_lib', ...
        'LookUnderMasks','all', ...
        'FollowLinks','on', ...
        'RegExp','on', ...
        'Name',patterns{k});

    if isempty(matches)
        fprintf('No matching blocks found.\n');
    else
        for j = 1:numel(matches)
            fprintf('\nBlock path: %s\n',matches{j});

            try
                fprintf('Mask type: %s\n', ...
                    get_param(matches{j},'MaskType'));
            catch
                fprintf('Mask type unavailable.\n');
            end
        end
    end
end

fprintf('\n============================================\n');
fprintf(' BLOCK DISCOVERY COMPLETED\n');
fprintf('============================================\n');
