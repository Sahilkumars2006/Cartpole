
%% PHASE 7: SIMSCAPE MULTIBODY
% Inverted Spherical Pendulum
% Step 1: Verify required Simscape libraries and blocks

clear;
clc;
close all;

fprintf('\n============================================\n');
fprintf(' PHASE 7: SIMSCAPE MULTIBODY SETUP\n');
fprintf('============================================\n');

%% 1. CHECK REQUIRED PRODUCTS

products = ver;

productNames = {products.Name};

requiredProducts = { ...
    'Simulink', ...
    'Simscape', ...
    'Simscape Multibody'};

fprintf('\n--- INSTALLED PRODUCTS ---\n');

for k = 1:numel(requiredProducts)

    found = any(strcmpi(productNames,requiredProducts{k}));

    fprintf('%-25s : %s\n', ...
        requiredProducts{k}, ...
        string(found));

    if ~found
        error('Required product not detected: %s', ...
            requiredProducts{k});
    end

end

%% 2. LOAD SIMSCAPE MULTIBODY LIBRARY

try

    load_system('sm_lib');

    fprintf('\nSimscape Multibody library loaded.\n');

catch ME

    error('Unable to load sm_lib: %s',ME.message);

end

%% 3. SEARCH REQUIRED MULTIBODY BLOCKS

blockKeywords = { ...
    'World Frame', ...
    'Mechanism Configuration', ...
    'Prismatic Joint', ...
    'Spherical Joint', ...
    'Rigid Transform', ...
    'Solid', ...
    'Transform Sensor', ...
    'Joint Sensor'};

fprintf('\n--- AVAILABLE MULTIBODY BLOCKS ---\n');

for k = 1:numel(blockKeywords)

    matches = find_system('sm_lib', ...
        'LookUnderMasks','all', ...
        'FollowLinks','on', ...
        'Name',blockKeywords{k});

    fprintf('\n%s:\n',blockKeywords{k});

    if isempty(matches)

        fprintf('  NOT FOUND\n');

    else

        for j = 1:numel(matches)

            fprintf('  %s\n',matches{j});

        end

    end

end

%% 4. CHECK SIMSCAPE SOLVER CONFIGURATION BLOCK

try

    load_system('nesl_utility');

    solverMatches = find_system('nesl_utility', ...
        'LookUnderMasks','all', ...
        'FollowLinks','on', ...
        'Name','Solver Configuration');

    fprintf('\n--- SOLVER CONFIGURATION ---\n');

    if isempty(solverMatches)

        fprintf('Solver Configuration block not found.\n');

    else

        for k = 1:numel(solverMatches)

            fprintf('%s\n',solverMatches{k});

        end

    end

catch ME

    fprintf('\nSolver library inspection warning: %s\n', ...
        ME.message);

end

%% 5. CHECK REFERENCE SIMULATION PARAMETERS

projectDir = fileparts(fileparts(mfilename('fullpath')));

referenceFile = fullfile(projectDir, ...
    'Results','MATLAB','controlled_simulation_results.mat');

if ~isfile(referenceFile)

    error('Controlled simulation results not found.');

end

data = load(referenceFile,'results');

p = data.results.parameters;

fprintf('\n--- REFERENCE SYSTEM PARAMETERS ---\n');

fprintf('Cart mass: %.3f kg\n',p.M);
fprintf('Bob mass: %.3f kg\n',p.m);
fprintf('Rod length: %.3f m\n',p.L);
fprintf('Gravity: %.3f m/s^2\n',p.g);

fprintf('Initial inclination: %.3f degrees\n', ...
    rad2deg(acos(data.results.bob_position(1,3)/p.L)));

fprintf('\n============================================\n');
fprintf(' SIMSCAPE DIAGNOSTIC COMPLETED\n');
fprintf('============================================\n');
