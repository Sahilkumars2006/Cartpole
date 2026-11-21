
%% PHASE 10.2: CONTROLLER DESIGN INSPECTION
%
% Read-only inspection of existing Phase 10 controller files.
% Does not execute control_design.m.
% Does not modify controller gains or models.

clc;
clear;

fprintf('\n============================================\n');
fprintf('PHASE 10.2: CONTROLLER DESIGN INSPECTION\n');
fprintf('============================================\n');

%% 1. PATHS

scriptFolder = fileparts(mfilename('fullpath'));

designFile = fullfile(scriptFolder,'control_design.m');
dataFile = fullfile(scriptFolder,'controller_data.mat');

%% 2. INSPECT CONTROL DESIGN SOURCE

fprintf('\n============================================\n');
fprintf('EXISTING CONTROL DESIGN SOURCE CODE\n');
fprintf('============================================\n');

if isfile(designFile)

    sourceCode = fileread(designFile);

    fprintf('\n--- control_design.m CONTENTS ---\n\n');
    fprintf('%s\n',sourceCode);

else

    error('control_design.m was not found.');

end

%% 3. INSPECT CONTROLLER DATA FILE

fprintf('\n============================================\n');
fprintf('CONTROLLER DATA FILE INSPECTION\n');
fprintf('============================================\n');

if isfile(dataFile)

    fprintf('\nFile found:\n%s\n',dataFile);

    variableInfo = whos('-file',dataFile);

    fprintf('\n--- VARIABLES STORED IN MAT FILE ---\n');

    for i = 1:numel(variableInfo)

        fprintf('\nVariable: %s\n',variableInfo(i).name);
        fprintf('Class: %s\n',variableInfo(i).class);
        fprintf('Size: %s\n',mat2str(variableInfo(i).size));

    end

    %% Load data into a structure to avoid workspace conflicts

    S = load(dataFile);

    variableNames = fieldnames(S);

    fprintf('\n============================================\n');
    fprintf('SAVED CONTROLLER DATA VALUES\n');
    fprintf('============================================\n');

    for i = 1:numel(variableNames)

        variableName = variableNames{i};

        fprintf('\n--- %s ---\n',variableName);

        value = S.(variableName);

        if isnumeric(value) || islogical(value)

            disp(value);

        elseif ischar(value) || isstring(value)

            disp(value);

        elseif isstruct(value)

            disp(value);

            nestedNames = fieldnames(value);

            for j = 1:numel(nestedNames)

                fprintf('\nField: %s\n',nestedNames{j});
                disp(value.(nestedNames{j}));

            end

        elseif iscell(value)

            disp(value);

        else

            fprintf('Data type: %s\n',class(value));
            disp(value);

        end

    end

else

    error('controller_data.mat was not found.');

end

%% 4. CHECK EXPECTED LQR VARIABLES

fprintf('\n============================================\n');
fprintf('LQR VARIABLE IDENTIFICATION\n');
fprintf('============================================\n');

expectedNames = {'K','Q','R','A','B','maximumForce', ...
    'maximum_force','max_force'};

for i = 1:numel(expectedNames)

    name = expectedNames{i};

    found = isfield(S,name);

    if found

        fprintf('%-20s : FOUND\n',name);

    else

        fprintf('%-20s : NOT FOUND AS TOP-LEVEL VARIABLE\n',name);

    end

end

%% 5. PHASE 8 REFERENCE GAIN

fprintf('\n============================================\n');
fprintf('PHASE 8 VERIFIED LQR REFERENCE\n');
fprintf('============================================\n');

K_reference = [
    -4.4721  0       -99.4112  0       -8.5016  0       -29.0870  0
     0      -4.4721   0       -99.4112  0       -8.5016  0       -29.0870
];

fprintf('\nPreviously verified Phase 8 gain:\n');
disp(K_reference);

fprintf('Reference gain dimensions: %d x %d\n', ...
    size(K_reference,1),size(K_reference,2));

fprintf('\nReference maximum force: 30 N\n');

%% 6. FINAL SUMMARY

fprintf('\n============================================\n');
fprintf('PHASE 10.2 INSPECTION SUMMARY\n');
fprintf('============================================\n');

fprintf('Controller source inspected: YES\n');
fprintf('Controller MAT file inspected: YES\n');
fprintf('MAT file loaded read-only: YES\n');
fprintf('Phase 8 reference gain displayed: YES\n');

fprintf('\nNo controller design script executed.\n');
fprintf('No controller gains modified.\n');
fprintf('No simulation performed.\n');
fprintf('No model modified.\n');

fprintf('\n============================================\n');
fprintf('PHASE 10.2 INSPECTION COMPLETED\n');
fprintf('============================================\n');
