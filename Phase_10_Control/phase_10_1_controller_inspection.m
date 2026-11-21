
%% PHASE 10.1: EXISTING CONTROLLER INSPECTION
%
% Project:
% Mathematical Modelling, Simulation and Analysis of a
% 3D X-Y Cart with an Inverted Spherical Pendulum
%
% Purpose:
% Inspect existing Phase 10 files and verified Phase 8
% controller artifacts.
%
% IMPORTANT:
% This script is read-only.
% It does not modify any model, controller or result file.

clc;
clear;

fprintf('\n============================================\n');
fprintf('PHASE 10.1: CONTROLLER INSPECTION\n');
fprintf('============================================\n');

%% 1. PROJECT PATHS

scriptFolder = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptFolder);

controlFolder = fullfile(projectRoot,'Phase_10_Control');

fprintf('\nProject root:\n%s\n',projectRoot);
fprintf('\nPhase 10 control folder:\n%s\n',controlFolder);

if ~exist(controlFolder,'dir')
    error('Phase_10_Control folder does not exist.');
end

%% 2. INSPECT PHASE 10 CONTROL FOLDER

fprintf('\n============================================\n');
fprintf('PHASE 10 CONTROL FOLDER CONTENTS\n');
fprintf('============================================\n');

folderContents = dir(fullfile(controlFolder,'**','*'));

folderContents = folderContents(~[folderContents.isdir]);

if isempty(folderContents)

    fprintf('No files currently exist in Phase_10_Control.\n');

else

    fileName = strings(numel(folderContents),1);
    fileExtension = strings(numel(folderContents),1);
    fileSizeKB = zeros(numel(folderContents),1);
    modifiedDate = strings(numel(folderContents),1);

    for i = 1:numel(folderContents)

        fileName(i) = string(fullfile( ...
            folderContents(i).folder, ...
            folderContents(i).name));

        [~,~,ext] = fileparts(folderContents(i).name);

        fileExtension(i) = string(ext);

        fileSizeKB(i) = folderContents(i).bytes/1024;

        modifiedDate(i) = string( ...
            folderContents(i).date);

        fprintf('\nFile: %s\n',folderContents(i).name);
        fprintf('Path: %s\n',folderContents(i).folder);
        fprintf('Size: %.3f KB\n',fileSizeKB(i));
        fprintf('Modified: %s\n',modifiedDate(i));

    end

    folderInventory = table( ...
        fileName,fileExtension,fileSizeKB,modifiedDate);

    fprintf('\n--- PHASE 10 FILE INVENTORY ---\n');
    disp(folderInventory);

end

%% 3. INSPECT EXISTING CONTROLLED SIMSCAPE MODEL

fprintf('\n============================================\n');
fprintf('EXISTING CONTROLLED MODEL INSPECTION\n');
fprintf('============================================\n');

controlledModel = fullfile( ...
    projectRoot, ...
    'Phase_7_Simscape', ...
    'sm_inverted_spherical_pendulum_lqr.slx');

uncontrolledModel = fullfile( ...
    projectRoot, ...
    'Phase_7_Simscape', ...
    'sm_inverted_spherical_pendulum.slx');

if isfile(controlledModel)

    info = dir(controlledModel);

    fprintf('\nCONTROLLED MODEL: FOUND\n');
    fprintf('File: %s\n',controlledModel);
    fprintf('Size: %.3f MB\n',info.bytes/1024^2);

else

    fprintf('\nCONTROLLED MODEL: NOT FOUND\n');

end

if isfile(uncontrolledModel)

    info = dir(uncontrolledModel);

    fprintf('\nORIGINAL UNCONTROLLED MODEL: FOUND\n');
    fprintf('File: %s\n',uncontrolledModel);
    fprintf('Size: %.3f MB\n',info.bytes/1024^2);

else

    fprintf('\nORIGINAL UNCONTROLLED MODEL: NOT FOUND\n');

end

%% 4. INSPECT PHASE 8 VALIDATION FILES

fprintf('\n============================================\n');
fprintf('PHASE 8 VALIDATION FILES\n');
fprintf('============================================\n');

phase8Folder = fullfile(projectRoot,'Phase_8_Validation');

if isfolder(phase8Folder)

    phase8Files = dir(fullfile(phase8Folder,'**','*'));

    phase8Files = phase8Files(~[phase8Files.isdir]);

    if isempty(phase8Files)

        fprintf('No files found in Phase 8 validation folder.\n');

    else

        for i = 1:numel(phase8Files)

            fprintf('%s\n',fullfile( ...
                phase8Files(i).folder, ...
                phase8Files(i).name));

        end

    end

else

    fprintf('Phase 8 validation folder not found.\n');

end

%% 5. INSPECT CONTROLLED SIMSCAPE RESULTS

fprintf('\n============================================\n');
fprintf('CONTROLLED SIMSCAPE RESULTS\n');
fprintf('============================================\n');

controlledResults = fullfile( ...
    projectRoot, ...
    'Results', ...
    'Simscape', ...
    'controlled_simscape_results_with_forces.mat');

if isfile(controlledResults)

    info = dir(controlledResults);

    fprintf('\nControlled results file: FOUND\n');
    fprintf('Size: %.3f MB\n',info.bytes/1024^2);

    fprintf('\n--- SAVED MATLAB VARIABLES ---\n');

    resultVariables = whos('-file',controlledResults);

    for i = 1:numel(resultVariables)

        fprintf('%-35s ',resultVariables(i).name);

        fprintf('%-15s ',resultVariables(i).class);

        fprintf('%s\n', ...
            mat2str(resultVariables(i).size));

    end

else

    fprintf('\nControlled Simscape results file: NOT FOUND\n');

end

%% 6. INSPECT PHASE 8 CONSOLIDATED VALIDATION

fprintf('\n============================================\n');
fprintf('PHASE 8 THREE-MODEL VALIDATION\n');
fprintf('============================================\n');

validationFile = fullfile( ...
    projectRoot, ...
    'Phase_8_Validation', ...
    'phase_8_three_model_validation.mat');

if isfile(validationFile)

    fprintf('Phase 8 validation MAT file: FOUND\n');

    validationVariables = whos('-file',validationFile);

    fprintf('\n--- VALIDATION VARIABLES ---\n');

    for i = 1:numel(validationVariables)

        fprintf('%-35s ',validationVariables(i).name);

        fprintf('%-15s ',validationVariables(i).class);

        fprintf('%s\n', ...
            mat2str(validationVariables(i).size));

    end

else

    fprintf('Phase 8 validation MAT file: NOT FOUND\n');

end

%% 7. INSPECT CONTROLLER SOURCE FILES

fprintf('\n============================================\n');
fprintf('CONTROLLER SOURCE FILE SEARCH\n');
fprintf('============================================\n');

searchFolders = {
    fullfile(projectRoot,'Phase_10_Control')
    fullfile(projectRoot,'Phase_8_Validation')
    fullfile(projectRoot,'Scripts')
    fullfile(projectRoot,'Phase_7_Simscape')
};

for k = 1:numel(searchFolders)

    currentFolder = searchFolders{k};

    fprintf('\nSearching: %s\n',currentFolder);

    if isfolder(currentFolder)

        sourceFiles = dir(fullfile(currentFolder,'**','*.m'));

        if isempty(sourceFiles)

            fprintf('No MATLAB source files found.\n');

        else

            for i = 1:numel(sourceFiles)

                fprintf('  %s\n',fullfile( ...
                    sourceFiles(i).folder, ...
                    sourceFiles(i).name));

            end

        end

    else

        fprintf('Folder not found.\n');

    end

end

%% 8. INSPECT SAVED LQR GAIN FILES

fprintf('\n============================================\n');
fprintf('LQR GAIN FILE SEARCH\n');
fprintf('============================================\n');

matSearchFolders = {
    fullfile(projectRoot,'Phase_10_Control')
    fullfile(projectRoot,'Phase_8_Validation')
    fullfile(projectRoot,'Results')
};

gainFilesFound = 0;

for k = 1:numel(matSearchFolders)

    currentFolder = matSearchFolders{k};

    if isfolder(currentFolder)

        matFiles = dir(fullfile(currentFolder,'**','*.mat'));

        for i = 1:numel(matFiles)

            currentFile = fullfile( ...
                matFiles(i).folder,matFiles(i).name);

            variables = whos('-file',currentFile);

            for j = 1:numel(variables)

                variableName = lower(variables(j).name);

                if contains(variableName,'gain') || ...
                   strcmp(variableName,'k') || ...
                   contains(variableName,'riccati')

                    fprintf('\nPotential controller variable found:\n');
                    fprintf('File: %s\n',currentFile);
                    fprintf('Variable: %s\n',variables(j).name);
                    fprintf('Size: %s\n',mat2str(variables(j).size));

                    gainFilesFound = gainFilesFound + 1;

                end

            end

        end

    end

end

if gainFilesFound == 0

    fprintf('No separately saved LQR gain variables identified.\n');
    fprintf('The gain may be stored in MATLAB source code or model blocks.\n');

end

%% 9. PHASE 10.1 FINAL SUMMARY

fprintf('\n============================================\n');
fprintf('PHASE 10.1 INSPECTION SUMMARY\n');
fprintf('============================================\n');

fprintf('Phase 10 folder inspected: YES\n');
fprintf('Controlled Simscape model inspected: YES\n');
fprintf('Original uncontrolled model inspected: YES\n');
fprintf('Phase 8 validation files inspected: YES\n');
fprintf('Controlled simulation results inspected: YES\n');
fprintf('Controller source folders searched: YES\n');
fprintf('LQR gain files searched: YES\n');

fprintf('\nNo simulations performed.\n');
fprintf('No controller gains modified.\n');
fprintf('No Simulink or Simscape models modified.\n');

fprintf('\n============================================\n');
fprintf('PHASE 10.1 INSPECTION COMPLETED\n');
fprintf('============================================\n');
