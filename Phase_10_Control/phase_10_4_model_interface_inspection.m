
%% PHASE 10.4: MODEL INTERFACE INSPECTION
%
% Inspect the controlled Cartesian model and its simulation interface.
% This script does not execute simulations or modify models.

clc;
clear;

fprintf('\n============================================\n');
fprintf('PHASE 10.4: MODEL INTERFACE INSPECTION\n');
fprintf('============================================\n');

%% 1. PROJECT PATHS

scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);

fprintf('\nProject directory:\n%s\n',projectDir);

%% 2. CONTROLLED CARTESIAN EQUATIONS

fprintf('\n============================================\n');
fprintf('CONTROLLED CARTESIAN MODEL SOURCE\n');
fprintf('============================================\n');

candidateFiles = {
    fullfile(projectDir, ...
    'Phase_4_MATLAB_Simulation', ...
    'controlled_cartesian_equations_of_motion.m')

    fullfile(projectDir, ...
    'Phase_4_MATLAB_Simulation', ...
    'cartesian_equations_of_motion.m')

    fullfile(projectDir, ...
    'Phase_10_Control', ...
    'control_design.m')
};

for k = 1:numel(candidateFiles)

    filePath = candidateFiles{k};

    fprintf('\n--------------------------------------------\n');

    if isfile(filePath)

        fprintf('FILE: %s\n',filePath);

        sourceText = fileread(filePath);

        fprintf('%s\n',sourceText);

    else

        fprintf('FILE NOT FOUND: %s\n',filePath);

    end

end

%% 3. CONTROLLED SIMSCAPE MODEL INSPECTION

fprintf('\n============================================\n');
fprintf('CONTROLLED SIMSCAPE MODEL\n');
fprintf('============================================\n');

modelFile = fullfile(projectDir, ...
    'Phase_7_Simscape', ...
    'sm_inverted_spherical_pendulum_lqr.slx');

if isfile(modelFile)

    fprintf('Controlled model found:\n%s\n',modelFile);

    modelInfo = dir(modelFile);

    fprintf('File size: %.3f MB\n', ...
        modelInfo.bytes/1024^2);

else

    fprintf('Controlled Simscape model not found.\n');

end

%% 4. CONTROLLED SIMULATION RESULTS

fprintf('\n============================================\n');
fprintf('CONTROLLED SIMULATION RESULT INSPECTION\n');
fprintf('============================================\n');

resultFile = fullfile(projectDir, ...
    'Results', ...
    'Simscape', ...
    'controlled_simscape_results_with_forces.mat');

if isfile(resultFile)

    fprintf('Result file found:\n%s\n',resultFile);

    resultInfo = whos('-file',resultFile);

    for k = 1:numel(resultInfo)

        fprintf('%-35s %-25s ', ...
            resultInfo(k).name,resultInfo(k).class);

        fprintf('%s\n',mat2str(resultInfo(k).size));

    end

else

    fprintf('Controlled result file not found.\n');

end

%% 5. PARAMETER FUNCTION INSPECTION

fprintf('\n============================================\n');
fprintf('SYSTEM PARAMETER SOURCE\n');
fprintf('============================================\n');

parameterFile = fullfile(projectDir, ...
    'Phase_4_MATLAB_Simulation', ...
    'system_parameters.m');

if isfile(parameterFile)

    fprintf('Parameter function found:\n%s\n',parameterFile);

    fprintf('\n--- SOURCE CODE ---\n');

    fprintf('%s\n',fileread(parameterFile));

else

    fprintf('system_parameters.m not found.\n');

end

%% 6. FINAL SUMMARY

fprintf('\n============================================\n');
fprintf('PHASE 10.4 INSPECTION COMPLETED\n');
fprintf('============================================\n');

fprintf('No simulation performed.\n');
fprintf('No controller gains modified.\n');
fprintf('No Simulink or Simscape model modified.\n');
fprintf('No existing result files modified.\n');

fprintf('\nPlease inspect the output before proceeding.\n');
