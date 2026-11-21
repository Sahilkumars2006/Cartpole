
%% PHASE 11.1: PROJECT-WIDE FILE AND FOLDER INVENTORY
%
% Project:
% Mathematical Modelling, Simulation and Analysis of a
% 3D X-Y Cart with an Inverted Spherical Pendulum
%
% Purpose:
% 1. Inspect the complete project directory.
% 2. Inventory all project files.
% 3. Verify important models and result files.
% 4. Generate CSV and TXT inventory reports.
%
% No existing project files are modified.

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 11.1: PROJECT-WIDE FILE INVENTORY\n');
fprintf('============================================\n');

%% 1. DETERMINE PROJECT DIRECTORIES

scriptDir = fileparts(mfilename('fullpath'));

if isempty(scriptDir)
    error('Save this script inside Phase_11_Integration before running.');
end

projectRoot = fileparts(scriptDir);

fprintf('\nProject root:\n%s\n',projectRoot);

if ~isfolder(projectRoot)
    error('Project root directory does not exist.');
end

%% 2. VERIFY MAIN PROJECT FOLDERS

requiredFolders = {
    'Backups'
    'Data'
    'Documentation'
    'Final_Presentation'
    'Final_Report'
    'Phase_1_System_Definition'
    'Phase_2_Mathematical_Model'
    'Phase_3_Verification'
    'Phase_4_MATLAB_Simulation'
    'Phase_5_Animation'
    'Phase_6_Simulink'
    'Phase_7_Simscape'
    'Phase_8_Validation'
    'Phase_9_Analysis'
    'Phase_10_Control'
    'Scripts'
    'Results'
};

folderExists = false(numel(requiredFolders),1);

fprintf('\n============================================\n');
fprintf('PROJECT FOLDER STRUCTURE\n');
fprintf('============================================\n');

for k = 1:numel(requiredFolders)

    folderPath = fullfile(projectRoot,requiredFolders{k});

    folderExists(k) = isfolder(folderPath);

    if folderExists(k)
        status = 'FOUND';
    else
        status = 'MISSING';
    end

    fprintf('%-35s : %s\n', ...
        requiredFolders{k},status);

end

%% 3. RECURSIVE FILE INVENTORY

fprintf('\n============================================\n');
fprintf('SCANNING PROJECT FILES\n');
fprintf('============================================\n');

allEntries = dir(fullfile(projectRoot,'**','*'));

allEntries = allEntries(~[allEntries.isdir]);

numberOfFiles = numel(allEntries);

relativePaths = strings(numberOfFiles,1);
fileNames = strings(numberOfFiles,1);
extensions = strings(numberOfFiles,1);
fileSizesMB = zeros(numberOfFiles,1);
modifiedDates = strings(numberOfFiles,1);

for k = 1:numberOfFiles

    currentName = allEntries(k).name;
    currentFolder = allEntries(k).folder;

    fullPath = fullfile(currentFolder,currentName);

    relativePaths(k) = string( ...
        erase(fullPath,[projectRoot filesep]));

    fileNames(k) = string(currentName);

    [~,~,ext] = fileparts(currentName);

    extensions(k) = string(ext);

    fileSizesMB(k) = allEntries(k).bytes/(1024^2);

    modifiedDates(k) = string( ...
        datestr(allEntries(k).datenum, ...
        'yyyy-mm-dd HH:MM:SS'));

end

inventory = table( ...
    relativePaths, ...
    fileNames, ...
    extensions, ...
    fileSizesMB, ...
    modifiedDates, ...
    'VariableNames',{ ...
    'RelativePath', ...
    'FileName', ...
    'Extension', ...
    'Size_MB', ...
    'LastModified'});

fprintf('Total files discovered: %d\n',numberOfFiles);

totalSizeMB = sum(fileSizesMB);

fprintf('Total project file size: %.3f MB\n',totalSizeMB);

%% 4. FILE TYPE SUMMARY

fprintf('\n============================================\n');
fprintf('FILE TYPE SUMMARY\n');
fprintf('============================================\n');

uniqueExtensions = unique(extensions);

extensionCount = zeros(numel(uniqueExtensions),1);
extensionSizeMB = zeros(numel(uniqueExtensions),1);

for k = 1:numel(uniqueExtensions)

    mask = extensions == uniqueExtensions(k);

    extensionCount(k) = sum(mask);

    extensionSizeMB(k) = sum(fileSizesMB(mask));

end

extensionSummary = table( ...
    uniqueExtensions, ...
    extensionCount, ...
    extensionSizeMB, ...
    'VariableNames',{ ...
    'Extension', ...
    'NumberOfFiles', ...
    'TotalSize_MB'});

disp(extensionSummary);

%% 5. VERIFY CRITICAL PROJECT ARTIFACTS

criticalFiles = {
    'Phase_6_Simulink/inverted_spherical_pendulum_lqr.slx'
    'Phase_7_Simscape/sm_inverted_spherical_pendulum.slx'
    'Phase_7_Simscape/sm_inverted_spherical_pendulum_lqr.slx'
    'Phase_7_Simscape/sm_inverted_spherical_pendulum_pre_omega_transfer.slx'
    'Phase_10_Control/controller_data.mat'
    'Phase_10_Control/phase_10_5_baseline_results.mat'
    'Phase_10_Control/phase_10_22_fine_stabilization_boundary_results.mat'
    'Phase_10_Control/phase_10_23_stabilization_settling_verification_results.mat'
    'Phase_10_Control/phase_10_24_control_performance_assessment_results.mat'
    'Phase_10_Control/phase_10_25_final_control_validation_results.mat'
    'Phase_8_Validation/phase_8_three_model_validation.mat'
    'Results/MATLAB/cartesian_simulation_results.mat'
    'Results/Simulink/simulink_simulation_results.mat'
    'Results/Simscape/uncontrolled_simscape_results_with_omega.mat'
    'Results/Simscape/controlled_simscape_results_with_forces.mat'
    'Results/Phase_9/phase_9_final_consolidated_results.mat'
    'Phase_5_Animation/Inverted_Spherical_Pendulum_LQR_Comparison.avi'
};

criticalExists = false(numel(criticalFiles),1);
criticalSizeMB = nan(numel(criticalFiles),1);

fprintf('\n============================================\n');
fprintf('CRITICAL PROJECT ARTIFACT VERIFICATION\n');
fprintf('============================================\n');

for k = 1:numel(criticalFiles)

    currentPath = fullfile(projectRoot, ...
        strrep(criticalFiles{k},'/',filesep));

    criticalExists(k) = isfile(currentPath);

    if criticalExists(k)

        fileInfo = dir(currentPath);

        criticalSizeMB(k) = fileInfo.bytes/(1024^2);

        fprintf('[FOUND]   %s\n',criticalFiles{k});

    else

        fprintf('[MISSING] %s\n',criticalFiles{k});

    end

end

criticalTable = table( ...
    string(criticalFiles), ...
    criticalExists, ...
    criticalSizeMB, ...
    'VariableNames',{ ...
    'RelativePath', ...
    'Exists', ...
    'Size_MB'});

%% 6. PROJECT STRUCTURE SUMMARY

folderTable = table( ...
    string(requiredFolders), ...
    folderExists, ...
    'VariableNames',{ ...
    'FolderName', ...
    'Exists'});

fprintf('\n============================================\n');
fprintf('PROJECT STRUCTURE SUMMARY\n');
fprintf('============================================\n');

fprintf('Expected main folders: %d\n',numel(requiredFolders));

fprintf('Existing main folders: %d\n',sum(folderExists));

fprintf('Missing main folders: %d\n',sum(~folderExists));

fprintf('Critical artifacts expected: %d\n',numel(criticalFiles));

fprintf('Critical artifacts found: %d\n',sum(criticalExists));

fprintf('Critical artifacts missing: %d\n',sum(~criticalExists));

%% 7. GENERATE PROJECT INVENTORY REPORT

reportPath = fullfile(scriptDir, ...
    'Phase_11_1_Project_Inventory_Report.txt');

fid = fopen(reportPath,'w');

if fid == -1
    error('Unable to create project inventory report.');
end

fprintf(fid,'PHASE 11.1: PROJECT-WIDE INVENTORY REPORT\n');
fprintf(fid,'============================================\n\n');

fprintf(fid,'Project root:\n%s\n\n',projectRoot);

fprintf(fid,'TOTAL PROJECT FILES: %d\n',numberOfFiles);

fprintf(fid,'TOTAL PROJECT SIZE: %.3f MB\n\n',totalSizeMB);

fprintf(fid,'MAIN FOLDER VERIFICATION\n');
fprintf(fid,'------------------------\n');

for k = 1:numel(requiredFolders)

    fprintf(fid,'%s : %d\n', ...
        requiredFolders{k},folderExists(k));

end

fprintf(fid,'\nFILE TYPE SUMMARY\n');
fprintf(fid,'-----------------\n');

for k = 1:height(extensionSummary)

    fprintf(fid,'%s : %d files, %.3f MB\n', ...
        extensionSummary.Extension(k), ...
        extensionSummary.NumberOfFiles(k), ...
        extensionSummary.TotalSize_MB(k));

end

fprintf(fid,'\nCRITICAL ARTIFACT VERIFICATION\n');
fprintf(fid,'-----------------------------\n');

for k = 1:numel(criticalFiles)

    fprintf(fid,'%s : %d\n', ...
        criticalFiles{k},criticalExists(k));

end

fprintf(fid,'\nFINAL INVENTORY STATUS\n');
fprintf(fid,'----------------------\n');

fprintf(fid,'Existing main folders: %d / %d\n', ...
    sum(folderExists),numel(requiredFolders));

fprintf(fid,'Critical artifacts found: %d / %d\n', ...
    sum(criticalExists),numel(criticalFiles));

if all(folderExists) && all(criticalExists)

    fprintf(fid,'PROJECT INVENTORY STATUS: COMPLETE\n');

else

    fprintf(fid,'PROJECT INVENTORY STATUS: REQUIRES REVIEW\n');

end

fprintf(fid,'\nEND OF PHASE 11.1 REPORT\n');

fclose(fid);

%% 8. SAVE INVENTORY FILES

writetable(inventory,fullfile(scriptDir, ...
    'Phase_11_1_Complete_File_Inventory.csv'));

writetable(extensionSummary,fullfile(scriptDir, ...
    'Phase_11_1_FileType_Summary.csv'));

writetable(folderTable,fullfile(scriptDir, ...
    'Phase_11_1_Folder_Verification.csv'));

writetable(criticalTable,fullfile(scriptDir, ...
    'Phase_11_1_CriticalArtifact_Verification.csv'));

save(fullfile(scriptDir, ...
    'phase_11_1_project_inventory_results.mat'), ...
    'inventory', ...
    'extensionSummary', ...
    'folderTable', ...
    'criticalTable', ...
    'projectRoot', ...
    'totalSizeMB');

%% 9. FINAL SUMMARY

fprintf('\n============================================\n');
fprintf('PHASE 11.1 COMPLETED\n');
fprintf('============================================\n');

fprintf('Files inventoried: %d\n',numberOfFiles);

fprintf('Total project size: %.3f MB\n',totalSizeMB);

fprintf('Main folders found: %d / %d\n', ...
    sum(folderExists),numel(requiredFolders));

fprintf('Critical artifacts found: %d / %d\n', ...
    sum(criticalExists),numel(criticalFiles));

if all(folderExists) && all(criticalExists)

    fprintf('\nPROJECT INVENTORY: PASSED\n');

else

    fprintf('\nPROJECT INVENTORY: REVIEW REQUIRED\n');

end

fprintf('\nGenerated inventory files:\n');

fprintf('1. Phase_11_1_Complete_File_Inventory.csv\n');
fprintf('2. Phase_11_1_FileType_Summary.csv\n');
fprintf('3. Phase_11_1_Folder_Verification.csv\n');
fprintf('4. Phase_11_1_CriticalArtifact_Verification.csv\n');
fprintf('5. Phase_11_1_Project_Inventory_Report.txt\n');
fprintf('6. phase_11_1_project_inventory_results.mat\n');

fprintf('\nNo existing project files modified.\n');

fprintf('\n============================================\n');
fprintf('END OF PHASE 11.1\n');
fprintf('============================================\n');
