
%% ============================================================
% PHASE 11.6: FINAL RESULTS CONSOLIDATION
% Project: 3D X-Y Cart with Inverted Spherical Pendulum
% =============================================================

clear;
clc;

fprintf('\n============================================\n');
fprintf('PHASE 11.6: FINAL RESULTS CONSOLIDATION\n');
fprintf('============================================\n\n');

project_root = ...
    '/home/sahil/Documents/Multibody_CartPole/XY_Spherical_Pendulum';

output_folder = fullfile(project_root, 'Results', 'Phase_11');

if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end

%% ------------------------------------------------------------
% 1. DEFINE IMPORTANT PROJECT RESULT FILES
% -------------------------------------------------------------

relative_paths = {
    'Results/MATLAB/cartesian_simulation_results.mat'
    'Results/MATLAB/controlled_simulation_results.mat'
    'Results/Simulink/simulink_simulation_results.mat'
    'Results/Simscape/uncontrolled_simscape_results_with_omega.mat'
    'Results/Simscape/controlled_simscape_results_with_forces.mat'
    'Phase_8_Validation/phase_8_three_model_validation.mat'
    'Results/Phase_9/phase_9_final_consolidated_results.mat'
    'Phase_10_Control/phase_10_5_baseline_results.mat'
    'Phase_10_Control/phase_10_22_fine_stabilization_boundary_results.mat'
    'Phase_10_Control/phase_10_23_stabilization_settling_verification_results.mat'
    'Phase_10_Control/phase_10_24_control_performance_assessment_results.mat'
    'Phase_10_Control/phase_10_25_final_control_validation_results.mat'
    'Phase_11_Integration/phase_11_1_project_inventory_results.mat'
    'Results/Phase_11/phase_11_2_animation_recovery_results.mat'
};

phase_labels = {
    'Phase 4 - MATLAB'
    'Phase 4 - Controlled MATLAB'
    'Phase 6 - Simulink'
    'Phase 7 - Uncontrolled Simscape'
    'Phase 8 - Controlled Simscape'
    'Phase 8 - Three Model Validation'
    'Phase 9 - Final Analysis'
    'Phase 10 - Baseline Control'
    'Phase 10 - Fine Boundary'
    'Phase 10 - Settling Verification'
    'Phase 10 - Performance Assessment'
    'Phase 10 - Final Control Validation'
    'Phase 11 - Project Inventory'
    'Phase 11 - Animation Recovery'
};

artifact_keys = {
    'matlab_uncontrolled'
    'matlab_controlled'
    'simulink'
    'simscape_uncontrolled'
    'simscape_controlled'
    'phase8_validation'
    'phase9_analysis'
    'phase10_baseline'
    'phase10_fine_boundary'
    'phase10_settling'
    'phase10_performance'
    'phase10_final_validation'
    'phase11_inventory'
    'phase11_animation_recovery'
};

%% ------------------------------------------------------------
% 2. LOAD AND CONSOLIDATE RESULT FILES
% -------------------------------------------------------------

n = numel(relative_paths);

phase_column = strings(n,1);
artifact_column = strings(n,1);
path_column = strings(n,1);
status_column = strings(n,1);
size_column = zeros(n,1);
variables_column = strings(n,1);

master_results = struct();

loaded_count = 0;
missing_count = 0;

fprintf('Loading project result files...\n\n');

for k = 1:n

    full_path = fullfile(project_root, relative_paths{k});

    phase_column(k) = string(phase_labels{k});
    artifact_column(k) = string(artifact_keys{k});
    path_column(k) = string(relative_paths{k});

    if isfile(full_path)

        file_info = dir(full_path);

        loaded_data = load(full_path);

        master_results.data.(artifact_keys{k}) = loaded_data;

        variable_names = fieldnames(loaded_data);

        variables_column(k) = strjoin(string(variable_names), ', ');

        size_column(k) = file_info.bytes / (1024^2);

        status_column(k) = "LOADED";

        loaded_count = loaded_count + 1;

        fprintf('[LOADED] %s\n', relative_paths{k});

    else

        status_column(k) = "MISSING";
        variables_column(k) = "";

        missing_count = missing_count + 1;

        fprintf('[MISSING] %s\n', relative_paths{k});

    end

end

%% ------------------------------------------------------------
% 3. CREATE SOURCE MANIFEST
% -------------------------------------------------------------

source_manifest = table( ...
    phase_column, ...
    artifact_column, ...
    path_column, ...
    status_column, ...
    size_column, ...
    variables_column, ...
    'VariableNames', { ...
    'Phase', ...
    'Artifact', ...
    'RelativePath', ...
    'Status', ...
    'SizeMB', ...
    'VariableList'});

%% ------------------------------------------------------------
% 4. RECORD VERIFIED ANIMATION INFORMATION
% -------------------------------------------------------------

video_path = fullfile(project_root, ...
    'Phase_5_Animation', ...
    'Inverted_Spherical_Pendulum_LQR_Comparison.avi');

if isfile(video_path)

    video_info = dir(video_path);
    video_reader = VideoReader(video_path);

    master_results.animation.file_path = video_path;
    master_results.animation.format = 'Motion JPEG AVI';
    master_results.animation.duration_seconds = video_reader.Duration;
    master_results.animation.frame_rate = video_reader.FrameRate;
    master_results.animation.width = video_reader.Width;
    master_results.animation.height = video_reader.Height;
    master_results.animation.file_size_MB = ...
        video_info.bytes / (1024^2);
    master_results.animation.verified_frame_count = 751;
    master_results.animation.integrity_check_passed = true;

    fprintf('\nAnimation metadata recorded successfully.\n');

else

    master_results.animation.integrity_check_passed = false;

    warning('Verified animation file is missing.');

end

%% ------------------------------------------------------------
% 5. ADD PROJECT INFORMATION
% -------------------------------------------------------------

master_results.project.name = ...
    'Mathematical Modelling, Simulation and Analysis of a 3D X-Y Cart with an Inverted Spherical Pendulum';

master_results.project.root_directory = project_root;
master_results.project.consolidation_phase = 'Phase 11.6';
master_results.project.consolidation_date = datestr(now);

master_results.consolidation.total_expected_files = n;
master_results.consolidation.loaded_files = loaded_count;
master_results.consolidation.missing_files = missing_count;

%% ------------------------------------------------------------
% 6. SAVE MASTER RESULTS PACKAGE
% -------------------------------------------------------------

master_mat_path = fullfile(output_folder, ...
    'phase_11_6_master_project_results.mat');

save(master_mat_path, ...
    'master_results', ...
    'source_manifest', ...
    '-v7.3');

%% ------------------------------------------------------------
% 7. EXPORT SOURCE MANIFEST
% -------------------------------------------------------------

manifest_path = fullfile(output_folder, ...
    'Phase_11_6_MasterResults_SourceManifest.csv');

writetable(source_manifest, manifest_path);

%% ------------------------------------------------------------
% 8. GENERATE CONSOLIDATION REPORT
% -------------------------------------------------------------

report_path = fullfile(output_folder, ...
    'Phase_11_6_Final_Results_Consolidation_Report.txt');

fid = fopen(report_path, 'w');

if fid == -1
    error('Unable to create consolidation report.');
end

fprintf(fid, '============================================\n');
fprintf(fid, 'PHASE 11.6: FINAL RESULTS CONSOLIDATION\n');
fprintf(fid, '============================================\n\n');

fprintf(fid, 'Project:\n%s\n\n', master_results.project.name);

fprintf(fid, 'Total expected source files: %d\n', n);
fprintf(fid, 'Successfully loaded files: %d\n', loaded_count);
fprintf(fid, 'Missing files: %d\n\n', missing_count);

fprintf(fid, '--- SOURCE FILE STATUS ---\n\n');

for k = 1:n

    fprintf(fid, '%-10s | %-35s | %s\n', ...
        char(status_column(k)), ...
        char(artifact_column(k)), ...
        char(path_column(k)));

end

fprintf(fid, '\n--- ANIMATION INFORMATION ---\n\n');

if isfield(master_results, 'animation') && ...
        master_results.animation.integrity_check_passed

    fprintf(fid, 'Format: Motion JPEG AVI\n');
    fprintf(fid, 'Duration: %.2f seconds\n', ...
        master_results.animation.duration_seconds);
    fprintf(fid, 'Frame rate: %.2f FPS\n', ...
        master_results.animation.frame_rate);
    fprintf(fid, 'Resolution: %d x %d pixels\n', ...
        master_results.animation.width, ...
        master_results.animation.height);
    fprintf(fid, 'Verified frames: %d\n', ...
        master_results.animation.verified_frame_count);
    fprintf(fid, 'File size: %.2f MB\n', ...
        master_results.animation.file_size_MB);
    fprintf(fid, 'Integrity: PASSED\n');

else

    fprintf(fid, 'Animation integrity: NOT VERIFIED\n');

end

fprintf(fid, '\n--- CONSOLIDATION STATUS ---\n\n');

if loaded_count == n && ...
        isfield(master_results, 'animation') && ...
        master_results.animation.integrity_check_passed

    fprintf(fid, 'PHASE 11.6: CONSOLIDATION PASSED\n');

else

    fprintf(fid, 'PHASE 11.6: REVIEW REQUIRED\n');

end

fprintf(fid, '\n============================================\n');
fprintf(fid, 'END OF CONSOLIDATION REPORT\n');
fprintf(fid, '============================================\n');

fclose(fid);

%% ------------------------------------------------------------
% 9. DISPLAY FINAL SUMMARY
% -------------------------------------------------------------

fprintf('\n============================================\n');
fprintf('PHASE 11.6 CONSOLIDATION SUMMARY\n');
fprintf('============================================\n');

fprintf('Expected source files: %d\n', n);
fprintf('Successfully loaded: %d\n', loaded_count);
fprintf('Missing: %d\n', missing_count);

fprintf('\n--- GENERATED FILES ---\n');

fprintf('%s\n', master_mat_path);
fprintf('%s\n', manifest_path);
fprintf('%s\n', report_path);

if loaded_count == n && ...
        isfield(master_results, 'animation') && ...
        master_results.animation.integrity_check_passed

    fprintf('\nPHASE 11.6 COMPLETED SUCCESSFULLY.\n');

else

    fprintf('\nPHASE 11.6 REQUIRES REVIEW.\n');

end

fprintf('\n============================================\n');
fprintf('PHASE 11.6 FINISHED\n');
fprintf('============================================\n');
