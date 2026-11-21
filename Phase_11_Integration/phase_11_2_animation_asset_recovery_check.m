
%% ============================================================
% PHASE 11.2: ANIMATION ASSET RECOVERY CHECK
% =============================================================

clc;
clear;
close all;

project_root = ...
    '/home/sahil/Documents/Multibody_CartPole/XY_Spherical_Pendulum';

animation_folder = fullfile(project_root, 'Phase_5_Animation');
results_folder = fullfile(project_root, 'Results', 'Phase_11');

if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

fprintf('\n============================================================\n');
fprintf('PHASE 11.2: ANIMATION ASSET RECOVERY CHECK\n');
fprintf('============================================================\n');

%% 1. Search the complete project recursively

fprintf('\nSearching project for video files...\n');

all_files = dir(fullfile(project_root, '**', '*'));
all_files = all_files(~[all_files.isdir]);

video_extensions = {'.avi', '.mp4', '.mov', '.mkv', '.mpeg', '.mpg'};

video_files = all_files(false(size(all_files)));

for k = 1:numel(all_files)
    [~, ~, ext] = fileparts(all_files(k).name);

    if any(strcmpi(ext, video_extensions))
        video_files(end+1) = all_files(k); %#ok<SAGROW>
    end
end

fprintf('Video files discovered: %d\n', numel(video_files));

%% 2. Display all discovered videos

if ~isempty(video_files)

    fprintf('\n--- DISCOVERED VIDEO FILES ---\n');

    for k = 1:numel(video_files)

        fprintf('\nFile: %s\n', video_files(k).name);
        fprintf('Folder: %s\n', video_files(k).folder);
        fprintf('Size: %.3f MB\n', ...
            video_files(k).bytes / (1024^2));

    end

else
    fprintf('\nNo video files found in the project.\n');
end

%% 3. Search for likely animation candidates

candidate_indices = false(numel(video_files), 1);

for k = 1:numel(video_files)

    filename = lower(video_files(k).name);
    foldername = lower(video_files(k).folder);

    candidate_indices(k) = ...
        contains(filename, 'pendulum') || ...
        contains(filename, 'comparison') || ...
        contains(filename, 'animation') || ...
        contains(foldername, 'phase_5') || ...
        contains(foldername, 'backup');

end

candidates = video_files(candidate_indices);

fprintf('\n--- POTENTIAL ANIMATION RECOVERY CANDIDATES ---\n');
fprintf('Candidates found: %d\n', numel(candidates));

for k = 1:numel(candidates)

    fprintf('\nCandidate %d\n', k);
    fprintf('Name: %s\n', candidates(k).name);
    fprintf('Location: %s\n', candidates(k).folder);
    fprintf('Size: %.3f MB\n', ...
        candidates(k).bytes / (1024^2));

end

%% 4. Verify expected animation

expected_file = fullfile( ...
    animation_folder, ...
    'Inverted_Spherical_Pendulum_LQR_Comparison.avi');

expected_exists = exist(expected_file, 'file') == 2;

fprintf('\n--- EXPECTED ANIMATION VERIFICATION ---\n');

if expected_exists
    fprintf('[FOUND] Original comparison animation exists.\n');
else
    fprintf('[MISSING] Original comparison animation not found.\n');
end

%% 5. Create inventory table

if isempty(video_files)

    video_inventory = table( ...
        strings(0,1), strings(0,1), zeros(0,1), ...
        'VariableNames', {'FileName','Folder','Size_MB'});

else

    filenames = string({video_files.name})';
    folders = string({video_files.folder})';
    sizes = [video_files.bytes]' / (1024^2);

    video_inventory = table( ...
        filenames, folders, sizes, ...
        'VariableNames', {'FileName','Folder','Size_MB'});

end

%% 6. Save results

writetable(video_inventory, ...
    fullfile(results_folder, ...
    'Phase_11_2_Video_File_Inventory.csv'));

recovery_results.project_root = project_root;
recovery_results.video_inventory = video_inventory;
recovery_results.candidates = candidates;
recovery_results.expected_file = expected_file;
recovery_results.expected_exists = expected_exists;
recovery_results.total_video_files = numel(video_files);

save(fullfile(results_folder, ...
    'phase_11_2_animation_recovery_results.mat'), ...
    'recovery_results');

%% 7. Generate text report

report_file = fullfile(results_folder, ...
    'Phase_11_2_Animation_Recovery_Report.txt');

fid = fopen(report_file, 'w');

fprintf(fid, 'PHASE 11.2: ANIMATION ASSET RECOVERY REPORT\n');
fprintf(fid, '============================================\n\n');

fprintf(fid, 'Project root: %s\n', project_root);
fprintf(fid, 'Video files discovered: %d\n', numel(video_files));
fprintf(fid, 'Potential candidates: %d\n', numel(candidates));

fprintf(fid, 'Expected animation exists: %d\n\n', expected_exists);

for k = 1:numel(video_files)

    fprintf(fid, '%s\n', video_files(k).name);
    fprintf(fid, 'Location: %s\n', video_files(k).folder);
    fprintf(fid, 'Size: %.3f MB\n\n', ...
        video_files(k).bytes / (1024^2));

end

fclose(fid);

%% 8. Final status

fprintf('\n============================================================\n');
fprintf('PHASE 11.2 COMPLETED\n');
fprintf('============================================================\n');

fprintf('Video files discovered: %d\n', numel(video_files));
fprintf('Potential recovery candidates: %d\n', numel(candidates));

if expected_exists
    fprintf('Original animation: FOUND\n');
elseif ~isempty(candidates)
    fprintf('Original animation: NOT FOUND\n');
    fprintf('Potential alternative video files require inspection.\n');
else
    fprintf('Original animation: NOT FOUND\n');
    fprintf('No alternative video files discovered.\n');
end

fprintf('\nGenerated files:\n');
fprintf('1. Phase_11_2_Video_File_Inventory.csv\n');
fprintf('2. phase_11_2_animation_recovery_results.mat\n');
fprintf('3. Phase_11_2_Animation_Recovery_Report.txt\n');

fprintf('\nNo existing project files modified.\n');
fprintf('\nEND OF PHASE 11.2\n');
