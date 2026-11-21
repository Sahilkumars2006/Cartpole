%% PHASE 11.2: ANIMATION ASSET RECOVERY CHECK
clc;
clear;
close all;

project_root = '/home/sahil/Documents/Multibody_CartPole/XY_Spherical_Pendulum';
results_folder = fullfile(project_root, 'Results', 'Phase_11');
if ~exist(results_folder, 'dir'), mkdir(results_folder); end

fprintf('\n============================================================\n');
fprintf('PHASE 11.2: ANIMATION ASSET RECOVERY CHECK\n');
fprintf('============================================================\n');

all_files = dir(fullfile(project_root, '**', '*'));
all_files = all_files(~[all_files.isdir]);
extensions = {'.avi', '.mp4', '.mov', '.mkv', '.mpeg', '.mpg'};
video_files = all_files([]);

for k = 1:numel(all_files)
    [~,~,ext] = fileparts(all_files(k).name);
    if any(strcmpi(ext, extensions))
        video_files(end+1) = all_files(k);
    end
end

fprintf('\nVideo files discovered: %d\n', numel(video_files));

candidate_indices = false(numel(video_files),1);
for k = 1:numel(video_files)
    filename = lower(video_files(k).name);
    foldername = lower(video_files(k).folder);
    candidate_indices(k) = contains(filename,'pendulum') || ...
        contains(filename,'comparison') || ...
        contains(filename,'animation') || ...
        contains(foldername,'phase_5') || contains(foldername,'backup');
end

candidates = video_files(candidate_indices);
fprintf('\nPotential recovery candidates: %d\n', numel(candidates));

for k = 1:numel(video_files)
    fprintf('\nFile: %s\n', video_files(k).name);
    fprintf('Folder: %s\n', video_files(k).folder);
    fprintf('Size: %.3f MB\n', video_files(k).bytes/(1024^2));
end

fprintf('\n--- CANDIDATE FILES ---\n');
for k = 1:numel(candidates)
    fprintf('\nCandidate: %s\n', candidates(k).name);
    fprintf('Location: %s\n', candidates(k).folder);
end

expected_file = fullfile(project_root, 'Phase_5_Animation', ...
    'Inverted_Spherical_Pendulum_LQR_Comparison.avi');
expected_exists = exist(expected_file,'file') == 2;

if expected_exists
    fprintf('\nOriginal animation: FOUND\n');
else
    fprintf('\nOriginal animation: MISSING\n');
end

if isempty(video_files)
    video_inventory = table(strings(0,1),strings(0,1),zeros(0,1), ...
        'VariableNames',{'FileName','Folder','Size_MB'});
else
    video_inventory = table(string({video_files.name})', ...
        string({video_files.folder})', [video_files.bytes]'/(1024^2), ...
        'VariableNames',{'FileName','Folder','Size_MB'});
end

writetable(video_inventory,fullfile(results_folder, ...
    'Phase_11_2_Video_File_Inventory.csv'));

recovery_results.project_root = project_root;
recovery_results.video_inventory = video_inventory;
recovery_results.candidates = candidates;
recovery_results.expected_file = expected_file;
recovery_results.expected_exists = expected_exists;
recovery_results.total_video_files = numel(video_files);

save(fullfile(results_folder,'phase_11_2_animation_recovery_results.mat'), ...
    'recovery_results');

fid_report = fopen(fullfile(results_folder, ...
    'Phase_11_2_Animation_Recovery_Report.txt'),'w');
fprintf(fid_report,'PHASE 11.2 ANIMATION RECOVERY REPORT\n');
fprintf(fid_report,'Video files discovered: %d\n',numel(video_files));
fprintf(fid_report,'Potential candidates: %d\n',numel(candidates));
fprintf(fid_report,'Expected animation exists: %d\n',expected_exists);
for k = 1:numel(video_files)
    fprintf(fid_report,'\n%s\n%s\n%.3f MB\n', ...
        video_files(k).name,video_files(k).folder, ...
        video_files(k).bytes/(1024^2));
end
fclose(fid_report);

fprintf('\n============================================================\n');
fprintf('PHASE 11.2 COMPLETED\n');
fprintf('Video files: %d | Candidates: %d\n', ...
    numel(video_files),numel(candidates));
fprintf('Expected animation found: %d\n',expected_exists);
fprintf('No existing project files modified.\n');
fprintf('============================================================\n');
