
%% PHASE 7.4: TEST PRISMATIC JOINT ACTUATION PORTS

clear;
clc;

model = 'test_prismatic_actuation';

if bdIsLoaded(model)
    close_system(model,0);
end

new_system(model);
load_system('sm_lib');

add_block( ...
    'sm_lib/Joints/Prismatic Joint', ...
    [model '/Test_Prismatic'], ...
    'Position',[200 150 350 250]);

joint = [model '/Test_Prismatic'];

fprintf('\n--- DEFAULT CONFIGURATION ---\n');
disp(get_param(joint,'TorqueActuationMode'));

%% Enable force input

set_param(joint, ...
    'JointMode','Normal', ...
    'TorqueActuationMode','InputTorque');

fprintf('\n--- CONFIGURED ACTUATION ---\n');
disp(get_param(joint,'TorqueActuationMode'));

%% Inspect ports

ports = get_param(joint,'PortHandles');

fprintf('\n--- PORT COUNTS AFTER ACTUATION ---\n');

names = fieldnames(ports);

for k = 1:numel(names)

    handles = ports.(names{k});

    if ~isempty(handles)
        fprintf('%s: %d\n',names{k},numel(handles));
    end
end

%% Inspect block port connectivity information

try
    ph = get_param(joint,'PortHandles');

    for k = 1:numel(ph.LConn)
        fprintf('Physical left port handle: %d\n',ph.LConn(k));
    end

    for k = 1:numel(ph.RConn)
        fprintf('Physical right port handle: %d\n',ph.RConn(k));
    end
catch ME
    fprintf('Port inspection warning: %s\n',ME.message);
end

fprintf('\nActuation port test completed.\n');

close_system(model,0);
