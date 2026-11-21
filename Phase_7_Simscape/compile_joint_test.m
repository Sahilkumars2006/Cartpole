
%% PHASE 7.5: COMPILE PRISMATIC JOINT TEST

clear;
clc;

model = 'compile_joint_test';

if bdIsLoaded(model)
    close_system(model,0);
end

new_system(model);
load_system('sm_lib');
load_system('nesl_utility');

add_block('sm_lib/Joints/Prismatic Joint', ...
    [model '/Prismatic'], ...
    'Position',[250 150 400 250]);

add_block('nesl_utility/Solver Configuration', ...
    [model '/Solver'], ...
    'Position',[50 350 150 400]);

add_block('simulink/Sources/Constant', ...
    [model '/Force'], ...
    'Value','0', ...
    'Position',[50 150 100 190]);

add_block('nesl_utility/Simulink-PS Converter', ...
    [model '/Force_Converter'], ...
    'Position',[150 140 210 200]);

set_param([model '/Prismatic'], ...
    'JointMode','Normal', ...
    'TorqueActuationMode','InputTorque');

set_param([model '/Force_Converter'],'Unit','N');

% Connect the Simulink force signal to the converter.
add_line(model,'Force/1','Force_Converter/1');

% Connect the converter physical signal to the joint.
jointPorts = get_param([model '/Prismatic'],'PortHandles');
converterPorts = get_param([model '/Force_Converter'],'PortHandles');

fprintf('\nPrismatic joint physical ports:\n');
disp(jointPorts.LConn);

fprintf('Converter physical output:\n');
disp(converterPorts.RConn);

% Attempt compilation to inspect the configured model.
set_param(model,'StopTime','0.1');

try
    set_param(model,'SimulationCommand','update');
    fprintf('\nMODEL UPDATE: SUCCESS\n');
catch ME
    fprintf('\nMODEL UPDATE ERROR:\n%s\n',ME.message);
end

save_system(model, ...
    fullfile(pwd,'compile_joint_test.slx'));

fprintf('\nTest model saved.\n');
