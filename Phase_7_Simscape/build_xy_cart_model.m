
%% PHASE 7.6: BUILD AND TEST X-Y CART ASSEMBLY
%
% Mechanical architecture:
% World -> X Prismatic Joint -> Intermediate Carriage
%       -> Y-axis Rigid Transform -> Y Prismatic Joint
%       -> Main Cart
%
% X joint axis: global X
% Y joint axis: global Y
%
% Total moving cart assembly mass: 2 kg
% Intermediate carriage mass: 0.001 kg
% Main cart mass: 1.999 kg
%
% Test:
% Fx = 1 N
% Fy = 1 N
% Gravity disabled for isolated axis validation.

clear;
clc;
close all;

%% 1. INITIALIZE

model = 'sm_xy_cart_test';

if bdIsLoaded(model)
    close_system(model,0);
end

if exist([model '.slx'],'file')
    delete([model '.slx']);
end

new_system(model);

load_system('sm_lib');
load_system('nesl_utility');

fprintf('\n============================================\n');
fprintf(' PHASE 7.6: X-Y CART ASSEMBLY\n');
fprintf('============================================\n');

%% 2. ADD FRAME AND CONFIGURATION BLOCKS

add_block('sm_lib/Frames and Transforms/World Frame', ...
    [model '/World'], ...
    'Position',[50 200 100 250]);

add_block('sm_lib/Utilities/Mechanism Configuration', ...
    [model '/Mechanism'], ...
    'Position',[50 350 160 400]);

add_block('nesl_utility/Solver Configuration', ...
    [model '/Solver'], ...
    'Position',[50 450 160 500]);

%% 3. ADD X-AXIS ROTATION TRANSFORM

add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
    [model '/X_Axis_Orientation'], ...
    'Position',[180 190 280 250]);

% Rotate local Z axis into global X.
set_param([model '/X_Axis_Orientation'], ...
    'RotationMethod','StandardAxis', ...
    'RotationStandardAxis','+Y', ...
    'RotationAngle','90');

%% 4. ADD X PRISMATIC JOINT

add_block('sm_lib/Joints/Prismatic Joint', ...
    [model '/X_Prismatic'], ...
    'Position',[350 180 460 270]);

set_param([model '/X_Prismatic'], ...
    'JointMode','Normal', ...
    'TorqueActuationMode','InputTorque', ...
    'SensePosition','on', ...
    'SenseVelocity','on');

%% 5. ADD INTERMEDIATE CARRIAGE

add_block('sm_lib/Body Elements/Brick Solid', ...
    [model '/Intermediate_Carriage'], ...
    'Position',[530 180 650 270]);

set_param([model '/Intermediate_Carriage'], ...
    'BasedOnType','Mass', ...
    'Mass','0.001', ...
    'BrickDimensions','[0.1 0.1 0.02]');

%% 6. ADD Y-AXIS ROTATION TRANSFORM

add_block('sm_lib/Frames and Transforms/Rigid Transform', ...
    [model '/Y_Axis_Orientation'], ...
    'Position',[710 180 810 250]);

% Rotate local Z axis into global Y.
set_param([model '/Y_Axis_Orientation'], ...
    'RotationMethod','StandardAxis', ...
    'RotationStandardAxis','+X', ...
    'RotationAngle','-90');

%% 7. ADD Y PRISMATIC JOINT

add_block('sm_lib/Joints/Prismatic Joint', ...
    [model '/Y_Prismatic'], ...
    'Position',[870 180 980 270]);

set_param([model '/Y_Prismatic'], ...
    'JointMode','Normal', ...
    'TorqueActuationMode','InputTorque', ...
    'SensePosition','on', ...
    'SenseVelocity','on');

%% 8. ADD MAIN CART

add_block('sm_lib/Body Elements/Brick Solid', ...
    [model '/Main_Cart'], ...
    'Position',[1060 180 1180 270]);

set_param([model '/Main_Cart'], ...
    'BasedOnType','Mass', ...
    'Mass','1.999', ...
    'BrickDimensions','[0.3 0.3 0.08]');

%% 9. ADD FORCE INPUTS

add_block('simulink/Sources/Constant', ...
    [model '/Force_X'], ...
    'Value','1', ...
    'Position',[250 40 300 80]);

add_block('simulink/Sources/Constant', ...
    [model '/Force_Y'], ...
    'Value','1', ...
    'Position',[780 40 830 80]);

add_block('nesl_utility/Simulink-PS Converter', ...
    [model '/Force_X_Converter'], ...
    'Position',[350 30 410 90]);

add_block('nesl_utility/Simulink-PS Converter', ...
    [model '/Force_Y_Converter'], ...
    'Position',[880 30 940 90]);

set_param([model '/Force_X_Converter'],'Unit','N');
set_param([model '/Force_Y_Converter'],'Unit','N');

%% 10. ADD SENSOR CONVERTERS

% X position and velocity

add_block('nesl_utility/PS-Simulink Converter', ...
    [model '/X_Position_Converter'], ...
    'Position',[480 350 550 400]);

add_block('nesl_utility/PS-Simulink Converter', ...
    [model '/X_Velocity_Converter'], ...
    'Position',[480 430 550 480]);

% Y position and velocity

add_block('nesl_utility/PS-Simulink Converter', ...
    [model '/Y_Position_Converter'], ...
    'Position',[1000 350 1070 400]);

add_block('nesl_utility/PS-Simulink Converter', ...
    [model '/Y_Velocity_Converter'], ...
    'Position',[1000 430 1070 480]);

set_param([model '/X_Position_Converter'],'Unit','m');
set_param([model '/X_Velocity_Converter'],'Unit','m/s');
set_param([model '/Y_Position_Converter'],'Unit','m');
set_param([model '/Y_Velocity_Converter'],'Unit','m/s');

%% 11. ADD TO WORKSPACE BLOCKS

workspaceNames = { ...
    'x_position', ...
    'x_velocity', ...
    'y_position', ...
    'y_velocity'};

workspacePositions = { ...
    [600 340 710 390], ...
    [600 420 710 470], ...
    [1120 340 1230 390], ...
    [1120 420 1230 470]};

for k = 1:4

    blockName = ['Workspace_' num2str(k)];

    add_block('simulink/Sinks/To Workspace', ...
        [model '/' blockName], ...
        'VariableName',workspaceNames{k}, ...
        'SaveFormat','Timeseries', ...
        'Position',workspacePositions{k});

end

%% 12. DISABLE GRAVITY FOR INITIAL AXIS TEST

set_param([model '/Mechanism'], ...
    'GravityVector','[0 0 0]');

%% 13. GET PHYSICAL PORT HANDLES

world = get_param([model '/World'],'PortHandles');
mechanism = get_param([model '/Mechanism'],'PortHandles');
solver = get_param([model '/Solver'],'PortHandles');

transformX = get_param( ...
    [model '/X_Axis_Orientation'],'PortHandles');

jointX = get_param( ...
    [model '/X_Prismatic'],'PortHandles');

carriage = get_param( ...
    [model '/Intermediate_Carriage'],'PortHandles');

transformY = get_param( ...
    [model '/Y_Axis_Orientation'],'PortHandles');

jointY = get_param( ...
    [model '/Y_Prismatic'],'PortHandles');

cart = get_param([model '/Main_Cart'],'PortHandles');

forceX = get_param( ...
    [model '/Force_X_Converter'],'PortHandles');

forceY = get_param( ...
    [model '/Force_Y_Converter'],'PortHandles');

%% 14. CONNECT WORLD AND CONFIGURATION

add_line(model,world.RConn(1),transformX.LConn(1));

add_line(model,world.RConn(1),solver.RConn(1));

add_line(model,world.RConn(1),mechanism.RConn(1));

%% 15. CONNECT X MECHANICAL CHAIN

add_line(model,transformX.RConn(1),jointX.LConn(1));

add_line(model,jointX.RConn(1),carriage.RConn(1));

%% 16. CONNECT Y MECHANICAL CHAIN

add_line(model,carriage.RConn(1),transformY.LConn(1));

add_line(model,transformY.RConn(1),jointY.LConn(1));

add_line(model,jointY.RConn(1),cart.RConn(1));

%% 17. CONNECT FORCE ACTUATORS

add_line(model,'Force_X/1','Force_X_Converter/1');

add_line(model,'Force_Y/1','Force_Y_Converter/1');

add_line(model,forceX.RConn(1),jointX.LConn(2));

add_line(model,forceY.RConn(1),jointY.LConn(2));

%% 18. CONNECT SENSOR OUTPUTS

jointX = get_param([model '/X_Prismatic'],'PortHandles');
jointY = get_param([model '/Y_Prismatic'],'PortHandles');

if numel(jointX.RConn) < 3 || numel(jointY.RConn) < 3
    error('Expected position and velocity sensor ports were not found.');
end

% X measurements

xPos = get_param([model '/X_Position_Converter'],'PortHandles');
xVel = get_param([model '/X_Velocity_Converter'],'PortHandles');

add_line(model,jointX.RConn(2),xPos.LConn(1));
add_line(model,jointX.RConn(3),xVel.LConn(1));

add_line(model,xPos.Outport(1), ...
    get_param([model '/Workspace_1'],'PortHandles').Inport(1));

add_line(model,xVel.Outport(1), ...
    get_param([model '/Workspace_2'],'PortHandles').Inport(1));

% Y measurements

yPos = get_param([model '/Y_Position_Converter'],'PortHandles');
yVel = get_param([model '/Y_Velocity_Converter'],'PortHandles');

add_line(model,jointY.RConn(2),yPos.LConn(1));
add_line(model,jointY.RConn(3),yVel.LConn(1));

add_line(model,yPos.Outport(1), ...
    get_param([model '/Workspace_3'],'PortHandles').Inport(1));

add_line(model,yVel.Outport(1), ...
    get_param([model '/Workspace_4'],'PortHandles').Inport(1));

fprintf('\nMechanical and sensor connections completed.\n');

%% 19. SIMULATION SETTINGS

set_param(model, ...
    'StopTime','1', ...
    'Solver','ode15s');

%% 20. COMPILE MODEL

fprintf('\n============================================\n');
fprintf(' COMPILING X-Y CART MODEL\n');
fprintf('============================================\n');

set_param(model,'SimulationCommand','update');

fprintf('MODEL UPDATE: SUCCESS\n');

%% 21. SAVE MODEL

save_system(model,fullfile(pwd,[model '.slx']));

%% 22. RUN SIMULATION

fprintf('\nRunning X-Y force test...\n');

simOut = sim(model,'StopTime','1');

fprintf('SIMULATION: SUCCESS\n');

%% 23. EXTRACT RESULTS

xPosData = simOut.get('x_position');
xVelData = simOut.get('x_velocity');

yPosData = simOut.get('y_position');
yVelData = simOut.get('y_velocity');

t = xPosData.Time;

x = squeeze(xPosData.Data);
xd = squeeze(xVelData.Data);

y = squeeze(yPosData.Data);
yd = squeeze(yVelData.Data);

x = x(:);
xd = xd(:);
y = y(:);
yd = yd(:);

%% 24. DISPLAY RESULTS

fprintf('\n============================================\n');
fprintf(' X-Y CART RESPONSE\n');
fprintf('============================================\n');

fprintf('\nX AXIS\n');
fprintf('Final position: %.8f m\n',x(end));
fprintf('Final velocity: %.8f m/s\n',xd(end));

fprintf('\nY AXIS\n');
fprintf('Final position: %.8f m\n',y(end));
fprintf('Final velocity: %.8f m/s\n',yd(end));

fprintf('\nExpected approximate response:\n');
fprintf('X final position: 0.25 m\n');
fprintf('X final velocity: 0.50 m/s\n');
fprintf('Y final position: 0.250125 m\n');
fprintf('Y final velocity: 0.500250 m/s\n');

%% 25. PLOT RESULTS

figure('Name','X-Y Cart Response','Color','w');

subplot(2,2,1);
plot(t,x,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('X Position (m)');
title('X Position');

subplot(2,2,2);
plot(t,xd,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('X Velocity (m/s)');
title('X Velocity');

subplot(2,2,3);
plot(t,y,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Y Position (m)');
title('Y Position');

subplot(2,2,4);
plot(t,yd,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Y Velocity (m/s)');
title('Y Velocity');

%% 26. SAVE RESULTS

results.time = t;

results.x_position = x;
results.x_velocity = xd;

results.y_position = y;
results.y_velocity = yd;

results.force_x = 1;
results.force_y = 1;

results.intermediate_mass = 0.001;
results.main_cart_mass = 1.999;
results.total_cart_mass = 2;

save(fullfile(pwd,'xy_cart_test_results.mat'),'results');

fprintf('\nResults saved successfully.\n');

fprintf('\n============================================\n');
fprintf(' PHASE 7.6 TEST COMPLETED\n');
fprintf('============================================\n');
