
%% PHASE 7.5: PRISMATIC JOINT FORCE RESPONSE AND SENSOR TEST
%
% Purpose:
% 1. Build a Simscape Multibody Prismatic Joint test model.
% 2. Configure a 2 kg test cart.
% 3. Apply a constant 1 N generalized force.
% 4. Disable gravity to isolate force response.
% 5. Enable position and velocity sensing.
% 6. Record and plot position and velocity.
% 7. Compare measured response with theoretical results.

clear;
clc;
close all;

%% 1. INITIALIZE MODEL

model = 'sm_prismatic_force_response_test';

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
fprintf(' PHASE 7.5: PRISMATIC JOINT FORCE TEST\n');
fprintf('============================================\n');

%% 2. ADD MECHANICAL BLOCKS

add_block('sm_lib/Frames and Transforms/World Frame', ...
    [model '/World'], ...
    'Position',[80 180 130 230]);

add_block('sm_lib/Utilities/Mechanism Configuration', ...
    [model '/Mechanism'], ...
    'Position',[80 320 180 370]);

add_block('nesl_utility/Solver Configuration', ...
    [model '/Solver'], ...
    'Position',[80 420 180 470]);

add_block('sm_lib/Joints/Prismatic Joint', ...
    [model '/Prismatic'], ...
    'Position',[300 180 410 270]);

add_block('sm_lib/Body Elements/Brick Solid', ...
    [model '/Cart_Test'], ...
    'Position',[540 180 650 270]);

%% 3. CONFIGURE TEST CART MASS

% Use explicitly specified mass instead of density-based mass.

set_param([model '/Cart_Test'], ...
    'BasedOnType','Mass', ...
    'Mass','2', ...
    'BrickDimensions','[1 1 1]');

fprintf('\nTest cart configuration:\n');
fprintf('Mass: 2 kg\n');
fprintf('Dimensions: 1 x 1 x 1 m\n');

%% 4. ADD FORCE INPUT BLOCKS

add_block('simulink/Sources/Constant', ...
    [model '/Force'], ...
    'Value','1', ...
    'Position',[80 60 130 100]);

add_block('nesl_utility/Simulink-PS Converter', ...
    [model '/Force_Converter'], ...
    'Position',[200 50 260 110]);

%% 5. ADD MEASUREMENT BLOCKS

add_block('nesl_utility/PS-Simulink Converter', ...
    [model '/Position_Converter'], ...
    'Position',[500 350 560 400]);

add_block('nesl_utility/PS-Simulink Converter', ...
    [model '/Velocity_Converter'], ...
    'Position',[500 450 560 500]);

add_block('simulink/Sinks/To Workspace', ...
    [model '/Position_Data'], ...
    'VariableName','joint_position', ...
    'SaveFormat','Timeseries', ...
    'Position',[650 350 760 400]);

add_block('simulink/Sinks/To Workspace', ...
    [model '/Velocity_Data'], ...
    'VariableName','joint_velocity', ...
    'SaveFormat','Timeseries', ...
    'Position',[650 450 760 500]);

%% 6. CONFIGURE PRISMATIC JOINT

set_param([model '/Prismatic'], ...
    'JointMode','Normal', ...
    'TorqueActuationMode','InputTorque', ...
    'SensePosition','on', ...
    'SenseVelocity','on');

%% 7. CONFIGURE CONVERTER UNITS

set_param([model '/Force_Converter'], ...
    'Unit','N');

set_param([model '/Position_Converter'], ...
    'Unit','m');

set_param([model '/Velocity_Converter'], ...
    'Unit','m/s');

%% 8. DISABLE GRAVITY FOR FORCE TEST

set_param([model '/Mechanism'], ...
    'GravityVector','[0 0 0]');

fprintf('\nGravity disabled for isolated force test.\n');

%% 9. GET PHYSICAL PORT HANDLES

world = get_param([model '/World'],'PortHandles');
joint = get_param([model '/Prismatic'],'PortHandles');
cart = get_param([model '/Cart_Test'],'PortHandles');

solver = get_param([model '/Solver'],'PortHandles');
mechanism = get_param([model '/Mechanism'],'PortHandles');

forceConverter = get_param( ...
    [model '/Force_Converter'],'PortHandles');

positionConverter = get_param( ...
    [model '/Position_Converter'],'PortHandles');

velocityConverter = get_param( ...
    [model '/Velocity_Converter'],'PortHandles');

positionWorkspace = get_param( ...
    [model '/Position_Data'],'PortHandles');

velocityWorkspace = get_param( ...
    [model '/Velocity_Data'],'PortHandles');

%% 10. CONNECT MECHANICAL FRAME NETWORK

add_line(model,world.RConn(1),joint.LConn(1));

add_line(model,joint.RConn(1),cart.RConn(1));

add_line(model,world.RConn(1),solver.RConn(1));

add_line(model,world.RConn(1),mechanism.RConn(1));

%% 11. CONNECT FORCE INPUT

add_line(model,'Force/1','Force_Converter/1');

add_line(model,forceConverter.RConn(1),joint.LConn(2));

%% 12. CONNECT POSITION AND VELOCITY SENSORS

joint = get_param([model '/Prismatic'],'PortHandles');

fprintf('\n--- PRISMATIC JOINT PORTS ---\n');
disp(joint);

if numel(joint.RConn) < 3
    error('Required sensing output ports were not found.');
end

% Previously tested candidate sensor mapping.
add_line(model,joint.RConn(2),positionConverter.LConn(1));

add_line(model,joint.RConn(3),velocityConverter.LConn(1));

add_line(model,positionConverter.Outport(1), ...
    positionWorkspace.Inport(1));

add_line(model,velocityConverter.Outport(1), ...
    velocityWorkspace.Inport(1));

fprintf('\nForce and measurement connections completed.\n');

%% 13. CONFIGURE SIMULATION

set_param(model, ...
    'StopTime','1', ...
    'Solver','ode15s');

%% 14. COMPILE MODEL

fprintf('\n============================================\n');
fprintf(' COMPILING MODEL\n');
fprintf('============================================\n');

try

    set_param(model,'SimulationCommand','update');

    fprintf('MODEL UPDATE: SUCCESS\n');

catch ME

    fprintf('MODEL UPDATE: FAILED\n');
    fprintf('%s\n',ME.message);

    rethrow(ME);

end

%% 15. SAVE MODEL

save_system(model, ...
    fullfile(pwd,[model '.slx']));

fprintf('\nModel saved successfully.\n');

%% 16. RUN SIMULATION

fprintf('\n============================================\n');
fprintf(' RUNNING 1 N FORCE SIMULATION\n');
fprintf('============================================\n');

try

    simOut = sim(model,'StopTime','1');

    fprintf('SIMULATION: SUCCESS\n');

catch ME

    fprintf('SIMULATION: FAILED\n');
    fprintf('%s\n',ME.message);

    rethrow(ME);

end

%% 17. EXTRACT MEASUREMENTS

positionData = simOut.get('joint_position');
velocityData = simOut.get('joint_velocity');

time = positionData.Time;

position = squeeze(positionData.Data);
velocity = squeeze(velocityData.Data);

position = position(:);
velocity = velocity(:);

%% 18. THEORETICAL RESPONSE

M = 2;
F = 1;

a_theoretical = F/M;

v_theoretical = a_theoretical*time;

x_theoretical = 0.5*a_theoretical*time.^2;

%% 19. DISPLAY RESULTS

fprintf('\n============================================\n');
fprintf(' MEASURED VS THEORETICAL RESPONSE\n');
fprintf('============================================\n');

fprintf('\n--- THEORETICAL ---\n');

fprintf('Cart mass: %.3f kg\n',M);
fprintf('Applied force: %.3f N\n',F);
fprintf('Acceleration: %.6f m/s^2\n',a_theoretical);
fprintf('Final velocity: %.6f m/s\n',v_theoretical(end));
fprintf('Final displacement: %.6f m\n',x_theoretical(end));

fprintf('\n--- SIMSCAPE MEASUREMENTS ---\n');

fprintf('Initial position: %.8f m\n',position(1));
fprintf('Final position: %.8f m\n',position(end));

fprintf('Initial velocity: %.8f m/s\n',velocity(1));
fprintf('Final velocity: %.8f m/s\n',velocity(end));

fprintf('\n--- ERRORS ---\n');

position_error = abs(position(end)-x_theoretical(end));
velocity_error = abs(velocity(end)-v_theoretical(end));

fprintf('Position error: %.8e m\n',position_error);
fprintf('Velocity error: %.8e m/s\n',velocity_error);

%% 20. PLOT POSITION COMPARISON

figure( ...
    'Name','Prismatic Joint Position Comparison', ...
    'NumberTitle','off', ...
    'Color','w');

subplot(2,1,1);

plot(time,position,'b','LineWidth',1.6);
hold on;

plot(time,x_theoretical,'r--','LineWidth',1.4);

grid on;

xlabel('Time (s)');
ylabel('Position (m)');

title('Position: Simscape vs Theory');

legend('Simscape','Theory','Location','northwest');

%% 21. PLOT VELOCITY COMPARISON

subplot(2,1,2);

plot(time,velocity,'b','LineWidth',1.6);
hold on;

plot(time,v_theoretical,'r--','LineWidth',1.4);

grid on;

xlabel('Time (s)');
ylabel('Velocity (m/s)');

title('Velocity: Simscape vs Theory');

legend('Simscape','Theory','Location','northwest');

%% 22. SAVE RESULTS

results.time = time;
results.position = position;
results.velocity = velocity;

results.theoretical_position = x_theoretical;
results.theoretical_velocity = v_theoretical;

results.cart_mass = M;
results.applied_force = F;

results.theoretical_acceleration = a_theoretical;

results.position_error = position_error;
results.velocity_error = velocity_error;

save(fullfile(pwd,'prismatic_force_test_results.mat'), ...
    'results');

fprintf('\nResults saved successfully.\n');

%% 23. FINAL SUMMARY

fprintf('\n============================================\n');
fprintf(' PHASE 7.5 TEST COMPLETED\n');
fprintf('============================================\n');

fprintf('Cart mass: 2 kg\n');
fprintf('Applied force: 1 N\n');
fprintf('Gravity: Disabled\n');
fprintf('Model compilation: PASS\n');
fprintf('Simulation: PASS\n');

fprintf('\nExpected final displacement: 0.25 m\n');
fprintf('Expected final velocity: 0.50 m/s\n');

fprintf('\nPHASE 7.5 FINISHED\n');
fprintf('============================================\n');
