
%% PHASE 7.7B: BUILD INVERTED SPHERICAL PENDULUM
%
% Creates a separate model using the validated X-Y cart.
% The original sm_xy_cart_test.slx remains unchanged.
%
% Coordinate convention:
% X-Y cart moves horizontally.
% Rod points upward along local +Z.
% Initial tilt = 5 degrees.
% Initial azimuth = 30 degrees.

clear;
clc;

fprintf('\n============================================\n');
fprintf(' PHASE 7.7B: INVERTED SPHERICAL PENDULUM\n');
fprintf('============================================\n');

%% 1. MODEL FILES

folder = fileparts(mfilename('fullpath'));

sourceFile = fullfile(folder,'sm_xy_cart_test.slx');

outputFile = fullfile(folder, ...
    'sm_inverted_spherical_pendulum.slx');

sourceModel = 'sm_xy_cart_test';
model = 'sm_inverted_spherical_pendulum';

if ~isfile(sourceFile)
    error('Original X-Y cart model not found: %s',sourceFile);
end

%% 2. CLOSE PREVIOUS COPIES

if bdIsLoaded(model)
    close_system(model,0);
end

%% 3. COPY THE VALIDATED CART MODEL

copyfile(sourceFile,outputFile,'f');

load_system(outputFile);

fprintf('\nOriginal X-Y cart model copied.\n');

%% 4. SET GRAVITY

mechanism = [model '/Mechanism'];

set_param(mechanism, ...
    'GravityVector','[0 0 -9.81]');

fprintf('Gravity configured: [0 0 -9.81] m/s^2\n');

%% 5. INITIAL ORIENTATION

theta0 = 5;
phi0 = 30;

theta = deg2rad(theta0);
phi = deg2rad(phi0);

% Rotation mapping the rod's local +Z axis
% to the desired initial direction.

Rz = [
    cos(phi), -sin(phi), 0;
    sin(phi),  cos(phi), 0;
    0,         0,        1
];

Ry = [
    cos(theta), 0, sin(theta);
    0,          1, 0;
    -sin(theta),0, cos(theta)
];

R0 = Rz * Ry;

fprintf('\nInitial rotation matrix:\n');
disp(R0);

%% 6. ADD SPHERICAL JOINT

jointPath = [model '/Spherical_Joint'];

add_block( ...
    'sm_lib/Joints/Spherical Joint', ...
    jointPath, ...
    'Position',[1260 160 1330 230]);

set_param(jointPath, ...
    'JointMode','Normal', ...
    'PositionTargetSpecify','on', ...
    'PositionTargetPriority','High', ...
    'PositionTargetRotationMethod','RotationMatrix', ...
    'PositionTargetRotationMatrix',mat2str(R0,15), ...
    'TorqueActuationMode','NoTorque');

fprintf('Spherical Joint configured.\n');

%% 7. ADD ROD OFFSET TRANSFORM

rodTransform = [model '/Rod_Centre_Transform'];

add_block( ...
    'sm_lib/Frames and Transforms/Rigid Transform', ...
    rodTransform, ...
    'Position',[1390 100 1480 170]);

set_param(rodTransform, ...
    'TranslationMethod','Cartesian', ...
    'TranslationCartesianOffset','[0 0 0.5]', ...
    'RotationMethod','None');

%% 8. ADD CYLINDRICAL ROD

rod = [model '/Pendulum_Rod'];

add_block( ...
    'sm_lib/Body Elements/Cylindrical Solid', ...
    rod, ...
    'Position',[1540 100 1630 170]);

set_param(rod, ...
    'BasedOnType','Mass', ...
    'Mass','0.001', ...
    'CylinderRadius','0.01', ...
    'CylinderLength','1.0', ...
    'InertiaType','CalculateFromGeometry');

fprintf('Rod configured: 1 m, 0.001 kg.\n');

%% 9. ADD BOB OFFSET TRANSFORM

bobTransform = [model '/Bob_Centre_Transform'];

add_block( ...
    'sm_lib/Frames and Transforms/Rigid Transform', ...
    bobTransform, ...
    'Position',[1390 290 1480 360]);

set_param(bobTransform, ...
    'TranslationMethod','Cartesian', ...
    'TranslationCartesianOffset','[0 0 1.0]', ...
    'RotationMethod','None');

%% 10. ADD SPHERICAL BOB

bob = [model '/Pendulum_Bob'];

add_block( ...
    'sm_lib/Body Elements/Spherical Solid', ...
    bob, ...
    'Position',[1540 290 1630 360]);

set_param(bob, ...
    'BasedOnType','Mass', ...
    'Mass','0.5', ...
    'SphereRadius','0.05', ...
    'InertiaType','CalculateFromGeometry');

fprintf('Bob configured: 0.5 kg, radius 0.05 m.\n');

%% 11. GET PHYSICAL FRAME PORTS

cartPorts = get_param([model '/Main_Cart'],'PortHandles');

jointPorts = get_param(jointPath,'PortHandles');

rodTransformPorts = get_param(rodTransform,'PortHandles');

rodPorts = get_param(rod,'PortHandles');

bobTransformPorts = get_param(bobTransform,'PortHandles');

bobPorts = get_param(bob,'PortHandles');

%% 12. CONNECT CART TO SPHERICAL JOINT

add_line(model, ...
    cartPorts.RConn(1), ...
    jointPorts.LConn(1), ...
    'autorouting','on');

%% 13. CONNECT SPHERICAL JOINT TO ROD

add_line(model, ...
    jointPorts.RConn(1), ...
    rodTransformPorts.LConn(1), ...
    'autorouting','on');

add_line(model, ...
    rodTransformPorts.RConn(1), ...
    rodPorts.RConn(1), ...
    'autorouting','on');

%% 14. CONNECT SPHERICAL JOINT TO BOB

% The bob has its own transform from the spherical
% joint follower frame. Its centre is 1 m from pivot.

add_line(model, ...
    jointPorts.RConn(1), ...
    bobTransformPorts.LConn(1), ...
    'autorouting','on');

add_line(model, ...
    bobTransformPorts.RConn(1), ...
    bobPorts.RConn(1), ...
    'autorouting','on');

fprintf('\nMechanical connections completed.\n');

%% 15. CALCULATE EXPECTED INITIAL BOB POSITION

L = 1.0;

bobPosition = [
    L*sin(theta)*cos(phi);
    L*sin(theta)*sin(phi);
    L*cos(theta)
];

fprintf('\n============================================\n');
fprintf(' EXPECTED INITIAL BOB POSITION\n');
fprintf('============================================\n');

fprintf('X: %.8f m\n',bobPosition(1));
fprintf('Y: %.8f m\n',bobPosition(2));
fprintf('Z: %.8f m\n',bobPosition(3));

fprintf('Rod length: %.4f m\n',L);
fprintf('Tilt: %.4f degrees\n',theta0);
fprintf('Azimuth: %.4f degrees\n',phi0);

%% 16. UPDATE AND COMPILE MODEL

fprintf('\n============================================\n');
fprintf(' COMPILING SIMSCAPE MODEL\n');
fprintf('============================================\n');

set_param(model,'SimulationCommand','update');

fprintf('MODEL UPDATE: SUCCESS\n');

%% 17. SAVE MODEL

save_system(model,outputFile);

fprintf('\n============================================\n');
fprintf(' PHASE 7.7B BUILD COMPLETED\n');
fprintf('============================================\n');

fprintf('Model: %s\n',outputFile);

fprintf('\nNext step: initial geometry and gravity validation.\n');
