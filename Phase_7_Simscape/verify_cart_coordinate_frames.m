
%% ============================================================
% PHASE 7.10D: CART COORDINATE FRAME VERIFICATION
% ============================================================

clc;

model = 'sm_inverted_spherical_pendulum';

projectRoot = ...
    '/home/sahil/Documents/Multibody_CartPole/XY_Spherical_Pendulum';

if ~bdIsLoaded(model)
    load_system(fullfile(projectRoot, ...
        'Phase_7_Simscape',[model '.slx']));
end

fprintf('\n============================================\n');
fprintf(' PHASE 7.10D: COORDINATE FRAME VERIFICATION\n');
fprintf('============================================\n');

%% 1. SET ZERO INPUT FORCES

set_param([model '/Force_X'],'Value','0');
set_param([model '/Force_Y'],'Value','0');
set_param(model,'StopTime','15');

%% 2. RUN SIMULATION

fprintf('\nRunning Simscape simulation...\n');

simOut = sim(model,'ReturnWorkspaceOutputs','on');

fprintf('Simulation completed.\n');

%% 3. READ LOGGED SIGNALS

[tBob,bob] = readSignal(simOut,'bob_xyz');

[tRel,relative] = readSignal(simOut,'cart_to_bob_xyz');

[tQuat,quat] = readSignal(simOut,'cart_quaternion_data');

[tX,xJoint] = readSignal(simOut,'x_position');

[tY,yJoint] = readSignal(simOut,'y_position');

%% 4. INTERPOLATE TO COMMON TIME

t = tBob;

relative = interp1(tRel,relative,t,'linear','extrap');
quat = interp1(tQuat,quat,t,'linear','extrap');

xJoint = interp1(tX,xJoint,t,'linear','extrap');
yJoint = interp1(tY,yJoint,t,'linear','extrap');

if size(quat,2) ~= 4
    error('Cart quaternion must contain four components.');
end

if size(relative,2) ~= 3
    error('Relative position must contain three components.');
end

%% 5. RECONSTRUCT WORLD CART POSITION

% Quaternion convention: [w x y z].
% Rotation matrix maps cart-frame vectors into world coordinates.

cartWorld = zeros(numel(t),3);
relativeWorld = zeros(numel(t),3);

for k = 1:numel(t)

    q = quat(k,:);

    q = q/norm(q);

    % Scalar-first quaternion: [w x y z]

    w = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);

    % Quaternion-to-rotation-matrix conversion

    R = [ ...
        1-2*(qy^2+qz^2), 2*(qx*qy-w*qz),   2*(qx*qz+w*qy);
        2*(qx*qy+w*qz),   1-2*(qx^2+qz^2), 2*(qy*qz-w*qx);
        2*(qx*qz-w*qy),   2*(qy*qz+w*qx), 1-2*(qx^2+qy^2)];

    % Transform Sensor output is already expressed in world coordinates.
    relativeWorld(k,:) = relative(k,:);

    % Reconstruct cart origin in world coordinates.
    cartWorld(k,:) = bob(k,:) - relativeWorld(k,:);

end

%% 6. CALCULATE WORLD-FRAME PENDULUM ANGLES

relativeWorldFromCart = bob-cartWorld;

theta = atan2( ...
    hypot(relativeWorldFromCart(:,1), ...
          relativeWorldFromCart(:,2)), ...
    relativeWorldFromCart(:,3));

phi = atan2( ...
    relativeWorldFromCart(:,2), ...
    relativeWorldFromCart(:,1));

%% 7. PRINT RESULTS

fprintf('\n--- INITIAL MEASUREMENTS ---\n');

fprintf('Bob world position:\n');
disp(bob(1,:));

fprintf('Cart reconstructed world position:\n');
disp(cartWorld(1,:));

fprintf('Cart-relative bob vector in world frame:\n');
disp(relativeWorld(1,:));

fprintf('Inclination: %.6f degrees\n',rad2deg(theta(1)));
fprintf('Azimuth: %.6f degrees\n',rad2deg(phi(1)));

fprintf('\n--- FINAL MEASUREMENTS ---\n');

fprintf('Bob world position:\n');
disp(bob(end,:));

fprintf('Cart reconstructed world position:\n');
disp(cartWorld(end,:));

fprintf('Inclination: %.6f degrees\n',rad2deg(theta(end)));
fprintf('Azimuth: %.6f degrees\n',rad2deg(phi(end)));

fprintf('\n--- CART SENSOR COMPARISON ---\n');

fprintf('Final X joint position: %.8f m\n',xJoint(end));
fprintf('Final Y joint position: %.8f m\n',yJoint(end));

fprintf('\nFinal reconstructed world cart position:\n');
disp(cartWorld(end,:));

%% 8. SAVE DIAGNOSTIC RESULTS

diagnostic.time = t;
diagnostic.bob_world = bob;
diagnostic.cart_world = cartWorld;
diagnostic.relative_cart_frame = relative;
diagnostic.relative_world = relativeWorld;
diagnostic.quaternion = quat;
diagnostic.x_joint_position = xJoint;
diagnostic.y_joint_position = yJoint;
diagnostic.inclination_deg = rad2deg(theta);
diagnostic.azimuth_deg = rad2deg(unwrap(phi));

save(fullfile(projectRoot, ...
    'Results','Simscape', ...
    'coordinate_frame_verification.mat'),'diagnostic');

fprintf('\nResults saved:\n');
fprintf('Results/Simscape/coordinate_frame_verification.mat\n');

fprintf('\nPHASE 7.10D COMPLETED\n');

%% LOCAL FUNCTION

function [t,data] = readSignal(simOut,name)

    signal = simOut.get(name);

    if isempty(signal)
        error('Signal "%s" not found.',name);
    end

    if isa(signal,'timeseries')
        t = signal.Time(:);
        data = squeeze(signal.Data);
    elseif isstruct(signal) && ...
            isfield(signal,'time') && ...
            isfield(signal,'signals')
        t = signal.time(:);
        data = squeeze(signal.signals.values);
    else
        error('Unsupported signal format: %s',name);
    end

    if isvector(data) && numel(data)==numel(t)
        data = data(:);
    elseif size(data,1) ~= numel(t) && ...
            size(data,2) == numel(t)
        data = data.';
    end

    if size(data,1) ~= numel(t)
        error('Invalid signal dimensions: %s',name);
    end
end
