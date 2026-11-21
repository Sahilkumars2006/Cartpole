
%% PHASE 7.7: SPHERICAL JOINT CONFIGURATION INSPECTION
%
% Purpose:
% 1. Inspect Spherical Joint configuration parameters.
% 2. Identify initial-orientation settings.
% 3. Inspect angular position and velocity sensing.
% 4. Inspect Rigid Transform orientation settings.
% 5. Inspect rod and bob Solid parameters.
%
% This is an inspection script. It does not modify the
% existing X-Y cart model.

clear;
clc;

load_system('sm_lib');

%% BLOCK PATHS

sphericalPath = 'sm_lib/Joints/Spherical Joint';

transformPath = ...
    'sm_lib/Frames and Transforms/Rigid Transform';

cylinderPath = 'sm_lib/Body Elements/Cylindrical Solid';

spherePath = 'sm_lib/Body Elements/Spherical Solid';

%% 1. SPHERICAL JOINT PARAMETERS

fprintf('\n============================================\n');
fprintf(' SPHERICAL JOINT CONFIGURATION\n');
fprintf('============================================\n');

jointParams = get_param(sphericalPath,'DialogParameters');

jointNames = fieldnames(jointParams);

for k = 1:numel(jointNames)

    name = jointNames{k};

    relevant = contains(name,'Position') || ...
               contains(name,'Sense') || ...
               contains(name,'Actuat') || ...
               contains(name,'Torque') || ...
               contains(name,'Frame') || ...
               contains(name,'JointMode');

    if relevant

        fprintf('\n--- %s ---\n',name);

        try
            value = get_param(sphericalPath,name);
            disp(value);
        catch
            fprintf('Current value unavailable.\n');
        end

        if isfield(jointParams.(name),'Enum')

            fprintf('Available options:\n');
            disp(jointParams.(name).Enum);

        end

    end
end

%% 2. RIGID TRANSFORM PARAMETERS

fprintf('\n============================================\n');
fprintf(' RIGID TRANSFORM ORIENTATION SETTINGS\n');
fprintf('============================================\n');

transformParams = get_param(transformPath,'DialogParameters');

transformNames = fieldnames(transformParams);

for k = 1:numel(transformNames)

    name = transformNames{k};

    relevant = contains(name,'Rotation') || ...
               contains(name,'Translation');

    if relevant

        fprintf('\n--- %s ---\n',name);

        try
            value = get_param(transformPath,name);
            disp(value);
        catch
            fprintf('Current value unavailable.\n');
        end

        if isfield(transformParams.(name),'Enum')

            fprintf('Available options:\n');
            disp(transformParams.(name).Enum);

        end

    end
end

%% 3. CYLINDRICAL SOLID PARAMETERS

fprintf('\n============================================\n');
fprintf(' CYLINDRICAL SOLID PARAMETERS\n');
fprintf('============================================\n');

cylinderParams = get_param(cylinderPath,'DialogParameters');

cylinderNames = fieldnames(cylinderParams);

for k = 1:numel(cylinderNames)

    name = cylinderNames{k};

    if contains(name,'Radius') || ...
       contains(name,'Length') || ...
       contains(name,'Mass') || ...
       contains(name,'Density') || ...
       contains(name,'Inertia') || ...
       contains(name,'CenterOfMass') || ...
       contains(name,'BasedOnType')

        fprintf('\n--- %s ---\n',name);

        try
            disp(get_param(cylinderPath,name));
        catch
            fprintf('Current value unavailable.\n');
        end

        if isfield(cylinderParams.(name),'Enum')
            disp(cylinderParams.(name).Enum);
        end

    end
end

%% 4. SPHERICAL SOLID PARAMETERS

fprintf('\n============================================\n');
fprintf(' SPHERICAL SOLID PARAMETERS\n');
fprintf('============================================\n');

sphereParams = get_param(spherePath,'DialogParameters');

sphereNames = fieldnames(sphereParams);

for k = 1:numel(sphereNames)

    name = sphereNames{k};

    if contains(name,'Radius') || ...
       contains(name,'Mass') || ...
       contains(name,'Density') || ...
       contains(name,'Inertia') || ...
       contains(name,'CenterOfMass') || ...
       contains(name,'BasedOnType')

        fprintf('\n--- %s ---\n',name);

        try
            disp(get_param(spherePath,name));
        catch
            fprintf('Current value unavailable.\n');
        end

        if isfield(sphereParams.(name),'Enum')
            disp(sphereParams.(name).Enum);
        end

    end
end

%% 5. INITIAL CONDITION REFERENCE

fprintf('\n============================================\n');
fprintf(' INVERTED PENDULUM INITIAL CONDITIONS\n');
fprintf('============================================\n');

L = 1.0;
theta0 = 5.0;
phi0 = 30.0;

thetaRad = deg2rad(theta0);
phiRad = deg2rad(phi0);

bobPosition = [
    L*sin(thetaRad)*cos(phiRad);
    L*sin(thetaRad)*sin(phiRad);
    L*cos(thetaRad)
];

fprintf('Rod length       : %.4f m\n',L);
fprintf('Initial tilt     : %.4f deg\n',theta0);
fprintf('Initial azimuth  : %.4f deg\n',phi0);

fprintf('\nExpected bob position relative to pivot:\n');

fprintf('X = %.8f m\n',bobPosition(1));
fprintf('Y = %.8f m\n',bobPosition(2));
fprintf('Z = %.8f m\n',bobPosition(3));

fprintf('\nInitial velocities:\n');
fprintf('Angular velocity = 0 rad/s\n');

fprintf('\n============================================\n');
fprintf(' SPHERICAL JOINT INSPECTION COMPLETED\n');
fprintf('============================================\n');
