
%% PHASE 7.6: INSPECT X-Y JOINT CONFIGURATION

clear;
clc;

load_system('sm_lib');

jointPath = 'sm_lib/Joints/Prismatic Joint';
transformPath = 'sm_lib/Frames and Transforms/Rigid Transform';
solidPath = 'sm_lib/Body Elements/Brick Solid';

%% INSPECT PRISMATIC JOINT

fprintf('\n============================================\n');
fprintf(' PRISMATIC JOINT CONFIGURATION\n');
fprintf('============================================\n');

jointParams = get_param(jointPath,'DialogParameters');
jointNames = fieldnames(jointParams);

for k = 1:numel(jointNames)

    name = jointNames{k};

    if contains(name,'Sense') || ...
       contains(name,'Actuation') || ...
       contains(name,'JointMode')

        fprintf('\n%s\n',name);

        try
            disp(get_param(jointPath,name));
        catch
        end

        if isfield(jointParams.(name),'Enum')
            disp(jointParams.(name).Enum);
        end
    end
end

%% INSPECT RIGID TRANSFORM

fprintf('\n============================================\n');
fprintf(' RIGID TRANSFORM CONFIGURATION\n');
fprintf('============================================\n');

transformParams = get_param(transformPath,'DialogParameters');
transformNames = fieldnames(transformParams);

for k = 1:numel(transformNames)

    name = transformNames{k};

    if contains(name,'Rotation') || ...
       contains(name,'Translation')

        fprintf('\n%s\n',name);

        try
            disp(get_param(transformPath,name));
        catch
        end

        if isfield(transformParams.(name),'Enum')
            disp(transformParams.(name).Enum);
        end
    end
end

%% INSPECT BRICK SOLID

fprintf('\n============================================\n');
fprintf(' BRICK SOLID CONFIGURATION\n');
fprintf('============================================\n');

solidParams = get_param(solidPath,'DialogParameters');

solidNames = {'BasedOnType','BrickDimensions', ...
              'Mass','InertiaType','CenterOfMass'};

for k = 1:numel(solidNames)

    name = solidNames{k};

    if isfield(solidParams,name)

        fprintf('\n%s\n',name);

        try
            disp(get_param(solidPath,name));
        catch
        end

        if isfield(solidParams.(name),'Enum')
            disp(solidParams.(name).Enum);
        end
    end
end

fprintf('\n============================================\n');
fprintf(' X-Y CONFIGURATION INSPECTION COMPLETED\n');
fprintf('============================================\n');
