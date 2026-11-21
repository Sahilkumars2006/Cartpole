
%% PHASE 6: SIMULINK MODEL GENERATION
% Inverted Spherical Pendulum with Two-Axis LQR Control
%
% Plant:
%   Nonlinear Cartesian constrained dynamics
%
% Controller:
%   Two-axis LQR state feedback
%
% Plant state:
%   z = [x;y;X;Y;Z;dx;dy;dX;dY;dZ]
%
% LQR state:
%   s = [x;y;xi;eta;dx;dy;dxi;deta]
%
% where:
%   xi  = X-x
%   eta = Y-y
%
% Outputs:
%   Simulink model (.slx)
%   Simulink simulation results (.mat)

clear;
clc;
close all;

fprintf('\n============================================\n');
fprintf(' PHASE 6: SIMULINK MODEL GENERATION\n');
fprintf('============================================\n');

%% 1. LOCATE PROJECT DIRECTORIES

scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);

controlDir = fullfile(projectDir,'Phase_10_Control');

matlabResultsDir = fullfile(projectDir,'Results','MATLAB');
simulinkResultsDir = fullfile(projectDir,'Results','Simulink');

if ~exist(simulinkResultsDir,'dir')
    mkdir(simulinkResultsDir);
end

controllerFile = fullfile(controlDir,'controller_data.mat');

referenceFile = fullfile(matlabResultsDir, ...
    'controlled_simulation_results.mat');

if ~isfile(controllerFile)
    error('Controller file not found: %s',controllerFile);
end

if ~isfile(referenceFile)
    error('Controlled MATLAB results not found: %s',referenceFile);
end

%% 2. LOAD LQR CONTROLLER

controllerData = load(controllerFile,'controller');

if ~isfield(controllerData,'controller')
    error('The MAT file does not contain a controller structure.');
end

controller = controllerData.controller;

requiredControllerFields = {'K','maximum_force'};

for k = 1:numel(requiredControllerFields)

    fieldName = requiredControllerFields{k};

    if ~isfield(controller,fieldName)
        error('Controller structure is missing field: %s',fieldName);
    end

end

K = controller.K;
maximum_force = controller.maximum_force;

if ~isequal(size(K),[2 8])
    error('Expected a 2-by-8 LQR gain matrix. Found %s.', ...
        mat2str(size(K)));
end

%% 3. LOAD REFERENCE SIMULATION AND PARAMETERS

referenceData = load(referenceFile,'results');

if ~isfield(referenceData,'results')
    error('Reference file does not contain results.');
end

reference = referenceData.results;

if ~isfield(reference,'parameters')
    error('Reference results do not contain system parameters.');
end

p = reference.parameters;

requiredParameters = {'M','m','L','g', ...
                      't_end','sample_time', ...
                      'relative_tolerance','absolute_tolerance'};

for k = 1:numel(requiredParameters)

    fieldName = requiredParameters{k};

    if ~isfield(p,fieldName)
        error('Missing system parameter: %s',fieldName);
    end

end

if ~isfield(reference,'states') || ...
        size(reference.states,2) ~= 10

    error('Expected reference.states to contain 10 state columns.');

end

%% 4. INITIAL CONDITION

% State ordering:
% [x;y;X;Y;Z;dx;dy;dX;dY;dZ]

z0 = reference.states(1,:)';

% Check initial rod length
r0 = [z0(3)-z0(1);
      z0(4)-z0(2);
      z0(5)];

initialRodLength = norm(r0);

fprintf('\n--- LOADED PROJECT DATA ---\n');

fprintf('LQR gain dimensions: %d x %d\n',size(K,1),size(K,2));

fprintf('Maximum control force: %.2f N\n',maximum_force);

fprintf('Cart mass: %.3f kg\n',p.M);
fprintf('Bob mass: %.3f kg\n',p.m);
fprintf('Rod length: %.3f m\n',p.L);

fprintf('Initial rod length: %.12f m\n',initialRodLength);

if abs(initialRodLength-p.L) > 1e-8
    error('Initial state violates the rigid rod-length constraint.');
end

%% 5. PREPARE MATLAB WORKSPACE VARIABLES

assignin('base','K',K);
assignin('base','maximum_force',maximum_force);

assignin('base','M',p.M);
assignin('base','m',p.m);
assignin('base','L',p.L);
assignin('base','g',p.g);

assignin('base','z0',z0);

%% 6. CREATE SIMULINK MODEL

modelName = 'inverted_spherical_pendulum_lqr';

modelFile = fullfile(scriptDir,[modelName '.slx']);

if bdIsLoaded(modelName)
    close_system(modelName,0);
end

if isfile(modelFile)
    delete(modelFile);
end

new_system(modelName);
open_system(modelName);

%% 7. CONFIGURE SIMULATION

set_param(modelName, ...
    'Solver','ode45', ...
    'StopTime',num2str(p.t_end), ...
    'RelTol',num2str(p.relative_tolerance), ...
    'AbsTol',num2str(p.absolute_tolerance), ...
    'MaxStep',num2str(p.sample_time), ...
    'SaveTime','on', ...
    'TimeSaveName','tout', ...
    'ReturnWorkspaceOutputs','on');

%% 8. ADD INTEGRATOR

add_block('simulink/Continuous/Integrator', ...
    [modelName '/State Integrator'], ...
    'Position',[550 230 590 270], ...
    'InitialCondition','z0');

%% 9. ADD LQR CONTROLLER MATLAB FUNCTION BLOCK

controllerBlock = [modelName '/LQR Controller'];

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    controllerBlock, ...
    'Position',[280 80 450 180]);

controllerCode = sprintf([ ...
    'function u = lqr_controller(z,K,Fmax)\n' ...
    '%% Two-axis LQR controller\n' ...
    '%% Plant state: [x;y;X;Y;Z;dx;dy;dX;dY;dZ]\n' ...
    '\n' ...
    'xi = z(3)-z(1);\n' ...
    'eta = z(4)-z(2);\n' ...
    'dxi = z(8)-z(6);\n' ...
    'deta = z(9)-z(7);\n' ...
    '\n' ...
    's = [z(1); z(2); xi; eta; ...\n' ...
    '     z(6); z(7); dxi; deta];\n' ...
    '\n' ...
    'u = -K*s;\n' ...
    '\n' ...
    'u(1) = min(max(u(1),-Fmax),Fmax);\n' ...
    'u(2) = min(max(u(2),-Fmax),Fmax);\n' ...
    'end\n']);

chart = find(sfroot,'-isa','Stateflow.EMChart', ...
    'Path',controllerBlock);

if isempty(chart)
    error('Could not access the LQR MATLAB Function block.');
end

chart.Script = controllerCode;

%% 10. ADD CARTESIAN NONLINEAR PLANT BLOCK

plantBlock = [modelName '/Cartesian Nonlinear Plant'];

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    plantBlock, ...
    'Position',[650 100 880 260]);


plantCode = sprintf([ ...
    'function dz = cartesian_plant(z,Fx,Fy,M,m,L,g)\n' ...
    '%% Nonlinear Cartesian constrained pendulum dynamics\n' ...
    '\n' ...
    '%% Generalized coordinates and velocities\n' ...
    'x = z(1);\n' ...
    'y = z(2);\n' ...
    'X = z(3);\n' ...
    'Y = z(4);\n' ...
    'Z = z(5);\n' ...
    '\n' ...
    'dx = z(6);\n' ...
    'dy = z(7);\n' ...
    'dX = z(8);\n' ...
    'dY = z(9);\n' ...
    'dZ = z(10);\n' ...
    '\n' ...
    '%% Relative bob position and velocity\n' ...
    'r = [X-x; Y-y; Z];\n' ...
    'vrel = [dX-dx; dY-dy; dZ];\n' ...
    '\n' ...
    '%% Correct 1-by-5 constraint Jacobian\n' ...
    'J = [-r(1),-r(2),r(1),r(2),r(3)];\n' ...
    '\n' ...
    '%% Generalized mass matrix\n' ...
    'D = diag([M,M,m,m,m]);\n' ...
    '\n' ...
    '%% Applied generalized forces\n' ...
    'Q = [Fx;Fy;0;0;-m*g];\n' ...
    '\n' ...
    '%% Acceleration-level rod constraint\n' ...
    'constraintAcceleration = -dot(vrel,vrel);\n' ...
    '\n' ...
    '%% Lagrange multiplier system\n' ...
    'A = [D,-J'';J,0];\n' ...
    'b = [Q;constraintAcceleration];\n' ...
    'solution = A\\b;\n' ...
    '\n' ...
    '%% Generalized accelerations\n' ...
    'qdd = solution(1:5);\n' ...
    '\n' ...
    '%% Return ten state derivatives\n' ...
    'dz = [dx;dy;dX;dY;dZ;qdd];\n' ...
    'end\n']);

chart = find(sfroot,'-isa','Stateflow.EMChart', ...
    'Path',plantBlock);

if isempty(chart)
    error('Could not access the Cartesian plant MATLAB Function block.');
end

chart.Script = plantCode;

%% 11. ADD CONSTANT BLOCKS

constantNames = {'K Constant','Force Limit', ...
                 'Cart Mass','Bob Mass','Rod Length','Gravity'};

constantValues = {'K','maximum_force','M','m','L','g'};

constantPositions = [ ...
    50 80 130 110;
    50 140 130 170;
    50 230 130 260;
    50 290 130 320;
    50 350 130 380;
    50 410 130 440];

for k = 1:numel(constantNames)

    blockPath = [modelName '/' constantNames{k}];

    add_block('simulink/Sources/Constant', ...
        blockPath, ...
        'Value',constantValues{k}, ...
        'Position',constantPositions(k,:));

end

%% 12. ADD SCOPES
%%SPLIT TWO-AXIS CONTROL FORCE

add_block('simulink/Signal Routing/Demux', ...
    [modelName '/Control Force Demux'], ...
    'Outputs','2', ...
    'Position',[500 90 505 170]);


add_block('simulink/Sinks/Scope', ...
    [modelName '/State Scope'], ...
    'Position',[1030 290 1080 340]);

add_block('simulink/Sinks/Scope', ...
    [modelName '/Control Force Scope'], ...
    'Position',[650 350 700 400]);

%% 13. ADD TO WORKSPACE BLOCKS

add_block('simulink/Sinks/To Workspace', ...
    [modelName '/Save States'], ...
    'VariableName','sim_states', ...
    'SaveFormat','Timeseries', ...
    'Position',[1030 220 1160 250]);

add_block('simulink/Sinks/To Workspace', ...
    [modelName '/Save Control Forces'], ...
    'VariableName','sim_control_force', ...
    'SaveFormat','Timeseries', ...
    'Position',[850 350 980 380]);

%% 14. CONNECT INTEGRATOR TO CONTROLLER

add_line(modelName, ...
    'State Integrator/1','LQR Controller/1', ...
    'autorouting','on');

%% 15. CONNECT LQR CONSTANT INPUTS

add_line(modelName, ...
    'K Constant/1','LQR Controller/2', ...
    'autorouting','on');

add_line(modelName, ...
    'Force Limit/1','LQR Controller/3', ...
    'autorouting','on');

%% 16. CONNECT CONTROLLER TO PLANT


%%CONNECT CONTROLLER TO FORCE DEMUX

add_line(modelName, ...
    'LQR Controller/1','Control Force Demux/1', ...
    'autorouting','on');

%%CONNECT INDIVIDUAL CONTROL FORCES TO PLANT

add_line(modelName, ...
    'Control Force Demux/1', ...
    'Cartesian Nonlinear Plant/2', ...
    'autorouting','on');

add_line(modelName, ...
    'Control Force Demux/2', ...
    'Cartesian Nonlinear Plant/3', ...
    'autorouting','on');

%% 17. CONNECT PLANT STATE INPUT

add_line(modelName, ...
    'State Integrator/1','Cartesian Nonlinear Plant/1', ...
    'autorouting','on');

%% 18. CONNECT PHYSICAL PARAMETERS TO PLANT

%%CONNECT PHYSICAL PARAMETERS TO PLANT

add_line(modelName, ...
    'Cart Mass/1','Cartesian Nonlinear Plant/4', ...
    'autorouting','on');

add_line(modelName, ...
    'Bob Mass/1','Cartesian Nonlinear Plant/5', ...
    'autorouting','on');

add_line(modelName, ...
    'Rod Length/1','Cartesian Nonlinear Plant/6', ...
    'autorouting','on');

add_line(modelName, ...
    'Gravity/1','Cartesian Nonlinear Plant/7', ...
    'autorouting','on');


%% 19. CLOSE THE FEEDBACK LOOP

add_line(modelName, ...
    'Cartesian Nonlinear Plant/1','State Integrator/1', ...
    'autorouting','on');

%% 20. CONNECT DATA LOGGING

add_line(modelName, ...
    'State Integrator/1','Save States/1', ...
    'autorouting','on');

add_line(modelName, ...
    'LQR Controller/1','Save Control Forces/1', ...
    'autorouting','on');

add_line(modelName, ...
    'State Integrator/1','State Scope/1', ...
    'autorouting','on');

add_line(modelName, ...
    'LQR Controller/1','Control Force Scope/1', ...
    'autorouting','on');

%% 21. UPDATE MODEL

set_param(modelName,'SimulationCommand','update');

%% 22. SAVE MODEL

save_system(modelName,modelFile);

fprintf('\nSimulink model created successfully.\n');
fprintf('Model location:\n%s\n',modelFile);

%% 23. RUN SIMULINK SIMULATION

fprintf('\nStarting Simulink simulation...\n');

simOut = sim(modelName, ...
    'StopTime',num2str(p.t_end), ...
    'ReturnWorkspaceOutputs','on');

fprintf('Simulink simulation finished.\n');

%% 24. EXTRACT SIMULATION OUTPUTS

simStates = simOut.get('sim_states');
simForces = simOut.get('sim_control_force');

if isempty(simStates) || isempty(simForces)
    error('Simulink did not return the expected logged signals.');
end

simTime = simStates.Time(:);
simStateData = squeeze(simStates.Data);
simForceData = squeeze(simForces.Data);

% Normalize state data to N-by-10
if size(simStateData,2) ~= 10 && size(simStateData,1) == 10
    simStateData = simStateData.';
end

% Normalize force data to N-by-2
if size(simForceData,2) ~= 2 && size(simForceData,1) == 2
    simForceData = simForceData.';
end

if size(simStateData,2) ~= 10
    error('Unexpected Simulink state output dimensions: %s', ...
        mat2str(size(simStateData)));
end

if size(simForceData,2) ~= 2
    error('Unexpected Simulink control-force dimensions: %s', ...
        mat2str(size(simForceData)));
end

%% 25. VERIFY ROD-LENGTH CONSTRAINT

relativePosition = [ ...
    simStateData(:,3)-simStateData(:,1), ...
    simStateData(:,4)-simStateData(:,2), ...
    simStateData(:,5)];

rodLength = vecnorm(relativePosition,2,2);

rodError = abs(rodLength-p.L);

maximumRodError = max(rodError);

%% 26. VERIFY CONTROL FORCE LIMIT

maximumFx = max(abs(simForceData(:,1)));
maximumFy = max(abs(simForceData(:,2)));

%% 27. COMPARE AGAINST MATLAB REFERENCE

referenceTime = reference.time(:);

referenceStates = reference.states;

referenceInterpolated = interp1( ...
    referenceTime, ...
    referenceStates, ...
    simTime,'linear');

stateDifference = simStateData-referenceInterpolated;

maximumStateDifference = max(abs(stateDifference(:)));

%% 28. CALCULATE FINAL INCLINATION

finalRelative = relativePosition(end,:);

finalInclination = atan2d( ...
    hypot(finalRelative(1),finalRelative(2)), ...
    finalRelative(3));

%% 29. SAVE SIMULINK RESULTS

simulink_results.time = simTime;
simulink_results.states = simStateData;
simulink_results.control_force = simForceData;

simulink_results.rod_length = rodLength;
simulink_results.rod_length_error = rodError;

simulink_results.maximum_rod_error = maximumRodError;

simulink_results.maximum_Fx = maximumFx;
simulink_results.maximum_Fy = maximumFy;

simulink_results.final_inclination = finalInclination;

simulink_results.reference_state_difference = stateDifference;
simulink_results.maximum_state_difference = maximumStateDifference;

simulink_results.parameters = p;
simulink_results.controller = controller;

simulink_results.simulation_completed = ...
    abs(simTime(end)-p.t_end) < 1e-6;

save(fullfile(simulinkResultsDir, ...
    'simulink_simulation_results.mat'), ...
    'simulink_results');

%% 30. DISPLAY RESULTS

fprintf('\n============================================\n');
fprintf(' SIMULINK SIMULATION RESULTS\n');
fprintf('============================================\n');

fprintf('Requested duration: %.4f s\n',p.t_end);
fprintf('Actual duration: %.4f s\n',simTime(end));

fprintf('Number of samples: %d\n',numel(simTime));

fprintf('\n--- BALANCING PERFORMANCE ---\n');

fprintf('Final inclination: %.8f degrees\n', ...
    finalInclination);

fprintf('\n--- CONTROL FORCE ---\n');

fprintf('Maximum |Fx|: %.6f N\n',maximumFx);
fprintf('Maximum |Fy|: %.6f N\n',maximumFy);

fprintf('\n--- CONSTRAINT VERIFICATION ---\n');

fprintf('Maximum rod length error: %.12e m\n', ...
    maximumRodError);

fprintf('\n--- MATLAB VS SIMULINK ---\n');

fprintf('Maximum state difference: %.12e\n', ...
    maximumStateDifference);

fprintf('\n--- FILES ---\n');

fprintf('Model:\n%s\n',modelFile);

fprintf('\nResults:\n%s\n', ...
    fullfile(simulinkResultsDir, ...
    'simulink_simulation_results.mat'));

fprintf('\n============================================\n');
fprintf(' PHASE 6 SIMULATION FINISHED\n');
fprintf('============================================\n');

if simulink_results.simulation_completed
    fprintf('Simulation duration: PASS\n');
else
    fprintf('Simulation duration: CHECK\n');
end

if maximumRodError < 1e-6
    fprintf('Rod constraint: PASS\n');
else
    fprintf('Rod constraint: CHECK\n');
end

if maximumFx <= maximum_force+1e-8 && ...
        maximumFy <= maximum_force+1e-8
    fprintf('Control force limits: PASS\n');
else
    fprintf('Control force limits: FAIL\n');
end

fprintf('============================================\n');
