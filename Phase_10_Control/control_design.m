
%% LQR CONTROL DESIGN
% Two-axis inverted spherical pendulum

clear;
clc;
close all;

%% 1. Locate project

scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);

phase4Dir = fullfile(projectDir, ...
    'Phase_4_MATLAB_Simulation');

addpath(phase4Dir);

%% 2. Load physical parameters

p = system_parameters();

M = p.M;
m = p.m;
L = p.L;
g = p.g;

%% 3. Define linearized state-space model
%
% State:
% s = [x; y; xi; eta; dx; dy; dxi; deta]
%
% Input:
% u = [Fx; Fy]

A = zeros(8,8);
B = zeros(8,2);

% Position derivatives
A(1,5) = 1;
A(2,6) = 1;
A(3,7) = 1;
A(4,8) = 1;

% Cart accelerations
A(5,3) = -m*g/(M*L);
A(6,4) = -m*g/(M*L);

% Relative bob accelerations
A(7,3) = (M+m)*g/(M*L);
A(8,4) = (M+m)*g/(M*L);

% Control input matrix
B(5,1) = 1/M;
B(6,2) = 1/M;
B(7,1) = -1/M;
B(8,2) = -1/M;

%% 4. Check controllability

Co = ctrb(A,B);

controllabilityRank = rank(Co);

fprintf('\n============================================\n');
fprintf(' TWO-AXIS LQR CONTROL DESIGN\n');
fprintf('============================================\n');

fprintf('\nControllability rank: %d / 8\n', ...
    controllabilityRank);

if controllabilityRank ~= 8
    error('The linearized system is not fully controllable.');
end

fprintf('Controllability: PASS\n');

%% 5. Define LQR weighting matrices

Q = diag([ ...
    10, ...    % Cart X position
    10, ...    % Cart Y position
    500, ...   % Bob relative X displacement
    500, ...   % Bob relative Y displacement
    2, ...     % Cart X velocity
    2, ...     % Cart Y velocity
    20, ...    % Relative X velocity
    20]);      % Relative Y velocity

R = diag([0.5, 0.5]);

%% 6. Calculate LQR gain

[K, S, poles] = lqr(A,B,Q,R);

%% 7. Closed-loop stability check

closedLoopA = A - B*K;

closedLoopPoles = eig(closedLoopA);

isStable = all(real(closedLoopPoles) < 0);

fprintf('\n--- LQR RESULTS ---\n');

disp('LQR gain matrix K:');
disp(K);

disp('Closed-loop eigenvalues:');
disp(closedLoopPoles);

if isStable
    fprintf('Closed-loop stability: PASS\n');
else
    error('The closed-loop system is not asymptotically stable.');
end

%% 8. Save controller data

controller.A = A;
controller.B = B;
controller.Q = Q;
controller.R = R;
controller.K = K;
controller.S = S;
controller.poles = poles;
controller.closed_loop_poles = closedLoopPoles;
controller.controllability_rank = controllabilityRank;

controller.maximum_force = 30;

save(fullfile(scriptDir, ...
    'controller_data.mat'), 'controller');

fprintf('\nController saved successfully.\n');
fprintf('Location:\n%s\n', ...
    fullfile(scriptDir,'controller_data.mat'));

fprintf('\n============================================\n');
fprintf('       LQR DESIGN COMPLETED\n');
fprintf('============================================\n');
