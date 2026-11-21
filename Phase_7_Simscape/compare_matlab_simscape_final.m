
%% ============================================================
% PHASE 7.10O: FINAL MATLAB-SIMSCAPE DYNAMIC VALIDATION
% =============================================================

clc;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 7.10O: FINAL DYNAMIC VALIDATION\n');
fprintf('============================================\n');

%% 1. LOAD RESULTS

projectRoot = ...
    '/home/sahil/Documents/Multibody_CartPole/XY_Spherical_Pendulum';

matlabFile = fullfile(projectRoot, ...
    'Results/MATLAB/cartesian_simulation_results.mat');

simscapeFile = fullfile(projectRoot, ...
    'Results/Simscape/uncontrolled_simscape_results_with_omega.mat');

M = load(matlabFile);
S = load(simscapeFile);

R = M.results;
simOut = S.simOut;

%% 2. MATLAB DATA

tM = R.time(:);

matlabCartPosition = R.cart_position;
matlabBobPosition = R.bob_position;

matlabCartVelocity = R.cart_velocity;
matlabBobVelocity = R.bob_velocity;

matlabTheta = R.theta(:);

% Reconstruct cart and bob relative positions
matlabCartXYZ = [matlabCartPosition, zeros(length(tM),1)];

matlabRelativePosition = ...
    matlabBobPosition - matlabCartXYZ;

matlabPhi = atan2( ...
    matlabRelativePosition(:,2), ...
    matlabRelativePosition(:,1));

%% 3. EXTRACT SIMSCAPE DATA

getVector = @(ts) ...
    squeeze(permute(ts.Data,[3 1 2]));

bobXYZ = getVector(simOut.get('bob_xyz'));
relativeXYZ = getVector(simOut.get('cart_to_bob_xyz'));

xPosition = simOut.get('x_position');
yPosition = simOut.get('y_position');

xVelocity = simOut.get('x_velocity');
yVelocity = simOut.get('y_velocity');

bobOmega = getVector(S.bobOmega);

tS = simOut.get('bob_xyz').Time(:);

cartX = xPosition.Data(:);
cartY = yPosition.Data(:);

cartVX = xVelocity.Data(:);
cartVY = yVelocity.Data(:);

cartPosition = [cartX, cartY, zeros(length(tS),1)];
cartVelocity = [cartVX, cartVY, zeros(length(tS),1)];

%% 4. INTERPOLATE SIMSCAPE DATA ONTO MATLAB TIME

simBobPosition = interp1(tS,bobXYZ,tM,'pchip');
simRelativePosition = interp1(tS,relativeXYZ,tM,'pchip');

simCartPosition = interp1(tS,cartPosition,tM,'pchip');
simCartVelocity = interp1(tS,cartVelocity,tM,'pchip');

simOmega = interp1(tS,bobOmega,tM,'pchip');

% Simscape pendulum angles
simTheta = acos(max(-1,min(1, ...
    simRelativePosition(:,3) ./ ...
    vecnorm(simRelativePosition,2,2))));

simPhi = atan2( ...
    simRelativePosition(:,2), ...
    simRelativePosition(:,1));

%% 5. POSITION COMPARISON

cartPositionError = ...
    matlabCartPosition - simCartPosition(:,1:2);

bobPositionError = ...
    matlabBobPosition - simBobPosition;

relativePositionError = ...
    matlabRelativePosition - simRelativePosition;

%% 6. ANGLE COMPARISON

thetaError = matlabTheta - simTheta;

% Wrap azimuth difference to [-pi, pi]
phiError = atan2( ...
    sin(matlabPhi-simPhi), ...
    cos(matlabPhi-simPhi));

thetaErrorDeg = rad2deg(thetaError);
phiErrorDeg = rad2deg(phiError);

%% 7. VELOCITY COMPARISON

simBobVelocity = simCartVelocity + ...
    cross(simOmega,simRelativePosition,2);

cartVelocityError = ...
    matlabCartVelocity - simCartVelocity(:,1:2);

bobVelocityError = ...
    matlabBobVelocity - simBobVelocity;

%% 8. ANGULAR VELOCITY COMPARISON

% Angular velocity associated with the pendulum direction.
% Axial spin about the rod does not change the bob position.

matlabOmega = cross( ...
    matlabRelativePosition, ...
    matlabBobVelocity - ...
    [matlabCartVelocity,zeros(length(tM),1)],2);

matlabOmega = matlabOmega ./ ...
    vecnorm(matlabRelativePosition,2,2).^2;

% Remove any Simscape angular velocity component along the rod.
rodDirection = simRelativePosition ./ ...
    vecnorm(simRelativePosition,2,2);

simOmegaTransverse = simOmega - ...
    sum(simOmega.*rodDirection,2).*rodDirection;

omegaError = matlabOmega - simOmegaTransverse;

%% 9. ERROR METRICS

fullWindow = true(size(tM));
shortWindow = tM <= 3;

fprintf('\n============================================\n');
fprintf('FULL 15-SECOND COMPARISON\n');
fprintf('============================================\n');

printMetrics('Cart position',cartPositionError,fullWindow);
printMetrics('Bob absolute position',bobPositionError,fullWindow);
printMetrics('Pendulum relative position',relativePositionError,fullWindow);
printMetrics('Cart velocity',cartVelocityError,fullWindow);
printMetrics('Bob velocity',bobVelocityError,fullWindow);

printMetrics('Angular velocity',omegaError,fullWindow);

fprintf('\nANGLE ERRORS\n');
fprintf('Theta RMS error: %.6f deg\n', ...
    rms(thetaErrorDeg(fullWindow)));

fprintf('Theta maximum error: %.6f deg\n', ...
    max(abs(thetaErrorDeg(fullWindow))));

fprintf('Phi RMS error: %.6f deg\n', ...
    rms(phiErrorDeg(fullWindow)));

fprintf('Phi maximum error: %.6f deg\n', ...
    max(abs(phiErrorDeg(fullWindow))));

fprintf('\n============================================\n');
fprintf('SHORT-TERM 0-3 SECOND COMPARISON\n');
fprintf('============================================\n');

printMetrics('Cart position',cartPositionError,shortWindow);
printMetrics('Bob absolute position',bobPositionError,shortWindow);
printMetrics('Pendulum relative position',relativePositionError,shortWindow);
printMetrics('Angular velocity',omegaError,shortWindow);

fprintf('\nANGLE ERRORS\n');
fprintf('Theta RMS error: %.6f deg\n', ...
    rms(thetaErrorDeg(shortWindow)));

fprintf('Phi RMS error: %.6f deg\n', ...
    rms(phiErrorDeg(shortWindow)));

%% 10. PLOTS

figure('Name','Cart Position Comparison');

subplot(2,1,1);
plot(tM,matlabCartPosition(:,1),'LineWidth',1.4);
hold on;
plot(tM,simCartPosition(:,1),'--','LineWidth',1.4);
grid on;
ylabel('X position (m)');
legend('MATLAB','Simscape');

subplot(2,1,2);
plot(tM,matlabCartPosition(:,2),'LineWidth',1.4);
hold on;
plot(tM,simCartPosition(:,2),'--','LineWidth',1.4);
grid on;
xlabel('Time (s)');
ylabel('Y position (m)');
legend('MATLAB','Simscape');

figure('Name','Bob Position Comparison');

for k = 1:3
    subplot(3,1,k);
    plot(tM,matlabBobPosition(:,k),'LineWidth',1.4);
    hold on;
    plot(tM,simBobPosition(:,k),'--','LineWidth',1.4);
    grid on;
    ylabel(['Position ',char('X'+k-1),' (m)']);
    legend('MATLAB','Simscape');
end
xlabel('Time (s)');

figure('Name','Pendulum Angle Comparison');

subplot(2,1,1);
plot(tM,rad2deg(matlabTheta),'LineWidth',1.4);
hold on;
plot(tM,rad2deg(simTheta),'--','LineWidth',1.4);
grid on;
ylabel('Inclination (deg)');
legend('MATLAB','Simscape');

subplot(2,1,2);
plot(tM,rad2deg(unwrap(matlabPhi)),'LineWidth',1.4);
hold on;
plot(tM,rad2deg(unwrap(simPhi)),'--','LineWidth',1.4);
grid on;
xlabel('Time (s)');
ylabel('Azimuth (deg)');
legend('MATLAB','Simscape');

figure('Name','Angular Velocity Comparison');

for k = 1:3
    subplot(3,1,k);
    plot(tM,matlabOmega(:,k),'LineWidth',1.4);
    hold on;
    plot(tM,simOmegaTransverse(:,k),'--','LineWidth',1.4);
    grid on;
    ylabel(sprintf('\\omega_%d (rad/s)',k));
    legend('MATLAB','Simscape');
end
xlabel('Time (s)');

figure('Name','Position Error');

plot(tM,vecnorm(relativePositionError,2,2),'LineWidth',1.4);
grid on;
xlabel('Time (s)');
ylabel('Relative position error (m)');
title('MATLAB-Simscape Pendulum Position Error');

%% 11. SAVE RESULTS

validation.time = tM;

validation.cart_position_error = cartPositionError;
validation.bob_position_error = bobPositionError;
validation.relative_position_error = relativePositionError;

validation.cart_velocity_error = cartVelocityError;
validation.bob_velocity_error = bobVelocityError;

validation.theta_error_deg = thetaErrorDeg;
validation.phi_error_deg = phiErrorDeg;

validation.matlab_angular_velocity = matlabOmega;
validation.simscape_angular_velocity = simOmegaTransverse;
validation.angular_velocity_error = omegaError;

validation.short_window = shortWindow;

outputFolder = fullfile(projectRoot,'Results','Comparisons');

if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end

save(fullfile(outputFolder, ...
    'phase_7_10O_final_validation.mat'),'validation');

fprintf('\n============================================\n');
fprintf('PHASE 7.10O COMPARISON COMPLETED\n');
fprintf('============================================\n');

fprintf('Validation results saved successfully.\n');
fprintf('Location: %s\n',outputFolder);

%% LOCAL METRICS FUNCTION

function printMetrics(label,errorData,mask)

    selected = errorData(mask,:);

    magnitude = vecnorm(selected,2,2);

    fprintf('\n%s:\n',label);
    fprintf('RMS error:     %.8e\n', ...
        sqrt(mean(magnitude.^2)));

    fprintf('Maximum error: %.8e\n', ...
        max(magnitude));
end
