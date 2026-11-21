
%% PHASE 7.10N: COMPLETE SIMSCAPE ENERGY CALCULATION

clearvars -except simOut

clc;

fprintf('\n============================================\n');
fprintf('PHASE 7.10N: COMPLETE MECHANICAL ENERGY\n');
fprintf('============================================\n');

%% PARAMETERS

M  = 2.000;       % Total cart mass (kg)
m  = 0.500;       % Bob mass (kg)
mr = 0.001;       % Rod mass (kg)
L  = 1.000;       % Rod length (m)
g  = 9.81;        % Gravity (m/s^2)

rodRadius = 0.01;
bobRadius = 0.05;

%% LOAD RESULTS

if ~exist('simOut','var')
    error('simOut is not available. Run the original Simscape simulation first.');
end

bobXYZ   = simOut.get('bob_xyz');
bobOmega = simOut.get('bob_omega');

xPos = simOut.get('x_position');
yPos = simOut.get('y_position');

xVel = simOut.get('x_velocity');
yVel = simOut.get('y_velocity');

t = bobOmega.Time(:);
N = numel(t);

%% HELPER: RESAMPLE TIMESERIES TO COMMON TIME

getData = @(ts) reshape(squeeze(ts.Data), ...
    size(squeeze(ts.Data),1), []);

xyz = getData(bobXYZ);

if size(xyz,1) ~= 3
    xyz = xyz.';
end

if size(xyz,2) ~= numel(bobXYZ.Time)
    error('Unexpected bob position data dimensions.');
end

bobPosition = interp1(bobXYZ.Time(:),xyz.',t,'pchip').';

cartX = interp1(xPos.Time(:),squeeze(xPos.Data),t,'pchip');
cartY = interp1(yPos.Time(:),squeeze(yPos.Data),t,'pchip');

cartVX = interp1(xVel.Time(:),squeeze(xVel.Data),t,'pchip');
cartVY = interp1(yVel.Time(:),squeeze(yVel.Data),t,'pchip');

omega = squeeze(bobOmega.Data);

if size(omega,1) ~= 3
    omega = omega.';
end

if size(omega,2) ~= N
    error('Unexpected angular velocity data dimensions.');
end

%% CART POSITION AND VELOCITY

cartPosition = [cartX(:).'; cartY(:).'; zeros(1,N)];

cartVelocity = [cartVX(:).'; cartVY(:).'; zeros(1,N)];

%% BOB AND ROD VELOCITIES FROM ANGULAR VELOCITY

relativePosition = bobPosition - cartPosition;

relativeVelocity = cross(omega.', ...
    relativePosition.',2).';

bobVelocity = cartVelocity + relativeVelocity;

%% ROD CENTER OF MASS

rodPosition = 0.5*(cartPosition + bobPosition);

rodVelocity = cartVelocity + 0.5*relativeVelocity;

%% ROD AXIS UNIT VECTOR

rodDirection = bobPosition - cartPosition;

rodDirection = rodDirection ./ vecnorm(rodDirection);

%% TRANSLATIONAL KINETIC ENERGY

T_cart = 0.5*M*sum(cartVelocity.^2,1);

T_bob = 0.5*m*sum(bobVelocity.^2,1);

T_rod = 0.5*mr*sum(rodVelocity.^2,1);

%% ROD ROTATIONAL KINETIC ENERGY

I_perp = mr*(L^2/12 + rodRadius^2/4);

I_parallel = 0.5*mr*rodRadius^2;

omegaSquared = sum(omega.^2,1);

omegaAlongRod = sum(omega.*rodDirection,1);

T_rod_rotation = 0.5*( ...
    I_perp*omegaSquared + ...
    (I_parallel-I_perp)*omegaAlongRod.^2);

%% BOB SPHERICAL ROTATIONAL KINETIC ENERGY

I_bob = (2/5)*m*bobRadius^2;

T_bob_rotation = 0.5*I_bob*omegaSquared;

%% POTENTIAL ENERGY

V_bob = m*g*bobPosition(3,:);

V_rod = mr*g*rodPosition(3,:);

V_total = V_bob + V_rod;

%% TOTAL ENERGY

T_total = T_cart + T_bob + T_rod + ...
    T_rod_rotation + T_bob_rotation;

E_total = T_total + V_total;

%% ENERGY DRIFT

E_initial = E_total(1);

energyDrift = E_total - E_initial;

rmsDrift = sqrt(mean(energyDrift.^2));

maxDrift = max(abs(energyDrift));

relativeDrift = 100*maxDrift/max(abs(E_initial),eps);

%% RESULTS

fprintf('\nENERGY COMPONENTS AT INITIAL TIME\n');

fprintf('Cart translational KE:       %.10f J\n',T_cart(1));
fprintf('Bob translational KE:        %.10f J\n',T_bob(1));
fprintf('Rod translational KE:        %.10f J\n',T_rod(1));
fprintf('Rod rotational KE:           %.10f J\n',T_rod_rotation(1));
fprintf('Bob rotational KE:           %.10f J\n',T_bob_rotation(1));
fprintf('Gravitational potential:     %.10f J\n',V_total(1));

fprintf('\nTOTAL ENERGY\n');

fprintf('Initial energy:              %.10f J\n',E_total(1));
fprintf('Final energy:                %.10f J\n',E_total(end));

fprintf('\nENERGY CONSERVATION\n');

fprintf('RMS energy drift:             %.10e J\n',rmsDrift);
fprintf('Maximum energy drift:         %.10e J\n',maxDrift);
fprintf('Maximum relative drift:       %.8f %%\n',relativeDrift);

%% SAVE RESULTS

results = struct();

results.time = t;
results.T_cart = T_cart;
results.T_bob = T_bob;
results.T_rod = T_rod;
results.T_rod_rotation = T_rod_rotation;
results.T_bob_rotation = T_bob_rotation;
results.V_total = V_total;
results.total_energy = E_total;
results.energy_drift = energyDrift;
results.rms_energy_drift = rmsDrift;
results.max_energy_drift = maxDrift;
results.relative_energy_drift_percent = relativeDrift;

projectRoot = ...
    '/home/sahil/Documents/Multibody_CartPole/XY_Spherical_Pendulum';

save(fullfile(projectRoot,'Results','Simscape', ...
    'complete_simscape_energy_results.mat'),'results');

%% ENERGY PLOT

figure('Name','Complete Simscape Energy');

plot(t,E_total,'LineWidth',1.5);
hold on;
yline(E_initial,'--','Initial energy');

xlabel('Time (s)');
ylabel('Mechanical Energy (J)');
title('Complete Simscape Mechanical Energy');
legend('Total energy','Initial energy');
grid on;

fprintf('\nComplete energy results saved successfully.\n');
