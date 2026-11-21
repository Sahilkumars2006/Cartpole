
%% PHASE 9.16: COMBINED PARAMETER INTERACTION ANALYSIS
% 3D X-Y Cart with Inverted Spherical Pendulum

clc;
clear;
close all;

fprintf('\n============================================\n');
fprintf('PHASE 9.16: COMBINED PARAMETER INTERACTION\n');
fprintf('============================================\n');

%% 1. PROJECT PATHS

analysisFolder = fileparts(mfilename('fullpath'));
projectRoot = fileparts(analysisFolder);

addpath(genpath(fullfile(projectRoot, ...
    'Phase_4_MATLAB_Simulation')));

resultsFolder = fullfile(projectRoot, ...
    'Results', 'Phase_9');

if ~exist(resultsFolder, 'dir')
    mkdir(resultsFolder);
end

%% 2. BASELINE PARAMETERS

p.M = 2.0;
p.m = 0.5;
p.L = 1.0;
p.g = 9.81;

p.Fx = 1.0;
p.Fy = 0.0;

p.x0 = 0;
p.y0 = 0;

p.theta0 = deg2rad(5);
p.phi0 = deg2rad(30);

p.dx0 = 0;
p.dy0 = 0;

p.t_start = 0;
p.t_end = 15;
p.sample_time = 0.01;

%% 3. PARAMETER ARRAYS

cartMassValues = [1.0 2.0 3.0];

bobMassValues = [0.25 0.50 0.75];

rodLengthValues = [0.5 1.0 1.5];

forceValues = [0.5 1.0 1.5];

nM = numel(cartMassValues);
nm = numel(bobMassValues);
nL = numel(rodLengthValues);
nF = numel(forceValues);

totalCases = nM*nm*nL*nF;

fprintf('\n--- BASELINE PARAMETERS ---\n');

fprintf('Gravity: %.3f m/s^2\n', p.g);
fprintf('Initial inclination: %.3f deg\n', rad2deg(p.theta0));
fprintf('Initial azimuth: %.3f deg\n', rad2deg(p.phi0));
fprintf('Force direction: +X\n');

fprintf('\n--- PARAMETER VALUES ---\n');

fprintf('Cart masses (kg):\n');
disp(cartMassValues);

fprintf('Bob masses (kg):\n');
disp(bobMassValues);

fprintf('Rod lengths (m):\n');
disp(rodLengthValues);

fprintf('External forces (N):\n');
disp(forceValues);

fprintf('Total simulations: %d\n', totalCases);

%% 4. SOLVER SETTINGS

timeVector = (p.t_start:p.sample_time:p.t_end)';

options = odeset( ...
    'RelTol', 1e-11, ...
    'AbsTol', 1e-13, ...
    'MaxStep', p.sample_time/5);

%% 5. INITIALIZE RESULT MATRICES

maxRadialMatrix = zeros(nM,nm,nL,nF);
maxSpeedMatrix = zeros(nM,nm,nL,nF);
maxInclinationMatrix = zeros(nM,nm,nL,nF);

maxEnergyMatrix = zeros(nM,nm,nL,nF);
maxRodErrorMatrix = zeros(nM,nm,nL,nF);

finalWorkMatrix = zeros(nM,nm,nL,nF);

results = struct([]);

caseNumber = 0;

%% 6. RUN COMBINED PARAMETER STUDY

fprintf('\n============================================\n');
fprintf('RUNNING COMBINED PARAMETER SIMULATIONS\n');
fprintf('============================================\n');

for i = 1:nM

    for j = 1:nm

        for k = 1:nL

            for q = 1:nF

                caseNumber = caseNumber + 1;

                %% Assign parameters

                pCase = p;

                pCase.M = cartMassValues(i);
                pCase.m = bobMassValues(j);
                pCase.L = rodLengthValues(k);

                pCase.Fx = forceValues(q);
                pCase.Fy = 0;

                %% Initial Cartesian geometry

                theta = pCase.theta0;
                phi = pCase.phi0;

                X0 = pCase.L*sin(theta)*cos(phi);
                Y0 = pCase.L*sin(theta)*sin(phi);
                Z0 = pCase.L*cos(theta);

                z0 = [ ...
                    pCase.x0;
                    pCase.y0;
                    X0;
                    Y0;
                    Z0;
                    pCase.dx0;
                    pCase.dy0;
                    0;
                    0;
                    0];

                %% Solve equations of motion

                fprintf('\nCase %d of %d\n', ...
                    caseNumber,totalCases);

                fprintf('M = %.2f kg, m = %.2f kg\n', ...
                    pCase.M,pCase.m);

                fprintf('L = %.2f m, Fx = %.2f N\n', ...
                    pCase.L,pCase.Fx);

                [t,states] = ode45( ...
                    @(t,z) cartesian_equations_of_motion(t,z,pCase), ...
                    timeVector,z0,options);

                %% Extract states

                x = states(:,1);
                y = states(:,2);

                X = states(:,3);
                Y = states(:,4);
                Z = states(:,5);

                dx = states(:,6);
                dy = states(:,7);

                dX = states(:,8);
                dY = states(:,9);
                dZ = states(:,10);

                %% Cart response

                cartRadial = sqrt( ...
                    (x-pCase.x0).^2 + ...
                    (y-pCase.y0).^2);

                cartSpeed = sqrt(dx.^2+dy.^2);

                %% Pendulum geometry

                rx = X-x;
                ry = Y-y;
                rz = Z;

                relativeLength = sqrt( ...
                    rx.^2+ry.^2+rz.^2);

                inclination = acosd(max(-1,min(1,rz/pCase.L)));

                %% Bob displacement from initial position

                bobDisplacement = sqrt( ...
                    (X-X0).^2+ ...
                    (Y-Y0).^2+ ...
                    (Z-Z0).^2);

                %% Mechanical energy

                cartKE = 0.5*pCase.M*(dx.^2+dy.^2);

                bobKE = 0.5*pCase.m*( ...
                    dX.^2+dY.^2+dZ.^2);

                potentialEnergy = pCase.m*pCase.g*Z;

                totalEnergy = cartKE+bobKE+potentialEnergy;

                %% External work

                externalWork = ...
                    pCase.Fx*(x-pCase.x0)+ ...
                    pCase.Fy*(y-pCase.y0);

                energyResidual = ...
                    totalEnergy-totalEnergy(1)-externalWork;

                %% Rod constraint error

                rodError = abs(relativeLength-pCase.L);

                %% Calculate performance metrics

                maxRadial = max(cartRadial);
                maxSpeed = max(cartSpeed);

                maxTilt = max(inclination);
                finalTilt = inclination(end);

                maxBobMotion = max(bobDisplacement);

                maxEnergyError = max(abs(energyResidual));
                maxConstraintError = max(rodError);

                finalWork = externalWork(end);

                %% Store response matrices

                maxRadialMatrix(i,j,k,q) = maxRadial;

                maxSpeedMatrix(i,j,k,q) = maxSpeed;

                maxInclinationMatrix(i,j,k,q) = maxTilt;

                maxEnergyMatrix(i,j,k,q) = maxEnergyError;

                maxRodErrorMatrix(i,j,k,q) = maxConstraintError;

                finalWorkMatrix(i,j,k,q) = finalWork;

                %% Store detailed results

                results(caseNumber).cart_mass = pCase.M;
                results(caseNumber).bob_mass = pCase.m;
                results(caseNumber).rod_length = pCase.L;
                results(caseNumber).force = pCase.Fx;

                results(caseNumber).time = t;
                results(caseNumber).states = states;

                results(caseNumber).cart_radial = cartRadial;
                results(caseNumber).cart_speed = cartSpeed;

                results(caseNumber).inclination = inclination;

                results(caseNumber).bob_displacement = bobDisplacement;

                results(caseNumber).total_energy = totalEnergy;

                results(caseNumber).external_work = externalWork;

                results(caseNumber).energy_residual = energyResidual;

                results(caseNumber).rod_error = rodError;

                results(caseNumber).max_radial_displacement = maxRadial;
                results(caseNumber).max_cart_speed = maxSpeed;

                results(caseNumber).max_inclination = maxTilt;
                results(caseNumber).final_inclination = finalTilt;

                results(caseNumber).max_bob_displacement = maxBobMotion;

                results(caseNumber).max_energy_residual = maxEnergyError;

                results(caseNumber).max_rod_error = maxConstraintError;

                results(caseNumber).final_external_work = finalWork;

                %% Display case results

                fprintf('Maximum cart displacement: %.6f m\n',maxRadial);

                fprintf('Maximum cart speed: %.6f m/s\n',maxSpeed);

                fprintf('Maximum inclination: %.6f deg\n',maxTilt);

                fprintf('Maximum energy residual: %.6e J\n', ...
                    maxEnergyError);

                fprintf('Maximum rod error: %.6e m\n', ...
                    maxConstraintError);

            end
        end
    end
end

%% 7. CREATE SUMMARY TABLE

MColumn = [results.cart_mass]';
mColumn = [results.bob_mass]';
LColumn = [results.rod_length]';
FColumn = [results.force]';

radialColumn = [results.max_radial_displacement]';
speedColumn = [results.max_cart_speed]';

maxTiltColumn = [results.max_inclination]';
finalTiltColumn = [results.final_inclination]';

bobMotionColumn = [results.max_bob_displacement]';

workColumn = [results.final_external_work]';

energyColumn = [results.max_energy_residual]';
rodColumn = [results.max_rod_error]';

summaryTable = table( ...
    MColumn, ...
    mColumn, ...
    LColumn, ...
    FColumn, ...
    radialColumn, ...
    speedColumn, ...
    maxTiltColumn, ...
    finalTiltColumn, ...
    bobMotionColumn, ...
    workColumn, ...
    energyColumn, ...
    rodColumn, ...
    'VariableNames',{ ...
    'Cart_Mass_kg', ...
    'Bob_Mass_kg', ...
    'Rod_Length_m', ...
    'External_Force_N', ...
    'Max_Cart_Displacement_m', ...
    'Max_Cart_Speed_mps', ...
    'Max_Inclination_deg', ...
    'Final_Inclination_deg', ...
    'Max_Bob_Displacement_m', ...
    'Final_External_Work_J', ...
    'Max_Energy_Residual_J', ...
    'Max_Rod_Error_m'});

fprintf('\n============================================\n');
fprintf('COMBINED PARAMETER SUMMARY TABLE\n');
fprintf('============================================\n');

disp(summaryTable);

%% 8. OVERALL RESPONSE RANGES

fprintf('\n============================================\n');
fprintf('OVERALL RESPONSE RANGES\n');
fprintf('============================================\n');

fprintf('Maximum cart displacement range: %.6f to %.6f m\n', ...
    min(radialColumn),max(radialColumn));

fprintf('Maximum cart speed range: %.6f to %.6f m/s\n', ...
    min(speedColumn),max(speedColumn));

fprintf('Maximum inclination range: %.6f to %.6f deg\n', ...
    min(maxTiltColumn),max(maxTiltColumn));

fprintf('Maximum bob displacement range: %.6f to %.6f m\n', ...
    min(bobMotionColumn),max(bobMotionColumn));

%% 9. BASELINE INDEX

baselineMIndex = find(cartMassValues==2.0,1);
baselineMIndexBob = find(bobMassValues==0.5,1);
baselineLIndex = find(rodLengthValues==1.0,1);
baselineFIndex = find(forceValues==1.0,1);

%% 10. CART MASS VS BOB MASS INTERACTION

figure('Name','Cart Mass Bob Mass Interaction');

imagesc(bobMassValues,cartMassValues, ...
    maxRadialMatrix(:,:,baselineLIndex,baselineFIndex));

set(gca,'YDir','normal');

colorbar;

xlabel('Bob mass (kg)');
ylabel('Cart mass (kg)');

title('Maximum Cart Displacement: M vs m');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_16_Cart_Bob_Mass_Interaction.png'));

%% 11. CART MASS VS ROD LENGTH INTERACTION

figure('Name','Cart Mass Rod Length Interaction');

imagesc(rodLengthValues,cartMassValues, ...
    squeeze(maxRadialMatrix(:,baselineMIndexBob,:,baselineFIndex)));

set(gca,'YDir','normal');

colorbar;

xlabel('Rod length (m)');
ylabel('Cart mass (kg)');

title('Maximum Cart Displacement: M vs L');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_16_Cart_Mass_Rod_Length_Interaction.png'));

%% 12. BOB MASS VS ROD LENGTH INTERACTION

figure('Name','Bob Mass Rod Length Interaction');

imagesc(rodLengthValues,bobMassValues, ...
    squeeze(maxRadialMatrix(baselineMIndex,:,:,baselineFIndex)));

set(gca,'YDir','normal');

colorbar;

xlabel('Rod length (m)');
ylabel('Bob mass (kg)');

title('Maximum Cart Displacement: m vs L');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_16_Bob_Mass_Rod_Length_Interaction.png'));

%% 13. CART MASS VS FORCE INTERACTION

figure('Name','Cart Mass Force Interaction');

imagesc(forceValues,cartMassValues, ...
    squeeze(maxRadialMatrix(:,baselineMIndexBob, ...
    baselineLIndex,:)));

set(gca,'YDir','normal');

colorbar;

xlabel('External force (N)');
ylabel('Cart mass (kg)');

title('Maximum Cart Displacement: M vs F');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_16_Cart_Mass_Force_Interaction.png'));

%% 14. SPEED INTERACTION: CART MASS VS BOB MASS

figure('Name','Cart Speed Mass Interaction');

imagesc(bobMassValues,cartMassValues, ...
    maxSpeedMatrix(:,:,baselineLIndex,baselineFIndex));

set(gca,'YDir','normal');

colorbar;

xlabel('Bob mass (kg)');
ylabel('Cart mass (kg)');

title('Maximum Cart Speed: M vs m');

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_16_Cart_Speed_Mass_Interaction.png'));

%% 15. FORCE EFFECT AT BASELINE MASSES AND LENGTH

figure('Name','Force Effect');

forceDisplacement = squeeze(maxRadialMatrix( ...
    baselineMIndex,baselineMIndexBob,baselineLIndex,:));

forceSpeed = squeeze(maxSpeedMatrix( ...
    baselineMIndex,baselineMIndexBob,baselineLIndex,:));

yyaxis left;

plot(forceValues,forceDisplacement,'-o','LineWidth',1.5);

ylabel('Maximum cart displacement (m)');

yyaxis right;

plot(forceValues,forceSpeed,'-s','LineWidth',1.5);

ylabel('Maximum cart speed (m/s)');

xlabel('External force (N)');

title('External Force Effect at Baseline Parameters');

grid on;

saveas(gcf,fullfile(resultsFolder, ...
    'Phase_9_16_Force_Response.png'));

%% 16. VALIDATION

maximumEnergyError = max(energyColumn);
maximumRodError = max(rodColumn);

fprintf('\n============================================\n');
fprintf('PHASE 9.16 VALIDATION\n');
fprintf('============================================\n');

fprintf('Total simulations: %d\n',totalCases);

fprintf('Maximum energy balance residual: %.6e J\n', ...
    maximumEnergyError);

fprintf('Maximum rod constraint error: %.6e m\n', ...
    maximumRodError);

if maximumEnergyError < 1e-5
    fprintf('Work-energy validation: PASSED\n');
else
    fprintf('Work-energy validation: REVIEW REQUIRED\n');
end

if maximumRodError < 1e-7
    fprintf('Rigid rod constraint: PASSED\n');
else
    fprintf('Rigid rod constraint: REVIEW REQUIRED\n');
end

%% 17. SAVE RESULTS

save(fullfile(resultsFolder, ...
    'phase_9_16_combined_parameter_results.mat'), ...
    'results', ...
    'summaryTable', ...
    'maxRadialMatrix', ...
    'maxSpeedMatrix', ...
    'maxInclinationMatrix', ...
    'maxEnergyMatrix', ...
    'maxRodErrorMatrix', ...
    'finalWorkMatrix', ...
    'cartMassValues', ...
    'bobMassValues', ...
    'rodLengthValues', ...
    'forceValues', ...
    'p', ...
    'timeVector');

writetable(summaryTable,fullfile(resultsFolder, ...
    'Phase_9_16_Combined_Parameter_Summary.csv'));

%% 18. COMPLETION

fprintf('\n============================================\n');
fprintf('PHASE 9.16 ANALYSIS COMPLETED\n');
fprintf('============================================\n');

fprintf('\nResults directory:\n%s\n',resultsFolder);

fprintf('\nSaved files:\n');

fprintf('1. phase_9_16_combined_parameter_results.mat\n');

fprintf('2. Phase_9_16_Combined_Parameter_Summary.csv\n');

fprintf('\nGenerated figures:\n');

fprintf('1. Phase_9_16_Cart_Bob_Mass_Interaction.png\n');

fprintf('2. Phase_9_16_Cart_Mass_Rod_Length_Interaction.png\n');

fprintf('3. Phase_9_16_Bob_Mass_Rod_Length_Interaction.png\n');

fprintf('4. Phase_9_16_Cart_Mass_Force_Interaction.png\n');

fprintf('5. Phase_9_16_Cart_Speed_Mass_Interaction.png\n');

fprintf('6. Phase_9_16_Force_Response.png\n');

fprintf('\n============================================\n');
fprintf('PHASE 9.16 SCRIPT FINISHED\n');
fprintf('============================================\n');
