
%% PHASE 10.3: LQR NUMERICAL VERIFICATION
%
% Independently verifies the saved LQR controller.
% No controller redesign or simulation is performed.

clc;
clear;

fprintf('\n============================================\n');
fprintf('PHASE 10.3: LQR NUMERICAL VERIFICATION\n');
fprintf('============================================\n');

%% 1. LOAD EXISTING CONTROLLER

scriptFolder = fileparts(mfilename('fullpath'));

dataFile = fullfile(scriptFolder,'controller_data.mat');

if ~isfile(dataFile)
    error('controller_data.mat not found.');
end

loadedData = load(dataFile);

if ~isfield(loadedData,'controller')
    error('Controller structure not found.');
end

controller = loadedData.controller;

A = controller.A;
B = controller.B;
Q = controller.Q;
R = controller.R;
K = controller.K;
S = controller.S;

maximumForce = controller.maximum_force;

%% 2. MATRIX DIMENSION VERIFICATION

fprintf('\n============================================\n');
fprintf('STATE-SPACE MATRIX DIMENSIONS\n');
fprintf('============================================\n');

fprintf('A: %d x %d\n',size(A,1),size(A,2));
fprintf('B: %d x %d\n',size(B,1),size(B,2));
fprintf('Q: %d x %d\n',size(Q,1),size(Q,2));
fprintf('R: %d x %d\n',size(R,1),size(R,2));
fprintf('K: %d x %d\n',size(K,1),size(K,2));
fprintf('S: %d x %d\n',size(S,1),size(S,2));

assert(isequal(size(A),[8 8]),'Incorrect A dimensions.');
assert(isequal(size(B),[8 2]),'Incorrect B dimensions.');
assert(isequal(size(Q),[8 8]),'Incorrect Q dimensions.');
assert(isequal(size(R),[2 2]),'Incorrect R dimensions.');
assert(isequal(size(K),[2 8]),'Incorrect K dimensions.');
assert(isequal(size(S),[8 8]),'Incorrect S dimensions.');

fprintf('\nMatrix dimension verification: PASSED\n');

%% 3. CONTROLLABILITY VERIFICATION

fprintf('\n============================================\n');
fprintf('CONTROLLABILITY VERIFICATION\n');
fprintf('============================================\n');

Co = ctrb(A,B);

controllabilityRank = rank(Co);

fprintf('Controllability rank: %d / 8\n', ...
    controllabilityRank);

controllabilityPassed = controllabilityRank == 8;

if controllabilityPassed
    fprintf('Controllability: PASSED\n');
else
    fprintf('Controllability: FAILED\n');
end

%% 4. RECONSTRUCT LQR GAIN FROM RICCATI MATRIX

fprintf('\n============================================\n');
fprintf('LQR GAIN RECONSTRUCTION\n');
fprintf('============================================\n');

K_reconstructed = R \ (B' * S);

gainError = norm(K-K_reconstructed,'fro');

fprintf('Gain reconstruction error: %.12e\n',gainError);

gainPassed = gainError < 1e-8;

if gainPassed
    fprintf('LQR gain reconstruction: PASSED\n');
else
    fprintf('LQR gain reconstruction: FAILED\n');
end

%% 5. ALGEBRAIC RICCATI EQUATION

fprintf('\n============================================\n');
fprintf('RICCATI EQUATION VERIFICATION\n');
fprintf('============================================\n');

riccatiResidual = ...
    A'*S + S*A - S*B*(R\(B'*S)) + Q;

riccatiError = norm(riccatiResidual,'fro');

fprintf('Riccati residual norm: %.12e\n',riccatiError);

riccatiPassed = riccatiError < 1e-7;

if riccatiPassed
    fprintf('Riccati equation: PASSED\n');
else
    fprintf('Riccati equation: FAILED\n');
end

%% 6. CLOSED-LOOP STABILITY

fprintf('\n============================================\n');
fprintf('CLOSED-LOOP STABILITY VERIFICATION\n');
fprintf('============================================\n');

Acl = A-B*K;

closedLoopPoles = eig(Acl);

disp('Closed-loop eigenvalues:');
disp(closedLoopPoles);

maximumRealPart = max(real(closedLoopPoles));

fprintf('Maximum pole real part: %.12e\n', ...
    maximumRealPart);

stabilityPassed = maximumRealPart < 0;

if stabilityPassed
    fprintf('Asymptotic stability: PASSED\n');
else
    fprintf('Asymptotic stability: FAILED\n');
end

%% 7. COMPARE WITH PHASE 8 REFERENCE GAIN

fprintf('\n============================================\n');
fprintf('PHASE 8 GAIN COMPARISON\n');
fprintf('============================================\n');

K_phase8 = [
    -4.4721  0       -99.4112  0       -8.5016  0       -29.0870  0
     0      -4.4721   0       -99.4112  0       -8.5016  0       -29.0870
];

referenceError = norm(K-K_phase8,'fro');

fprintf('Difference from displayed Phase 8 gain: %.12e\n', ...
    referenceError);

referencePassed = referenceError < 1e-3;

if referencePassed
    fprintf('Phase 8 gain agreement: PASSED\n');
else
    fprintf('Phase 8 gain agreement: REVIEW REQUIRED\n');
end

%% 8. FORCE SATURATION VERIFICATION

fprintf('\n============================================\n');
fprintf('ACTUATOR FORCE LIMIT\n');
fprintf('============================================\n');

fprintf('Maximum force: %.4f N\n',maximumForce);

forceLimitPassed = maximumForce == 30;

if forceLimitPassed
    fprintf('Force limit verification: PASSED\n');
else
    fprintf('Force limit verification: REVIEW REQUIRED\n');
end

%% 9. SAVE VERIFICATION RESULTS

verification = struct();

verification.controllability_rank = controllabilityRank;
verification.gain_reconstruction_error = gainError;
verification.riccati_residual = riccatiError;
verification.closed_loop_poles = closedLoopPoles;
verification.maximum_pole_real_part = maximumRealPart;
verification.phase8_gain_difference = referenceError;
verification.maximum_force = maximumForce;

verification.dimension_passed = true;
verification.controllability_passed = controllabilityPassed;
verification.gain_passed = gainPassed;
verification.riccati_passed = riccatiPassed;
verification.stability_passed = stabilityPassed;
verification.reference_gain_passed = referencePassed;
verification.force_limit_passed = forceLimitPassed;

verification.all_checks_passed = ...
    verification.dimension_passed && ...
    verification.controllability_passed && ...
    verification.gain_passed && ...
    verification.riccati_passed && ...
    verification.stability_passed && ...
    verification.reference_gain_passed && ...
    verification.force_limit_passed;

outputFile = fullfile(scriptFolder, ...
    'phase_10_3_lqr_verification_results.mat');

save(outputFile,'verification');

%% 10. FINAL SUMMARY

fprintf('\n============================================\n');
fprintf('PHASE 10.3 FINAL SUMMARY\n');
fprintf('============================================\n');

fprintf('Matrix dimensions: %s\n', ...
    string(verification.dimension_passed));

fprintf('Controllability: %s\n', ...
    string(controllabilityPassed));

fprintf('Gain reconstruction: %s\n', ...
    string(gainPassed));

fprintf('Riccati equation: %s\n', ...
    string(riccatiPassed));

fprintf('Closed-loop stability: %s\n', ...
    string(stabilityPassed));

fprintf('Phase 8 gain agreement: %s\n', ...
    string(referencePassed));

fprintf('Force limit: %s\n', ...
    string(forceLimitPassed));

fprintf('\nResults saved:\n%s\n',outputFile);

if verification.all_checks_passed

    fprintf('\nPHASE 10.3 VERIFICATION PASSED\n');

else

    fprintf('\nPHASE 10.3 VERIFICATION NEEDS REVIEW\n');

end

fprintf('============================================\n');
