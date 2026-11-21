
%% SYMBOLIC VERIFICATION
% 3D X-Y Cart with an Inverted Spherical Pendulum
%
% This script:
% 1. Derives equations using the Lagrangian formulation.
% 2. Verifies the analytical mass matrix.
% 3. Verifies the nonlinear RHS vector.
% 4. Checks mass matrix symmetry.
% 5. Checks the analytical determinant.
% 6. Verifies the mechanical energy balance.
% 7. Saves verification results.
%
% Coordinate convention:
% theta = 0 represents the upright configuration.
% phi   = azimuth measured from +X toward +Y.
%
% Potential energy:
% V = mp*g*L*cos(theta)

clear;
clc;

fprintf('\n============================================\n');
fprintf(' INVERTED SPHERICAL PENDULUM VERIFICATION\n');
fprintf('============================================\n\n');

%% 1. DEFINE SYMBOLIC VARIABLES

syms Mc mp L g real
syms x y th ph real
syms dx dy dth dph real
syms ddx ddy ddth ddph real
syms Fx Fy real

q = [x; y; th; ph];

qd = [dx; dy; dth; dph];

qdd = [ddx; ddy; ddth; ddph];

Q = [Fx; Fy; 0; 0];

%% 2. DEFINE BOB POSITION

rp = [
    x + L*sin(th)*cos(ph);
    y + L*sin(th)*sin(ph);
    L*cos(th)
];

fprintf('Bob position defined.\n');

%% 3. DERIVE BOB VELOCITY

Jp = jacobian(rp, q);

vp = simplify(Jp * qd);

fprintf('Bob velocity derived.\n');

%% 4. DERIVE KINETIC AND POTENTIAL ENERGY

T_cart = 0.5 * Mc * (dx^2 + dy^2);

T_bob = 0.5 * mp * (vp.' * vp);

T = simplify(T_cart + T_bob);

% Gravitational potential energy for an inverted pendulum

V = mp*g*L*cos(th);

% Lagrangian

Lag = simplify(T - V);

% Total mechanical energy

E = simplify(T + V);

fprintf('Kinetic energy derived.\n');
fprintf('Potential energy derived.\n');
fprintf('Lagrangian constructed.\n');

%% 5. APPLY EULER-LAGRANGE EQUATIONS

dLdqd = jacobian(Lag, qd).';

% Total time derivative:
% d/dt(dL/dqd) = Jacobian(dL/dqd,[q;qd])*[qd;qdd]

d_dt_dLdqd = jacobian(dLdqd, [q; qd]) * [qd; qdd];

dLdq = jacobian(Lag, q).';

% Euler-Lagrange equations:
% d/dt(dL/dqd) - dL/dq = Q

EL = simplify(d_dt_dLdqd - dLdq - Q);

fprintf('Euler-Lagrange equations derived.\n');

%% 6. EXTRACT DERIVED MASS MATRIX AND RHS

% EL = Mderived*qdd - Rderived

Mderived = simplify(jacobian(EL, qdd));

hderived = simplify(subs(EL, qdd, zeros(4,1)));

Rderived = simplify(-hderived);

fprintf('Derived mass matrix and RHS extracted.\n');

%% 7. DEFINE ANALYTICAL MASS MATRIX

st = sin(th);
ct = cos(th);

sp = sin(ph);
cp = cos(ph);

Manalytical = [
    Mc+mp,          0,          mp*L*ct*cp,   -mp*L*st*sp;
    0,              Mc+mp,      mp*L*ct*sp,    mp*L*st*cp;
    mp*L*ct*cp,     mp*L*ct*sp, mp*L^2,        0;
    -mp*L*st*sp,    mp*L*st*cp, 0,              mp*L^2*st^2
];

%% 8. DEFINE ANALYTICAL NONLINEAR RHS

Ranalytical = [
    Fx + mp*L*st*cp*(dth^2 + dph^2) ...
       + 2*mp*L*ct*sp*dth*dph;

    Fy + mp*L*st*sp*(dth^2 + dph^2) ...
       - 2*mp*L*ct*cp*dth*dph;

    mp*L^2*st*ct*dph^2 ...
       + mp*g*L*st;

    -2*mp*L^2*st*ct*dth*dph
];

Ranalytical = simplify(Ranalytical);

%% 9. VERIFY MASS MATRIX

fprintf('\n--- MASS MATRIX VERIFICATION ---\n');

mass_error = simplify(Mderived - Manalytical);

mass_check = isAlways(mass_error == 0, 'Unknown', 'false');

mass_pass = all(mass_check(:));

if mass_pass
    fprintf('PASS: Derived and analytical mass matrices are identical.\n');
else
    fprintf('FAIL: Mass matrices are different.\n');
    disp(mass_error);
end

%% 10. VERIFY NONLINEAR RHS

fprintf('\n--- NONLINEAR RHS VERIFICATION ---\n');

rhs_error = simplify(Rderived - Ranalytical);

rhs_check = isAlways(rhs_error == 0, 'Unknown', 'false');

rhs_pass = all(rhs_check(:));

if rhs_pass
    fprintf('PASS: Derived and analytical RHS vectors are identical.\n');
else
    fprintf('FAIL: RHS vectors are different.\n');
    disp(rhs_error);
end

%% 11. VERIFY MASS MATRIX SYMMETRY

fprintf('\n--- MASS MATRIX SYMMETRY ---\n');

symmetry_error = simplify(Manalytical - Manalytical.');

symmetry_check = isAlways(symmetry_error == 0, 'Unknown', 'false');

symmetry_pass = all(symmetry_check(:));

if symmetry_pass
    fprintf('PASS: Mass matrix is symmetric.\n');
else
    fprintf('FAIL: Mass matrix is not symmetric.\n');
    disp(symmetry_error);
end


%% 12. VERIFY MASS MATRIX DETERMINANT

fprintf('\n--- MASS MATRIX DETERMINANT ---\n');

% Calculate the complete determinant as one symbolic expression
det_derived = simplify(det(Manalytical), 'Steps', 100);

% Correct analytical determinant
det_expected = ...
    Mc * mp^2 * L^4 * sin(th)^2 * ...
    (Mc + mp*sin(th)^2);

% Simplify the difference
det_error = simplify(det_derived - det_expected, 'Steps', 100);

% Verify equality
det_check = isAlways(det_error == 0, 'Unknown', 'false');

det_pass = all(det_check(:));

if det_pass
    fprintf('PASS: Determinant matches the analytical expression.\n');
else
    fprintf('FAIL: Determinant does not match.\n');

    fprintf('\nDerived determinant:\n');
    disp(det_derived);

    fprintf('\nExpected determinant:\n');
    disp(det_expected);

    fprintf('\nDifference:\n');
    disp(det_error);
end

fprintf('\nAnalytical determinant:\n');
pretty(det_expected);


%% 13. VERIFY MECHANICAL ENERGY BALANCE

fprintf('\n--- ENERGY BALANCE VERIFICATION ---\n');

% Solve analytical equations of motion for acceleration

qdd_analytical = simplify(Manalytical \ Ranalytical);

% Total mechanical energy:
% E = T + V

% Calculate total time derivative of energy

energy_rate = simplify( ...
    jacobian(E, [q; qd]) * [qd; qdd_analytical]);

% Expected energy rate:
% dE/dt = Fx*dx + Fy*dy

expected_energy_rate = Fx*dx + Fy*dy;

energy_error = simplify( ...
    energy_rate - expected_energy_rate);

energy_check = isAlways(energy_error == 0, 'Unknown', 'false');

energy_pass = all(energy_check(:));

if energy_pass
    fprintf('PASS: Mechanical energy balance is verified.\n');
    fprintf('dE/dt = Fx*dx + Fy*dy\n');
else
    fprintf('FAIL: Energy balance verification failed.\n');
    disp(energy_error);
end

%% 14. FINAL VERIFICATION SUMMARY

fprintf('\n============================================\n');
fprintf('       FINAL VERIFICATION SUMMARY\n');
fprintf('============================================\n');

fprintf('Mass matrix verification:       %s\n', ...
    pass_fail(mass_pass));

fprintf('Nonlinear RHS verification:     %s\n', ...
    pass_fail(rhs_pass));

fprintf('Mass matrix symmetry:           %s\n', ...
    pass_fail(symmetry_pass));

fprintf('Mass matrix determinant:        %s\n', ...
    pass_fail(det_pass));

fprintf('Energy balance:                 %s\n', ...
    pass_fail(energy_pass));

% Overall verification status

all_pass = all([
    mass_pass;
    rhs_pass;
    symmetry_pass;
    det_pass;
    energy_pass
]);

if all_pass
    fprintf('\nOVERALL RESULT: ALL TESTS PASSED.\n');
else
    fprintf('\nOVERALL RESULT: SOME TESTS FAILED.\n');
end

fprintf('============================================\n');

%% 15. SAVE VERIFICATION RESULTS

scriptDir = fileparts(mfilename('fullpath'));

projectDir = fileparts(scriptDir);

resultsDir = fullfile(projectDir, 'Results', 'MATLAB');

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

verification = struct();

verification.mass_error = mass_error;
verification.rhs_error = rhs_error;
verification.symmetry_error = symmetry_error;

verification.determinant = det_derived;
verification.expected_determinant = det_expected;
verification.determinant_error = det_error;

verification.energy_error = energy_error;

verification.mass_pass = mass_pass;
verification.rhs_pass = rhs_pass;
verification.symmetry_pass = symmetry_pass;
verification.det_pass = det_pass;
verification.energy_pass = energy_pass;

verification.all_pass = all_pass;

verification.model = ...
    '3D X-Y Cart with Inverted Spherical Pendulum';

save(fullfile(resultsDir, 'verification_results.mat'), ...
    'verification');

fprintf('\nVerification results saved successfully.\n');

fprintf('Saved location:\n%s\n', ...
    fullfile(resultsDir, 'verification_results.mat'));

%% LOCAL FUNCTION

function result = pass_fail(condition)

    if condition
        result = 'PASS';
    else
        result = 'FAIL';
    end

end
