clear;clc;close all;

restoredefaultpath;
addpath(genpath('casadi'));

import casadi.*

q = SX.sym('q', 2);
dq = SX.sym('dq', 2);
ddq = SX.sym('ddq', 2);
u = SX.sym('u', 2);

I1 = 0.001;
I2 = 0.002;
I1rot = 0.00001;
I2rot = 0.00003;
m1 = 0.3;
m2 = 0.5;
l1 = 0.3;
l2 = 0.26;
c1 = 0.15;
c2 = 0.13;
N1 = 6;
N2 = 10;
g = -9.81;

ddt = @(x)(jacobian(x, q) * dq + jacobian(x, dq) * ddq);
F2Q = @(F, r) simplify(jacobian(r, q)' * (F));
M2Q = @(M, w) simplify(jacobian(w, dq)' * (M));

%% Kinemeatics
th1 = q(1);
th2 = q(2);
dth1 = dq(1);
dth2 = dq(2);

rO = [0 0 0]'; % position of link 1
ehat1 = [cos(th1) sin(th1) 0]'; % Define unit vector along Leg 1
ehat2 = [cos(th1 + th2) sin(th1 + th2) 0]'; % Define unit vector along Leg 2
ghat = [-1 0 0]';

rc1 = c1 * ehat1; % Position of link 1 CoM
rB = l1 * ehat1; % Position of base of link 2
rc2 = rB + c2 * ehat2; % Position of CoM of link 2
rC = rB + l2 * ehat2; % Position of end of link 3

vc1 = ddt(rc1); % Velocity of link 1 CoM
omega1 = dth1; % Angular velocity of Link 1
vc2 = ddt(rc2); % Velocity of link 2 CoM
omega2 = dth1 + dth2; % Angular velocity of link 2

%% Lagrangian
% Kinetic and Potential Energy of link 1
T1 = 1/2 * m1 * (vc1.' * vc1) + ...
    1/2 * I1 * omega1 ^ 2 + ...
    1/2 * I1rot * (N1 * omega1) ^ 2;
T1_ref_inertia = 1/2 * m1 * (vc1.' * vc1) + ...
    1/2 * I1 * omega1 ^ 2 + ...
    1/2 * I1rot * (N1 * dth1) ^ 2;
V1 = m1 * g * dot(rc1, -ghat);

% Kinetic and Potential Energy of link 2
T2 = 1/2 * m2 * (vc2.' * vc2) + ...
    1/2 * I2 * omega2 ^ 2 + ...
    1/2 * I2rot * (dth2 * N2 + dth1) ^ 2;
T2_ref_inertia = 1/2 * m2 * (vc2.' * vc2) + ...
    1/2 * I2 * omega2 ^ 2 + ...
    1/2 * I2rot * (N2 * dth2) ^ 2;
V2 = m2 * g * dot(rc2, -ghat);

T = T1 + T2;
T_ref_inertia = T1_ref_inertia + T2_ref_inertia;
V = V1 + V2;
% Compute Lagrangian and total energy
L = simplify(T - V);
L_ref_inertia = simplify(T_ref_inertia - V);

%% Generalized forces
tau1 = u(1);
tau2 = u(2);
Q = M2Q(tau1, omega1) - M2Q(tau2, omega1) + M2Q(tau2, omega2);

%% Equations of motion
eom = computeSymbolicEquationsOfMotion(q, dq, ddq, u, L, Q);
eom_ref_inertia = computeSymbolicEquationsOfMotion(q, dq, ddq, u, L_ref_inertia, Q);

% Code generation
f_eom = Function('OpenChain2DofEom', {q, dq, ddq, u}, {eom});
f_eom_rf = Function('OpenChain2DofEomReflectedInertia', {q, dq, ddq, u}, {eom_ref_inertia});
opts = struct ('cpp', true, 'with_header', true);
gen_code(f_eom, 'open_chain_2dof_eom.cpp', opts);
gen_code(f_eom_rf, 'open_chain_2dof_eom_reflected_inertia.cpp', opts);

%% Helper functions
function eom = computeSymbolicEquationsOfMotion(q, dq, ddq, u, L, Q)
    ddt = @(x)(jacobian(x, q) * dq + jacobian(x, dq) * ddq);
    dL_dq = jacobian(L, q)';
    dL_dqd = jacobian(L, dq)';
    eom = ddt(dL_dqd) - dL_dq - Q;
end

function gen_code(f, name, opts)
    f.generate(name, opts);
    movefile(name, ['../source/' name]);
    movefile([name(1:end - 3) 'h'], ['../header/' name(1:end - 3) 'h']);
end
