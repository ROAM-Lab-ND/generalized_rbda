clear;clc;close all;

restoredefaultpath;
addpath(genpath('casadi'));

for N = [2 4]

    fwd_dyn = derive_revolute_with_rotor(N);

    opts = struct ('cpp', true, 'with_header', true);
    name = ['rev_w_rotor_', num2str(N), 'dof_FD'];
    gen_code(fwd_dyn.exact, name, opts);
    gen_code(fwd_dyn.rf, [name, '_ref_inertia'], opts);
    gen_code(fwd_dyn.rf_diag, [name, '_ref_inertia_diag'], opts)

end
