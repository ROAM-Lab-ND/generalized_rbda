clear;clc;close all;

restoredefaultpath;
addpath(genpath('casadi'));

thd = derive_tello_hip_differential();
tkad = derive_tello_knee_ankle_differential();

opts = struct('cpp', true, 'with_header', true);

gen_code(thd.phi, 'thd_phi', opts);
gen_code(thd.G, 'thd_G', opts);
gen_code(thd.G_dot, 'thd_G_dot', opts);
gen_code(thd.g, 'thd_small_g', opts);
gen_code(thd.K, 'thd_K', opts);
gen_code(thd.k, 'thd_small_k', opts);
gen_code(thd.IK_pos, 'thd_IK_pos', opts);
gen_code(thd.IK_vel, 'thd_IK_vel', opts);
% TODO(@nicholasadr): can we remove?
% gen_code(thd.IK_acc, 'thd_IK_acc', opts);
gen_code(thd.jacobian, 'thd_jacobian', opts);
gen_code(thd.bias, 'thd_bias', opts);

gen_code(tkad.phi, 'tkad_phi', opts);
gen_code(tkad.G, 'tkad_G', opts);
gen_code(tkad.G_dot, 'tkad_G_dot', opts);
gen_code(tkad.g, 'tkad_small_g', opts);
gen_code(tkad.K, 'tkad_K', opts);
gen_code(tkad.k, 'tkad_small_k', opts);
gen_code(tkad.IK_pos, 'tkad_IK_pos', opts);
gen_code(tkad.IK_vel, 'tkad_IK_vel', opts);
% TODO(@nicholasadr): can we remove?
% gen_code(tkad.IK_acc, 'tkad_IK_acc', opts);
gen_code(tkad.jacobian, 'tkad_jacobian', opts);
gen_code(tkad.bias, 'tkad_bias', opts);
