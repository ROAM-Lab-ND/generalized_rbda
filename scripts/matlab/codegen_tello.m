clear;clc;close all;

restoredefaultpath;
addpath(genpath('casadi'));

thd = derive_tello_hip_differential();
tkad = derive_tello_knee_ankle_differential();

opts = struct('cpp', true, 'with_header', true);

gen_code(thd.IK_pos, 'thd_IK_pos', opts);
gen_code(thd.IK_vel, 'thd_IK_vel', opts);
gen_code(thd.jacobian, 'thd_jacobian', opts);
gen_code(thd.bias, 'thd_bias', opts);

gen_code(tkad.IK_pos, 'tkad_IK_pos', opts);
gen_code(tkad.IK_vel, 'tkad_IK_vel', opts);
gen_code(tkad.jacobian, 'tkad_jacobian', opts);
gen_code(tkad.bias, 'tkad_bias', opts);
