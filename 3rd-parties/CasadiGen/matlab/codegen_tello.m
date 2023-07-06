clear;clc;close all;

thd = derive_tello_hip_differential();
tkad = derive_tello_knee_ankle_differential();

opts = struct('cpp', true, 'with_header', true);
gen_code(thd.phi, 'thd_phi', opts);
gen_code(thd.kikd, 'thd_kikd', opts);
gen_code(thd.J_dy_2_dqd, 'thd_J_dy_2_dqd', opts);
gen_code(thd.k, 'thd_k', opts);
gen_code(thd.g, 'thd_g', opts);
gen_code(thd.IK_pos, 'thd_IK_pos', opts);
gen_code(thd.IK_vel, 'thd_IK_vel', opts);
gen_code(thd.IK_acc, 'thd_IK_acc', opts);
gen_code(tkad.phi, 'tkad_phi', opts);
gen_code(tkad.kikd, 'tkad_kikd', opts);
gen_code(tkad.J_dy_2_dqd, 'tkad_J_dy_2_dqd', opts);
gen_code(tkad.k, 'tkad_k', opts);
gen_code(tkad.g, 'tkad_g', opts);
gen_code(tkad.IK_pos, 'tkad_IK_pos', opts);
gen_code(tkad.IK_vel, 'tkad_IK_vel', opts);
gen_code(tkad.IK_acc, 'tkad_IK_acc', opts);