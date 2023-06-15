clear;clc;close all;

thd = derive_tello_hip_differential();

opts = struct('cpp', true, 'with_header', true);
gen_code(thd.phi, 'thd_phi', opts);
gen_code(thd.kikd, 'thd_kikd', opts);
gen_code(thd.J_dy_2_dqd, 'thd_J_dy_2_dqd', opts);
gen_code(thd.k, 'thd_k', opts);
gen_code(thd.g, 'thd_g', opts);