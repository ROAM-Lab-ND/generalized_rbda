/* Auto-generated CasADi codegen declarations for Tello Generic constraints. */
#ifndef GRBDA_CODEGEN_TELLO_KG_DGDQ_H
#define GRBDA_CODEGEN_TELLO_KG_DGDQ_H

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

extern "C" int tello_hip_kg_dgdq(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" const casadi_int* tello_hip_kg_dgdq_sparsity_out(casadi_int i);
extern "C" int tello_hip_kg_dgdq_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);

extern "C" int tello_knee_kg_dgdq(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" const casadi_int* tello_knee_kg_dgdq_sparsity_out(casadi_int i);
extern "C" int tello_knee_kg_dgdq_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);

#endif // GRBDA_CODEGEN_TELLO_KG_DGDQ_H
