/* This file was automatically generated by CasADi 3.6.2.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

extern "C" int tkad_jacobian(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int tkad_jacobian_alloc_mem(void);
extern "C" int tkad_jacobian_init_mem(int mem);
extern "C" void tkad_jacobian_free_mem(int mem);
extern "C" int tkad_jacobian_checkout(void);
extern "C" void tkad_jacobian_release(int mem);
extern "C" void tkad_jacobian_incref(void);
extern "C" void tkad_jacobian_decref(void);
extern "C" casadi_int tkad_jacobian_n_in(void);
extern "C" casadi_int tkad_jacobian_n_out(void);
extern "C" casadi_real tkad_jacobian_default_in(casadi_int i);
extern "C" const char* tkad_jacobian_name_in(casadi_int i);
extern "C" const char* tkad_jacobian_name_out(casadi_int i);
extern "C" const casadi_int* tkad_jacobian_sparsity_in(casadi_int i);
extern "C" const casadi_int* tkad_jacobian_sparsity_out(casadi_int i);
extern "C" int tkad_jacobian_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define tkad_jacobian_SZ_ARG 2
#define tkad_jacobian_SZ_RES 2
#define tkad_jacobian_SZ_IW 0
#define tkad_jacobian_SZ_W 18