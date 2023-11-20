/* This file was automatically generated by CasADi 3.6.3.
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

extern "C" int thd_bias(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int thd_bias_alloc_mem(void);
extern "C" int thd_bias_init_mem(int mem);
extern "C" void thd_bias_free_mem(int mem);
extern "C" int thd_bias_checkout(void);
extern "C" void thd_bias_release(int mem);
extern "C" void thd_bias_incref(void);
extern "C" void thd_bias_decref(void);
extern "C" casadi_int thd_bias_n_in(void);
extern "C" casadi_int thd_bias_n_out(void);
extern "C" casadi_real thd_bias_default_in(casadi_int i);
extern "C" const char* thd_bias_name_in(casadi_int i);
extern "C" const char* thd_bias_name_out(casadi_int i);
extern "C" const casadi_int* thd_bias_sparsity_in(casadi_int i);
extern "C" const casadi_int* thd_bias_sparsity_out(casadi_int i);
extern "C" int thd_bias_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define thd_bias_SZ_ARG 4
#define thd_bias_SZ_RES 2
#define thd_bias_SZ_IW 0
#define thd_bias_SZ_W 95