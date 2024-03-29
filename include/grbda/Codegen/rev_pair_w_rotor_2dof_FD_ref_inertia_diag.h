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

extern "C" int RevPairWithRotors2DofFwdDynReflectedInertiaDiag(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int RevPairWithRotors2DofFwdDynReflectedInertiaDiag_alloc_mem(void);
extern "C" int RevPairWithRotors2DofFwdDynReflectedInertiaDiag_init_mem(int mem);
extern "C" void RevPairWithRotors2DofFwdDynReflectedInertiaDiag_free_mem(int mem);
extern "C" int RevPairWithRotors2DofFwdDynReflectedInertiaDiag_checkout(void);
extern "C" void RevPairWithRotors2DofFwdDynReflectedInertiaDiag_release(int mem);
extern "C" void RevPairWithRotors2DofFwdDynReflectedInertiaDiag_incref(void);
extern "C" void RevPairWithRotors2DofFwdDynReflectedInertiaDiag_decref(void);
extern "C" casadi_int RevPairWithRotors2DofFwdDynReflectedInertiaDiag_n_in(void);
extern "C" casadi_int RevPairWithRotors2DofFwdDynReflectedInertiaDiag_n_out(void);
extern "C" casadi_real RevPairWithRotors2DofFwdDynReflectedInertiaDiag_default_in(casadi_int i);
extern "C" const char* RevPairWithRotors2DofFwdDynReflectedInertiaDiag_name_in(casadi_int i);
extern "C" const char* RevPairWithRotors2DofFwdDynReflectedInertiaDiag_name_out(casadi_int i);
extern "C" const casadi_int* RevPairWithRotors2DofFwdDynReflectedInertiaDiag_sparsity_in(casadi_int i);
extern "C" const casadi_int* RevPairWithRotors2DofFwdDynReflectedInertiaDiag_sparsity_out(casadi_int i);
extern "C" int RevPairWithRotors2DofFwdDynReflectedInertiaDiag_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define RevPairWithRotors2DofFwdDynReflectedInertiaDiag_SZ_ARG 3
#define RevPairWithRotors2DofFwdDynReflectedInertiaDiag_SZ_RES 1
#define RevPairWithRotors2DofFwdDynReflectedInertiaDiag_SZ_IW 0
#define RevPairWithRotors2DofFwdDynReflectedInertiaDiag_SZ_W 34
