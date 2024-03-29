/* This file was automatically generated by CasADi 3.6.3.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) tkad_IK_vel_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[6] = {2, 1, 0, 2, 0, 1};

/* tkad_IK_vel:(i0[2],i1[2])->(o0[2]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[1]? arg[1][0] : 0;
  a1=2.0799999999999999e-02;
  a2=7.;
  a3=arg[0]? arg[0][1] : 0;
  a4=1.9896753472735356e+00;
  a3=(a3+a4);
  a4=cos(a3);
  a4=(a2*a4);
  a5=2500.;
  a4=(a4/a5);
  a1=(a1-a4);
  a4=3.1573672058406521e-03;
  a1=(a1-a4);
  a4=2.0522886837964237e-02;
  a6=91.;
  a7=cos(a3);
  a7=(a6*a7);
  a8=5000.;
  a7=(a7/a8);
  a4=(a4+a7);
  a7=1.3813481525552854e+02;
  a8=cos(a3);
  a8=(a7*a8);
  a9=50000.;
  a8=(a8/a9);
  a4=(a4-a8);
  a8=5.0276961068873298e+01;
  a10=sin(a3);
  a10=(a8*a10);
  a10=(a10/a9);
  a4=(a4-a10);
  a10=2.6135840000000000e-02;
  a4=(a4-a10);
  a10=(a1*a4);
  a9=-1.1491876815742470e-03;
  a11=sin(a3);
  a11=(a2*a11);
  a11=(a11/a5);
  a9=(a9-a11);
  a11=casadi_sq(a9);
  a5=casadi_sq(a1);
  a11=(a11+a5);
  a5=casadi_sq(a4);
  a11=(a11-a5);
  a11=sqrt(a11);
  a5=(a9*a11);
  a10=(a10-a5);
  a5=(a1*a11);
  a12=(a9*a4);
  a5=(a5+a12);
  a12=casadi_sq(a5);
  a13=casadi_sq(a10);
  a12=(a12+a13);
  a10=(a10/a12);
  a13=4.0000000000000002e-04;
  a14=sin(a3);
  a15=arg[1]? arg[1][1] : 0;
  a14=(a14*a15);
  a14=(a2*a14);
  a14=(a13*a14);
  a16=(a11*a14);
  a17=(a1+a1);
  a17=(a17*a14);
  a18=(a9+a9);
  a19=cos(a3);
  a19=(a19*a15);
  a2=(a2*a19);
  a13=(a13*a2);
  a18=(a18*a13);
  a17=(a17-a18);
  a18=(a4+a4);
  a2=2.0000000000000002e-05;
  a19=sin(a3);
  a19=(a19*a15);
  a7=(a7*a19);
  a7=(a2*a7);
  a19=2.0000000000000001e-04;
  a20=sin(a3);
  a20=(a20*a15);
  a6=(a6*a20);
  a19=(a19*a6);
  a7=(a7-a19);
  a3=cos(a3);
  a3=(a3*a15);
  a8=(a8*a3);
  a2=(a2*a8);
  a7=(a7-a2);
  a18=(a18*a7);
  a17=(a17-a18);
  a18=(a11+a11);
  a17=(a17/a18);
  a18=(a1*a17);
  a16=(a16+a18);
  a18=(a9*a7);
  a2=(a4*a13);
  a18=(a18-a2);
  a16=(a16+a18);
  a10=(a10*a16);
  a5=(a5/a12);
  a4=(a4*a14);
  a1=(a1*a7);
  a4=(a4+a1);
  a9=(a9*a17);
  a11=(a11*a13);
  a9=(a9-a11);
  a4=(a4-a9);
  a5=(a5*a4);
  a10=(a10-a5);
  a5=(a0-a10);
  if (res[0]!=0) res[0][0]=a5;
  a0=(a0+a10);
  if (res[0]!=0) res[0][1]=a0;
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int tkad_IK_vel(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

extern "C" CASADI_SYMBOL_EXPORT int tkad_IK_vel_alloc_mem(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int tkad_IK_vel_init_mem(int mem) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void tkad_IK_vel_free_mem(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT int tkad_IK_vel_checkout(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void tkad_IK_vel_release(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT void tkad_IK_vel_incref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT void tkad_IK_vel_decref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT casadi_int tkad_IK_vel_n_in(void) { return 2;}

extern "C" CASADI_SYMBOL_EXPORT casadi_int tkad_IK_vel_n_out(void) { return 1;}

extern "C" CASADI_SYMBOL_EXPORT casadi_real tkad_IK_vel_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* tkad_IK_vel_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* tkad_IK_vel_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* tkad_IK_vel_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* tkad_IK_vel_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT int tkad_IK_vel_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


