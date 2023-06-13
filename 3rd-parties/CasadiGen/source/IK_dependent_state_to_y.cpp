/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) IK_dependent_state_to_y_ ## ID
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

/* IK_dependent_state_to_y:(i0[2])->(o0[2]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=8.;
  a1=arg[0]? arg[0][1] : 0;
  a2=cos(a1);
  a2=(a0*a2);
  a3=625.;
  a2=(a2/a3);
  a4=2.2800000000000001e-02;
  a5=arg[0]? arg[0][0] : 0;
  a6=cos(a5);
  a6=(a0*a6);
  a7=sin(a1);
  a6=(a6*a7);
  a6=(a6/a3);
  a6=(a4-a6);
  a7=7.;
  a8=sin(a5);
  a8=(a7*a8);
  a8=(a8/a3);
  a6=(a6-a8);
  a8=casadi_sq(a6);
  a9=casadi_sq(a2);
  a8=(a8+a9);
  a9=49.;
  a10=cos(a5);
  a10=(a9*a10);
  a11=5000.;
  a10=(a10/a11);
  a12=399.;
  a13=sin(a5);
  a13=(a12*a13);
  a14=20000.;
  a13=(a13/a14);
  a10=(a10+a13);
  a13=57.;
  a15=cos(a5);
  a15=(a13*a15);
  a16=sin(a1);
  a15=(a15*a16);
  a16=2500.;
  a15=(a15/a16);
  a10=(a10+a15);
  a15=sin(a5);
  a15=(a7*a15);
  a17=sin(a1);
  a15=(a15*a17);
  a15=(a15/a3);
  a10=(a10-a15);
  a15=1.8881249999999999e-02;
  a10=(a10-a15);
  a17=casadi_sq(a10);
  a8=(a8-a17);
  a8=sqrt(a8);
  a17=(a2*a8);
  a18=(a6*a10);
  a17=(a17+a18);
  a6=(a6*a8);
  a2=(a2*a10);
  a6=(a6-a2);
  a17=atan2(a17,a6);
  if (res[0]!=0) res[0][0]=a17;
  a17=cos(a1);
  a17=(a0*a17);
  a17=(a17/a3);
  a6=sin(a5);
  a6=(a7*a6);
  a6=(a6/a3);
  a2=cos(a5);
  a0=(a0*a2);
  a2=sin(a1);
  a0=(a0*a2);
  a0=(a0/a3);
  a6=(a6-a0);
  a6=(a6+a4);
  a4=casadi_sq(a6);
  a0=casadi_sq(a17);
  a4=(a4+a0);
  a0=cos(a5);
  a9=(a9*a0);
  a9=(a9/a11);
  a11=sin(a5);
  a12=(a12*a11);
  a12=(a12/a14);
  a9=(a9-a12);
  a12=cos(a5);
  a13=(a13*a12);
  a12=sin(a1);
  a13=(a13*a12);
  a13=(a13/a16);
  a9=(a9+a13);
  a5=sin(a5);
  a7=(a7*a5);
  a1=sin(a1);
  a7=(a7*a1);
  a7=(a7/a3);
  a9=(a9+a7);
  a9=(a9-a15);
  a15=casadi_sq(a9);
  a4=(a4-a15);
  a4=sqrt(a4);
  a15=(a17*a4);
  a7=(a6*a9);
  a15=(a15+a7);
  a6=(a6*a4);
  a17=(a17*a9);
  a6=(a6-a17);
  a15=atan2(a15,a6);
  if (res[0]!=0) res[0][1]=a15;
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int IK_dependent_state_to_y(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

extern "C" CASADI_SYMBOL_EXPORT int IK_dependent_state_to_y_alloc_mem(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int IK_dependent_state_to_y_init_mem(int mem) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void IK_dependent_state_to_y_free_mem(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT int IK_dependent_state_to_y_checkout(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void IK_dependent_state_to_y_release(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT void IK_dependent_state_to_y_incref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT void IK_dependent_state_to_y_decref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT casadi_int IK_dependent_state_to_y_n_in(void) { return 1;}

extern "C" CASADI_SYMBOL_EXPORT casadi_int IK_dependent_state_to_y_n_out(void) { return 1;}

extern "C" CASADI_SYMBOL_EXPORT casadi_real IK_dependent_state_to_y_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* IK_dependent_state_to_y_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* IK_dependent_state_to_y_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* IK_dependent_state_to_y_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* IK_dependent_state_to_y_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT int IK_dependent_state_to_y_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

