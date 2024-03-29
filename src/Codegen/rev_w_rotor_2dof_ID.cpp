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
  #define CASADI_PREFIX(ID) rev_w_rotor_2dof_ID_ ## ID
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

/* RevWithRotors2DofInvDyn:(i0[2],i1[2],i2[2])->(o0[2]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a4, a5, a6, a7, a8, a9;
  a0=1.0037000000000000e+00;
  a1=arg[0]? arg[0][0] : 0;
  a2=cos(a1);
  a3=5.0000000000000000e-01;
  a4=arg[0]? arg[0][1] : 0;
  a4=(a1+a4);
  a5=cos(a4);
  a6=(a3*a5);
  a2=(a2+a6);
  a6=(a2+a2);
  a6=(a3*a6);
  a6=(a2*a6);
  a0=(a0+a6);
  a6=sin(a1);
  a7=sin(a4);
  a8=(a3*a7);
  a6=(a6+a8);
  a8=(a6+a6);
  a8=(a3*a8);
  a8=(a6*a8);
  a0=(a0+a8);
  a8=1.;
  a0=(a0+a8);
  a8=cos(a1);
  a8=(a3*a8);
  a9=casadi_sq(a8);
  a0=(a0+a9);
  a9=sin(a1);
  a9=(a3*a9);
  a10=casadi_sq(a9);
  a0=(a0+a10);
  a10=arg[2]? arg[2][0] : 0;
  a0=(a0*a10);
  a11=1.0005999999999999e+00;
  a12=(a3*a5);
  a12=(a2*a12);
  a11=(a11+a12);
  a12=(a3*a7);
  a12=(a6*a12);
  a11=(a11+a12);
  a12=arg[2]? arg[2][1] : 0;
  a13=(a11*a12);
  a0=(a0+a13);
  a13=arg[1]? arg[1][0] : 0;
  a14=cos(a1);
  a14=(a3*a14);
  a15=(a13*a14);
  a15=(a15+a15);
  a15=(a9*a15);
  a9=(a9*a13);
  a16=(a9+a9);
  a16=(a16*a14);
  a15=(a15+a16);
  a15=(a3*a15);
  a16=sin(a1);
  a16=(a3*a16);
  a14=(a13*a16);
  a14=(a14+a14);
  a14=(a8*a14);
  a8=(a8*a13);
  a17=(a8+a8);
  a17=(a17*a16);
  a14=(a14+a17);
  a14=(a3*a14);
  a15=(a15-a14);
  a14=cos(a1);
  a17=cos(a4);
  a16=(a3*a17);
  a14=(a14+a16);
  a16=(a13*a14);
  a18=arg[1]? arg[1][1] : 0;
  a19=(a3*a17);
  a20=(a18*a19);
  a16=(a16+a20);
  a16=(a16+a16);
  a20=(a6*a16);
  a21=(a6*a13);
  a7=(a3*a7);
  a22=(a7*a18);
  a21=(a21+a22);
  a22=(a21+a21);
  a14=(a22*a14);
  a20=(a20+a14);
  a20=(a3*a20);
  a15=(a15+a20);
  a20=sin(a1);
  a14=sin(a4);
  a23=(a3*a14);
  a20=(a20+a23);
  a23=(a13*a20);
  a24=(a3*a14);
  a25=(a18*a24);
  a23=(a23+a25);
  a23=(a23+a23);
  a25=(a2*a23);
  a26=(a2*a13);
  a5=(a3*a5);
  a27=(a5*a18);
  a26=(a26+a27);
  a27=(a26+a26);
  a20=(a27*a20);
  a25=(a25+a20);
  a25=(a3*a25);
  a15=(a15-a25);
  a15=(a15*a13);
  a25=(a3*a17);
  a20=(a13*a25);
  a17=(a3*a17);
  a28=(a18*a17);
  a20=(a20+a28);
  a20=(a20+a20);
  a6=(a6*a20);
  a25=(a22*a25);
  a6=(a6+a25);
  a6=(a3*a6);
  a25=(a3*a14);
  a28=(a13*a25);
  a14=(a3*a14);
  a29=(a18*a14);
  a28=(a28+a29);
  a28=(a28+a28);
  a2=(a2*a28);
  a25=(a27*a25);
  a2=(a2+a25);
  a2=(a3*a2);
  a6=(a6-a2);
  a6=(a6*a18);
  a15=(a15+a6);
  a9=(a9+a9);
  a6=cos(a1);
  a6=(a3*a6);
  a6=(a13*a6);
  a9=(a9*a6);
  a9=(a3*a9);
  a8=(a8+a8);
  a6=sin(a1);
  a6=(a3*a6);
  a6=(a13*a6);
  a8=(a8*a6);
  a8=(a3*a8);
  a9=(a9-a8);
  a21=(a21+a21);
  a8=cos(a1);
  a6=cos(a4);
  a2=(a3*a6);
  a8=(a8+a2);
  a8=(a13*a8);
  a2=(a3*a6);
  a2=(a18*a2);
  a8=(a8+a2);
  a8=(a21*a8);
  a8=(a3*a8);
  a9=(a9+a8);
  a26=(a26+a26);
  a8=sin(a1);
  a2=sin(a4);
  a25=(a3*a2);
  a8=(a8+a25);
  a8=(a13*a8);
  a25=(a3*a2);
  a25=(a18*a25);
  a8=(a8+a25);
  a8=(a26*a8);
  a8=(a3*a8);
  a9=(a9-a8);
  a8=1.4715000000000000e+01;
  a1=sin(a1);
  a8=(a8*a1);
  a9=(a9-a8);
  a8=4.9050000000000002e+00;
  a4=sin(a4);
  a1=(a8*a4);
  a9=(a9-a1);
  a15=(a15-a9);
  a0=(a0+a15);
  if (res[0]!=0) res[0][0]=a0;
  a11=(a11*a10);
  a10=1.0036000000000000e+00;
  a0=casadi_sq(a5);
  a10=(a10+a0);
  a0=casadi_sq(a7);
  a10=(a10+a0);
  a10=(a10*a12);
  a11=(a11+a10);
  a16=(a7*a16);
  a19=(a22*a19);
  a16=(a16+a19);
  a16=(a3*a16);
  a23=(a5*a23);
  a24=(a27*a24);
  a23=(a23+a24);
  a23=(a3*a23);
  a16=(a16-a23);
  a16=(a16*a13);
  a7=(a7*a20);
  a22=(a22*a17);
  a7=(a7+a22);
  a7=(a3*a7);
  a5=(a5*a28);
  a27=(a27*a14);
  a5=(a5+a27);
  a5=(a3*a5);
  a7=(a7-a5);
  a7=(a7*a18);
  a16=(a16+a7);
  a7=(a3*a6);
  a7=(a13*a7);
  a6=(a3*a6);
  a6=(a18*a6);
  a7=(a7+a6);
  a21=(a21*a7);
  a21=(a3*a21);
  a7=(a3*a2);
  a13=(a13*a7);
  a2=(a3*a2);
  a18=(a18*a2);
  a13=(a13+a18);
  a26=(a26*a13);
  a3=(a3*a26);
  a21=(a21-a3);
  a8=(a8*a4);
  a21=(a21-a8);
  a16=(a16-a21);
  a11=(a11+a16);
  if (res[0]!=0) res[0][1]=a11;
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int RevWithRotors2DofInvDyn(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

extern "C" CASADI_SYMBOL_EXPORT int RevWithRotors2DofInvDyn_alloc_mem(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int RevWithRotors2DofInvDyn_init_mem(int mem) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void RevWithRotors2DofInvDyn_free_mem(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT int RevWithRotors2DofInvDyn_checkout(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void RevWithRotors2DofInvDyn_release(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT void RevWithRotors2DofInvDyn_incref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT void RevWithRotors2DofInvDyn_decref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT casadi_int RevWithRotors2DofInvDyn_n_in(void) { return 3;}

extern "C" CASADI_SYMBOL_EXPORT casadi_int RevWithRotors2DofInvDyn_n_out(void) { return 1;}

extern "C" CASADI_SYMBOL_EXPORT casadi_real RevWithRotors2DofInvDyn_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* RevWithRotors2DofInvDyn_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* RevWithRotors2DofInvDyn_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* RevWithRotors2DofInvDyn_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s0;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* RevWithRotors2DofInvDyn_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT int RevWithRotors2DofInvDyn_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


