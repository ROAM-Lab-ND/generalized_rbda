/* This file was automatically generated by CasADi 3.6.2.
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
  #define CASADI_PREFIX(ID) tkad_small_k_ ## ID
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

static const casadi_int casadi_s0[6] = {2, 1, 0, 2, 0, 1};

/* tkad_small_k:(i0[2],i1[2],i2[2],i3[2])->(o0[2]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a3, a4, a5, a6, a7, a8, a9;
  a0=0.;
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[2]? arg[2][0] : 0;
  a1=1.6000000000000001e-03;
  a2=13.;
  a3=5.0000000000000000e-01;
  a4=arg[0]? arg[0][0] : 0;
  a5=2.;
  a6=(a4/a5);
  a7=arg[0]? arg[0][1] : 0;
  a8=(a7/a5);
  a6=(a6-a8);
  a8=1.0325367854798453e+00;
  a6=(a6+a8);
  a8=cos(a6);
  a9=(a3*a0);
  a10=arg[2]? arg[2][1] : 0;
  a11=(a3*a10);
  a9=(a9-a11);
  a8=(a8*a9);
  a9=(a3*a8);
  a9=(a2*a9);
  a9=(a1*a9);
  a11=1.6000000000000001e-04;
  a12=21.;
  a13=(a4/a5);
  a14=(a7/a5);
  a13=(a13-a14);
  a14=1.3816026358787112e+00;
  a13=(a13+a14);
  a14=cos(a13);
  a15=(a3*a0);
  a16=(a3*a10);
  a15=(a15-a16);
  a14=(a14*a15);
  a15=(a3*a14);
  a15=(a12*a15);
  a15=(a11*a15);
  a9=(a9-a15);
  a15=4.0000000000000002e-04;
  a16=7.;
  a4=(a4/a5);
  a7=(a7/a5);
  a4=(a4-a7);
  a7=arg[1]? arg[1][1] : 0;
  a4=(a4+a7);
  a5=1.4514158059584845e+00;
  a4=(a4+a5);
  a5=sin(a4);
  a17=(a3*a0);
  a18=(a3*a10);
  a17=(a17-a18);
  a18=arg[3]? arg[3][1] : 0;
  a17=(a17+a18);
  a5=(a5*a17);
  a19=(a3*a5);
  a19=(a16*a19);
  a19=(a15*a19);
  a9=(a9+a19);
  a19=2.0000000000000001e-04;
  a20=91.;
  a21=4.1887902047863906e-01;
  a21=(a7+a21);
  a22=cos(a21);
  a22=(a20*a22);
  a22=(a19*a22);
  a23=cos(a4);
  a23=(a16*a23);
  a23=(a15*a23);
  a22=(a22-a23);
  a23=2.0000000000000002e-05;
  a24=147.;
  a25=6.9813170079773182e-02;
  a7=(a7+a25);
  a25=cos(a7);
  a25=(a24*a25);
  a25=(a23*a25);
  a22=(a22-a25);
  a9=(a9/a22);
  a6=sin(a6);
  a25=(a3*a6);
  a25=(a2*a25);
  a25=(a1*a25);
  a13=sin(a13);
  a26=(a3*a13);
  a26=(a12*a26);
  a26=(a11*a26);
  a25=(a25-a26);
  a26=cos(a4);
  a3=(a3*a26);
  a3=(a16*a3);
  a3=(a15*a3);
  a25=(a25-a3);
  a25=(a25/a22);
  a25=(a25/a22);
  a4=sin(a4);
  a4=(a4*a17);
  a4=(a16*a4);
  a4=(a15*a4);
  a21=sin(a21);
  a21=(a21*a18);
  a20=(a20*a21);
  a19=(a19*a20);
  a4=(a4-a19);
  a7=sin(a7);
  a7=(a7*a18);
  a24=(a24*a7);
  a23=(a23*a24);
  a4=(a4+a23);
  a25=(a25*a4);
  a9=(a9-a25);
  a0=(a0*a9);
  a9=-5.0000000000000000e-01;
  a8=(a9*a8);
  a8=(a2*a8);
  a8=(a1*a8);
  a14=(a9*a14);
  a14=(a12*a14);
  a14=(a11*a14);
  a8=(a8-a14);
  a5=(a9*a5);
  a5=(a16*a5);
  a5=(a15*a5);
  a8=(a8+a5);
  a8=(a8/a22);
  a6=(a9*a6);
  a2=(a2*a6);
  a1=(a1*a2);
  a13=(a9*a13);
  a12=(a12*a13);
  a11=(a11*a12);
  a1=(a1-a11);
  a9=(a9*a26);
  a16=(a16*a9);
  a15=(a15*a16);
  a1=(a1-a15);
  a1=(a1/a22);
  a1=(a1/a22);
  a1=(a1*a4);
  a8=(a8-a1);
  a10=(a10*a8);
  a0=(a0+a10);
  a0=(-a0);
  if (res[0]!=0) res[0][1]=a0;
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int tkad_small_k(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

extern "C" CASADI_SYMBOL_EXPORT int tkad_small_k_alloc_mem(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int tkad_small_k_init_mem(int mem) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void tkad_small_k_free_mem(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT int tkad_small_k_checkout(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void tkad_small_k_release(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT void tkad_small_k_incref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT void tkad_small_k_decref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT casadi_int tkad_small_k_n_in(void) { return 4;}

extern "C" CASADI_SYMBOL_EXPORT casadi_int tkad_small_k_n_out(void) { return 1;}

extern "C" CASADI_SYMBOL_EXPORT casadi_real tkad_small_k_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* tkad_small_k_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* tkad_small_k_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* tkad_small_k_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s0;
    case 3: return casadi_s0;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* tkad_small_k_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT int tkad_small_k_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

