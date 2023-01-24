/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) usv_model_pf_ca_constr_h_fun_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
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

static const casadi_int casadi_s0[16] = {12, 1, 0, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const casadi_int casadi_s1[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[20] = {16, 1, 0, 16, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
static const casadi_int casadi_s4[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};

/* usv_model_pf_ca_constr_h_fun:(i0[12],i1[2],i2[],i3[16])->(o0[8]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1, w2, w3;
  /* #0: @0 = input[0][8] */
  w0 = arg[0] ? arg[0][8] : 0;
  /* #1: @1 = input[3][0] */
  w1 = arg[3] ? arg[3][0] : 0;
  /* #2: @1 = (@0-@1) */
  w1  = (w0-w1);
  /* #3: @1 = sq(@1) */
  w1 = casadi_sq( w1 );
  /* #4: @2 = input[0][9] */
  w2 = arg[0] ? arg[0][9] : 0;
  /* #5: @3 = input[3][1] */
  w3 = arg[3] ? arg[3][1] : 0;
  /* #6: @3 = (@2-@3) */
  w3  = (w2-w3);
  /* #7: @3 = sq(@3) */
  w3 = casadi_sq( w3 );
  /* #8: @1 = (@1+@3) */
  w1 += w3;
  /* #9: @1 = sqrt(@1) */
  w1 = sqrt( w1 );
  /* #10: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  /* #11: @1 = input[3][2] */
  w1 = arg[3] ? arg[3][2] : 0;
  /* #12: @1 = (@0-@1) */
  w1  = (w0-w1);
  /* #13: @1 = sq(@1) */
  w1 = casadi_sq( w1 );
  /* #14: @3 = input[3][3] */
  w3 = arg[3] ? arg[3][3] : 0;
  /* #15: @3 = (@2-@3) */
  w3  = (w2-w3);
  /* #16: @3 = sq(@3) */
  w3 = casadi_sq( w3 );
  /* #17: @1 = (@1+@3) */
  w1 += w3;
  /* #18: @1 = sqrt(@1) */
  w1 = sqrt( w1 );
  /* #19: output[0][1] = @1 */
  if (res[0]) res[0][1] = w1;
  /* #20: @1 = input[3][4] */
  w1 = arg[3] ? arg[3][4] : 0;
  /* #21: @1 = (@0-@1) */
  w1  = (w0-w1);
  /* #22: @1 = sq(@1) */
  w1 = casadi_sq( w1 );
  /* #23: @3 = input[3][5] */
  w3 = arg[3] ? arg[3][5] : 0;
  /* #24: @3 = (@2-@3) */
  w3  = (w2-w3);
  /* #25: @3 = sq(@3) */
  w3 = casadi_sq( w3 );
  /* #26: @1 = (@1+@3) */
  w1 += w3;
  /* #27: @1 = sqrt(@1) */
  w1 = sqrt( w1 );
  /* #28: output[0][2] = @1 */
  if (res[0]) res[0][2] = w1;
  /* #29: @1 = input[3][6] */
  w1 = arg[3] ? arg[3][6] : 0;
  /* #30: @1 = (@0-@1) */
  w1  = (w0-w1);
  /* #31: @1 = sq(@1) */
  w1 = casadi_sq( w1 );
  /* #32: @3 = input[3][7] */
  w3 = arg[3] ? arg[3][7] : 0;
  /* #33: @3 = (@2-@3) */
  w3  = (w2-w3);
  /* #34: @3 = sq(@3) */
  w3 = casadi_sq( w3 );
  /* #35: @1 = (@1+@3) */
  w1 += w3;
  /* #36: @1 = sqrt(@1) */
  w1 = sqrt( w1 );
  /* #37: output[0][3] = @1 */
  if (res[0]) res[0][3] = w1;
  /* #38: @1 = input[3][8] */
  w1 = arg[3] ? arg[3][8] : 0;
  /* #39: @1 = (@0-@1) */
  w1  = (w0-w1);
  /* #40: @1 = sq(@1) */
  w1 = casadi_sq( w1 );
  /* #41: @3 = input[3][9] */
  w3 = arg[3] ? arg[3][9] : 0;
  /* #42: @3 = (@2-@3) */
  w3  = (w2-w3);
  /* #43: @3 = sq(@3) */
  w3 = casadi_sq( w3 );
  /* #44: @1 = (@1+@3) */
  w1 += w3;
  /* #45: @1 = sqrt(@1) */
  w1 = sqrt( w1 );
  /* #46: output[0][4] = @1 */
  if (res[0]) res[0][4] = w1;
  /* #47: @1 = input[3][10] */
  w1 = arg[3] ? arg[3][10] : 0;
  /* #48: @1 = (@0-@1) */
  w1  = (w0-w1);
  /* #49: @1 = sq(@1) */
  w1 = casadi_sq( w1 );
  /* #50: @3 = input[3][11] */
  w3 = arg[3] ? arg[3][11] : 0;
  /* #51: @3 = (@2-@3) */
  w3  = (w2-w3);
  /* #52: @3 = sq(@3) */
  w3 = casadi_sq( w3 );
  /* #53: @1 = (@1+@3) */
  w1 += w3;
  /* #54: @1 = sqrt(@1) */
  w1 = sqrt( w1 );
  /* #55: output[0][5] = @1 */
  if (res[0]) res[0][5] = w1;
  /* #56: @1 = input[3][12] */
  w1 = arg[3] ? arg[3][12] : 0;
  /* #57: @1 = (@0-@1) */
  w1  = (w0-w1);
  /* #58: @1 = sq(@1) */
  w1 = casadi_sq( w1 );
  /* #59: @3 = input[3][13] */
  w3 = arg[3] ? arg[3][13] : 0;
  /* #60: @3 = (@2-@3) */
  w3  = (w2-w3);
  /* #61: @3 = sq(@3) */
  w3 = casadi_sq( w3 );
  /* #62: @1 = (@1+@3) */
  w1 += w3;
  /* #63: @1 = sqrt(@1) */
  w1 = sqrt( w1 );
  /* #64: output[0][6] = @1 */
  if (res[0]) res[0][6] = w1;
  /* #65: @1 = input[3][14] */
  w1 = arg[3] ? arg[3][14] : 0;
  /* #66: @0 = (@0-@1) */
  w0 -= w1;
  /* #67: @0 = sq(@0) */
  w0 = casadi_sq( w0 );
  /* #68: @1 = input[3][15] */
  w1 = arg[3] ? arg[3][15] : 0;
  /* #69: @2 = (@2-@1) */
  w2 -= w1;
  /* #70: @2 = sq(@2) */
  w2 = casadi_sq( w2 );
  /* #71: @0 = (@0+@2) */
  w0 += w2;
  /* #72: @0 = sqrt(@0) */
  w0 = sqrt( w0 );
  /* #73: output[0][7] = @0 */
  if (res[0]) res[0][7] = w0;
  return 0;
}

CASADI_SYMBOL_EXPORT int usv_model_pf_ca_constr_h_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int usv_model_pf_ca_constr_h_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int usv_model_pf_ca_constr_h_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void usv_model_pf_ca_constr_h_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int usv_model_pf_ca_constr_h_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void usv_model_pf_ca_constr_h_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void usv_model_pf_ca_constr_h_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void usv_model_pf_ca_constr_h_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int usv_model_pf_ca_constr_h_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int usv_model_pf_ca_constr_h_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real usv_model_pf_ca_constr_h_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* usv_model_pf_ca_constr_h_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* usv_model_pf_ca_constr_h_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* usv_model_pf_ca_constr_h_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* usv_model_pf_ca_constr_h_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int usv_model_pf_ca_constr_h_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 4;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
