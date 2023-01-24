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
  #define CASADI_PREFIX(ID) usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
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

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

static const casadi_int casadi_s0[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s1[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[20] = {16, 1, 0, 16, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
static const casadi_int casadi_s4[27] = {9, 8, 0, 2, 4, 6, 8, 10, 12, 14, 16, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7};
static const casadi_int casadi_s5[3] = {8, 0, 0};

/* usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt:(i0[8],i1,i2[],i3[16])->(o0[8],o1[9x8,16nz],o2[8x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real *rr, *ss;
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, w19, w20, w21, w22, w23, w24, *w25=w+25, *w26=w+41;
  /* #0: @0 = input[0][5] */
  w0 = arg[0] ? arg[0][5] : 0;
  /* #1: @1 = input[3][0] */
  w1 = arg[3] ? arg[3][0] : 0;
  /* #2: @1 = (@0-@1) */
  w1  = (w0-w1);
  /* #3: @2 = sq(@1) */
  w2 = casadi_sq( w1 );
  /* #4: @3 = input[0][6] */
  w3 = arg[0] ? arg[0][6] : 0;
  /* #5: @4 = input[3][1] */
  w4 = arg[3] ? arg[3][1] : 0;
  /* #6: @4 = (@3-@4) */
  w4  = (w3-w4);
  /* #7: @5 = sq(@4) */
  w5 = casadi_sq( w4 );
  /* #8: @2 = (@2+@5) */
  w2 += w5;
  /* #9: @2 = sqrt(@2) */
  w2 = sqrt( w2 );
  /* #10: output[0][0] = @2 */
  if (res[0]) res[0][0] = w2;
  /* #11: @5 = input[3][2] */
  w5 = arg[3] ? arg[3][2] : 0;
  /* #12: @5 = (@0-@5) */
  w5  = (w0-w5);
  /* #13: @6 = sq(@5) */
  w6 = casadi_sq( w5 );
  /* #14: @7 = input[3][3] */
  w7 = arg[3] ? arg[3][3] : 0;
  /* #15: @7 = (@3-@7) */
  w7  = (w3-w7);
  /* #16: @8 = sq(@7) */
  w8 = casadi_sq( w7 );
  /* #17: @6 = (@6+@8) */
  w6 += w8;
  /* #18: @6 = sqrt(@6) */
  w6 = sqrt( w6 );
  /* #19: output[0][1] = @6 */
  if (res[0]) res[0][1] = w6;
  /* #20: @8 = input[3][4] */
  w8 = arg[3] ? arg[3][4] : 0;
  /* #21: @8 = (@0-@8) */
  w8  = (w0-w8);
  /* #22: @9 = sq(@8) */
  w9 = casadi_sq( w8 );
  /* #23: @10 = input[3][5] */
  w10 = arg[3] ? arg[3][5] : 0;
  /* #24: @10 = (@3-@10) */
  w10  = (w3-w10);
  /* #25: @11 = sq(@10) */
  w11 = casadi_sq( w10 );
  /* #26: @9 = (@9+@11) */
  w9 += w11;
  /* #27: @9 = sqrt(@9) */
  w9 = sqrt( w9 );
  /* #28: output[0][2] = @9 */
  if (res[0]) res[0][2] = w9;
  /* #29: @11 = input[3][6] */
  w11 = arg[3] ? arg[3][6] : 0;
  /* #30: @11 = (@0-@11) */
  w11  = (w0-w11);
  /* #31: @12 = sq(@11) */
  w12 = casadi_sq( w11 );
  /* #32: @13 = input[3][7] */
  w13 = arg[3] ? arg[3][7] : 0;
  /* #33: @13 = (@3-@13) */
  w13  = (w3-w13);
  /* #34: @14 = sq(@13) */
  w14 = casadi_sq( w13 );
  /* #35: @12 = (@12+@14) */
  w12 += w14;
  /* #36: @12 = sqrt(@12) */
  w12 = sqrt( w12 );
  /* #37: output[0][3] = @12 */
  if (res[0]) res[0][3] = w12;
  /* #38: @14 = input[3][8] */
  w14 = arg[3] ? arg[3][8] : 0;
  /* #39: @14 = (@0-@14) */
  w14  = (w0-w14);
  /* #40: @15 = sq(@14) */
  w15 = casadi_sq( w14 );
  /* #41: @16 = input[3][9] */
  w16 = arg[3] ? arg[3][9] : 0;
  /* #42: @16 = (@3-@16) */
  w16  = (w3-w16);
  /* #43: @17 = sq(@16) */
  w17 = casadi_sq( w16 );
  /* #44: @15 = (@15+@17) */
  w15 += w17;
  /* #45: @15 = sqrt(@15) */
  w15 = sqrt( w15 );
  /* #46: output[0][4] = @15 */
  if (res[0]) res[0][4] = w15;
  /* #47: @17 = input[3][10] */
  w17 = arg[3] ? arg[3][10] : 0;
  /* #48: @17 = (@0-@17) */
  w17  = (w0-w17);
  /* #49: @18 = sq(@17) */
  w18 = casadi_sq( w17 );
  /* #50: @19 = input[3][11] */
  w19 = arg[3] ? arg[3][11] : 0;
  /* #51: @19 = (@3-@19) */
  w19  = (w3-w19);
  /* #52: @20 = sq(@19) */
  w20 = casadi_sq( w19 );
  /* #53: @18 = (@18+@20) */
  w18 += w20;
  /* #54: @18 = sqrt(@18) */
  w18 = sqrt( w18 );
  /* #55: output[0][5] = @18 */
  if (res[0]) res[0][5] = w18;
  /* #56: @20 = input[3][12] */
  w20 = arg[3] ? arg[3][12] : 0;
  /* #57: @20 = (@0-@20) */
  w20  = (w0-w20);
  /* #58: @21 = sq(@20) */
  w21 = casadi_sq( w20 );
  /* #59: @22 = input[3][13] */
  w22 = arg[3] ? arg[3][13] : 0;
  /* #60: @22 = (@3-@22) */
  w22  = (w3-w22);
  /* #61: @23 = sq(@22) */
  w23 = casadi_sq( w22 );
  /* #62: @21 = (@21+@23) */
  w21 += w23;
  /* #63: @21 = sqrt(@21) */
  w21 = sqrt( w21 );
  /* #64: output[0][6] = @21 */
  if (res[0]) res[0][6] = w21;
  /* #65: @23 = input[3][14] */
  w23 = arg[3] ? arg[3][14] : 0;
  /* #66: @0 = (@0-@23) */
  w0 -= w23;
  /* #67: @23 = sq(@0) */
  w23 = casadi_sq( w0 );
  /* #68: @24 = input[3][15] */
  w24 = arg[3] ? arg[3][15] : 0;
  /* #69: @3 = (@3-@24) */
  w3 -= w24;
  /* #70: @24 = sq(@3) */
  w24 = casadi_sq( w3 );
  /* #71: @23 = (@23+@24) */
  w23 += w24;
  /* #72: @23 = sqrt(@23) */
  w23 = sqrt( w23 );
  /* #73: output[0][7] = @23 */
  if (res[0]) res[0][7] = w23;
  /* #74: @25 = zeros(9x8,16nz) */
  casadi_clear(w25, 16);
  /* #75: @1 = (2.*@1) */
  w1 = (2.* w1 );
  /* #76: @26 = ones(9x1,8nz) */
  casadi_fill(w26, 8, 1.);
  /* #77: {NULL, NULL, NULL, NULL, NULL, NULL, @24, NULL, NULL} = vertsplit(@26) */
  w24 = w26[6];
  /* #78: @1 = (@1*@24) */
  w1 *= w24;
  /* #79: @2 = (2.*@2) */
  w2 = (2.* w2 );
  /* #80: @1 = (@1/@2) */
  w1 /= w2;
  /* #81: @5 = (2.*@5) */
  w5 = (2.* w5 );
  /* #82: @5 = (@5*@24) */
  w5 *= w24;
  /* #83: @6 = (2.*@6) */
  w6 = (2.* w6 );
  /* #84: @5 = (@5/@6) */
  w5 /= w6;
  /* #85: @8 = (2.*@8) */
  w8 = (2.* w8 );
  /* #86: @8 = (@8*@24) */
  w8 *= w24;
  /* #87: @9 = (2.*@9) */
  w9 = (2.* w9 );
  /* #88: @8 = (@8/@9) */
  w8 /= w9;
  /* #89: @11 = (2.*@11) */
  w11 = (2.* w11 );
  /* #90: @11 = (@11*@24) */
  w11 *= w24;
  /* #91: @12 = (2.*@12) */
  w12 = (2.* w12 );
  /* #92: @11 = (@11/@12) */
  w11 /= w12;
  /* #93: @14 = (2.*@14) */
  w14 = (2.* w14 );
  /* #94: @14 = (@14*@24) */
  w14 *= w24;
  /* #95: @15 = (2.*@15) */
  w15 = (2.* w15 );
  /* #96: @14 = (@14/@15) */
  w14 /= w15;
  /* #97: @17 = (2.*@17) */
  w17 = (2.* w17 );
  /* #98: @17 = (@17*@24) */
  w17 *= w24;
  /* #99: @18 = (2.*@18) */
  w18 = (2.* w18 );
  /* #100: @17 = (@17/@18) */
  w17 /= w18;
  /* #101: @20 = (2.*@20) */
  w20 = (2.* w20 );
  /* #102: @20 = (@20*@24) */
  w20 *= w24;
  /* #103: @21 = (2.*@21) */
  w21 = (2.* w21 );
  /* #104: @20 = (@20/@21) */
  w20 /= w21;
  /* #105: @0 = (2.*@0) */
  w0 = (2.* w0 );
  /* #106: @0 = (@0*@24) */
  w0 *= w24;
  /* #107: @23 = (2.*@23) */
  w23 = (2.* w23 );
  /* #108: @0 = (@0/@23) */
  w0 /= w23;
  /* #109: @26 = vertcat(@1, @5, @8, @11, @14, @17, @20, @0) */
  rr=w26;
  *rr++ = w1;
  *rr++ = w5;
  *rr++ = w8;
  *rr++ = w11;
  *rr++ = w14;
  *rr++ = w17;
  *rr++ = w20;
  *rr++ = w0;
  /* #110: (@25[:16:2] = @26) */
  for (rr=w25+0, ss=w26; rr!=w25+16; rr+=2) *rr = *ss++;
  /* #111: @4 = (2.*@4) */
  w4 = (2.* w4 );
  /* #112: @1 = ones(9x1,1nz) */
  w1 = 1.;
  /* #113: {NULL, NULL, NULL, NULL, NULL, NULL, NULL, @5, NULL} = vertsplit(@1) */
  w5 = w1;
  /* #114: @4 = (@4*@5) */
  w4 *= w5;
  /* #115: @4 = (@4/@2) */
  w4 /= w2;
  /* #116: @7 = (2.*@7) */
  w7 = (2.* w7 );
  /* #117: @7 = (@7*@5) */
  w7 *= w5;
  /* #118: @7 = (@7/@6) */
  w7 /= w6;
  /* #119: @10 = (2.*@10) */
  w10 = (2.* w10 );
  /* #120: @10 = (@10*@5) */
  w10 *= w5;
  /* #121: @10 = (@10/@9) */
  w10 /= w9;
  /* #122: @13 = (2.*@13) */
  w13 = (2.* w13 );
  /* #123: @13 = (@13*@5) */
  w13 *= w5;
  /* #124: @13 = (@13/@12) */
  w13 /= w12;
  /* #125: @16 = (2.*@16) */
  w16 = (2.* w16 );
  /* #126: @16 = (@16*@5) */
  w16 *= w5;
  /* #127: @16 = (@16/@15) */
  w16 /= w15;
  /* #128: @19 = (2.*@19) */
  w19 = (2.* w19 );
  /* #129: @19 = (@19*@5) */
  w19 *= w5;
  /* #130: @19 = (@19/@18) */
  w19 /= w18;
  /* #131: @22 = (2.*@22) */
  w22 = (2.* w22 );
  /* #132: @22 = (@22*@5) */
  w22 *= w5;
  /* #133: @22 = (@22/@21) */
  w22 /= w21;
  /* #134: @3 = (2.*@3) */
  w3 = (2.* w3 );
  /* #135: @3 = (@3*@5) */
  w3 *= w5;
  /* #136: @3 = (@3/@23) */
  w3 /= w23;
  /* #137: @26 = vertcat(@4, @7, @10, @13, @16, @19, @22, @3) */
  rr=w26;
  *rr++ = w4;
  *rr++ = w7;
  *rr++ = w10;
  *rr++ = w13;
  *rr++ = w16;
  *rr++ = w19;
  *rr++ = w22;
  *rr++ = w3;
  /* #138: (@25[1:17:2] = @26) */
  for (rr=w25+1, ss=w26; rr!=w25+17; rr+=2) *rr = *ss++;
  /* #139: output[1][0] = @25 */
  casadi_copy(w25, 16, res[1]);
  return 0;
}

CASADI_SYMBOL_EXPORT int usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_incref(void) {
}

CASADI_SYMBOL_EXPORT void usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int usv_model_guidance_ca1_constr_h_fun_jac_uxt_zt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 12;
  if (sz_res) *sz_res = 12;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 49;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
