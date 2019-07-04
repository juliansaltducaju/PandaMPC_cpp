/* Produced by CVXGEN, 2019-06-03 11:00:40 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif




typedef struct msg_t {
                    double j1[500];
                    double j2[500];
                    double j3[500];
                    double j4[500];
                    double j5[500];
                    double j6[500];
                    double j7[500];
                } msg;

typedef struct Params_t {
  double z_0[3];
  double Q[9];
  double R[1];
  double z_min[3];
  double z_max[3];
  double u_min[1];
  double u_max[1];
  double A[9];
  double x_0[3];
  double B[3];
  double Dz[3];
  double goal[3];
  double *z[1];
  double *x[1];
} Params;
typedef struct Vars_t {
  double *u_0; /* 1 rows. */
  double *z_1; /* 3 rows. */
  double *u_1; /* 1 rows. */
  double *z_2; /* 3 rows. */
  double *u_2; /* 1 rows. */
  double *z_3; /* 3 rows. */
  double *u_3; /* 1 rows. */
  double *z_4; /* 3 rows. */
  double *u_4; /* 1 rows. */
  double *z_5; /* 3 rows. */
  double *u_5; /* 1 rows. */
  double *z_6; /* 3 rows. */
  double *u_6; /* 1 rows. */
  double *z_7; /* 3 rows. */
  double *u_7; /* 1 rows. */
  double *z_8; /* 3 rows. */
  double *u_8; /* 1 rows. */
  double *z_9; /* 3 rows. */
  double *u_9; /* 1 rows. */
  double *z_10; /* 3 rows. */
  double *u_10; /* 1 rows. */
  double *z_11; /* 3 rows. */
  double *u_11; /* 1 rows. */
  double *z_12; /* 3 rows. */
  double *u_12; /* 1 rows. */
  double *z_13; /* 3 rows. */
  double *u_13; /* 1 rows. */
  double *z_14; /* 3 rows. */
  double *u_14; /* 1 rows. */
  double *z_15; /* 3 rows. */
  double *u_15; /* 1 rows. */
  double *z_16; /* 3 rows. */
  double *u_16; /* 1 rows. */
  double *z_17; /* 3 rows. */
  double *u_17; /* 1 rows. */
  double *z_18; /* 3 rows. */
  double *u_18; /* 1 rows. */
  double *z_19; /* 3 rows. */
  double *u_19; /* 1 rows. */
  double *z_20; /* 3 rows. */
  double *u_20; /* 1 rows. */
  double *z_21; /* 3 rows. */
  double *u_21; /* 1 rows. */
  double *z_22; /* 3 rows. */
  double *u_22; /* 1 rows. */
  double *z_23; /* 3 rows. */
  double *u_23; /* 1 rows. */
  double *z_24; /* 3 rows. */
  double *u_24; /* 1 rows. */
  double *z_25; /* 3 rows. */
  double *u_25; /* 1 rows. */
  double *x_1; /* 3 rows. */
  double *x_2; /* 3 rows. */
  double *x_3; /* 3 rows. */
  double *x_4; /* 3 rows. */
  double *x_5; /* 3 rows. */
  double *x_6; /* 3 rows. */
  double *x_7; /* 3 rows. */
  double *x_8; /* 3 rows. */
  double *x_9; /* 3 rows. */
  double *x_10; /* 3 rows. */
  double *x_11; /* 3 rows. */
  double *x_12; /* 3 rows. */
  double *x_13; /* 3 rows. */
  double *x_14; /* 3 rows. */
  double *x_15; /* 3 rows. */
  double *x_16; /* 3 rows. */
  double *x_17; /* 3 rows. */
  double *x_18; /* 3 rows. */
  double *x_19; /* 3 rows. */
  double *x_20; /* 3 rows. */
  double *x_21; /* 3 rows. */
  double *x_22; /* 3 rows. */
  double *x_23; /* 3 rows. */
  double *x_24; /* 3 rows. */
  double *x_25; /* 3 rows. */
  double *u[26];
  double *z[26];
  double *x[26];
} Vars;

typedef struct data_t {
	int T;
        double resample;
        double span;
        double posgoal [7];
	int size_res;
	int flag;
	
	double x_prev_j1_1 [26];
        double x_prev_j1_2 [26];
        double x_prev_j1_3 [26];
        double z_prev_j1_1 [26];
        double z_prev_j1_2 [26];
        double z_prev_j1_3 [26];
        
	double x_prev_j2_1 [26];
        double x_prev_j2_2 [26];
        double x_prev_j2_3 [26];
        double z_prev_j2_1 [26];
        double z_prev_j2_2 [26];
        double z_prev_j2_3 [26];
	
	double x_prev_j3_1 [26];
        double x_prev_j3_2 [26];
        double x_prev_j3_3 [26];
        double z_prev_j3_1 [26];
        double z_prev_j3_2 [26];
        double z_prev_j3_3 [26];

	double x_prev_j4_1 [26];
        double x_prev_j4_2 [26];
        double x_prev_j4_3 [26];
        double z_prev_j4_1 [26];
        double z_prev_j4_2 [26];
        double z_prev_j4_3 [26];
	
	double x_prev_j5_1 [26];
        double x_prev_j5_2 [26];
        double x_prev_j5_3 [26];
        double z_prev_j5_1 [26];
        double z_prev_j5_2 [26];
        double z_prev_j5_3 [26];
	
	double x_prev_j6_1 [26];
        double x_prev_j6_2 [26];
        double x_prev_j6_3 [26];
        double z_prev_j6_1 [26];
        double z_prev_j6_2 [26];
        double z_prev_j6_3 [26];

	double x_prev_j7_1 [26];
        double x_prev_j7_2 [26];
        double x_prev_j7_3 [26];
        double z_prev_j7_1 [26];
        double z_prev_j7_2 [26];
        double z_prev_j7_3[26];

} DataMPC;

typedef struct Workspace_t {
  double h[202];
  double s_inv[202];
  double s_inv_z[202];
  double b[156];
  double q[176];
  double rhs[736];
  double x[736];
  double *s;
  double *z;
  double *y;
  double lhs_aff[736];
  double lhs_cc[736];
  double buffer[736];
  double buffer2[736];
  double KKT[1581];
  double L[1684];
  double d[736];
  double v[736];
  double d_inv[736];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_57302585344[1];
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);


#ifdef __cplusplus
    extern "C"
    {
#endif
    double eval_gap(void);
		void set_defaults(void);
		void setup_pointers(void);
		void setup_indexed_params(void);
		void setup_indexed_optvars(void);
		void setup_indexing(void);
		void set_start(void);
		double eval_objv(void);
		void fillrhs_aff(void);
		void fillrhs_cc(void);
		void refine(double *target, double *var);
		double calc_ineq_resid_squared(void);
		double calc_eq_resid_squared(void);
		void better_start(void);
		void fillrhs_start(void);
		long solve(void);
#ifdef __cplusplus
    }
#endif

/* Function definitions in MPC.cc: */
int main();
void load_static_data(int joint);
void model(double h);
void goal(double posgoal);
void init_z(double zpos, double zvel, double zacel);
void init_x(double xpos, double xvel, double xacel);
void initial_state(double hprev, double resample, int k, int joint, DataMPC dataMPC);
int send(int internal_flag1, msg msgMPC2);
int receive_first(int internal_flag2, double veloc1[200], double veloc2[200], double veloc3[200], double veloc4[200], double veloc5[200], double veloc6[200], double veloc7[200], msg msgMPC);
int receive(int internal_flag2, double veloc1[200], double veloc2[200], double veloc3[200], double veloc4[200], double veloc5[200], double veloc6[200], double veloc7[200], int indice, msg msgMPC);
void store_next_initial(int joint, Vars vars);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#endif
