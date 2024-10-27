#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3832217124692423628) {
   out_3832217124692423628[0] = delta_x[0] + nom_x[0];
   out_3832217124692423628[1] = delta_x[1] + nom_x[1];
   out_3832217124692423628[2] = delta_x[2] + nom_x[2];
   out_3832217124692423628[3] = delta_x[3] + nom_x[3];
   out_3832217124692423628[4] = delta_x[4] + nom_x[4];
   out_3832217124692423628[5] = delta_x[5] + nom_x[5];
   out_3832217124692423628[6] = delta_x[6] + nom_x[6];
   out_3832217124692423628[7] = delta_x[7] + nom_x[7];
   out_3832217124692423628[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8550848976003632472) {
   out_8550848976003632472[0] = -nom_x[0] + true_x[0];
   out_8550848976003632472[1] = -nom_x[1] + true_x[1];
   out_8550848976003632472[2] = -nom_x[2] + true_x[2];
   out_8550848976003632472[3] = -nom_x[3] + true_x[3];
   out_8550848976003632472[4] = -nom_x[4] + true_x[4];
   out_8550848976003632472[5] = -nom_x[5] + true_x[5];
   out_8550848976003632472[6] = -nom_x[6] + true_x[6];
   out_8550848976003632472[7] = -nom_x[7] + true_x[7];
   out_8550848976003632472[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6150847992731042195) {
   out_6150847992731042195[0] = 1.0;
   out_6150847992731042195[1] = 0;
   out_6150847992731042195[2] = 0;
   out_6150847992731042195[3] = 0;
   out_6150847992731042195[4] = 0;
   out_6150847992731042195[5] = 0;
   out_6150847992731042195[6] = 0;
   out_6150847992731042195[7] = 0;
   out_6150847992731042195[8] = 0;
   out_6150847992731042195[9] = 0;
   out_6150847992731042195[10] = 1.0;
   out_6150847992731042195[11] = 0;
   out_6150847992731042195[12] = 0;
   out_6150847992731042195[13] = 0;
   out_6150847992731042195[14] = 0;
   out_6150847992731042195[15] = 0;
   out_6150847992731042195[16] = 0;
   out_6150847992731042195[17] = 0;
   out_6150847992731042195[18] = 0;
   out_6150847992731042195[19] = 0;
   out_6150847992731042195[20] = 1.0;
   out_6150847992731042195[21] = 0;
   out_6150847992731042195[22] = 0;
   out_6150847992731042195[23] = 0;
   out_6150847992731042195[24] = 0;
   out_6150847992731042195[25] = 0;
   out_6150847992731042195[26] = 0;
   out_6150847992731042195[27] = 0;
   out_6150847992731042195[28] = 0;
   out_6150847992731042195[29] = 0;
   out_6150847992731042195[30] = 1.0;
   out_6150847992731042195[31] = 0;
   out_6150847992731042195[32] = 0;
   out_6150847992731042195[33] = 0;
   out_6150847992731042195[34] = 0;
   out_6150847992731042195[35] = 0;
   out_6150847992731042195[36] = 0;
   out_6150847992731042195[37] = 0;
   out_6150847992731042195[38] = 0;
   out_6150847992731042195[39] = 0;
   out_6150847992731042195[40] = 1.0;
   out_6150847992731042195[41] = 0;
   out_6150847992731042195[42] = 0;
   out_6150847992731042195[43] = 0;
   out_6150847992731042195[44] = 0;
   out_6150847992731042195[45] = 0;
   out_6150847992731042195[46] = 0;
   out_6150847992731042195[47] = 0;
   out_6150847992731042195[48] = 0;
   out_6150847992731042195[49] = 0;
   out_6150847992731042195[50] = 1.0;
   out_6150847992731042195[51] = 0;
   out_6150847992731042195[52] = 0;
   out_6150847992731042195[53] = 0;
   out_6150847992731042195[54] = 0;
   out_6150847992731042195[55] = 0;
   out_6150847992731042195[56] = 0;
   out_6150847992731042195[57] = 0;
   out_6150847992731042195[58] = 0;
   out_6150847992731042195[59] = 0;
   out_6150847992731042195[60] = 1.0;
   out_6150847992731042195[61] = 0;
   out_6150847992731042195[62] = 0;
   out_6150847992731042195[63] = 0;
   out_6150847992731042195[64] = 0;
   out_6150847992731042195[65] = 0;
   out_6150847992731042195[66] = 0;
   out_6150847992731042195[67] = 0;
   out_6150847992731042195[68] = 0;
   out_6150847992731042195[69] = 0;
   out_6150847992731042195[70] = 1.0;
   out_6150847992731042195[71] = 0;
   out_6150847992731042195[72] = 0;
   out_6150847992731042195[73] = 0;
   out_6150847992731042195[74] = 0;
   out_6150847992731042195[75] = 0;
   out_6150847992731042195[76] = 0;
   out_6150847992731042195[77] = 0;
   out_6150847992731042195[78] = 0;
   out_6150847992731042195[79] = 0;
   out_6150847992731042195[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2114866482915183913) {
   out_2114866482915183913[0] = state[0];
   out_2114866482915183913[1] = state[1];
   out_2114866482915183913[2] = state[2];
   out_2114866482915183913[3] = state[3];
   out_2114866482915183913[4] = state[4];
   out_2114866482915183913[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2114866482915183913[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2114866482915183913[7] = state[7];
   out_2114866482915183913[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7930859491116433429) {
   out_7930859491116433429[0] = 1;
   out_7930859491116433429[1] = 0;
   out_7930859491116433429[2] = 0;
   out_7930859491116433429[3] = 0;
   out_7930859491116433429[4] = 0;
   out_7930859491116433429[5] = 0;
   out_7930859491116433429[6] = 0;
   out_7930859491116433429[7] = 0;
   out_7930859491116433429[8] = 0;
   out_7930859491116433429[9] = 0;
   out_7930859491116433429[10] = 1;
   out_7930859491116433429[11] = 0;
   out_7930859491116433429[12] = 0;
   out_7930859491116433429[13] = 0;
   out_7930859491116433429[14] = 0;
   out_7930859491116433429[15] = 0;
   out_7930859491116433429[16] = 0;
   out_7930859491116433429[17] = 0;
   out_7930859491116433429[18] = 0;
   out_7930859491116433429[19] = 0;
   out_7930859491116433429[20] = 1;
   out_7930859491116433429[21] = 0;
   out_7930859491116433429[22] = 0;
   out_7930859491116433429[23] = 0;
   out_7930859491116433429[24] = 0;
   out_7930859491116433429[25] = 0;
   out_7930859491116433429[26] = 0;
   out_7930859491116433429[27] = 0;
   out_7930859491116433429[28] = 0;
   out_7930859491116433429[29] = 0;
   out_7930859491116433429[30] = 1;
   out_7930859491116433429[31] = 0;
   out_7930859491116433429[32] = 0;
   out_7930859491116433429[33] = 0;
   out_7930859491116433429[34] = 0;
   out_7930859491116433429[35] = 0;
   out_7930859491116433429[36] = 0;
   out_7930859491116433429[37] = 0;
   out_7930859491116433429[38] = 0;
   out_7930859491116433429[39] = 0;
   out_7930859491116433429[40] = 1;
   out_7930859491116433429[41] = 0;
   out_7930859491116433429[42] = 0;
   out_7930859491116433429[43] = 0;
   out_7930859491116433429[44] = 0;
   out_7930859491116433429[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7930859491116433429[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7930859491116433429[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7930859491116433429[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7930859491116433429[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7930859491116433429[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7930859491116433429[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7930859491116433429[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7930859491116433429[53] = -9.8000000000000007*dt;
   out_7930859491116433429[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7930859491116433429[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7930859491116433429[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7930859491116433429[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7930859491116433429[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7930859491116433429[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7930859491116433429[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7930859491116433429[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7930859491116433429[62] = 0;
   out_7930859491116433429[63] = 0;
   out_7930859491116433429[64] = 0;
   out_7930859491116433429[65] = 0;
   out_7930859491116433429[66] = 0;
   out_7930859491116433429[67] = 0;
   out_7930859491116433429[68] = 0;
   out_7930859491116433429[69] = 0;
   out_7930859491116433429[70] = 1;
   out_7930859491116433429[71] = 0;
   out_7930859491116433429[72] = 0;
   out_7930859491116433429[73] = 0;
   out_7930859491116433429[74] = 0;
   out_7930859491116433429[75] = 0;
   out_7930859491116433429[76] = 0;
   out_7930859491116433429[77] = 0;
   out_7930859491116433429[78] = 0;
   out_7930859491116433429[79] = 0;
   out_7930859491116433429[80] = 1;
}
void h_25(double *state, double *unused, double *out_1573131070607391915) {
   out_1573131070607391915[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7622478135120030861) {
   out_7622478135120030861[0] = 0;
   out_7622478135120030861[1] = 0;
   out_7622478135120030861[2] = 0;
   out_7622478135120030861[3] = 0;
   out_7622478135120030861[4] = 0;
   out_7622478135120030861[5] = 0;
   out_7622478135120030861[6] = 1;
   out_7622478135120030861[7] = 0;
   out_7622478135120030861[8] = 0;
}
void h_24(double *state, double *unused, double *out_8236009456035901526) {
   out_8236009456035901526[0] = state[4];
   out_8236009456035901526[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5445263711512880888) {
   out_5445263711512880888[0] = 0;
   out_5445263711512880888[1] = 0;
   out_5445263711512880888[2] = 0;
   out_5445263711512880888[3] = 0;
   out_5445263711512880888[4] = 1;
   out_5445263711512880888[5] = 0;
   out_5445263711512880888[6] = 0;
   out_5445263711512880888[7] = 0;
   out_5445263711512880888[8] = 0;
   out_5445263711512880888[9] = 0;
   out_5445263711512880888[10] = 0;
   out_5445263711512880888[11] = 0;
   out_5445263711512880888[12] = 0;
   out_5445263711512880888[13] = 0;
   out_5445263711512880888[14] = 1;
   out_5445263711512880888[15] = 0;
   out_5445263711512880888[16] = 0;
   out_5445263711512880888[17] = 0;
}
void h_30(double *state, double *unused, double *out_7740245110578526218) {
   out_7740245110578526218[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5104145176612782234) {
   out_5104145176612782234[0] = 0;
   out_5104145176612782234[1] = 0;
   out_5104145176612782234[2] = 0;
   out_5104145176612782234[3] = 0;
   out_5104145176612782234[4] = 1;
   out_5104145176612782234[5] = 0;
   out_5104145176612782234[6] = 0;
   out_5104145176612782234[7] = 0;
   out_5104145176612782234[8] = 0;
}
void h_26(double *state, double *unused, double *out_5704825695470179753) {
   out_5704825695470179753[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4317952165359230260) {
   out_4317952165359230260[0] = 0;
   out_4317952165359230260[1] = 0;
   out_4317952165359230260[2] = 0;
   out_4317952165359230260[3] = 0;
   out_4317952165359230260[4] = 0;
   out_4317952165359230260[5] = 0;
   out_4317952165359230260[6] = 0;
   out_4317952165359230260[7] = 1;
   out_4317952165359230260[8] = 0;
}
void h_27(double *state, double *unused, double *out_3428542197053453730) {
   out_3428542197053453730[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2880551105428839017) {
   out_2880551105428839017[0] = 0;
   out_2880551105428839017[1] = 0;
   out_2880551105428839017[2] = 0;
   out_2880551105428839017[3] = 1;
   out_2880551105428839017[4] = 0;
   out_2880551105428839017[5] = 0;
   out_2880551105428839017[6] = 0;
   out_2880551105428839017[7] = 0;
   out_2880551105428839017[8] = 0;
}
void h_29(double *state, double *unused, double *out_7290225673481955382) {
   out_7290225673481955382[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4593913832298390050) {
   out_4593913832298390050[0] = 0;
   out_4593913832298390050[1] = 1;
   out_4593913832298390050[2] = 0;
   out_4593913832298390050[3] = 0;
   out_4593913832298390050[4] = 0;
   out_4593913832298390050[5] = 0;
   out_4593913832298390050[6] = 0;
   out_4593913832298390050[7] = 0;
   out_4593913832298390050[8] = 0;
}
void h_28(double *state, double *unused, double *out_3827601520039997979) {
   out_3827601520039997979[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8770431224341630992) {
   out_8770431224341630992[0] = 1;
   out_8770431224341630992[1] = 0;
   out_8770431224341630992[2] = 0;
   out_8770431224341630992[3] = 0;
   out_8770431224341630992[4] = 0;
   out_8770431224341630992[5] = 0;
   out_8770431224341630992[6] = 0;
   out_8770431224341630992[7] = 0;
   out_8770431224341630992[8] = 0;
}
void h_31(double *state, double *unused, double *out_122499677978508748) {
   out_122499677978508748[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4944160267592581736) {
   out_4944160267592581736[0] = 0;
   out_4944160267592581736[1] = 0;
   out_4944160267592581736[2] = 0;
   out_4944160267592581736[3] = 0;
   out_4944160267592581736[4] = 0;
   out_4944160267592581736[5] = 0;
   out_4944160267592581736[6] = 0;
   out_4944160267592581736[7] = 0;
   out_4944160267592581736[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_3832217124692423628) {
  err_fun(nom_x, delta_x, out_3832217124692423628);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8550848976003632472) {
  inv_err_fun(nom_x, true_x, out_8550848976003632472);
}
void car_H_mod_fun(double *state, double *out_6150847992731042195) {
  H_mod_fun(state, out_6150847992731042195);
}
void car_f_fun(double *state, double dt, double *out_2114866482915183913) {
  f_fun(state,  dt, out_2114866482915183913);
}
void car_F_fun(double *state, double dt, double *out_7930859491116433429) {
  F_fun(state,  dt, out_7930859491116433429);
}
void car_h_25(double *state, double *unused, double *out_1573131070607391915) {
  h_25(state, unused, out_1573131070607391915);
}
void car_H_25(double *state, double *unused, double *out_7622478135120030861) {
  H_25(state, unused, out_7622478135120030861);
}
void car_h_24(double *state, double *unused, double *out_8236009456035901526) {
  h_24(state, unused, out_8236009456035901526);
}
void car_H_24(double *state, double *unused, double *out_5445263711512880888) {
  H_24(state, unused, out_5445263711512880888);
}
void car_h_30(double *state, double *unused, double *out_7740245110578526218) {
  h_30(state, unused, out_7740245110578526218);
}
void car_H_30(double *state, double *unused, double *out_5104145176612782234) {
  H_30(state, unused, out_5104145176612782234);
}
void car_h_26(double *state, double *unused, double *out_5704825695470179753) {
  h_26(state, unused, out_5704825695470179753);
}
void car_H_26(double *state, double *unused, double *out_4317952165359230260) {
  H_26(state, unused, out_4317952165359230260);
}
void car_h_27(double *state, double *unused, double *out_3428542197053453730) {
  h_27(state, unused, out_3428542197053453730);
}
void car_H_27(double *state, double *unused, double *out_2880551105428839017) {
  H_27(state, unused, out_2880551105428839017);
}
void car_h_29(double *state, double *unused, double *out_7290225673481955382) {
  h_29(state, unused, out_7290225673481955382);
}
void car_H_29(double *state, double *unused, double *out_4593913832298390050) {
  H_29(state, unused, out_4593913832298390050);
}
void car_h_28(double *state, double *unused, double *out_3827601520039997979) {
  h_28(state, unused, out_3827601520039997979);
}
void car_H_28(double *state, double *unused, double *out_8770431224341630992) {
  H_28(state, unused, out_8770431224341630992);
}
void car_h_31(double *state, double *unused, double *out_122499677978508748) {
  h_31(state, unused, out_122499677978508748);
}
void car_H_31(double *state, double *unused, double *out_4944160267592581736) {
  H_31(state, unused, out_4944160267592581736);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
