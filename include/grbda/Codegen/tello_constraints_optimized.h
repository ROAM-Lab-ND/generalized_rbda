/* Auto-generated optimized Tello constraint functions */
#ifndef GRBDA_TELLO_CONSTRAINTS_OPTIMIZED_H
#define GRBDA_TELLO_CONSTRAINTS_OPTIMIZED_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Optimized G matrix evaluation for hip differential
 * Replaces CasADi symbolic evaluation with direct C code
 *
 * Input:  q[4] = [rotor1, rotor2, gimbal, thigh]
 * Output: G[4][2] = explicit Jacobian matrix
 */
static inline void tello_hip_G_optimized(const double* q, double* G_flat)
{
    const double N = 6.0;
    const double y_1 = q[0] / N;
    const double y_2 = q[1] / N;
    const double ql_1 = q[2];  // gimbal
    const double ql_2 = q[3];  // thigh

    // Pre-compute trig functions
    const double cos_y1 = cos(y_1);
    const double sin_y1 = sin(y_1);
    const double cos_y2 = cos(y_2);
    const double sin_y2 = sin(y_2);
    const double cos_ql1 = cos(ql_1);
    const double sin_ql1 = sin(ql_1);
    const double cos_ql2 = cos(ql_2);
    const double sin_ql2 = sin(ql_2);

    // K matrix (constraint Jacobian dphi/dq)
    // This is derived from the phi equations in Tello.cpp lines 151-153
    double K[2][4];

    // dphi1/dq (first constraint)
    K[0][0] = (57.0*cos_y1)/(2500.0*N) - (8.0*cos_ql2*(-sin_y1))/(625.0*N) -
              (7.0*cos_y1*sin_ql1)/(625.0*N) - (8.0*cos_ql1*cos_y1*sin_ql2)/(625.0*N);
    K[0][1] = 0.0;  // phi1 doesn't depend on y_2
    K[0][2] = (49.0*sin_ql1)/5000.0 - (399.0*cos_ql1)/20000.0 -
              (57.0*(-sin_ql1)*sin_ql2)/2500.0 + (7.0*sin_y1*cos_ql1)/625.0 +
              (7.0*cos_ql1*sin_ql2)/625.0 - (8.0*(-sin_ql1)*sin_y1*sin_ql2)/625.0;
    K[0][3] = -(8.0*cos_y1*(-sin_ql2))/625.0 - (57.0*cos_ql1*cos_ql2)/2500.0 +
               (7.0*sin_ql1*cos_ql2)/625.0 - (8.0*cos_ql1*sin_y1*cos_ql2)/625.0;

    // dphi2/dq (second constraint)
    K[1][0] = 0.0;  // phi2 doesn't depend on y_1
    K[1][1] = (57.0*cos_y2)/(2500.0*N) - (8.0*cos_ql2*(-sin_y2))/(625.0*N) +
              (7.0*cos_y2*sin_ql1)/(625.0*N) - (8.0*cos_ql1*cos_y2*sin_ql2)/(625.0*N);
    K[1][2] = (49.0*sin_ql1)/5000.0 - (399.0*cos_ql1)/20000.0 -
              (57.0*(-sin_ql1)*sin_ql2)/2500.0 - (7.0*sin_y2*cos_ql1)/625.0 +
              (7.0*cos_ql1*sin_ql2)/625.0 - (8.0*(-sin_ql1)*sin_y2*sin_ql2)/625.0;
    K[1][3] = -(8.0*cos_y2*(-sin_ql2))/625.0 - (57.0*cos_ql1*cos_ql2)/2500.0 -
               (7.0*sin_ql1*cos_ql2)/625.0 - (8.0*cos_ql1*sin_y2*cos_ql2)/625.0;

    // Split K into independent (Ki) and dependent (Kd) parts
    // Ki = K[:, 0:2], Kd = K[:, 2:4]
    double Ki[2][2] = {{K[0][0], K[0][1]},
                       {K[1][0], K[1][1]}};
    double Kd[2][2] = {{K[0][2], K[0][3]},
                       {K[1][2], K[1][3]}};

    // Compute G = [I; -inv(Kd)*Ki] where I is 2x2 identity
    // For 2x2 matrix, use analytical inverse: inv([[a,b],[c,d]]) = (1/det)*[[d,-b],[-c,a]]
    double a = Kd[0][0], b = Kd[0][1];
    double c = Kd[1][0], d = Kd[1][1];
    double det = a*d - b*c;

    // inv(Kd) * Ki for each column
    double inv_Kd_Ki[2][2];
    for (int j = 0; j < 2; j++) {
        double e = Ki[0][j];
        double f = Ki[1][j];
        inv_Kd_Ki[0][j] = (d*e - b*f) / det;
        inv_Kd_Ki[1][j] = (-c*e + a*f) / det;
    }

    // Build G matrix (4x2 in column-major order)
    // G[0:2, :] = I (identity)
    G_flat[0] = 1.0; G_flat[4] = 0.0;  // Column 0, rows 0-1
    G_flat[1] = 0.0; G_flat[5] = 1.0;  // Column 1, rows 0-1

    // G[2:4, :] = -inv(Kd)*Ki
    G_flat[2] = -inv_Kd_Ki[0][0]; G_flat[6] = -inv_Kd_Ki[0][1];  // Column 0-1, row 2
    G_flat[3] = -inv_Kd_Ki[1][0]; G_flat[7] = -inv_Kd_Ki[1][1];  // Column 0-1, row 3
}

/**
 * Optimized g vector evaluation for hip differential
 *
 * Input:  q[4] = positions, v[4] = velocities
 * Output: g[4] = explicit bias vector
 */
static inline void tello_hip_g_optimized(const double* q, const double* v, double* g)
{
    // For now, return zeros (full implementation would compute k and solve Kd*g = k)
    // This is a simplified version - full version would follow same pattern as G
    g[0] = 0.0;
    g[1] = 0.0;
    g[2] = 0.0;
    g[3] = 0.0;
}

#ifdef __cplusplus
}
#endif

#endif /* GRBDA_TELLO_CONSTRAINTS_OPTIMIZED_H */
