/*
 * Generic 2nd-order ADRC: Extended State Observer + PD controller +
 * disturbance cancellation, with saturation and rate limiting.
 *
 * The ESO matrices (ESO_AD, ESO_M, ESO_L) below are PRECOMPUTED OFFLINE
 * for a specific omega_o and dt (see generate_eso_matrices.py). This is
 * deliberate: computing a matrix exponential on a microcontroller isn't
 * practical, and Ad/M don't change at runtime anyway since they only
 * depend on omega_o and dt, both fixed at design time. b0 CAN change at
 * runtime (e.g. for gain scheduling) since it only affects the forcing
 * term, not the discretization matrices themselves.
 *
 * Regenerate these constants any time you change omega_o or your loop's
 * sample rate.
 */

#ifndef ADRC_H
#define ADRC_H

/* ---- Precomputed for omega_o = 20 rad/s, dt = 0.005 s (200 Hz) ---- */
static const double ESO_AD[3][3] = {
    {7.2839412152e-01, 4.2979777357e-03, 1.1310467725e-05},
    {-5.2480570246e+00, 9.8627278566e-01, 4.9766057992e-03},
    {-3.4383821885e+01, -9.0483741804e-02, 9.9984534693e-01},
};
static const double ESO_M[3][3] = {
    {4.2979777357e-03, 1.1310467725e-05, 1.9331633783e-08},
    {-1.3727214341e-02, 4.9766057992e-03, 1.2470365752e-05},
    {-9.0483741804e-02, -1.5465307026e-04, 4.9998037597e-03},
};
static const double ESO_L[3] = {60.0, 1200.0, 8000.0};
/* --------------------------------------------------------------------- */

typedef struct {
    double z[3];   /* estimates: z[0]=x1_hat, z[1]=x2_hat, z[2]=x3_hat (disturbance) */
} ESO3;

typedef struct {
    ESO3 eso;
    double kp, kd;          /* controller gains, from omega_c */
    double b0;               /* nominal control gain (can be updated for gain scheduling) */
    double u_min, u_max;     /* actuator saturation limits */
    double u_rate_max;       /* actuator slew rate limit */
    double u_prev;           /* last commanded output, for rate limiting */
} ADRCLoop;

/* ---- Initialization ---- */

static inline void eso3_init(ESO3 *eso) {
    eso->z[0] = 0.0;
    eso->z[1] = 0.0;
    eso->z[2] = 0.0;
}

static inline void adrc_loop_init(ADRCLoop *loop, double omega_c, double b0_nominal,
                                   double u_min, double u_max, double u_rate_max) {
    loop->kp = omega_c * omega_c;
    loop->kd = 2.0 * omega_c;
    loop->b0 = b0_nominal;
    loop->u_min = u_min;
    loop->u_max = u_max;
    loop->u_rate_max = u_rate_max;
    loop->u_prev = 0.0;
    eso3_init(&loop->eso);
}

/* ---- Helpers ---- */

static inline double clampd(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline double rate_limitd(double target, double previous, double max_rate, double dt) {
    double max_step = max_rate * dt;
    double delta = target - previous;
    if (delta > max_step)  delta = max_step;
    if (delta < -max_step) delta = -max_step;
    return previous + delta;
}

/* ---- ESO update: z[k+1] = Ad @ z[k] + M @ (B*u + L*y) ---- */

static inline void eso3_update(ESO3 *eso, double y, double u, double b0) {
    double Bu[3] = {0.0, b0 * u, 0.0};
    double forcing[3];
    double z_new[3];
    int i, j;

    for (i = 0; i < 3; i++) {
        forcing[i] = Bu[i] + ESO_L[i] * y;
    }

    for (i = 0; i < 3; i++) {
        z_new[i] = 0.0;
        for (j = 0; j < 3; j++) {
            z_new[i] += ESO_AD[i][j] * eso->z[j];
            z_new[i] += ESO_M[i][j]  * forcing[j];
        }
    }

    for (i = 0; i < 3; i++) {
        eso->z[i] = z_new[i];
    }
}

/* ---- Full ADRC step: controller + disturbance cancellation + limits ---- */

static inline double adrc_step(ADRCLoop *loop, double ref, double ref_dot,
                                double y_meas, double dt) {
    double z1 = loop->eso.z[0];
    double z2 = loop->eso.z[1];
    double z3 = loop->eso.z[2];

    double u0 = loop->kp * (ref - z1) + loop->kd * (ref_dot - z2);
    double u_raw = (u0 - z3) / loop->b0;

    double u_rl = rate_limitd(u_raw, loop->u_prev, loop->u_rate_max, dt);
    double u_final = clampd(u_rl, loop->u_min, loop->u_max);
    loop->u_prev = u_final;

    /* feed the ACTUAL (saturated, rate-limited) command back into the ESO,
       not the raw pre-limit value -- this is the anti-windup fix from
       earlier in the design discussion */
    eso3_update(&loop->eso, y_meas, u_final, loop->b0);

    return u_final;
}

#endif /* ADRC_H */
