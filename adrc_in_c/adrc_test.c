#include <stdio.h>
#include "adrc3.h"

int main(void) {
    ADRCLoop loop;
    double omega_c = 8.0;
    double b0_nominal = 2.0;
    double b0_true = 2.4;

    adrc_loop_init(&loop, omega_c, b0_nominal, -5.0, 5.0, 50.0);

    double x = 0.0, x_dot = 0.0;
    double dt = 0.005;
    double ref = 10.0;
    int steps = 2000;
  
    FILE *fp = fopen("log.csv", "w");
    if (fp == NULL) {
        printf("Error: could not open log.csv for writing\n");
        return 1;
    }

    fprintf(fp, "t,x,u,z1_hat,z2_hat,z3_hat\n");

    for (int k = 0; k < steps; k++) {
        double t = k * dt;
        double disturbance = (t > 5.0) ? 3.0 : 0.0;

        double u = adrc_step(&loop, ref, 0.0, x, dt);

        double x_ddot = disturbance + b0_true * u;
        x_dot += x_ddot * dt;
        x += x_dot * dt;

        fprintf(fp, "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                t, x, u, loop.eso.z[0], loop.eso.z[1], loop.eso.z[2]);
    }

    fclose(fp);

    printf("Wrote %d rows to log.csv\n", steps);
    printf("Final x = %.4f (target = %.4f)\n", x, ref);
    return 0;
}
