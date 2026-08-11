"""
Lightweight ADRC autopilot simulator.

Simulates seven ADRC loops (inner: roll/pitch/yaw/airspeed, outer:
altitude/lateral-path, airspeed shared) driving a simplified nonlinear-ish
rigid body model, with:
  - b0 gain scheduling (aileron/elevator/rudder/altitude effectiveness
    scale with dynamic pressure ~ V^2)
  - actuator saturation and rate limiting, correctly fed back into each ESO
  - reference saturation and rate limiting on the outer-loop outputs
  - simple wind gust disturbances and sensor noise (no full EKF here --
    measurements are the true state plus noise, which is enough for
    tuning/validating the ADRC loops themselves)

This is meant as the "first step" simulator: fast to iterate on, full
visibility into every internal ESO state. Swap the Plant class for
JSBSim or a higher-fidelity model later without touching the ADRC code.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm, inv

# ----------------------------------------------------------------------
# Utilities
# ----------------------------------------------------------------------

def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def rate_limit(target, previous, max_rate, dt):
    """Move `previous` toward `target` by at most max_rate*dt this step."""
    max_step = max_rate * dt
    delta = clamp(target - previous, -max_step, max_step)
    return previous + delta


def b0_schedule(b0_nominal, V_hat, V_nominal, V_floor=3.0):
    """Aerodynamic surface effectiveness scales with dynamic pressure,
    i.e. roughly V^2. V_floor guards against division blow-up / near-zero
    airspeed during startup."""
    V_eff = max(V_hat, V_floor)
    return b0_nominal * (V_eff / V_nominal) ** 2


# ----------------------------------------------------------------------
# Extended State Observers
# ----------------------------------------------------------------------

class ESO3:
    """3-state ESO for a 2nd-order channel: x1 (output), x2 (rate), x3 (disturbance).

    Uses EXACT zero-order-hold discretization of z_dot = (A-LC)z + B*b0*u + L*y,
    rather than naive forward-Euler. This matters a lot in practice: with
    omega_o in the tens-to-hundreds of rad/s (typical for a fast inner
    attitude loop), beta3 = omega_o**3 makes forward-Euler blow up unless
    dt is absurdly small. Exact ZOH discretization is unconditionally
    stable regardless of dt, matching how a careful embedded observer
    implementation should be built.
    """

    def __init__(self, omega_o, b0, dt):
        beta1, beta2, beta3 = 3 * omega_o, 3 * omega_o ** 2, omega_o ** 3
        L = np.array([[beta1], [beta2], [beta3]])
        A = np.array([[0.0, 1.0, 0.0],
                       [0.0, 0.0, 1.0],
                       [0.0, 0.0, 0.0]])
        C = np.array([[1.0, 0.0, 0.0]])
        Abar = A - L @ C

        self.Ad = expm(Abar * dt)
        self.M = inv(Abar) @ (self.Ad - np.eye(3))  # precomputed ZOH input map
        self.L = L
        self.b0 = b0
        self.z = np.zeros((3, 1))

    def update(self, y, u, dt):
        Bu = np.array([[0.0], [self.b0], [0.0]]) * u
        forcing = Bu + self.L * y
        self.z = self.Ad @ self.z + self.M @ forcing
        return self.z[0, 0], self.z[1, 0], self.z[2, 0]

    @property
    def z1(self):
        return self.z[0, 0]

    @property
    def z2(self):
        return self.z[1, 0]

    @property
    def z3(self):
        return self.z[2, 0]


class ESO2:
    """2-state ESO for a 1st-order channel: x1 (output), x2 (disturbance).
    Same exact ZOH discretization approach as ESO3."""

    def __init__(self, omega_o, b0, dt):
        beta1, beta2 = 2 * omega_o, omega_o ** 2
        L = np.array([[beta1], [beta2]])
        A = np.array([[0.0, 1.0], [0.0, 0.0]])
        C = np.array([[1.0, 0.0]])
        Abar = A - L @ C

        self.Ad = expm(Abar * dt)
        self.M = inv(Abar) @ (self.Ad - np.eye(2))
        self.L = L
        self.b0 = b0
        self.z = np.zeros((2, 1))

    def update(self, y, u, dt):
        Bu = np.array([[self.b0], [0.0]]) * u
        forcing = Bu + self.L * y
        self.z = self.Ad @ self.z + self.M @ forcing
        return self.z[0, 0], self.z[1, 0]

    @property
    def z1(self):
        return self.z[0, 0]

    @property
    def z2(self):
        return self.z[1, 0]


# ----------------------------------------------------------------------
# ADRC loop wrapper (handles controller + ESO + saturation/rate limiting
# + b0 scheduling in one place, for a 2nd-order channel)
# ----------------------------------------------------------------------

class ADRCLoop3:
    def __init__(self, name, omega_c, omega_o, b0_nominal,
                 u_min, u_max, u_rate_max, dt,
                 schedule_b0=False, V_nominal=None):
        self.name = name
        self.kp = omega_c ** 2
        self.kd = 2 * omega_c
        self.b0_nominal = b0_nominal
        self.schedule_b0 = schedule_b0
        self.V_nominal = V_nominal
        self.eso = ESO3(omega_o, b0_nominal, dt)
        self.u_min, self.u_max, self.u_rate_max = u_min, u_max, u_rate_max
        self.u_prev = 0.0

    def step(self, ref, y_meas, dt, V_hat=None):
        if self.schedule_b0:
            b0 = b0_schedule(self.b0_nominal, V_hat, self.V_nominal)
        else:
            b0 = self.b0_nominal
        self.eso.b0 = b0

        z1, z2, z3 = self.eso.z1, self.eso.z2, self.eso.z3
        u0 = self.kp * (ref - z1) - self.kd * z2
        u_raw = (u0 - z3) / b0

        u_rl = rate_limit(u_raw, self.u_prev, self.u_rate_max, dt)
        u_final = clamp(u_rl, self.u_min, self.u_max)
        self.u_prev = u_final

        # feed the ACTUAL (saturated, rate-limited) command back into the ESO
        self.eso.update(y_meas, u_final, dt)
        return u_final


class ADRCLoop2:
    """Same idea, for a 1st-order channel (yaw rate, airspeed)."""

    def __init__(self, name, omega_c, omega_o, b0_nominal,
                 u_min, u_max, u_rate_max, dt,
                 schedule_b0=False, V_nominal=None):
        self.name = name
        self.kp = omega_c
        self.b0_nominal = b0_nominal
        self.schedule_b0 = schedule_b0
        self.V_nominal = V_nominal
        self.eso = ESO2(omega_o, b0_nominal, dt)
        self.u_min, self.u_max, self.u_rate_max = u_min, u_max, u_rate_max
        self.u_prev = 0.0

    def step(self, ref, y_meas, dt, V_hat=None):
        if self.schedule_b0:
            b0 = b0_schedule(self.b0_nominal, V_hat, self.V_nominal)
        else:
            b0 = self.b0_nominal
        self.eso.b0 = b0

        z1, z2 = self.eso.z1, self.eso.z2
        u0 = self.kp * (ref - z1)
        u_raw = (u0 - z2) / b0

        u_rl = rate_limit(u_raw, self.u_prev, self.u_rate_max, dt)
        u_final = clamp(u_rl, self.u_min, self.u_max)
        self.u_prev = u_final

        self.eso.update(y_meas, u_final, dt)
        return u_final


# ----------------------------------------------------------------------
# Simplified "true" plant (unknown to the controller -- this is reality)
# ----------------------------------------------------------------------

class Plant:
    def __init__(self):
        self.phi, self.phidot = 0.0, 0.0
        self.theta, self.thetadot = 0.0, 0.0
        self.psi, self.r = 0.0, 0.0
        self.z, self.zdot = 100.0, 0.0
        self.ylat, self.ylatdot = 50.0, 0.0
        self.V = 14.0

        # true (unknown to controller) effectiveness at V_nominal
        self.b0_phi_true = 8.0
        self.b0_theta_true = 6.0
        self.b0_psi_true = 5.0
        self.b0z_true = 3.0
        self.b0V_true = 2.0
        self.g = 9.81
        self.V_nominal = 20.0

        # light aerodynamic damping
        self.c_phi, self.c_theta, self.c_psi = 0.5, 0.6, 0.8
        self.c_z, self.c_lat = 0.3, 0.2
        self.adverse_yaw_gain = -0.6  # aileron -> opposing yaw moment

    def step(self, delta_a, delta_e, delta_r, delta_t, t, dt, rng):
        q_ratio = (max(self.V, 3.0) / self.V_nominal) ** 2

        gust_phi = 0.3 * np.sin(0.4 * t) + rng.normal(0, 0.05)
        gust_theta = 0.2 * np.sin(0.3 * t + 1.0) + rng.normal(0, 0.05)
        gust_psi = rng.normal(0, 0.03)
        gust_z = 0.4 * np.sin(0.1 * t) + rng.normal(0, 0.05)
        gust_V = rng.normal(0, 0.05)

        phi_ddot = (-self.c_phi * self.phidot
                    + self.b0_phi_true * q_ratio * delta_a + gust_phi)
        theta_ddot = (-self.c_theta * self.thetadot
                      + self.b0_theta_true * q_ratio * delta_e + gust_theta)
        r_dot = (-self.c_psi * self.r
                 + self.b0_psi_true * q_ratio * delta_r
                 + self.adverse_yaw_gain * delta_a  # adverse yaw coupling
                 + gust_psi)
        z_ddot = (-self.c_z * self.zdot
                  + self.b0z_true * q_ratio * self.theta + gust_z)
        ylat_ddot = -self.c_lat * self.ylatdot + self.g * self.phi
        V_dot = self.b0V_true * delta_t - 0.05 * (self.V - 14.0) + gust_V

        self.phidot += phi_ddot * dt
        self.phi += self.phidot * dt
        self.thetadot += theta_ddot * dt
        self.theta += self.thetadot * dt
        self.r += r_dot * dt
        self.psi += self.r * dt
        self.zdot += z_ddot * dt
        self.z += self.zdot * dt
        self.ylatdot += ylat_ddot * dt
        self.ylat += self.ylatdot * dt
        self.V += V_dot * dt


# ----------------------------------------------------------------------
# Simulation
# ----------------------------------------------------------------------

def run_sim(t_end=60.0, dt_inner=0.005, outer_decimation=4, seed=0):
    """Multi-rate simulation, matching the earlier rate-mismatch discussion:
    inner loops + plant integrate at dt_inner (200 Hz here), outer loops
    only update every `outer_decimation` steps (50 Hz here)."""
    rng = np.random.default_rng(seed)
    plant = Plant()

    V_nom = 20.0
    dt_outer = dt_inner * outer_decimation

    # NOTE on gains: the flight-controller document's example numbers
    # (inner omega_c=20, outer omega_c=3) assume an airframe with enough
    # control authority per degree of surface deflection that those
    # bandwidths don't demand more than the actuator can deliver. For
    # THIS toy plant's b0 values, omega_c=20 demands >1000 deg of
    # equivalent elevator for a routine 20 deg pitch error -- it saturates
    # immediately and, combined with the actuator rate limit, rings in a
    # sustained limit cycle instead of converging. This is a real, common
    # failure mode (rate-limit-induced oscillation), not a simulation bug.
    # Gains below are chosen to match this toy plant's actual authority;
    # for a real airframe, size omega_c against your real b0 and surface
    # limits the same way, rather than reusing numbers from a different
    # aircraft.

    # --- outer loops (their own ESO discretized at the OUTER rate) ---
    alt_loop = ADRCLoop3("altitude", omega_c=1.5, omega_o=6.0, b0_nominal=3.0,
                          u_min=-0.4, u_max=0.4, u_rate_max=1.0, dt=dt_outer,
                          schedule_b0=True, V_nominal=V_nom)
    lat_loop = ADRCLoop3("lateral", omega_c=0.3, omega_o=1.2, b0_nominal=9.81,
                          u_min=-0.785, u_max=0.785, u_rate_max=1.047, dt=dt_outer)

    # --- inner loops (their ESO discretized at the fast INNER rate) ---
    roll_loop = ADRCLoop3("roll", omega_c=6.0, omega_o=24.0, b0_nominal=8.0,
                           u_min=-0.436, u_max=0.436, u_rate_max=3.0, dt=dt_inner,
                           schedule_b0=True, V_nominal=V_nom)
    pitch_loop = ADRCLoop3("pitch", omega_c=8.0, omega_o=32.0, b0_nominal=6.0,
                            u_min=-0.436, u_max=0.436, u_rate_max=3.0, dt=dt_inner,
                            schedule_b0=True, V_nominal=V_nom)
    yaw_loop = ADRCLoop2("yaw", omega_c=6.0, omega_o=24.0, b0_nominal=5.0,
                          u_min=-0.436, u_max=0.436, u_rate_max=3.0, dt=dt_inner,
                          schedule_b0=True, V_nominal=V_nom)
    speed_loop = ADRCLoop2("airspeed", omega_c=1.0, omega_o=4.0, b0_nominal=2.0,
                            u_min=0.0, u_max=1.0, u_rate_max=0.5, dt=dt_inner)

    n = int(t_end / dt_inner)
    log = {k: np.zeros(n) for k in
           ["t", "z", "z_ref", "ylat", "phi", "phi_d", "theta", "theta_d",
            "V", "delta_a", "delta_e", "delta_r", "delta_t", "r"]}

    z_ref = 120.0        # climb command
    theta_d_limited = 0.0
    phi_d_limited = 0.0

    for k in range(n):
        t = k * dt_inner

        # --- noisy "measurements" (stand-in for a full EKF output) ---
        y_z = plant.z + rng.normal(0, 0.3)
        y_ylat = plant.ylat + rng.normal(0, 1.0)
        y_phi = plant.phi + rng.normal(0, 0.01)
        y_theta = plant.theta + rng.normal(0, 0.01)
        y_r = plant.r + rng.normal(0, 0.01)
        y_V = plant.V + rng.normal(0, 0.2)

        # --- outer loops: only update every outer_decimation steps ---
        if k % outer_decimation == 0:
            theta_d_raw = alt_loop.step(z_ref, y_z, dt_outer, V_hat=y_V)
            theta_d_limited = rate_limit(theta_d_raw, theta_d_limited, 0.5, dt_outer)
            theta_d_limited = clamp(theta_d_limited, -0.35, 0.35)

            phi_d_raw = lat_loop.step(0.0, y_ylat, dt_outer)
            phi_d_limited = rate_limit(phi_d_raw, phi_d_limited, 1.047, dt_outer)
            phi_d_limited = clamp(phi_d_limited, -0.785, 0.785)

        # --- inner loops: every step ---
        delta_e = pitch_loop.step(theta_d_limited, y_theta, dt_inner, V_hat=y_V)
        delta_a = roll_loop.step(phi_d_limited, y_phi, dt_inner, V_hat=y_V)
        delta_r = yaw_loop.step(0.0, y_r, dt_inner, V_hat=y_V)
        delta_t = speed_loop.step(16.0, y_V, dt_inner)

        plant.step(delta_a, delta_e, delta_r, delta_t, t, dt_inner, rng)

        for key, val in zip(log.keys(),
                             [t, plant.z, z_ref, plant.ylat, plant.phi,
                              phi_d_limited, plant.theta, theta_d_limited,
                              plant.V, delta_a, delta_e, delta_r, delta_t,
                              plant.r]):
            log[key][k] = val

    return log


def plot_results(log):
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))

    axes[0, 0].plot(log["t"], log["z"], label="z (altitude)")
    axes[0, 0].plot(log["t"], log["z_ref"], "--", label="z_ref")
    axes[0, 0].set_title("Altitude")
    axes[0, 0].legend()

    axes[0, 1].plot(log["t"], log["ylat"])
    axes[0, 1].axhline(0, color="gray", linestyle="--")
    axes[0, 1].set_title("Lateral cross-track offset")

    axes[1, 0].plot(log["t"], np.degrees(log["phi"]), label="phi")
    axes[1, 0].plot(log["t"], np.degrees(log["phi_d"]), "--", label="phi_d")
    axes[1, 0].set_title("Roll angle (deg)")
    axes[1, 0].legend()

    axes[1, 1].plot(log["t"], np.degrees(log["theta"]), label="theta")
    axes[1, 1].plot(log["t"], np.degrees(log["theta_d"]), "--", label="theta_d")
    axes[1, 1].set_title("Pitch angle (deg)")
    axes[1, 1].legend()

    axes[2, 0].plot(log["t"], log["V"])
    axes[2, 0].set_title("Airspeed (m/s)")

    axes[2, 1].plot(log["t"], np.degrees(log["delta_a"]), label="aileron")
    axes[2, 1].plot(log["t"], np.degrees(log["delta_e"]), label="elevator")
    axes[2, 1].plot(log["t"], np.degrees(log["delta_r"]), label="rudder")
    axes[2, 1].set_title("Actuator commands (deg)")
    axes[2, 1].legend()

    for ax in axes.flat:
        ax.set_xlabel("time (s)")
        ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig("adrc_uav_sim_results.png", dpi=130)
    print("Saved plot to adrc_uav_sim_results.png")


if __name__ == "__main__":
    log = run_sim()
    plot_results(log)
