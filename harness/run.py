"""Headless run + measure + verdict for both twin-fidelity cases.

Usage:
    python -m harness.run            # runs both cases, writes results/*.json
    python -m harness.run --case 1
    python -m harness.run --case 2
"""

from __future__ import annotations

import argparse
import json
import os

import mujoco
import numpy as np
from scipy.signal import find_peaks

from . import analytical, models

RESULTS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")

CASE1_TOLERANCE_REL_ERR = 0.01  # 1% relative error => reproduced
CASE2_TOLERANCE_REL_ERR = 0.05  # 5% on e_eff vs target
CASE2_TOLERANCE_R2 = 0.99


CASE1_SETTLE_TIME_S = 0.3  # skip the initial drop-onto-plane transient before fitting


def run_case1(theta_deg=models.INCLINE_THETA_DEG, mu=models.INCLINE_MU, duration=2.0):
    xml_path = os.path.join("models", "incline.xml")
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)

    n_steps = int(duration / m.opt.timestep)
    t = np.empty(n_steps)
    x = np.empty(n_steps)

    for i in range(n_steps):
        t[i] = d.time
        x[i] = d.qpos[0]  # the down-slope DOF; qpos[1] is the normal-settling DOF
        mujoco.mj_step(m, d)

    if not np.all(np.isfinite(x)):
        raise RuntimeError("case 1: non-finite position trace -- simulation diverged")

    # Drop the initial drop-onto-plane settling transient (the body starts
    # with a small gap above the surface, see models.py); fit only the
    # steady sliding phase. Fit, not eyeball: x(t) = x0 + v0*t + 0.5*a*t^2.
    settle_mask = t >= CASE1_SETTLE_TIME_S
    t_fit = t[settle_mask]
    x_fit = x[settle_mask]
    design = np.vstack([np.ones_like(t_fit), t_fit, 0.5 * t_fit ** 2]).T
    coeffs, *_ = np.linalg.lstsq(design, x_fit, rcond=None)
    x0_fit, v0_fit, a_measured = coeffs
    predicted = design @ coeffs
    ss_res = float(np.sum((x_fit - predicted) ** 2))
    ss_tot = float(np.sum((x_fit - np.mean(x_fit)) ** 2))
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0

    theta_rad = np.radians(theta_deg)
    a_analytical = analytical.incline_acceleration(theta_rad, mu)
    rel_err = abs(a_measured - a_analytical) / abs(a_analytical)
    verdict = "reproduced" if rel_err < CASE1_TOLERANCE_REL_ERR else "deviation-flagged"

    result = {
        "case": 1,
        "name": "friction_incline_sliding",
        "configured_inputs": {
            "theta_deg": theta_deg,
            "mu": mu,
            "g": 9.81,
            "duration_s": duration,
            "timestep_s": float(m.opt.timestep),
            "n_samples": n_steps,
            "settle_time_excluded_from_fit_s": CASE1_SETTLE_TIME_S,
        },
        "analytical_value_m_s2": float(a_analytical),
        "measured_value_m_s2": float(a_measured),
        "fit_quality_r_squared": float(r_squared),
        "fit_initial_position_m": float(x0_fit),
        "fit_initial_velocity_m_s": float(v0_fit),
        "relative_error": float(rel_err),
        "tolerance_relative_error": CASE1_TOLERANCE_REL_ERR,
        "verdict": verdict,
    }
    return result


def run_case2(
    e_target=models.BALL_E_TARGET,
    h0=models.BALL_DROP_HEIGHT,
    radius=models.BALL_RADIUS,
    duration=6.0,
    min_bounces=5,
):
    xml_path = os.path.join("models", "bounce.xml")
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    # mj_forward populates d.xpos (Cartesian body positions) from qpos0;
    # without it the first recorded sample is a stale zero-initialized
    # array, not the true initial condition, which corrupts peak-finding.
    mujoco.mj_forward(m, d)

    n_steps = int(duration / m.opt.timestep)
    t = np.empty(n_steps)
    z = np.empty(n_steps)  # ball center world z

    ball_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ball")

    for i in range(n_steps):
        t[i] = d.time
        z[i] = d.xpos[ball_body_id, 2]
        mujoco.mj_step(m, d)

    if not np.all(np.isfinite(z)):
        raise RuntimeError("case 2: non-finite position trace -- simulation diverged")

    height_above_contact = z - radius  # h(t): 0 == touching the plane

    # Post-bounce apexes: local maxima in height, well above ground noise.
    peak_idx, _ = find_peaks(height_above_contact, height=radius * 0.02, distance=int(0.02 / m.opt.timestep))
    measured_bounce_heights = height_above_contact[peak_idx]
    measured_bounce_times = t[peak_idx]

    n_bounces_found = len(peak_idx)
    if n_bounces_found < min_bounces:
        raise RuntimeError(
            f"case 2: only found {n_bounces_found} post-bounce apexes in {duration}s "
            f"(need >= {min_bounces}) -- increase `duration` or check the contact model"
        )

    # n=0 is the exact initial drop height (an input, not a measurement);
    # n=1..N are the measured post-bounce apex heights.
    bounce_indices = [0] + list(range(1, n_bounces_found + 1))
    bounce_heights = [h0] + list(measured_bounce_heights)

    e_eff, h0_fit, r_squared = analytical.fit_effective_restitution(bounce_indices, bounce_heights)

    rel_err = abs(e_eff - e_target) / e_target
    reproduced = rel_err < CASE2_TOLERANCE_REL_ERR and r_squared > CASE2_TOLERANCE_R2
    verdict = "reproduced" if reproduced else "deviation-flagged"

    result = {
        "case": 2,
        "name": "bounce_restitution",
        "e_convention": (
            "velocity coefficient of restitution (e = |v_after|/|v_before|); "
            "height decays as h_n = e^(2n) * h_0 because h ~ v^2"
        ),
        "configured_inputs": {
            "e_target": e_target,
            "h0_m": h0,
            "ball_radius_m": radius,
            "ball_mass_kg": models.BALL_MASS,
            "contact_freq_hz": models.CONTACT_FREQ_HZ,
            "duration_s": duration,
            "timestep_s": float(m.opt.timestep),
        },
        "n_bounces_measured": n_bounces_found,
        "bounce_indices": bounce_indices,
        "bounce_heights_m": [float(h) for h in bounce_heights],
        "bounce_times_s": [float(x) for x in measured_bounce_times],
        "analytical_value_e": e_target,
        "measured_value_e_eff": e_eff,
        "h0_fit_m": h0_fit,
        "decay_law_fit_r_squared": r_squared,
        "relative_error_e": float(rel_err),
        "tolerance_relative_error_e": CASE2_TOLERANCE_REL_ERR,
        "tolerance_r_squared": CASE2_TOLERANCE_R2,
        "verdict": verdict,
    }
    return result


def _print_summary(result):
    print(f"\n=== Case {result['case']}: {result['name']} ===")
    if result["case"] == 1:
        print(f"  theta={result['configured_inputs']['theta_deg']} deg, mu={result['configured_inputs']['mu']}")
        print(f"  analytical a = {result['analytical_value_m_s2']:.6f} m/s^2")
        print(f"  measured   a = {result['measured_value_m_s2']:.6f} m/s^2  (fit R^2={result['fit_quality_r_squared']:.6f})")
        print(f"  relative error = {result['relative_error']*100:.4f}%  (tolerance {result['tolerance_relative_error']*100:.1f}%)")
    else:
        print(f"  e_target={result['analytical_value_e']}, h0={result['configured_inputs']['h0_m']} m")
        print(f"  bounce heights (m): {['%.5f' % h for h in result['bounce_heights_m']]}")
        print(f"  measured e_eff = {result['measured_value_e_eff']:.6f}  (decay-fit R^2={result['decay_law_fit_r_squared']:.6f})")
        print(f"  relative error on e = {result['relative_error_e']*100:.4f}%  (tolerance {result['tolerance_relative_error_e']*100:.1f}%, R^2 > {result['tolerance_r_squared']})")
    print(f"  VERDICT: {result['verdict'].upper()}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", type=int, choices=[1, 2], default=None)
    args = parser.parse_args()

    os.makedirs(RESULTS_DIR, exist_ok=True)
    results = {}

    if args.case in (None, 1):
        results["case1"] = run_case1()
        _print_summary(results["case1"])
        with open(os.path.join(RESULTS_DIR, "case1_incline.json"), "w") as f:
            json.dump(results["case1"], f, indent=2)

    if args.case in (None, 2):
        results["case2"] = run_case2()
        _print_summary(results["case2"])
        with open(os.path.join(RESULTS_DIR, "case2_bounce.json"), "w") as f:
            json.dump(results["case2"], f, indent=2)

    return results


if __name__ == "__main__":
    main()
