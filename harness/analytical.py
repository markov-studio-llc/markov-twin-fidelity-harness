"""Closed-form physics for the twin-fidelity harness.

Every number the harness grades against is derived here, from Newton's
second law and the standard definition of the coefficient of restitution.
Nothing is hardcoded from memory of "what MuJoCo usually gives you."

Case 1 -- incline (friction, sliding)
--------------------------------------
A rigid block on a frictional incline of angle ``theta``. Resolve forces
perpendicular and parallel to the slope. Perpendicular: no acceleration,
so the normal force balances the perpendicular weight component,
``N = m g cos(theta)``. Kinetic friction opposes sliding with magnitude
``f = mu * N``. Parallel to the slope (down-slope positive):

    m*a = m*g*sin(theta) - mu*m*g*cos(theta)
    a   = g*(sin(theta) - mu*cos(theta))

This holds only while the block actually slides, i.e. while
``mu < tan(theta)`` -- otherwise static friction can balance the entire
down-slope weight component and the block never moves.

Case 2 -- bounce (restitution)
--------------------------------------
The coefficient of restitution ``e`` is defined here as the standard
VELOCITY ratio at a collision: ``e = |v_after| / |v_before|``. This is the
textbook definition (Newton's experimental law of impact) and is also the
definition SimBenchmark's (leggedrobotics) bouncing-benchmark option struct
uses -- its ``Option::e`` is documented as "restitution coefficient (0-1)"
and the codebase only trusts the analytic solution at ``e=1`` ("only elastic
collision analytic solution is supported"), i.e. full energy conservation
at the standard velocity-based definition. See DECISION.md for the
verification trail (SimBenchmark source read directly, not from memory).

For a ball dropped from rest at height ``h_0`` above the contact point and
bouncing on a fixed (infinitely massive) surface, height and speed are
related by free-fall energy conservation, ``h = v^2 / (2g)``. Each bounce
scales speed by ``e``, so it scales height by ``e^2``. After ``n`` bounces:

    h_n = e^(2n) * h_0

This is convention (i) from the build brief. The rejected alternative,
convention (ii) (``e`` as a raw per-bounce HEIGHT ratio, ``h_n = e^n * h_0``),
is not used -- it is not how "coefficient of restitution" is defined in the
physics or robotics-sim literature, it was only ever the parent document's
loose shorthand.
"""

from __future__ import annotations

import numpy as np


def incline_acceleration(theta_rad: float, mu: float, g: float = 9.81) -> float:
    """Closed-form sliding acceleration down a frictional incline."""
    tan_theta = np.tan(theta_rad)
    if mu >= tan_theta:
        raise ValueError(
            f"mu={mu} >= tan(theta)={tan_theta:.4f}: this is a static case "
            "(the block would not slide) -- the closed form assumes kinetic "
            "sliding throughout."
        )
    return g * (np.sin(theta_rad) - mu * np.cos(theta_rad))


def restitution_height_ratio(e: float, n: int) -> float:
    """h_n / h_0 under the velocity-coefficient-of-restitution convention."""
    return e ** (2 * n)


def fit_effective_restitution(bounce_indices, heights):
    """Recover e_eff from measured apex heights via log-linear regression.

    Fits ``ln(h_n) = ln(h_0) + 2*n*ln(e)`` by ordinary least squares over
    the supplied ``(n, h_n)`` pairs (n=0 is the initial drop height, an
    exact input rather than a simulated measurement, and is included as an
    anchor point). Returns ``(e_eff, h0_fit, r_squared)``.
    """
    n = np.asarray(bounce_indices, dtype=float)
    h = np.asarray(heights, dtype=float)
    if np.any(h <= 0):
        raise ValueError("all heights must be positive to take a log")
    log_h = np.log(h)
    design = np.vstack([np.ones_like(n), n]).T
    (intercept, slope), *_ = np.linalg.lstsq(design, log_h, rcond=None)
    predicted = intercept + slope * n
    ss_res = float(np.sum((log_h - predicted) ** 2))
    ss_tot = float(np.sum((log_h - np.mean(log_h)) ** 2))
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
    e_eff = float(np.exp(slope / 2))
    h0_fit = float(np.exp(intercept))
    return e_eff, h0_fit, r_squared


def damping_ratio_for_restitution(e_target: float) -> float:
    """Invert e = exp(-zeta*pi / sqrt(1-zeta^2)) for the damping ratio zeta.

    This is the restitution coefficient of a single-DOF linear
    spring-dashpot (Kelvin-Voigt) contact undergoing one half-period of
    oscillation against a fixed wall -- a standard result in contact
    mechanics / vibrations (e.g. Goldsmith, "Impact", ch. 3). It is the
    natural heuristic to reach for here because MuJoCo's OWN documentation
    (doc/modeling.rst, "Solver parameters") identifies its ``dampratio``
    with exactly this same damping ratio: "A dampratio of 1 (positive
    format) is equivalent to damping = 2*sqrt(stiffness) (direct format)",
    i.e. zeta = damping / (2*sqrt(stiffness*mass)) in MuJoCo's own direct
    solref parameterization.

    This is a HEURISTIC STARTING POINT, not a guarantee of the simulated
    outcome: MuJoCo's actual contact impedance d(r) (set by solimp) is not
    perfectly constant across the contact's penetration depth the way the
    idealized derivation above assumes, and MuJoCo's constraint model is a
    continuous soft-penalty method, not an instantaneous-impulse restitution
    law. The gap between this formula's target and the measured e_eff (see
    run.py) is exactly what the harness is built to surface, honestly.
    """
    if not (0.0 < e_target < 1.0):
        raise ValueError("e_target must be in (0, 1) for the underdamped formula")
    log_e = -np.log(e_target)
    zeta = log_e / np.sqrt(np.pi ** 2 + log_e ** 2)
    return float(zeta)


def solref_direct_for_target(e_target: float, mass: float, contact_freq_hz: float):
    """MuJoCo direct-format solref = (-stiffness, -damping) targeting e_target.

    ``contact_freq_hz`` sets the natural frequency of the virtual
    spring-dashpot (how short/hard the collision feels); it must be low
    enough relative to 1/dt for the integrator to remain stable (see
    CONTACT_FREQ_HZ / DT in run.py).
    """
    zeta = damping_ratio_for_restitution(e_target)
    omega0 = 2.0 * np.pi * contact_freq_hz
    stiffness = mass * omega0 ** 2
    damping = 2.0 * zeta * mass * omega0
    return -stiffness, -damping
