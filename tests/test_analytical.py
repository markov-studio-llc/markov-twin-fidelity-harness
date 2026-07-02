"""Unit tests for the closed-form physics grader (harness/analytical.py).

These are the most important tests in the repo: analytical.py is the
ground truth every MuJoCo verdict is graded against. If a formula here is
wrong, a "reproduced" verdict means nothing. So each function is checked
against a value computed independently by hand (in the test, from first
principles) and against the physics invariants it must satisfy -- never
against the simulator.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from harness import analytical


# --------------------------------------------------------------------------
# Case 1: incline_acceleration -- a = g*(sin(theta) - mu*cos(theta))
# --------------------------------------------------------------------------

def test_incline_acceleration_matches_hand_computed_value():
    # theta=30deg, mu=0.15, g=9.81 -> the exact published Case-1 analytical value.
    theta = math.radians(30.0)
    expected = 9.81 * (math.sin(theta) - 0.15 * math.cos(theta))
    assert analytical.incline_acceleration(theta, 0.15, g=9.81) == pytest.approx(expected)
    # sanity: this is the 3.630644 m/s^2 quoted in the README / finding.
    assert analytical.incline_acceleration(theta, 0.15) == pytest.approx(3.630643618, rel=1e-6)


def test_incline_acceleration_frictionless_is_g_sin_theta():
    theta = math.radians(30.0)
    assert analytical.incline_acceleration(theta, 0.0) == pytest.approx(9.81 * math.sin(theta))
    # frictionless 30deg incline: a = g/2.
    assert analytical.incline_acceleration(theta, 0.0) == pytest.approx(9.81 / 2)


def test_incline_acceleration_decreases_with_friction():
    theta = math.radians(30.0)
    a0 = analytical.incline_acceleration(theta, 0.0)
    a1 = analytical.incline_acceleration(theta, 0.15)
    a2 = analytical.incline_acceleration(theta, 0.3)
    assert a0 > a1 > a2 > 0


def test_incline_acceleration_rejects_static_case():
    # mu >= tan(theta) means the block would not slide; the closed form is
    # invalid there and must fail loudly rather than return a bogus number.
    theta = math.radians(30.0)  # tan(30) ~= 0.577
    with pytest.raises(ValueError):
        analytical.incline_acceleration(theta, 0.6)
    # exactly at the threshold is also static.
    with pytest.raises(ValueError):
        analytical.incline_acceleration(theta, math.tan(theta))


# --------------------------------------------------------------------------
# Case 2: restitution_height_ratio -- h_n / h_0 = e^(2n)
# --------------------------------------------------------------------------

def test_restitution_height_ratio_basic_values():
    assert analytical.restitution_height_ratio(0.75, 0) == pytest.approx(1.0)
    assert analytical.restitution_height_ratio(0.75, 1) == pytest.approx(0.75 ** 2)
    assert analytical.restitution_height_ratio(0.9, 2) == pytest.approx(0.9 ** 4)


def test_restitution_height_ratio_elastic_conserves_height():
    # e=1 (perfectly elastic) -> every bounce returns to the drop height.
    for n in range(5):
        assert analytical.restitution_height_ratio(1.0, n) == pytest.approx(1.0)


# --------------------------------------------------------------------------
# fit_effective_restitution -- recover e from apex heights by log-linear fit
# --------------------------------------------------------------------------

def test_fit_recovers_exact_synthetic_restitution():
    # Build a perfect e^(2n) decay and confirm the fit inverts it exactly.
    e_true, h0 = 0.8, 1.3
    n = list(range(7))
    heights = [h0 * e_true ** (2 * k) for k in n]
    e_eff, h0_fit, r2 = analytical.fit_effective_restitution(n, heights)
    assert e_eff == pytest.approx(e_true, rel=1e-9)
    assert h0_fit == pytest.approx(h0, rel=1e-9)
    assert r2 == pytest.approx(1.0, abs=1e-12)


def test_fit_r_squared_drops_on_noisy_data():
    e_true, h0 = 0.8, 1.0
    n = list(range(8))
    rng = np.random.default_rng(0)
    heights = [h0 * e_true ** (2 * k) * (1 + 0.1 * rng.standard_normal()) for k in n]
    _e_eff, _h0, r2 = analytical.fit_effective_restitution(n, heights)
    assert r2 < 1.0  # imperfect data => imperfect fit


def test_fit_rejects_nonpositive_heights():
    with pytest.raises(ValueError):
        analytical.fit_effective_restitution([0, 1, 2], [1.0, 0.0, 0.5])
    with pytest.raises(ValueError):
        analytical.fit_effective_restitution([0, 1], [1.0, -0.2])


# --------------------------------------------------------------------------
# damping_ratio_for_restitution -- invert e = exp(-zeta*pi/sqrt(1-zeta^2))
# --------------------------------------------------------------------------

@pytest.mark.parametrize("e_target", [0.1, 0.5, 0.75, 0.95])
def test_damping_ratio_round_trips_through_restitution_formula(e_target):
    zeta = analytical.damping_ratio_for_restitution(e_target)
    assert 0.0 < zeta < 1.0  # must be underdamped
    e_recovered = math.exp(-zeta * math.pi / math.sqrt(1 - zeta ** 2))
    assert e_recovered == pytest.approx(e_target, rel=1e-9)


def test_damping_ratio_monotonic_more_bouncy_less_damped():
    # Higher restitution => less damping.
    z_low = analytical.damping_ratio_for_restitution(0.9)
    z_high = analytical.damping_ratio_for_restitution(0.3)
    assert z_high > z_low


def test_damping_ratio_rejects_out_of_range():
    for bad in (0.0, 1.0, -0.1, 1.5):
        with pytest.raises(ValueError):
            analytical.damping_ratio_for_restitution(bad)


# --------------------------------------------------------------------------
# solref_direct_for_target -- (-stiffness, -damping) for MuJoCo direct format
# --------------------------------------------------------------------------

def test_solref_signs_and_magnitudes():
    stiffness, damping = analytical.solref_direct_for_target(0.75, mass=0.5, contact_freq_hz=120.0)
    # MuJoCo direct format encodes both as negatives.
    assert stiffness < 0
    assert damping < 0


def test_solref_encodes_target_damping_ratio_exactly():
    # The whole point: |damping| / (2*sqrt(|stiffness|*mass)) must equal the
    # damping ratio derived from e_target -- otherwise the solref does not
    # actually target the requested restitution.
    e_target, mass = 0.75, 0.5
    stiffness, damping = analytical.solref_direct_for_target(e_target, mass=mass, contact_freq_hz=120.0)
    zeta_encoded = abs(damping) / (2 * math.sqrt(abs(stiffness) * mass))
    assert zeta_encoded == pytest.approx(analytical.damping_ratio_for_restitution(e_target), rel=1e-9)


def test_solref_stiffness_tracks_natural_frequency():
    # stiffness = mass * omega0^2, omega0 = 2*pi*f: doubling f quadruples stiffness.
    k1, _ = analytical.solref_direct_for_target(0.75, mass=1.0, contact_freq_hz=100.0)
    k2, _ = analytical.solref_direct_for_target(0.75, mass=1.0, contact_freq_hz=200.0)
    assert abs(k2) == pytest.approx(4 * abs(k1), rel=1e-9)
