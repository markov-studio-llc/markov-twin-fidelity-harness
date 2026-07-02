"""End-to-end regression tests: run both cases through MuJoCo and assert the
published verdicts.

MuJoCo is deterministic given a fixed model and timestep (no RNG), so these
pin the exact claims made in README.md and the corpus finding. In
particular test_case2 asserts the verdict is *deviation-flagged*: that is
the honesty guarantee of the whole artifact. If someone later tunes the
solver until Case 2 "passes", this test fails -- which is the point. A
green Case 2 would mean the harness had become a rubber stamp.

These load and step MuJoCo; they run in a couple of seconds but are marked
`integration` so they can be deselected with `-m "not integration"`.
"""

from __future__ import annotations

import pytest

from harness import models, run


@pytest.fixture(autouse=True)
def _models_present(in_repo_root):
    # run.run_case* read models/*.xml from the cwd; regenerate them fresh at
    # the repo root so the test never depends on stale committed XML.
    models.write_models("models")


@pytest.mark.integration
def test_case1_friction_reproduced():
    result = run.run_case1()
    assert result["verdict"] == "reproduced"
    # published: 0.005% relative error, well inside the 1% tolerance.
    assert result["relative_error"] < 1e-3
    assert result["fit_quality_r_squared"] > 0.9999
    assert result["measured_value_m_s2"] == pytest.approx(3.6306, rel=1e-3)


@pytest.mark.integration
def test_case2_bounce_deviation_flagged():
    result = run.run_case2()
    # THE honesty guard: this case is KNOWN to deviate; it must not silently pass.
    assert result["verdict"] == "deviation-flagged"
    # the decay LAW still fits well (heights decay like a constant-e process)...
    assert result["decay_law_fit_r_squared"] > 0.99
    # ...but at a different constant than requested: e_eff ~= 0.82 vs target 0.75.
    assert result["relative_error_e"] > 0.05  # exceeds the 5% tolerance => flagged
    assert 0.79 < result["measured_value_e_eff"] < 0.85
    assert result["measured_value_e_eff"] == pytest.approx(0.8200, rel=5e-3)


@pytest.mark.integration
def test_case2_finds_enough_bounces():
    # The peak-finder must recover the >=5 bounces the fit needs; a contact
    # model that dies early would silently starve the regression.
    result = run.run_case2()
    assert result["n_bounces_measured"] >= 5
