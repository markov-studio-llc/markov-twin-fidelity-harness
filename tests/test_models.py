"""Tests for the MJCF model generators (harness/models.py).

These confirm the generated XML is well-formed and loadable by the
installed MuJoCo, that the models carry the degrees of freedom the
DECISION.md write-up says they must (the whole friction result hinges on
the incline body having a normal-direction DOF), and that the
generation-time sanity guard fires.
"""

from __future__ import annotations

import math

import mujoco
import pytest

from harness import models


def test_incline_xml_loads_in_mujoco():
    m = mujoco.MjModel.from_xml_string(models.incline_xml())
    assert m is not None


def test_incline_has_two_slide_dofs_no_rotation():
    # The fix documented in DECISION.md: sphere + two orthogonal slide
    # joints (down-slope + normal), no rotation. If this regresses to a
    # single DOF the normal force becomes a solver artifact.
    m = mujoco.MjModel.from_xml_string(models.incline_xml())
    assert m.njnt == 2
    joint_types = {m.jnt_type[i] for i in range(m.njnt)}
    assert joint_types == {int(mujoco.mjtJoint.mjJNT_SLIDE)}


def test_incline_xml_rejects_static_case():
    # mu >= tan(theta) is not a sliding (MATCH) case; generation must refuse.
    with pytest.raises(ValueError):
        models.incline_xml(theta_deg=30.0, mu=0.6)


def test_incline_body_starts_above_the_plane():
    # Body is offset along the (rotated) plane normal by radius + gap, so it
    # starts just clear of the surface and settles onto it.
    m = mujoco.MjModel.from_xml_string(models.incline_xml())
    # body 0 is world; body 1 is the block.
    block_pos = m.body_pos[1]
    expected_norm = models.INCLINE_RADIUS + 0.01
    # rel=1e-3: the MJCF template writes coords to 6 decimals, so the offset
    # magnitude is exact only to that rounding -- a loose check is what this
    # test means anyway ("starts just clear of the plane along the normal").
    assert math.hypot(block_pos[0], block_pos[2]) == pytest.approx(expected_norm, rel=1e-3)


def test_bounce_xml_loads_and_has_free_body():
    m = mujoco.MjModel.from_xml_string(models.bounce_xml())
    assert m.njnt == 1
    assert m.jnt_type[0] == int(mujoco.mjtJoint.mjJNT_FREE)


def test_bounce_ball_starts_at_drop_height():
    m = mujoco.MjModel.from_xml_string(models.bounce_xml())
    ball_pos = m.body_pos[1]
    assert ball_pos[2] == pytest.approx(models.BALL_RADIUS + models.BALL_DROP_HEIGHT)


def test_write_models_produces_both_files(tmp_path):
    models.write_models(str(tmp_path))
    incline = tmp_path / "incline.xml"
    bounce = tmp_path / "bounce.xml"
    assert incline.exists() and bounce.exists()
    # both must load.
    assert mujoco.MjModel.from_xml_path(str(incline)) is not None
    assert mujoco.MjModel.from_xml_path(str(bounce)) is not None
