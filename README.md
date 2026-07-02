# Twin-fidelity harness (Cycle 1)

A small headless test rig that checks a MuJoCo simulation against a
closed-form physics result, on two cases. It's a validation artifact --
"does this simulator reproduce a known answer" -- not a new physics
finding. Two things went into it: MuJoCo 3.10.0, installed and run locally
on CPU, and two textbook mechanics problems with derivations in-repo
(`harness/analytical.py`), so nothing here is graded against a number
pulled from memory.

## What it does

**Case 1 -- friction on an incline.** A sphere slides down a 30-degree
incline with friction coefficient 0.15 (below `tan(30deg) = 0.577`, so it
slides instead of sticking). Closed form: `a = g*(sin(theta) -
mu*cos(theta))`. The simulated position trace is fit (least squares, not
eyeballed) to a quadratic in time to recover the acceleration.

- Analytical: **3.630644 m/s^2**
- Measured: **3.630461 m/s^2**
- Relative error: **0.005%** (tolerance: 1%)
- Fit quality: R^2 = 1.000000
- **Verdict: reproduced**

**Case 2 -- restitution on a bounce.** A ball is dropped from 1.0 m and
bounces on a plane at least 5 times; peak heights per bounce are
extracted and fit to `h_n = e^(2n) * h_0` (the standard velocity-based
coefficient of restitution -- see `DECISION.md` for why that convention
and not a raw height-ratio one). Target `e = 0.75`.

- Analytical target: **e = 0.75**
- Measured (fit from 15 bounces): **e_eff = 0.820**
- Relative error: **9.3%** (tolerance: 5%)
- Decay-law fit quality: R^2 = 0.99995 (tolerance: > 0.99)
- **Verdict: deviation-flagged**

That second verdict is not a bug we didn't get around to fixing. MuJoCo
has no first-class "coefficient of restitution" parameter -- contacts are
a continuous spring-damper model (`solref`/`solimp`), not an
instantaneous-impulse restitution law. This harness derives a target
`solref` from a standard damped-harmonic-oscillator restitution formula,
then *measures* what actually comes out and reports the gap, instead of
retuning the solver until the number matches. The decay law itself fits
almost perfectly (R^2 = 0.99995) -- the ball's bounce heights decay
exactly like a constant-`e` process. It's just a different constant than
the one we asked for. That gap, not the fit quality, is the honest
MuJoCo-fidelity result of this case. Full derivation and the exact
numbers in `results/case2_bounce.json`.

## Two failed model attempts, kept for the lesson

The first two versions of Case 1 didn't fail on the physics -- they failed
on the *contact modeling*, in ways worth naming because they're generic
MuJoCo traps, not specific to this harness:

1. A box resting flush on the incline creates four exactly-coplanar corner
   contacts. That's a statically-indeterminate setup: each corner
   independently reported close to the *entire* body weight as its normal
   force. Friction, bounded per-contact by `mu * that contact's normal
   force`, then resisted about 4x what it should have, and the block
   never moved.
2. Swapping to a sphere (one contact point, no redundancy) fixed that, but
   a version with only a down-slope degree of freedom removed the contact
   *normal* direction from the model entirely -- not "constrained by
   contact," just absent. With no way to move in that direction, MuJoCo's
   contact solver had nothing to solve for there, and reported an
   arbitrary, physically meaningless normal force.

The version that ships gives the sphere two orthogonal sliding degrees of
freedom (down-slope, which we measure, and normal, which lets contact
settle to the correct equilibrium) and no rotation. Full writeup in
`DECISION.md`.

## Layout

```
models/incline.xml   -- generated MJCF, case 1 (see harness/models.py)
models/bounce.xml    -- generated MJCF, case 2
harness/analytical.py -- closed-form derivations (the actual grader)
harness/models.py    -- MJCF generation + the solref->target-e heuristic
harness/run.py        -- headless run, fit, verdict, JSON output
results/*.json         -- machine-readable output from the last run
DECISION.md            -- verification trail: version checks, source reads,
                          the two failed model attempts, why case 2 misses
```

## Running it

```
pip install -r requirements.txt
python -m harness.models   # (re)generate models/*.xml
python -m harness.run      # run both cases, print + write results/*.json
```

Runs in well under a minute on a laptop CPU. No GPU, no cloud, no cost.

## Tests

```
pip install -r requirements-dev.txt
pytest                     # 28 tests: unit (analytical + models) + integration
pytest -m "not integration"  # unit-only, skips the MuJoCo runs
```

The suite is deliberately weighted toward `tests/test_analytical.py` — those
check the closed-form grader itself (against hand-computed values and physics
invariants like the damping-ratio round-trip), because that module is the
ground truth every verdict is measured against. `tests/test_harness.py` runs
both cases end-to-end and pins the published verdicts; note that the Case 2
test asserts the verdict is *deviation-flagged* — that is the honesty guard,
so a change that silently tuned the solver until Case 2 "passed" would fail
the suite, which is the intent.

## Limits, stated plainly

- Two cases, both single-body. This is a floor check, not a general
  MuJoCo-vs-analytical audit.
- Case 2's target-restitution heuristic is a known-approximate physics
  model (constant contact impedance) applied to a solver that doesn't
  actually hold impedance constant during a contact. The 9.3% gap is
  expected to move if the target `e`, ball mass, or contact stiffness
  change -- it is not a fixed "MuJoCo is always off by 9.3%" constant.
- Nothing here has been pushed anywhere or posted anywhere. This is the
  BUILD step; publishing is a separate, deliberate step after review.
