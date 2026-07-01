# Decisions and verification trail

This file records the choices that aren't obvious from reading the code, and
how each was checked against a primary source rather than assumed from
memory. "Physics is the grader" only means something if the physics was
actually re-derived and actually checked -- this is the paper trail for that.

## MuJoCo version and contact API (verified, not assumed)

Installed via `pip install mujoco` on 2026-07-01: **MuJoCo 3.10.0**.

Checked directly against the installed build, not against memory of an
older MuJoCo release: `mujoco.MjModel`'s field list contains no
`elasticity` / `bounce` / `restitution` attribute for rigid-body geoms
(`flex_stiffness` / `flex_damping` exist, but those govern soft-body flex
elements, not rigid contacts). **MuJoCo has no first-class
coefficient-of-restitution knob.** This is also stated directly in
MuJoCo's own documentation (`doc/modeling.rst`, "Solver parameters"
section, fetched from the `google-deepmind/mujoco` GitHub repo at `main`):
contacts are governed by `solref`/`solimp`, which parameterize a
continuous spring-damper *reference acceleration* model for the
constraint, not an instantaneous-impulse restitution law. The docs
explicitly recommend the **"direct" solref format** (`-stiffness
-damping`, as opposed to the default `(timeconst, dampratio)` format) for
this kind of work: *"allowing for perfectly elastic collisions
(damping = 0) ... the recommended format for system identification."*
That is the format `harness/models.py` uses for the bounce case.

## Why the "block" in Case 1 is a sphere, and carries two slide joints

Two failed attempts preceded the working model, both left in `DECISION.md`
rather than memory-holed, because the failure modes are the actual
MuJoCo-fidelity lesson here:

1. **Box + one slide joint (down-slope only).** A box lying flush on an
   infinite plane produces 4 exactly-coplanar corner contacts -- a
   statically-indeterminate configuration. Measured directly
   (`mj_contactForce` on each contact): each of the 4 corners
   independently reported a normal force close to the ball's *entire*
   weight component, summing to ~4x the true `m*g*cos(theta)`. Since
   friction is bounded *per contact* by `mu * normal_at_that_contact`,
   this let aggregate friction resist far more than
   `mu*m*g*cos(theta)`, and the block never moved (measured acceleration
   ~0, when `mu < tan(theta)` should have made it slide).
2. **Sphere + one slide joint (down-slope only).** Fixed the redundant-contact
   problem (a sphere touches an infinite plane at exactly one point), but
   introduced a different bug: with only the down-slope DOF modeled, the
   *normal* direction has zero degrees of freedom at all -- it's not
   "constrained by contact", it's simply absent from the model. MuJoCo's
   contact Jacobian for the normal direction, projected onto the one
   remaining DOF, is then identically zero (sliding along the slope
   doesn't change the sphere's distance from the plane to first order).
   With a zero Jacobian the solver has no dynamical requirement to
   converge on any particular normal force -- confirmed by reading
   `mj_contactForce` at rest: it reported an arbitrary ~5.4 N rather than
   the true equilibrium value `m*g*cos(theta)` = 8.50 N, and friction
   saturated at `mu` times that wrong number.

The working model: **sphere + two orthogonal slide joints** (down-slope
and contact-normal, no rotational joint). The normal-direction joint gives
the contact solver a real DOF to equilibrate the normal force against (it
settles at the correct `m*g*cos(theta)` through genuine dynamics, not a
solver artifact), a small damping on that joint only settles the
initial drop-onto-plane transient quickly, and the missing rotational DOF
keeps the sphere from spinning up under friction torque -- which matters
because a *free-spinning* sphere would transition from sliding to rolling
partway through and no longer match the closed form's pure-translation
assumption. The first 0.3 s (settling transient) is excluded from the
fit window; see `harness/run.py::CASE1_SETTLE_TIME_S`.

## The restitution-convention choice (verified against SimBenchmark's source)

The build brief flagged two candidate conventions for the symbol `e`:
(i) the standard **velocity** coefficient of restitution
(`e = |v_after|/|v_before|`, giving `h_n = e^(2n) * h_0`), or (ii) a raw
per-bounce **height ratio** (`h_n = e^n * h_0`). This harness uses **(i)**.

Verification, not assumption: the brief asks to confirm which convention
SimBenchmark's (leggedrobotics) bouncing test uses. Its source was read
directly (`github.com/leggedrobotics/SimBenchmark`,
`benchmark/include/BouncingBenchmark.hpp`, fetched 2026-07-01): the
benchmark's `Option` struct documents `e` as `"restitution coefficient
(0-1)"`, and the code comments that *"only elastic collision analytic
solution is supported"* for `e=1` -- i.e. the only case they trust
analytically is full energy conservation at the standard velocity-ratio
definition. (Their actual "bouncing test" measures single-collision energy
error across an n*n grid of balls, not a multi-bounce single-ball height
decay -- but the restitution coefficient's *definition* is what mattered
here, and it is the textbook velocity ratio, not a height ratio.) This
independently corroborates the everywhere-else-standard physics
definition (Newton's experimental law of impact / any rigid-body dynamics
text) already used in `harness/analytical.py`.

## Why the measured Case 2 restitution misses the target, honestly

`harness/analytical.py::damping_ratio_for_restitution` derives a target
`solref` from a **linear** spring-dashpot (Kelvin-Voigt) restitution
model, `e = exp(-zeta*pi/sqrt(1-zeta^2))`. That model assumes a *constant*
constraint impedance throughout the contact. MuJoCo's real impedance,
`d(r)` (set by `solimp`), varies with penetration depth `r` by
construction -- the harness sets `solimp` to keep `d(r)` as close to
constant as the model allows (`0.998 0.9999 0.0002 0.5 2`: high floor and
ceiling impedance, narrow transition width), but it is not perfectly
constant, and MuJoCo's actual constraint solver is a numerical-optimization
based soft-penalty method, not a closed-form linear ODE. The gap between
the heuristic's target `e=0.75` and the measured `e_eff≈0.82` (see
`results/case2_bounce.json`) is the honest, expected consequence of using
a first-order analytical heuristic to aim a fundamentally different
(non-impulsive, variable-impedance) numerical contact model. This is
exactly the deviation the build brief anticipated and asked to be
surfaced, not hidden or tuned away.

## SimBenchmark citation scope

SimBenchmark is cited here **only** as prior-art *methodology* -- the
general idea of grading a physics engine against a closed-form or
elastic-collision result. It is developed by RaiSim's authors (a
competing physics engine); its cross-engine performance rankings are not
cited or relied on anywhere in this repo, and nothing here treats it as an
authority on MuJoCo specifically.
