# NOTE_DRAFT -- unposted, for review before any publish decision

**Target:** one build-in-public post, sim/robotics-adjacent channel (X/Twitter
thread or a MuJoCo/robotics-sim subreddit -- pick at publish time). Markov
voice. Framing: validate your MuJoCo twin against analytical ground truth,
in an afternoon, on a laptop CPU.

---

**Draft post:**

If you're using MuJoCo as a digital twin for anything, here's a 30-minute
check worth running before you trust it: drop a ball, slide a block, and
see if the numbers match the ones you can derive with a pencil.

I built a small headless harness that does exactly that:

1. A block slides down a 30-degree incline with friction. Closed form:
   `a = g*(sin(theta) - mu*cos(theta))`. Measured against the sim: 0.005%
   off. That's a match.

2. A ball bounces on a plane, 15+ bounces tracked. The textbook
   coefficient-of-restitution model says bounce heights should decay as
   `e^(2n)`. The decay LAW fits almost perfectly (R^2 = 0.99995) -- but
   the coefficient I asked MuJoCo for and the one I actually measured are
   9% apart.

That second result isn't a bug I didn't fix. MuJoCo doesn't have a
"coefficient of restitution" dial -- contacts are a spring-damper model
under the hood, not an instant-bounce law. You can aim it at a target
restitution using a standard formula from contact mechanics, but what you
get out is a measurement, not a guarantee. If your twin depends on
getting bounce elasticity exactly right, that's the kind of gap you want
to know about before it costs you, not after.

Both cases, closed forms re-derived in the repo (not copied from a
memory of "what MuJoCo usually does"), full numbers and a writeup of two
modeling mistakes I made getting there (a box that got stuck to the
ground because of a redundant-contact artifact was the fun one) -- link
in thread / repo TBD at publish time.

Runs on a laptop CPU. No GPU, no cloud bill, under a minute end to end.

---

**Notes for whoever reviews this before it goes out:**

- Repo is currently private (`github.com/markov-studio-llc`, per standing
  infra rule -- no personal handle in the history). A publish decision
  means deciding whether/when it goes public, separately from posting
  about it.
- The "9% off" number is real for THIS configuration (e_target=0.75, this
  ball mass, this contact stiffness) -- it is not a general "MuJoCo is
  always 9% off" claim, and the post above doesn't say it is. Worth a
  second read before it goes out to make sure that line still reads that
  way.
- No SimBenchmark ranking claims, no "MuJoCo is worse than X" framing --
  SimBenchmark shows up only in DECISION.md, as a source for the
  restitution-coefficient *definition*, not as a performance comparison.
