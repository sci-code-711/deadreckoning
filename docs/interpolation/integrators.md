# Integrators: the maths

This page derives the model behind each `AttitudeIntegrator`. It's theory
only — for what these classes are, how to wire one into a reckoner, and how
look-ahead is handled at the code level, see the
[overview page](../interpolation.md). For how the angular-rate function
`w(t)` these integrators consume is itself constructed, see the
[interpolators page](interpolators.md) — this page takes `w(t)` as given.

## The ODE being integrated

Attitude is represented as a unit quaternion `q`, and its evolution under a
body-frame angular rate `w(t)` (rad/s) is governed by

```
dq/dt = 0.5 * Omega(w(t)) * q
```

where `Omega(w)` is the skew-symmetric quaternion-rate matrix built from
`w`. "Integrating attitude across a step" means: given `q0` at time `t0`
and a continuous function `w(t)` defined over `[t0, t1]`, find `q1`, the
attitude at `t1`, that this ODE evolves `q0` into. There's no closed-form
solution for general `w(t)`, so it has to be approximated numerically —
that's the job of an `AttitudeIntegrator`.

## `RK4Integrator`

The classical 4-stage Runge-Kutta method, specialised to this ODE. Writing
`f(t, q) = 0.5 * Omega(w(t)) * q` for the right-hand side, and `dt = t1 -
t0`, the method computes four stage derivatives:

```
k1 = f(t0,        q0)
k2 = f(t0 + dt/2,  q0 + dt/2 * k1)
k3 = f(t0 + dt/2,  q0 + dt/2 * k2)
k4 = f(t1,        q0 + dt   * k3)
```

and combines them as a weighted average to produce the step:

```
q1 = q0 + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
```

(followed by renormalising `q1` back to unit length, since the ODE only
exactly preserves the unit-quaternion constraint in the continuous limit,
not after a discrete numerical step).

Each stage evaluates the angular rate at a specific point:

- `k1` at `t0`, against the known starting attitude `q0`.
- `k2` at the midpoint `t0 + dt/2`, against a trial attitude advanced
  half a step using `k1`.
- `k3` again at the midpoint, but against a *different* trial attitude —
  advanced half a step using `k2` instead of `k1`, refining the estimate
  of what's happening at the midpoint.
- `k4` at the endpoint `t1`, against a trial attitude advanced a full
  step using `k3`.

The final combination weights the two midpoint estimates twice as heavily
as the two endpoint estimates, which is what gives the method its
accuracy: RK4 is 4th-order accurate, meaning its local truncation error
per step scales as `O(dt^5)` (global error `O(dt^4)`) — a property of the
method itself, following from the classical Runge-Kutta stage weights and
error analysis, not something specific to this codebase's implementation
of it.

## `EulerIntegrator`

The simplest possible numerical scheme: forward (explicit) Euler,
approximating the whole step's derivative using a single evaluation at
its start:

```
q1 = q0 + dt * 0.5*Omega(w(t0))*q0
```

(renormalized). Local truncation error `O(dt²)` per step (1st-order
accurate globally) — the cheapest strategy available, one `w(t)`
evaluation per step, useful as a low-cost baseline to compare the other
integrators against.

## `ExactExponentialIntegrator`

When the angular rate is genuinely constant over the step — fixed axis,
fixed magnitude `w` — the ODE has a closed-form solution: composing `q0`
with the rotation quaternion for angle `|w|·dt` about axis `w/|w|`,

```
q1 = q0 ⊗ (cos(|w|dt/2) + sin(|w|dt/2)·(w/|w|))
```

exact regardless of step size, with no truncation error at all. It
samples `w(t)` once — a single point estimate is all a genuinely constant
rate needs, since there's no "which point" ambiguity when the rate isn't
actually changing. Pairs naturally with a model that holds the rate
constant over the step.

## `MuntheKaasIntegrator`

Instead of stepping the quaternion ODE directly, this integrates the
angular rate itself to get a single net rotation vector for the whole
step, then composes that vector onto `q0` with one exact exponential
(the same composition `ExactExponentialIntegrator` uses, but with a
rotation vector built from the *whole* step rather than assumed
constant). For a rate-only vector ODE, RK4-style quadrature of `w(t)`
reduces exactly to Simpson's rule:

```
rotation_vector = dt/6 * (w(t0) + 4*w(tm) + w(t1))
q1 = q0 ⊗ exp(rotation_vector)
```

This composes via a true rotation exponential rather than a linear ODE
step in the ambient 4D quaternion space, structurally different from
`RK4Integrator`. It does *not*, however, correct for the non-commutativity
between rotations about *different* axes within the same step ("coning"
error) — its rotation vector only captures the plain integral of `w(t)`,
with no correction for how the rotation's own instantaneous axis moves
during the step. `MagnusIntegrator`, below, adds exactly that correction.

## `MagnusIntegrator`

The Magnus expansion solves the same linear ODE by writing the exact
solution as a single exponential of a rotation vector `Θ`, built as a
series:

```
Θ = ∫w dt - 1/2 ∫₀ᵗ∫₀ˢ [w(s), w(r)] dr ds + ...
```

where `[a, b]` is the so(3) commutator (`a × b` under the vector
representation). The first term alone is just the plain integral of the
rate — what `MuntheKaasIntegrator` computes. The second term is the
leading *coning correction*: it accounts for the rotation's axis itself
moving during the step, which a bare integral of `w(t)` can't capture.

Truncating after this term and assuming `w(t)` varies linearly between
the step's two endpoint samples `a = w(t0)`, `b = w(t1)` gives a closed
form for both integrals — no numerical double integration needed:

```
Θ = dt/2*(a + b) + dt²/12*(a × b)
q1 = q0 ⊗ exp(Θ)
```

This only needs two `w(t)` evaluations — cheaper than
`MuntheKaasIntegrator`'s three — while adding the correction
`MuntheKaasIntegrator` lacks. When the endpoint vectors are parallel
(including the constant-rate case), the cross term vanishes exactly and
this reduces to the plain trapezoidal single-axis case, which is itself
exact. For non-parallel endpoints, though, this is *not* an exact
solution — the Magnus series has further, uncomputed commutator terms in
general — only 2nd-order accurate (local truncation error `O(dt³)`), the
same order as the classical two-sample coning-compensation formulas from
the strapdown-INS literature this is closely related to.

## Currently implemented, and what's next

Five integration strategies are implemented, trading `w(t)`-evaluation
cost against accuracy and coning-correction: `EulerIntegrator` (1
evaluation, 1st-order) through `RK4Integrator`/`MuntheKaasIntegrator`
(3-4 evaluations, higher-order but no coning correction) to
`MagnusIntegrator` (2 evaluations, 2nd-order with coning correction).
Further strategies — closed-form strapdown-INS coning algorithms, higher-
order Magnus expansions, or multistep methods using rate history across
steps — are conceivable future additions, without committing to any
particular one here.

## How we handle look-ahead windows

Integration itself never looks beyond the step's own `[t0, t1]` — every
`w(t)` evaluation above falls strictly within that interval, at `t0`, the
midpoint, or `t1`. Whatever look-ahead a reconstruction needs comes
entirely from the interpolator supplying `w(t)`, not from the integration
method. See the [interpolators page](interpolators.md#how-we-handle-look-ahead-windows)
for why some interpolators need samples beyond the step's own endpoints.
