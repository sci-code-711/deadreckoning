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

`RK4Integrator` is currently the only integration strategy implemented.
Other strategies — trading accuracy for fewer `w(t)` evaluations per step,
or vice versa — are conceivable future additions, without committing to
any particular one here.

## How we handle look-ahead windows

Integration itself never looks beyond the step's own `[t0, t1]` — every
`w(t)` evaluation above falls strictly within that interval, at `t0`, the
midpoint, or `t1`. Whatever look-ahead a reconstruction needs comes
entirely from the interpolator supplying `w(t)`, not from the integration
method. See the [interpolators page](interpolators.md#how-we-handle-look-ahead-windows)
for why some interpolators need samples beyond the step's own endpoints.
