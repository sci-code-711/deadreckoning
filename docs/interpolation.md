# Interpolation & integration

`DeadReckoner` and its EKF subclasses reconstruct attitude by propagating a
quaternion forward one IMU sample at a time. Doing that needs a
continuous angular-rate function `w(t)`: the attitude ODE

```
dq/dt = 0.5 * Omega(w(t)) * q
```

has to be evaluated at times that fall *between* the discrete gyro readings
a sensor actually produces, and how you fill in the gaps between samples -
as well as how you numerically integrate the ODE once you have `w(t)` - is a
real, measurable accuracy/cost tradeoff, not an implementation detail. This
page covers what's implemented and how to use it; the maths behind each
strategy lives on its own page:

- [Interpolators: the maths behind `AngularRateInterpolator`](interpolation/interpolators.md)
- [Integrators: the maths behind `AttitudeIntegrator`](interpolation/integrators.md)

## The orthogonal split

Two abstract base classes, defined in `deadrec/interpolation.py` and
`deadrec/attitude_integration.py`, split the problem into two independent
concerns:

- **`AngularRateInterpolator`** turns the IMU samples spanning one
  integration step (plus, for some strategies, extra context samples
  before and/or after) into a continuous `omega(t)` function.
- **`AttitudeIntegrator`** propagates a quaternion attitude across a step,
  querying `omega(t)` at whatever times its numerical scheme needs.

The entire contract between them is `omega(t)`, a plain
`Callable[[float], np.ndarray]`. An integrator never knows how many
samples of context went into building `omega(t)`, or how it was
constructed; an interpolator never knows how many times, or at what times,
`omega(t)` will be queried. Any interpolator can be paired with any
integrator.

## What's implemented

**Interpolators** (see [interpolators.md](interpolation/interpolators.md)
for the maths behind each):

- `TwoPointLinearInterpolator`
- `ZeroOrderHoldInterpolator`
- `CentredCubicHermiteInterpolator`

**Integrators** (see [integrators.md](interpolation/integrators.md)):

- `RK4Integrator`, `EulerIntegrator`, `ExactExponentialIntegrator`,
  `MuntheKaasIntegrator`, `MagnusIntegrator`, `ConingIntegrator`,
  `AdamsBashforth2Integrator`

## Using them in code

`DeadReckoner`, `GravityCorrectedEKF`, and `WindowedGravityCorrectedEKF`
all accept `interpolator=`/`integrator=` keyword arguments:

```python
from deadrec.ekf import WindowedGravityCorrectedEKF
from deadrec.interpolation import CentredCubicHermiteInterpolator
from deadrec.attitude_integration import RK4Integrator

reckoner = WindowedGravityCorrectedEKF(
    initial_attitude,
    gravity_magnitude,
    interpolator=CentredCubicHermiteInterpolator(),
    integrator=RK4Integrator(),
)
```

Both default to `TwoPointLinearInterpolator()` and `RK4Integrator()` if
omitted.

## The `needs_lookahead` compatibility rule

Some interpolators need samples that haven't happened yet relative to the
step they're building `omega(t)` for - `CentredCubicHermiteInterpolator`
is the one implemented here, since its tangent estimates need a sample
beyond each endpoint. `AngularRateInterpolator.needs_lookahead` reports
this (`True` iff `context_after > 0`).

`DeadReckoner` and `GravityCorrectedEKF` process samples one at a time as
they arrive and can't provide look-ahead, so they reject such an
interpolator at construction time:

```
ValueError: CentredCubicHermiteInterpolator needs look-ahead samples, so
it can't be used with DeadReckoner, which processes samples one at a time
as they arrive. Use WindowedGravityCorrectedEKF instead.
```

`WindowedGravityCorrectedEKF` is the only reckoner that accepts a
look-ahead-needing interpolator, since its `run()` always holds the full
sample sequence up front rather than streaming one sample at a time.

## CLI usage

The `deadrec run` and `deadrec stream` subcommands both expose
`--interpolator {linear,zoh,cubic-hermite}` and `--integrator {rk4}`,
defaulting to `linear`/`rk4`:

```bash
uv run deadrec run --in readings.csv --out trajectory.csv \
  --method ekf-fut --interpolator cubic-hermite --integrator rk4
```

`cubic-hermite` needs the same look-ahead `WindowedGravityCorrectedEKF`
provides, so it only works with `--method ekf-fut`; `deadrec stream` has
no windowed method, so passing `--interpolator cubic-hermite` there fails
with the same `ValueError` shown above, just raised earlier (at reckoner
construction, before any streaming starts).
