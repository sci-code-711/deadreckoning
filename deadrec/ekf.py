"""Attitude correction using gravity-vector measurements."""

import numpy as np

from .attitude import attitude_aligning_vectors, gravity_deviation
from .dead_reckoning import DeadReckoner, accel_to_nav_frame
from .kinematics import rk4_attitude_step
from .quaternion import Quaternion
from .samples import ImuSample, TrajectoryState


def _heading_only(attitude: Quaternion) -> Quaternion:
    """
    Extract the component of ``attitude`` that rotates about the gravity
    axis (its "heading"), discarding tilt. Assumes gravity is along z, i.e.
    the default ``gravity_direction=(0, 0, 1)`` convention used elsewhere in
    this package.

    Args:
        * attitude {``Quaternion``} -- The attitude to extract heading from.

    Returns:
        * {``Quaternion``} -- The heading-only component.

    """
    norm = np.sqrt(attitude.w**2 + attitude.z**2)
    return Quaternion(attitude.w / norm, 0, 0, attitude.z / norm)


def _blend_attitudes(a: Quaternion, b: Quaternion, weight_b: float) -> Quaternion:
    """
    Average two attitude quaternions via the eigenvector method (as used in
    :func:`deadrec.attitude.initial_attitude_from_gravity`, but for two
    unevenly-weighted quaternions instead of many equally-weighted ones).

    Args:
        * a {``Quaternion``} -- The first attitude, with weight ``1 - weight_b``.
        * b {``Quaternion``} -- The second attitude, with weight ``weight_b``.
        * weight_b {``float``} -- Weight given to ``b``, between 0 and 1.

    Returns:
        * {``Quaternion``} -- The blended attitude.

    """
    av = np.array([a.w, a.x, a.y, a.z])
    bv = np.array([b.w, b.x, b.y, b.z])
    mat = np.outer(av, av) * (1 - weight_b) + np.outer(bv, bv) * weight_b

    eig_val, eig_vec = np.linalg.eig(mat)
    q = eig_vec[:, np.argmax(eig_val)].real

    return Quaternion(*(q / np.linalg.norm(q)))


class GravityCorrectedEKF(DeadReckoner):
    """
    A :class:`DeadReckoner` that additionally fuses in a gravity-vector
    attitude correction at each step: whenever the predicted, gravity-removed
    acceleration is small enough to plausibly be measurement noise (rather
    than real acceleration), the predicted attitude is blended with an
    attitude estimated directly from the current accelerometer reading
    (assuming the system is stationary), correcting for gyroscope drift.

    Args:
        * initial_attitude, gravity_magnitude, gravity_direction,
          initial_velocity, initial_position -- See :class:`DeadReckoner`.
        * deviation_threshold {``float``} -- Maximum
          :func:`deadrec.attitude.gravity_deviation` at which the gravity
          correction is applied; above this, only the gyroscope-predicted
          attitude is used for the step. Defaults to ``0.02``.
        * beta {``float``} -- Weight given to the gravity-derived attitude
          when blending it with the predicted attitude (0 = ignore it
          entirely, 1 = use it exclusively). Defaults to ``0.2``.

    """

    def __init__(
        self,
        initial_attitude: Quaternion,
        gravity_magnitude: float,
        *,
        gravity_direction=(0, 0, 1),
        initial_velocity=(0.0, 0.0, 0.0),
        initial_position=(0.0, 0.0, 0.0),
        deviation_threshold: float = 0.02,
        beta: float = 0.2,
    ):
        super().__init__(
            initial_attitude,
            gravity_magnitude,
            gravity_direction=gravity_direction,
            initial_velocity=initial_velocity,
            initial_position=initial_position,
        )
        self.deviation_threshold = deviation_threshold
        self.beta = beta

    def _predict_attitude(self, sample: ImuSample, dt: float) -> Quaternion:
        predicted = super()._predict_attitude(sample, dt)
        predicted_accel_nav = accel_to_nav_frame(
            sample.accel, predicted, self.gravity_magnitude, self.gravity_direction
        )

        if gravity_deviation(predicted_accel_nav) >= self.deviation_threshold:
            return predicted

        gravity_estimate = _heading_only(predicted) * attitude_aligning_vectors(
            sample.accel, self.gravity_direction
        )

        return _blend_attitudes(predicted, gravity_estimate, self.beta)


class WindowedGravityCorrectedEKF(GravityCorrectedEKF):
    """
    A :class:`GravityCorrectedEKF` variant that decides whether to apply its
    gravity correction using a window of samples on both sides of the
    current one, rather than just the current one, and averages the
    gravity-derived attitude over that whole window rather than a single
    reading.

    Because it needs samples that haven't happened yet relative to the one
    it's currently deciding on, it can't process a live/streaming sample at
    a time like :class:`DeadReckoner`/:class:`GravityCorrectedEKF` can -
    :meth:`step` is disabled, and the full sequence of samples must be given
    to :meth:`run` up front.

    Args:
        * initial_attitude, gravity_magnitude, gravity_direction,
          initial_velocity, initial_position, beta -- See
          :class:`GravityCorrectedEKF`.
        * deviation_threshold {``float``} -- As in :class:`GravityCorrectedEKF`,
          but compared against the *worst* deviation anywhere in the
          window, not just the current sample. Defaults to ``0.035``.
        * window_radius {``int``} -- Number of samples either side of the
          current one to include in the window. Defaults to ``5``.

    """

    def __init__(
        self,
        initial_attitude: Quaternion,
        gravity_magnitude: float,
        *,
        gravity_direction=(0, 0, 1),
        initial_velocity=(0.0, 0.0, 0.0),
        initial_position=(0.0, 0.0, 0.0),
        deviation_threshold: float = 0.035,
        beta: float = 0.2,
        window_radius: int = 5,
    ):
        super().__init__(
            initial_attitude,
            gravity_magnitude,
            gravity_direction=gravity_direction,
            initial_velocity=initial_velocity,
            initial_position=initial_position,
            deviation_threshold=deviation_threshold,
            beta=beta,
        )
        self.window_radius = window_radius

    def step(self, sample: ImuSample) -> TrajectoryState:
        raise NotImplementedError(
            "WindowedGravityCorrectedEKF needs samples from both before and "
            "after each one to decide whether to apply its correction, so it "
            "can't process one live/streaming sample at a time - call run() "
            "with the full sequence of samples instead."
        )

    def run(self, samples) -> list[TrajectoryState]:
        samples = list(samples)
        if not samples:
            return []

        states = [self._seed_state(samples[0])]
        window_size = 2 * self.window_radius + 1

        for r in range(1, len(samples)):
            prev_attitude = self.attitude
            dt = samples[r].t - samples[r - 1].t
            rmin, rmax = self._window_bounds(r, len(samples))

            deviations = [gravity_deviation(states[row].accel_nav) for row in range(rmin, r)]

            predicted_attitude, predicted_accel_nav = None, None
            for row in range(r, rmax):
                q_pred = rk4_attitude_step(
                    prev_attitude, samples[row - 1].gyro, samples[row].gyro, dt
                )
                accel_nav_pred = accel_to_nav_frame(
                    samples[row].accel, q_pred, self.gravity_magnitude, self.gravity_direction
                )
                if row == r:
                    predicted_attitude, predicted_accel_nav = q_pred, accel_nav_pred
                deviations.append(gravity_deviation(accel_nav_pred))

            if max(deviations) < self.deviation_threshold:
                attitude = self._windowed_correction(
                    samples, rmin, rmax, window_size, predicted_attitude
                )
                accel_nav = accel_to_nav_frame(
                    samples[r].accel, attitude, self.gravity_magnitude, self.gravity_direction
                )
            else:
                attitude, accel_nav = predicted_attitude, predicted_accel_nav

            prev_state = states[-1]
            velocity = prev_state.velocity + (prev_state.accel_nav + accel_nav) * dt / 2
            position = (
                prev_state.position
                + prev_state.velocity * dt
                + (prev_state.accel_nav + accel_nav) * dt**2 / 4
            )
            states.append(
                TrajectoryState(
                    t=samples[r].t,
                    attitude=attitude,
                    accel_nav=accel_nav,
                    velocity=velocity,
                    position=position,
                    euler=attitude.to_euler_angles(),
                )
            )
            self.attitude = attitude

        return states

    def _seed_state(self, sample: ImuSample) -> TrajectoryState:
        accel_nav = accel_to_nav_frame(
            sample.accel, self.attitude, self.gravity_magnitude, self.gravity_direction
        )
        return TrajectoryState(
            t=sample.t,
            attitude=self.attitude,
            accel_nav=accel_nav,
            velocity=self.initial_velocity,
            position=self.initial_position,
            euler=self.attitude.to_euler_angles(),
        )

    def _window_bounds(self, r: int, n: int) -> tuple[int, int]:
        if r < self.window_radius:
            rmin, rmax = 0, r + self.window_radius + 1
        elif r > n - (self.window_radius + 1):
            rmin, rmax = r - self.window_radius, n
        else:
            rmin, rmax = r - self.window_radius, r + self.window_radius + 1

        return rmin, min(rmax, n)

    def _windowed_correction(self, samples, rmin, rmax, window_size, predicted_attitude):
        heading = _heading_only(predicted_attitude)

        gravity_mat = np.zeros((4, 4))
        for row in range(rmin, rmax):
            tilt = attitude_aligning_vectors(samples[row].accel, self.gravity_direction)
            qg = heading * tilt
            qgv = np.array([qg.w, qg.x, qg.y, qg.z])
            # Divides by the full window size even when the window is
            # clipped near either end of the sample sequence, matching
            # EKF_fut.py - so a clipped window's gravity estimate carries
            # proportionally less weight than a full one, rather than being
            # renormalised to it.
            gravity_mat = gravity_mat + np.outer(qgv, qgv) / window_size

        predicted_v = np.array(
            [predicted_attitude.w, predicted_attitude.x, predicted_attitude.y, predicted_attitude.z]
        )
        mat = np.outer(predicted_v, predicted_v) * (1 - self.beta) + gravity_mat * self.beta

        eig_val, eig_vec = np.linalg.eig(mat)
        q = eig_vec[:, np.argmax(eig_val)].real

        return Quaternion(*(q / np.linalg.norm(q)))
