from pathlib import Path

import numpy as np
import pytest

from deadrec.attitude import (
    attitude_aligning_vectors,
    gravity_deviation,
    initial_attitude_from_gravity,
)
from deadrec.quaternion import Quaternion


def _rotate(q: Quaternion, v: np.ndarray) -> np.ndarray:
    p = Quaternion(0, *v)
    r = q * p * q.conjugate()
    return np.array([r.x, r.y, r.z])


def test_attitude_aligning_vectors_identity_when_already_aligned():
    q = attitude_aligning_vectors([0.0, 0.0, 2.5], target=(0, 0, 1))

    assert q == Quaternion(1, 0, 0, 0)


def test_attitude_aligning_vectors_identity_when_anti_parallel():
    # The cross product of anti-parallel vectors is also zero, so the
    # original scripts (and this port) return the identity here too, even
    # though the vectors are opposed. This is a known limitation of the
    # method being preserved, not asserted as physically correct.
    q = attitude_aligning_vectors([0.0, 0.0, -1.0], target=(0, 0, 1))

    assert q == Quaternion(1, 0, 0, 0)


def test_attitude_aligning_vectors_rotates_source_onto_target():
    source = np.array([1.0, 0.3, -0.2])
    target = np.array([0.0, 0.0, 1.0])

    q = attitude_aligning_vectors(source, target)
    rotated = _rotate(q, source)

    assert np.allclose(rotated / np.linalg.norm(rotated), target, atol=1e-8)


def test_attitude_aligning_vectors_normalises_unnormalised_source():
    q_unit = attitude_aligning_vectors([0.0, 1.0, 0.0], target=(0, 0, 1))
    q_scaled = attitude_aligning_vectors([0.0, 9.8, 0.0], target=(0, 0, 1))

    assert q_unit == q_scaled


def test_initial_attitude_from_gravity_of_aligned_samples_is_identity():
    samples = np.tile([0.0, 0.0, 9.81], (10, 1))

    q = initial_attitude_from_gravity(samples)

    assert abs(q.w) == pytest.approx(1.0, abs=1e-8)
    assert q.x == pytest.approx(0.0, abs=1e-8)
    assert q.y == pytest.approx(0.0, abs=1e-8)
    assert q.z == pytest.approx(0.0, abs=1e-8)


def test_initial_attitude_from_gravity_matches_single_reading_for_one_sample():
    sample = np.array([[0.2, 9.7, 1.1]])

    from_average = initial_attitude_from_gravity(sample)
    from_single = attitude_aligning_vectors(sample[0], target=(0, 0, 1))

    # Eigenvectors are only defined up to an overall sign.
    same = all(
        getattr(from_average, attr) == pytest.approx(getattr(from_single, attr), abs=1e-8)
        for attr in ("w", "x", "y", "z")
    )
    flipped = all(
        getattr(from_average, attr) == pytest.approx(-getattr(from_single, attr), abs=1e-8)
        for attr in ("w", "x", "y", "z")
    )
    assert same or flipped


def test_initial_attitude_from_gravity_rejects_wrong_shape():
    with pytest.raises(ValueError):
        initial_attitude_from_gravity(np.array([1.0, 2.0, 3.0]))


def test_gravity_deviation():
    assert gravity_deviation([3.0, 4.0, 0.0]) == pytest.approx(5.0)
    assert gravity_deviation([0.0, 0.0, 0.0]) == pytest.approx(0.0)


def _reference_initial_attitude(accel_samples: np.ndarray) -> Quaternion:
    """Independent reference implementation of the eigenvector-averaging
    algorithm, kept only to regression-test against real data."""
    g = np.array([0, 0, 1])
    n = len(accel_samples)
    mat = np.zeros((4, 4))

    for row in accel_samples:
        t = np.asarray(row, dtype=float)
        t = t / np.linalg.norm(t)

        if np.linalg.norm(np.cross(t, g)) == 0:
            q = [1, 0, 0, 0]
        else:
            axis = np.cross(t, g) / np.linalg.norm(np.cross(t, g))
            phi = np.arctan(np.linalg.norm(np.cross(t, g)) / np.dot(t, g))

            if phi < 0:
                phi = phi + np.pi

            q = [
                np.cos(phi / 2),
                axis[0] * np.sin(phi / 2),
                axis[1] * np.sin(phi / 2),
                axis[2] * np.sin(phi / 2),
            ]

        mat = mat + np.outer(q, q) / n

    eig_val, eig_vec = np.linalg.eig(mat)
    c = np.argmax(eig_val)
    q = [eig_vec[0, c], eig_vec[1, c], eig_vec[2, c], eig_vec[3, c]]
    norm = np.linalg.norm(q)

    return Quaternion(q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def test_initial_attitude_from_gravity_matches_reference_on_example_data():
    path = Path(__file__).resolve().parent.parent / "example_data" / "Example_data.csv"
    data = np.genfromtxt(path, delimiter=",", names=True)
    accel = np.column_stack([data["ax"][:30], data["ay"][:30], data["az"][:30]])

    expected = _reference_initial_attitude(accel)
    result = initial_attitude_from_gravity(accel)

    assert result.w == pytest.approx(expected.w, abs=1e-9)
    assert result.x == pytest.approx(expected.x, abs=1e-9)
    assert result.y == pytest.approx(expected.y, abs=1e-9)
    assert result.z == pytest.approx(expected.z, abs=1e-9)
