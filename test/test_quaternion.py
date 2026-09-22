from deadrec.quaternion import Quaternion
import numpy as np
import pytest


def test_quaternion_init():
    quat = Quaternion(1, 2, 3, 4)

    assert quat.w == 1
    assert quat.x == 2
    assert quat.y == 3
    assert quat.z == 4


def test_quaternion_str():
    quat = Quaternion(1, 2, 3, 4)

    assert str(quat) == "Quaternion (1, 2i, 3j, 4k)"


def test_quaternion_repr():
    assert Quaternion.__repr__ == Quaternion.__str__


def test_quaternion_eq_and_neq():
    quat_1 = Quaternion(1, 2, 3, 4)
    quat_2 = Quaternion(1, 0, 0, 0)

    assert quat_1 == quat_1
    assert quat_1 != quat_2
    assert quat_2 != quat_1
    assert quat_2 == quat_2

    assert quat_1 != 1
    assert quat_1 != "Quaternion(1, 2i, 3j, 4k)"
    assert quat_1 != [1, 2, 3, 4]


def test_quaternion_add():
    quat_1 = Quaternion(1, 2, 3, 4)
    quat_2 = Quaternion(-1, 0, 3, 6)

    assert quat_1 + quat_2 == Quaternion(0, 2, 6, 10)
    assert quat_1 + quat_2 == quat_2 + quat_1

    with pytest.raises(TypeError):
        assert quat_1 + 3


def test_quaternion_radd():
    assert Quaternion.__radd__ == Quaternion.__add__


def test_quaternion_sub():
    quat_1 = Quaternion(1, 2, 3, 4)
    quat_2 = Quaternion(2, 3, 1, 2)

    assert (quat_1 - quat_2) == Quaternion(-1, -1, 2, 2)
    assert (quat_1 - quat_2) == -(quat_2 - quat_1)


def test_quaternion_rsub():
    with pytest.raises(TypeError):
        3 - Quaternion(1, 2, 3, 4)


def test_quaternion_iadd():
    quat_1 = Quaternion(1, 2, 3, 4)
    quat_2 = Quaternion(1, 4, -2, 3)

    quat_1 += quat_2

    assert quat_1 == Quaternion(2, 6, 1, 7)


def test_quaternion_mul():
    quat_1 = Quaternion(1, 2, 3, 4)

    assert quat_1 * 2 == Quaternion(2, 4, 6, 8)
    assert quat_1 * 2.0 == Quaternion(2, 4, 6, 8)

    assert 3 * quat_1 == quat_1 * 3

    assert quat_1 * Quaternion(1, 0, 0, 0) == quat_1

    assert quat_1 * Quaternion(2, 3, 7, -3) == Quaternion(-13, -30, 31, 10)

    with pytest.raises(TypeError):
        Quaternion(1, 2, 3, 4) * "test"


def test_quaternion_rmul():
    quat_1 = Quaternion(1, 2, 3, 4)

    assert 3 * quat_1 == Quaternion(3, 6, 9, 12)
    assert 3.0 * quat_1 == Quaternion(3, 6, 9, 12)

    with pytest.raises(TypeError):
        "test" * quat_1


def test_quaternion_abs():
    assert abs(Quaternion(0, 0, 0, 0)) == 0
    assert abs(Quaternion(1, 0, 0, 0)) == 1
    assert abs(Quaternion(0, -1, 0, 0)) == 1
    assert abs(Quaternion(0, 0, 1, 0)) == 1
    assert abs(Quaternion(0, 0, 0, -1)) == 1
    assert abs(Quaternion(3, 0, 4, 0)) == 5
    assert abs(Quaternion(1, 1, 1, -1)) == 2
    assert abs(Quaternion(0, 4, -3, 0)) == 5


def test_quaternion_conjugate():
    assert Quaternion(1, 2, 3, 4).conjugate() == Quaternion(1, -2, -3, -4)


def test_quaternion_len():
    assert len(Quaternion(1, 2, 3, 4)) == 4
    assert len(Quaternion(4, 6, 2, 3)) == 4
    assert len(Quaternion(-7, 3, -4, 5)) == 4


def test_quaternion_neg():
    assert -Quaternion(1, -2, -3, 4) == Quaternion(-1, 2, 3, -4)


def test_quaternion_to_euler_angles():
    assert Quaternion(1, 0, 0, 0).to_euler_angles() == (0, 0, 0)

    assert Quaternion(0, 1, 0, 0).to_euler_angles() == (np.pi, 0, 0)

    # Fun quirk of euler angles (np.pi, 0, np.pi) == (0, np.pi, 0)
    assert Quaternion(0, 0, 1, 0).to_euler_angles() == (np.pi, 0, np.pi)

    assert Quaternion(0, 0, 0, 1).to_euler_angles() == (0, 0, np.pi)

    assert Quaternion(0, 1 / np.sqrt(2), 1 / np.sqrt(2), 0).to_euler_angles() == (
        np.pi,
        0.0,
        pytest.approx(np.pi / 2),
    )

    with pytest.raises(ValueError):
        Quaternion(2, 3, 4, 0).to_euler_angles()


def test_quaternion_from_euler_angles():
    assert Quaternion.from_eul_angles(0, 0, 0) == Quaternion(1, 0, 0, 0)

    assert Quaternion.from_eul_angles(np.pi / 2, 0, 0) == (
        Quaternion(pytest.approx(0), pytest.approx(1), 0, 0)
    )
    assert Quaternion.from_eul_angles(-np.pi / 2, 0, 0) == (
        Quaternion(pytest.approx(0), pytest.approx(-1), 0, 0)
    )

    assert Quaternion.from_eul_angles(0, np.pi / 2, 0) == (
        Quaternion(pytest.approx(0), 0, pytest.approx(1), 0)
    )
    assert Quaternion.from_eul_angles(0, -np.pi / 2, 0) == (
        Quaternion(pytest.approx(0), 0, pytest.approx(-1), 0)
    )

    assert Quaternion.from_eul_angles(0, 0, np.pi / 2) == (
        Quaternion(pytest.approx(0), 0, 0, pytest.approx(1))
    )
    assert Quaternion.from_eul_angles(0, 0, -np.pi / 2) == (
        Quaternion(pytest.approx(0), 0, 0, pytest.approx(-1))
    )

    assert Quaternion.from_eul_angles(np.pi / np.sqrt(2), np.pi / np.sqrt(2), 0) == Quaternion(
        pytest.approx(-1), pytest.approx(0), pytest.approx(0), 0
    )


def test_quaternion_from_axis_angle_zero_angle_is_identity():
    assert Quaternion.from_axis_angle([0, 0, 1], 0) == Quaternion(1, 0, 0, 0)


def test_quaternion_from_axis_angle_zero_axis_is_identity():
    assert Quaternion.from_axis_angle([0, 0, 0], np.pi / 2) == Quaternion(1, 0, 0, 0)


def test_quaternion_from_axis_angle_matches_from_eul_angles_single_axis():
    assert Quaternion.from_axis_angle([1, 0, 0], np.pi / 2) == Quaternion(
        pytest.approx(np.cos(np.pi / 4)), pytest.approx(np.sin(np.pi / 4)), 0, 0
    )
    assert Quaternion.from_axis_angle([0, 1, 0], np.pi / 2) == Quaternion(
        pytest.approx(np.cos(np.pi / 4)), 0, pytest.approx(np.sin(np.pi / 4)), 0
    )
    assert Quaternion.from_axis_angle([0, 0, 1], np.pi / 2) == Quaternion(
        pytest.approx(np.cos(np.pi / 4)), 0, 0, pytest.approx(np.sin(np.pi / 4))
    )


def test_quaternion_from_axis_angle_normalises_non_unit_axis():
    result = Quaternion.from_axis_angle([0, 0, 5], np.pi / 2)

    assert abs(result) == pytest.approx(1.0)
    assert result == Quaternion(
        pytest.approx(np.cos(np.pi / 4)), 0, 0, pytest.approx(np.sin(np.pi / 4))
    )


def test_quaternion_from_axis_angle_composes_via_multiplication():
    # Two successive rotations by angle/2 about the same axis should equal
    # one rotation by angle.
    axis = [1.0, 2.0, -1.0]
    half = Quaternion.from_axis_angle(axis, np.pi / 3)
    full = Quaternion.from_axis_angle(axis, 2 * np.pi / 3)
    composed = half * half

    assert composed.w == pytest.approx(full.w)
    assert composed.x == pytest.approx(full.x)
    assert composed.y == pytest.approx(full.y)
    assert composed.z == pytest.approx(full.z)


def _assert_quaternions_close(a, b, abs_tol=1e-9):
    assert a.w == pytest.approx(b.w, abs=abs_tol)
    assert a.x == pytest.approx(b.x, abs=abs_tol)
    assert a.y == pytest.approx(b.y, abs=abs_tol)
    assert a.z == pytest.approx(b.z, abs=abs_tol)


def test_quaternion_slerp_endpoints():
    a = Quaternion(1, 0, 0, 0)
    b = Quaternion.from_axis_angle([0, 0, 1], np.pi / 2)

    _assert_quaternions_close(Quaternion.slerp(a, b, 0.0), a)
    _assert_quaternions_close(Quaternion.slerp(a, b, 1.0), b)


def test_quaternion_slerp_midpoint_bisects_the_angle():
    axis = [0, 0, 1]
    a = Quaternion(1, 0, 0, 0)
    b = Quaternion.from_axis_angle(axis, np.pi / 2)
    expected_mid = Quaternion.from_axis_angle(axis, np.pi / 4)

    _assert_quaternions_close(Quaternion.slerp(a, b, 0.5), expected_mid)


def test_quaternion_slerp_moves_at_a_constant_angular_rate():
    axis = [1.0, -2.0, 0.5]
    a = Quaternion(1, 0, 0, 0)
    b = Quaternion.from_axis_angle(axis, np.radians(120.0))

    ts = np.linspace(0, 1, 6)
    points = [Quaternion.slerp(a, b, t) for t in ts]

    def angle_between(p, q):
        dot = min(abs(p.w * q.w + p.x * q.x + p.y * q.y + p.z * q.z), 1.0)
        return 2 * np.arccos(dot)

    steps = [angle_between(points[i], points[i + 1]) for i in range(len(points) - 1)]

    for step in steps[1:]:
        assert step == pytest.approx(steps[0], abs=1e-9)


def test_quaternion_slerp_takes_the_shorter_path():
    a = Quaternion(1, 0, 0, 0)
    # Stored more than 90 degrees from `a` (dot < 0), even though it
    # represents a rotation only 10 degrees away from identity.
    b = -Quaternion.from_axis_angle([0, 0, 1], np.radians(10))

    result = Quaternion.slerp(a, b, 0.5)

    # The shorter path passes through a rotation close to identity, not
    # one close to the ~175 degree rotation the raw quaternions imply.
    assert abs(result.w) == pytest.approx(1.0, abs=1e-2)


def test_quaternion_slerp_of_identical_quaternions_is_stable():
    a = Quaternion.from_axis_angle([0, 1, 0], 0.3)

    _assert_quaternions_close(Quaternion.slerp(a, a, 0.5), a)
