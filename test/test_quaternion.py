from quaternion import Quaternion
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
    assert (quat_1 - quat_2) ==  - (quat_2 - quat_1)


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


def test_quaternion_rmul():
    quat_1 = Quaternion(1, 2, 3, 4)

    assert 3 * quat_1 == Quaternion(3, 6, 9, 12)
    assert 3.0 * quat_1 == Quaternion(3, 6, 9, 12)


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
    assert - Quaternion(1, -2, -3, 4) == Quaternion(-1, 2, 3, -4)


def test_quaternion_to_euler_angles():
    assert True


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

    assert Quaternion.from_eul_angles(
            np.pi / np.sqrt(2), np.pi / np.sqrt(2), 0
        ) == Quaternion(
            pytest.approx(-1), pytest.approx(0), pytest.approx(0), 0
        )
