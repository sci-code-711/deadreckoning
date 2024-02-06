from quaternion import Quaternion

def test_quaternion_init():
    quat = Quaternion(1, 2, 3, 4)

    assert quat.w == 1
    assert quat.x == 2
    assert quat.y == 3
    assert quat.z == 4


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
    quat_2 = Quaternion(2, 3, 1, 2)

    assert (quat_1 + quat_2) == Quaternion(3, 5, 4, 6)
    assert (quat_1 + quat_2) == (quat_2 + quat_1)


def test_quaternion_sub():
    quat_1 = Quaternion(1, 2, 3, 4)
    quat_2 = Quaternion(2, 3, 1, 2)

    assert (quat_1 - quat_2) == Quaternion(-1, -1, 2, 2)
    assert (quat_1 - quat_2) != (quat_2 - quat_1)


def test_quaternion_rsub():
    quat_1 = Quaternion(1, 2, 3, 4)

    assert - quat_1 == Quaternion(-1, -2, -3, -4)


def test_quaternion_iadd():

    quat_1 = Quaternion(1, 2, 3, 4)

    quat_1 += Quaternion(1, 1, 1, 1)

    assert quat_1 == Quaternion(2, 3, 4, 5)
