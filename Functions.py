"""
A selection of functions used for integrating angular rotation measurements and
converting between euler angles and quaternions.

"""

import numpy as np
import math as M
from deadrec.quaternion import Quaternion


GRAV_MOD = 9.8067
GRAV_VEC = np.array([0, 1, 0])
GRAV_ACCEL_VEC = GRAV_MOD * GRAV_VEC


def q_update(qi, vl, vm, vn, dt, *, steps=100):
    """
    Integrate a rate of change of euler angles over a time step dt to find a new
    attitude from the original given attitude qi.

    Arguments:
        * qi {``quaternion``} -- Original attitude quaternion
        * dl {``float``} -- rate od rotation about x axis in frame of IMU
        * dm {``float``} -- rate od rotation about y axis in frame of IMU
        * dn {``float``} -- rate od rotation about z axis in frame of IMU
        * dt {``float``} -- Time over which to integrate

    Keyword Arguments:
        ``int`` -- Number of step to break the integration down over.
            Default value is 100.

    """
    vl = M.radians(vl)
    vm = M.radians(vm)
    vn = M.radians(vn)

    dl = 0.5 * vl * dt
    dm = 0.5 * vm * dt
    dn = 0.5 * vn * dt

    w = [dl, dm, dn]
    w_norm = np.linalg.norm(w)

    if w_norm == 0:
        qw = Quaternion(1, 0, 0, 0)
    else:
        qw = Quaternion.from_eul_angles(dl / steps, dm / steps, dn / steps)

    # combines previous quaternion with small displacement to estimate current
    # attitude.
    for _ in range(steps):
        qi = qi * qw
        qi = qi / np.absolute(qi)

    qww = Quaternion.from_eul_angles(dl, dm, dn)

    return (qi, qww)


def Omega(w):
    """
    Convert a rate of angular rotation into a matrix used for multiplication
    with quaternions.

    Arguments:
        * w {``list``} -- Angular velocity 3 vector in (degrees/s)

    Returns:
        ``array`` -- multiplication matrix.

    """
    assert len(w) == 3, f"{w} is not a valid angular velocity vector."

    omega = np.array(
        [
            [0, -w[0], -w[1], -w[2]],
            [w[0], 0, w[2], -w[1]],
            [w[1], -w[2], 0, w[0]],
            [w[2], w[1], -w[0], 0],
        ]
    )
    return omega


def RK4(qi, w1, w2, dt):
    """
    Uses 4th Order Runge-Kutta integration to calculate a final attitude from
    an initial state and angular velocity  measurements.

    Arguments:
        * qi {``quaternion``} -- The initial attitude quaternion of the system
        * w1 {``Iter``} -- 3 vector of angular velocity measurement at the
            beginning of the time step in (degrees/s)
        * w2 {``Iter``} -- 3 vector of angular velocity measurement at the
            end of the time step in (degrees/s)
        * dt {``float``} -- The time step over which to perform the integration
            in (seconds)

    Returns:
        ``quaternion`` -- Final attitude quaternion of the system.

    """

    qt = np.array([qi.w, qi.x, qi.y, qi.z])
    w11 = np.array([M.radians(w1[0]), M.radians(w1[1]), M.radians(w1[2])])
    w22 = np.array([M.radians(w2[0]), M.radians(w2[1]), M.radians(w2[2])])
    q1 = qt
    k1 = 0.5 * Omega(w11) @ q1
    q2 = qt + dt * 0.5 * k1
    k2 = 0.5 * Omega(0.5 * (w11 + w22)) @ q2
    q3 = qt + dt * 0.5 * k2
    k3 = 0.5 * Omega(0.5 * (w11 + w22)) @ q3
    q4 = qt + dt * k3
    k4 = 0.5 * Omega(w22) @ q4

    qft = qt + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

    qf = qft / np.linalg.norm(qft)

    res = Quaternion(*qf)

    w = 0.5 * (w11 + w22)
    qww = Quaternion.from_eul_angles(*w)

    return (res, qww)


def q_from_g(a_vec):
    """
    Calculates the attitude quaternion of a system from a gravity measurement.
    This measures all attitude relative to acceleration due to gravity being
    vertically up.

    Arguments:
        * a_vec {``Iter``} -- 3 vector of measured acceleration in the frame
            of the IMU.

    Returns:
        ``quaternion`` -- The attitude quaternion of the system where positive y
            is considered the natural vertical coordinate.

    """
    a_vec = a_vec / np.linalg.norm(a_vec)

    cross_product = np.cross(a_vec, GRAV_VEC)
    cross_prod_norm = np.linalg.norm(cross_product)

    if cross_prod_norm == 0:
        qc = Quaternion(1, 0, 0, 0)
    else:
        n = cross_product / cross_prod_norm
        phi = np.arctan(cross_prod_norm / np.dot(a_vec, GRAV_VEC))

        if phi < 0:
            phi = phi + np.pi

        qw = np.cos(phi / 2)
        qx = n[0] * np.sin(phi / 2)
        qy = n[1] * np.sin(phi / 2)
        qz = n[2] * np.sin(phi / 2)

        qc = Quaternion(qw, qx, qy, qz)

    return qc
