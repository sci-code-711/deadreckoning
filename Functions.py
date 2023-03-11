"""
A selection of functions used for integrating angular rotation measurements and
converting between euler angles and quaternions.

"""

import numpy as np
import math as M
import quaternion as Q


def exp_q(dl, dm, dn):
    """
    Convert an angular velocity vector to a rotation quaternion

    Arguments:
        * dl {``float``} -- Rate of rotation about x axis
        * dm {``float``} -- Rate of rotation about y axis
        * dn {``float``} -- Rate of rotation about z axis

    Returns:
        ``quaternion`` -- Rotation quaternion

    """
    n = [dl, dm, dn]
    n_norm = np.linalg.norm(n)
    if n_norm == 0:
        quat = Q.quaternion(1, 0, 0, 0)
    else:
        qw = np.cos(n_norm)
        qx = (dl / n_norm) * np.sin(n_norm)
        qy = (dm / n_norm) * np.sin(n_norm)
        qz = (dn / n_norm) * np.sin(n_norm)

        quat = Q.quaternion(qw, qx, qy, qz)

    return quat


def eul_from_q(quat):
    """
    Convert a rotation quaternion into a set of euler angles: (yaw, pitch, and
    roll)

    Arguments:
        * quat {``quaternion``} -- Quaternion to be converted

    Returns:
        ``tuple`` -- A tuple of the form (yaw, pitch, roll)

    """
    l = M.degrees(
        M.atan2(
            2 * (quat.w * quat.x + quat.y * quat.z),
            (1 - 2 * (quat.x**2 + quat.y**2))
        )
    )
    m = M.degrees(M.asin(2 * (quat.w * quat.y - quat.z * quat.x)))

    n = M.degrees(
        M.atan2(
            2 * (quat.w * quat.z + quat.x * quat.y),
            (1 - 2 * (quat.y**2 + quat.z**2))
        )
    )

    return (l, m, n)


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
    vl=M.radians(vl)
    vm = M.radians(vm)
    vn = M.radians(vn)

    dl = 0.5 * vl * dt
    dm = 0.5 * vm * dt
    dn = 0.5 * vn * dt

    w = [dl, dm, dn]
    w_norm = np.linalg.norm(w)

    if w_norm == 0:
        qw = Q.quaternion(1, 0, 0, 0)
    else:
        qw = exp_q(dl / steps, dm / steps, dn / steps)

    # combines previous quaternion with small displacement to estimate current
    # attitude.
    for _ in range(steps):
        qi = qi * qw
        qi = qi / np.absolute(qi)

    qww = exp_q(dl, dm, dn)

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

    qft = qt + (dt/6) * (k1 + 2 * k2 + 2 * k3 + k4)

    qf = qft/np.linalg.norm(qft)

    res = Q.quaternion(*qf)

    w = 0.5 * (w11 + w22)
    qww = exp_q(*w)

    return(res, qww)


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
    a_vec = a_vec / (np.linalg.norm(a_vec))

    if np.linalg.norm(np.cross(ag, g)) == 0:
        qc = Q.quaternion(1, 0, 0, 0)
    else:
        n = np.cross(ag, g)/np.linalg.norm(np.cross(ag, g))
        phi = np.arctan(np.linalg.norm(np.cross(ag, g))/np.dot(ag, g))

        if phi<0:
            phi = phi + np.pi

        qw = np.cos(phi/2)
        qx = n[0] * np.sin(phi/2)
        qy = n[1] * np.sin(phi/2)
        qz = n[2] * np.sin(phi/2)

        qc = Q.quaternion(qw, qx, qy, qz)

    return(qc)


def quaternion(quat):
    """
    Produces a quaternion object from a 4 item Iterable with the associated
    quaternion values (qw, qx, qy, qz).

    Arguments:
        * quat {``Iter``} -- len 4 iterable containing the values of the
            quaternion in the order (qw, qx, qy, qz)

    Returns:
        ``quaternion`` -- The quaternion object

    """
    assert len(quat) == 4, f"Failed to provide a valid quaternion."

    return  Q.quaternion(*quat)
