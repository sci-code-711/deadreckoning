### a selection of functions my code relies upon ###

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
        * ``quaternion`` -- Rotation quaternion
    """

    n = [dl, dm, dn]
    n_norm = np.linalg.norm(n)
    if n_norm == 0:
        quat = Q.quaternion(1,0,0,0)
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
        * ``tuple`` -- Returns a tuple of the form (yaw, pitch, roll)

    """
    l = M.degrees(M.atan2(2 * (quat.w * quat.x + quat.y * quat.z), (1 - 2 * (quat.x**2 + quat.y**2))))
    m = M.degrees(M.asin(2 * (quat.w * quat.y - quat.z * quat.x)))
    n = M.degrees(M.atan2(2 * (quat.w *quat.z + quat.x * quat.y), (1 - 2 * (quat.y**2 + quat.z**2))))

    return (l, m, n)


######################################################################################################################
# this function updates a quaternion using an angular velocity input

def q_update(qi,vl,vm,vn,dt):

    vl=M.radians(vl)

    vm=M.radians(vm)

    vn=M.radians(vn)

    dl=vl*(dt)/2
    dm=vm*(dt)/2
    dn=vn*(dt)/2

    w=[dl,dm,dn]
    w_norm=np.linalg.norm(w)

    if w_norm == 0:
        qw=Q.quaternion(1,0,0,0)
    else:
        qw=exp_q(dl/100,dm/100,dn/100)

    # combines previous quaternion with small displacement to estimate current attitude
    for c in range(0,100):
        qi=qi*qw

    qc=qi/np.absolute(qi)
    qww=exp_q(dl,dm,dn)

    return(qc,qww)


######################################################################################################################
# this function generates an object used in below function

def Omega(w):
    omega=np.array([[0,-w[0],-w[1],-w[2]],[w[0],0,w[2],-w[1]],[w[1],-w[2],0,w[0]],[w[2],w[1],-w[0],0]])
    return(omega)

######################################################################################################################
# this function uses Runga Kutta 4th order intergrtion to up date a quaternion using angular velocities

def RK4(qi,w1,w2,dt):

    qt=np.array([qi.w,qi.x,qi.y,qi.z])
    w11=np.array([M.radians(w1[0]),M.radians(w1[1]),M.radians(w1[2])])
    w22=np.array([M.radians(w2[0]),M.radians(w2[1]),M.radians(w2[2])])
    q1=qt
    k1=0.5*Omega(w11) @ q1
    q2=qt+dt*0.5*k1
    k2=0.5*Omega(0.5*(w11+w22)) @ q2
    q3=qt+dt*0.5*k2
    k3=0.5*Omega(0.5*(w11+w22)) @ q3
    q4=qt+dt*k3
    k4=0.5*Omega(w22) @ q4

    qft=qt+(dt/6)*(k1+2*k2+2*k3+k4)

    qf=qft/np.linalg.norm(qft)

    res=Q.quaternion(qf[0],qf[1],qf[2],qf[3])

    w=0.5*(w11+w22)
    qww=exp_q(w[0],w[1],w[2])

    return(res,qww)

#######################################################################################################################
# this function takes a gravity vector and converts this into a quaternion

def q_from_g(a_vec):

    a_vec[:]=a_vec[:]/(np.linalg.norm(a_vec))

    if np.linalg.norm(np.cross(ag,g))==0:
        qc=Q.quaternion(1,0,0,0)
    else:
        n=np.cross(ag,g)/np.linalg.norm(np.cross(ag,g))
        phi=np.arctan(np.linalg.norm(np.cross(ag,g))/np.dot(ag,g))

        if phi<0:
            phi=phi+np.pi

        qw=np.cos(phi/2)
        qx=n[0]*np.sin(phi/2)
        qy=n[1]*np.sin(phi/2)
        qz=n[2]*np.sin(phi/2)

        qc=Q.quaternion(qw,qx,qy,qz)

    return(qc)

#######################################################################################################################
# this function turns things into quaternions for compatability reasons

def quaternion(q):

    q_out=Q.quaternion(q[0],q[1],q[2],q[3])

    return(q_out)