#######################################################################################################################
# Unfiltered dead reckoning.
#######################################################################################################################
# this code takes the calibrated IMU readings, inserts them into a larger array designed to store all positional, velocity, and acceleration data. It then uses initial acceleration readings during the calibration phasse to generate a gravity vector and from this calculates the rotation of the imu realtive to the gravitiational vector, which is asssumed to be vertical, and stores this value as a quaternian, the code then works through the data array one line at atime intergrating gyrosccope readings and converting these into a rotaion qauternian whic it then multiplies with the previous rotation to provide an updated orientation. this updated orientation is then used to intergrate the liner accelerations to velocities and then in turn displacements.

import pandas as pd
import numpy as np
from deadrec.quaternion import Quaternion
import Functions as func

#######################################################################################################################
# this section of the code opens the specified data file as a data frame, then generates the an expanded data frame
# used to store all calculated values, and then inputs the readings into this expanded data frame.


with open("Temp_calib_data.csv") as file:
    data = pd.read_csv(file, delimiter=",", skiprows=0)  # opens data file as an array

num_rows = data.shape[0]
P = pd.DataFrame(
    index=range(num_rows), columns=range(20)
)  # generates expanded data frame
P.iloc[0, 7:20] = 0  # sets initial values for displacements and velocities to 0
P.columns = [
    "t",
    "ax",
    "ay",
    "az",
    "vl",
    "vm",
    "vn",
    "VX",
    "VY",
    "VZ",
    "L",
    "M",
    "N",
    "X",
    "Y",
    "Z",
    "qw",
    "qx",
    "qy",
    "qz",
]  # adds indexes to data frame
P.update(
    data
)  # Inserts IMU data into  expanded Data frame using indices as references


#######################################################################################################################
# this section of the code calculates the initial attitude of the IMU by averaging the acceleration readings during the
# calibration period, which are due to gravity, and using these to find the rotation of the imu relative to the
# vertical direction in the navigation frame.
tav = 0
calib_length = 400
Mat = np.zeros([4, 4])

for r in range(0, calib_length):
    tx = P.loc[r, "ax"]
    ty = P.loc[r, "ay"]
    tz = P.loc[r, "az"]
    tav = tav + np.sqrt(tx**2 + ty**2 + tz**2) / calib_length

grav_accel = tav  # should be 9.806
# print(tav + "should be 9.806")

g = [0, 0, 1]

for c in range(0, 30):
    t = [P.loc[c, "ax"], P.loc[c, "ay"], P.loc[c, "az"]]
    t_norm = np.linalg.norm(t)
    t[:] = t[:] / t_norm

    if np.linalg.norm(np.cross(t, g)) == 0:
        q = [1, 0, 0, 0]
    else:
        n = np.cross(t, g) / np.linalg.norm(np.cross(t, g))
        phi = np.arctan(np.linalg.norm(np.cross(t, g)) / np.dot(t, g))

        if phi < 0:
            phi = phi + np.pi

        qw = np.cos(phi / 2)
        qx = n[0] * np.sin(phi / 2)
        qy = n[1] * np.sin(phi / 2)
        qz = n[2] * np.sin(phi / 2)

        q = [qw, qx, qy, qz]

    Mat = Mat + np.outer(q, q) / 30


eig_val, eig_vec = np.linalg.eig(Mat)

c = np.argmax(eig_val)
q = [eig_vec[0, c], eig_vec[1, c], eig_vec[2, c], eig_vec[3, c]]
Mat[:, :] = 0
norm = np.linalg.norm(q)

qc = Quaternion(q[0] / (norm), q[1] / (norm), q[2] / (norm), q[3] / (norm))
P.loc[0, "qw"] = round(qc.w)
P.loc[0, "qx"] = round(qc.x)
P.loc[0, "qy"] = round(qc.y)
P.loc[0, "qz"] = round(qc.z)

qcc = qc.conjugate()  # finds conjugate of initial quaternion

a = Quaternion(
    0, P.loc[0, "ax"], P.loc[0, "ay"], P.loc[0, "az"]
)  # converts initial accelerometer reading into a quaternion

# rotates acceleration quaternion using calculated rotational quaternions
A = qc * a * qcc

# extracts rotated accelerations and inserts them into data frame
P.loc[0, "ax"] = A.x
P.loc[0, "ay"] = A.y
P.loc[0, "az"] = A.z - grav_accel  # removes gravitational acceleration from readings

# converts initial rotation quaternion into euler angles and inputs them into data frame
[l, m, n] = qc.to_euler_angles()
P.loc[0, "L"] = round(l, 3)
P.loc[0, "M"] = round(m, 3)
P.loc[0, "N"] = round(n, 3)

#######################################################################################################################
# this section of the code iterates row by row through the data table calculating attitude, rotating acceleration
# readings into the navigation frame, and removing gravitational effects. the code then integrates linear accelerations to
# velocities and in turn displacements.
count = 0
alpha = 1 / 5
gMat = np.zeros([4, 4])
r_max = 0
fut_c = 0
g = [0, 0, 1]
activate = np.zeros([num_rows - 1, 2])

for r in range(1, num_rows):
    activate[r - 1, 0] = P.loc[r, "t"]

    qi = qc  # saves previous rotation quaternion for later use

    dt = P.loc[r, "t"] - P.loc[r - 1, "t"]

    # integrates angular velocities to find angular displacement over time step
    w1 = np.array([P.loc[r - 1, "vl"], P.loc[r - 1, "vm"], P.loc[r - 1, "vn"]])
    w2 = np.array([P.loc[r, "vl"], P.loc[r, "vm"], P.loc[r, "vn"]])

    [qc, qw] = func.RK4(qi, w1, w2, dt)
    qcc = qc.conjugate()

    aq = Quaternion(0, data.loc[r, "ax"], data.loc[r, "ay"], data.loc[r, "az"])

    A = qc * aq * qcc
    Av = [A.x, A.y, A.z - grav_accel]

    dev_bod = (
        np.absolute(
            ((P.loc[r, "ax"]) ** 2 + (P.loc[r, "ay"]) ** 2 + (P.loc[r, "az"]) ** 2)
            - grav_accel**2
        )
    ) ** 0.5
    dev_nav = np.linalg.norm(Av)
    # print(dev_bod,r)
    if dev_bod < dev_nav:
        dev = dev_nav
    else:
        dev = dev_nav

    if dev < 0.02:  # set threshold parameter for Kalman filter
        activate[r - 1, 1] = 0.1
        count = count + 1
        qcv = [qc.w, qc.x, qc.y, qc.z]

        # extract heading from attitude estimate
        q_head = Quaternion(
            qc.w / np.sqrt(qc.w**2 + qc.z**2),
            0,
            0,
            qc.z / np.sqrt(qc.w**2 + qc.z**2),
        )

        # form gravity estimate for pose
        ag = [data.loc[r, "ax"], data.loc[r, "ay"], data.loc[r, "az"]]
        ag[:] = ag[:] / (np.linalg.norm(ag))

        if np.linalg.norm(np.cross(ag, g)) == 0:
            qgv = [q_head.w, q_head.x, q_head.y, q_head.z]
        else:
            n = np.cross(ag, g) / np.linalg.norm(np.cross(ag, g))
            phi = np.arctan(np.linalg.norm(np.cross(ag, g)) / np.dot(ag, g))

            if phi < 0:
                phi = phi + np.pi

            qgw = np.cos(phi / 2)
            qgx = n[0] * np.sin(phi / 2)
            qgy = n[1] * np.sin(phi / 2)
            qgz = n[2] * np.sin(phi / 2)

            qg = q_head * Quaternion(qgw, qgx, qgy, qgz)
            qgv = [qg.w, qg.x, qg.y, qg.z]

        gMat = np.outer(qgv, qgv)

        beta = 1 / 5

        Mat = np.outer(qcv, qcv) * (1 - beta) + gMat * (beta)

        eig_val, eig_vec = np.linalg.eig(Mat)
        c = np.argmax(eig_val)
        q = [eig_vec[0, c].real, eig_vec[1, c].real, eig_vec[2, c].real, eig_vec[3, c].real]
        Mat[:, :] = 0
        norm = np.linalg.norm(q)

        qc = Quaternion(q[0] / (norm), q[1] / (norm), q[2] / (norm), q[3] / (norm))
        qcc = qc.conjugate()

        aq = Quaternion(0, data.loc[r, "ax"], data.loc[r, "ay"], data.loc[r, "az"])

        A = qc * aq * qcc

    else:
        activate[r - 1, 1] = -0.1
        count = count

    # saves rotation quaternion to data table
    P.loc[r, "qw"] = round(qc.w, 10)
    P.loc[r, "qx"] = round(qc.x, 10)
    P.loc[r, "qy"] = round(qc.y, 10)
    P.loc[r, "qz"] = round(qc.z, 10)

    # converts rotation quaternion into euler angles
    [l, m, n] = qc.to_euler_angles()
    P.loc[r, "L"] = round(l, 6)
    P.loc[r, "M"] = round(m, 6)
    P.loc[r, "N"] = round(n, 6)

    # rotates accelerations into navigation frame

    # inputs accelerations into data frame taking acount for gravitational effects and with a minimum cut off of 2cm/ss
    P.loc[r, "az"] = A.z - grav_accel
    P.loc[r, "ax"] = A.x
    P.loc[r, "ay"] = A.y  # -0.00473466*P.loc[r-1,"vl"]

    # integrates navigation frame accelerations to find linear velocities
    P.loc[r, "VX"] = P.loc[r - 1, "VX"] + (
        (P.loc[r - 1, "ax"] + P.loc[r, "ax"]) * dt / 2
    )
    P.loc[r, "VY"] = P.loc[r - 1, "VY"] + (
        (P.loc[r - 1, "ay"] + P.loc[r, "ay"]) * dt / 2
    )
    P.loc[r, "VZ"] = P.loc[r - 1, "VZ"] + (
        (P.loc[r - 1, "az"] + P.loc[r, "az"]) * dt / 2
    )

    # integrates linear velocities to find linear displacements
    P.loc[r, "X"] = (
        P.loc[r - 1, "X"]
        + ((P.loc[r - 1, "VX"]) * dt)
        + ((P.loc[r - 1, "ax"] + P.loc[r, "ax"]) * (dt**2) / 4)
    )
    P.loc[r, "Y"] = (
        P.loc[r - 1, "Y"]
        + ((P.loc[r - 1, "VY"]) * dt)
        + ((P.loc[r - 1, "ax"] + P.loc[r, "ax"]) * (dt**2) / 4)
    )
    P.loc[r, "Z"] = (
        P.loc[r - 1, "Z"]
        + ((P.loc[r - 1, "VZ"]) * dt)
        + ((P.loc[r - 1, "az"] + P.loc[r, "az"]) * (dt**2) / 4)
    )

print("g_cal_count:")
print(count)
P.to_csv("Temp_EKF_Positional.csv", index=False)

print("Dead reckoning complete")
