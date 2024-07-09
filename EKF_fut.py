#######################################################################################################################
# uses predictions about future motion to decide on activation of the Kalman filter
#######################################################################################################################
# this code takes the calibrated IMU readdings, inserts them into a larger array designed to store all positional, velocity, and acceleration data. It then uses initial acceleration readings during the calibration phasse to generate a gravity vector and from this calculates the rotation of the imu realtive to the gravitiational vector, which is asssumed to be vertical, and stores this value as a quaternian, the code then works through the data array one line at atime intergrating gyrosccope readings and converting these into a rotaion qauternian whic it then multiplies with the previous rotation to provide an updated orientation. this updated orientation is then used to intergrate the liner accelerations to velocities and then in turn displacements.

import pandas as pd
import numpy as np
from deadrec.quaternion import Quaternion
import Functions as func

#######################################################################################################################
# this section of the code opens the specified data file as a data frame, then generates the an expanded data frame
# used to store all calculated values, and then inputs the readings into this expanded data frame.


data = []
with open("Temp_calib_data.csv") as csvfile:
    data = pd.read_csv(
        csvfile, delimiter=",", skiprows=0
    )  # opens data file as an array

numRows = data.shape[0]
P = pd.DataFrame(
    index=range(numRows), columns=range(20)
)  # generates expanded dataframe
P.iloc[0, 7:20] = 0  # sets initial values for displacments and velocities to 0
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
)  # Inserts IMU data into  expanded Data frame using indicies as referneces

#######################################################################################################################
# this section of the code calculates the initial attitude of the IMU by averaging the acceleration readings during the
# calibration period, which are due to gravity, and using these to find the rotation of the imu relative to the
# vertical direction in the navigation frame.
tav = 0
callength = 300
Mat = np.zeros([4, 4])

for r in range(0, callength):
    tx = P.loc[r, "ax"]
    ty = P.loc[r, "ay"]
    tz = P.loc[r, "az"]
    tav = tav + np.sqrt(tx**2 + ty**2 + tz**2) / callength

gsize = tav  # 9.815 # should be 9.806
print(tav)

g = [0, 0, 1]
for c in range(0, 30):
    t = [P.loc[c, "ax"], P.loc[c, "ay"], P.loc[c, "az"]]
    tnorm = np.linalg.norm(t)
    t[:] = t[:] / tnorm

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


eigval, eigvec = np.linalg.eig(Mat)

c = np.argmax(eigval)
q = [eigvec[0, c], eigvec[1, c], eigvec[2, c], eigvec[3, c]]
Mat[:, :] = 0
norm = np.linalg.norm(q)

qc = Quaternion(q[0] / (norm), q[1] / (norm), q[2] / (norm), q[3] / (norm))
P.loc[0, "qw"] = round(qc.w)
P.loc[0, "qx"] = round(qc.x)
P.loc[0, "qy"] = round(qc.y)
P.loc[0, "qz"] = round(qc.z)

qcc = np.conjugate(qc)  # finds cojugate of initial quaternian

a = Quaternion(
    0, P.loc[0, "ax"], P.loc[0, "ay"], P.loc[0, "az"]
)  # converts initail accelerometer reading into a quaternian

# rotates acceleration quaternian using calculated rotationa quaternians
A = qc * a * qcc

# extracts rotated accelerations and inserts them into data frame
P.loc[0, "ax"] = round(A.x, 3)
P.loc[0, "ay"] = round(A.y, 3)
P.loc[0, "az"] = round(
    A.z - gsize, 3
)  # removes gravitiational accceleration from readings

# converts initial rotation quaternian into euler angles and inputs them into data frame
[l, m, n] = qc.to_euler_angles()
P.loc[0, "L"] = round(l, 3)
P.loc[0, "M"] = round(m, 3)
P.loc[0, "N"] = round(n, 3)

#######################################################################################################################
# this section of the code iterates row by row through data table calculating attitude roating cceleration readings
# into navigation frame and removing gravitiational effect. the code then intergrates linear accelerations to
# velocities and in turn displacements.
count = 0
gMat = np.zeros([4, 4])
fut_r = 5
acel_pred = np.zeros([fut_r * 2 + 1, 2])
activate_fut = np.zeros([numRows - 1, 2])
g = [0, 0, 1]

for r in range(1, numRows):
    activate_fut[r - 1, 0] = P.loc[r, "t"]
    qi = qc  # saves previous rotation quaternian for later use
    q_pred = qi

    dt = P.loc[r, "t"] - P.loc[r - 1, "t"]

    if r < fut_r:
        rmin = 0
        rmax = r + (fut_r + 1)
    elif r > numRows - (fut_r + 1):
        rmin = r - fut_r
        rmax = numRows
    else:
        rmin = r - fut_r
        rmax = r + (fut_r + 1)

    for row in range(rmin, r):
        acel_pred[fut_r - (r - row), 1] = (
            P.loc[row, "ax"] ** 2 + P.loc[row, "ay"] ** 2 + P.loc[row, "az"] ** 2
        ) ** 0.5
        acel_pred[fut_r - (r - row), 0] = P.loc[row, "t"]
    for row in range(r, rmax):
        w1 = np.array(
            [P.loc[row - 1, "vl"], P.loc[row - 1, "vm"], P.loc[row - 1, "vn"]]
        )
        w2 = np.array([P.loc[row, "vl"], P.loc[row, "vm"], P.loc[row, "vn"]])

        [q_pred, qw] = func.RK4(qi, w1, w2, dt)
        q_predc = np.conjugate(q_pred)

        aq_fut = Quaternion(
            0, data.loc[row, "ax"], data.loc[row, "ay"], data.loc[row, "az"]
        )

        A_pred = q_pred * aq_fut * q_predc
        Av_pred = [A_pred.x, A_pred.y, A_pred.z - gsize]
        if row == r:
            A = A_pred
            Av = Av_pred
            qc = q_pred

        acel_pred[row - r + fut_r, 1] = np.linalg.norm(Av_pred)
        acel_pred[row - r + fut_r, 0] = P.loc[row, "t"]

    dev = np.linalg.norm(Av)
    dev_max = acel_pred[np.argmax(acel_pred, axis=0)[1], 1]
    cum_dev = np.sum(acel_pred[:, 1])

    if dev_max < 0.035:
        activate_fut[r - 1, 1] = 0.1
        count = count + 1
        qcv = [qc.w, qc.x, qc.y, qc.z]

        # extract heading from attitude estimate
        qhead = Quaternion(
            qc.w / np.sqrt(qc.w**2 + qc.z**2),
            0,
            0,
            qc.z / np.sqrt(qc.w**2 + qc.z**2),
        )

        # form gravity estimate for pose
        for row in range(rmin, rmax):
            ag = [data.loc[row, "ax"], data.loc[row, "ay"], data.loc[row, "az"]]
            ag[:] = ag[:] / (np.linalg.norm(ag))

            if np.linalg.norm(np.cross(ag, g)) == 0:
                qgv = [qhead.w, qhead.x, qhead.y, qhead.z]
            else:
                n = np.cross(ag, g) / np.linalg.norm(np.cross(ag, g))
                phi = np.arctan(np.linalg.norm(np.cross(ag, g)) / np.dot(ag, g))

                if phi < 0:
                    phi = phi + np.pi

                qgw = np.cos(phi / 2)
                qgx = n[0] * np.sin(phi / 2)
                qgy = n[1] * np.sin(phi / 2)
                qgz = n[2] * np.sin(phi / 2)

                qg = qhead * Quaternion(qgw, qgx, qgy, qgz)
                qgv = [qg.w, qg.x, qg.y, qg.z]

            gMat = (np.outer(qgv, qgv) / (fut_r * 2 + 1)) + gMat

        beta = 1 / 5

        Mat = np.outer(qcv, qcv) * (1 - beta) + gMat * (beta)

        eigval, eigvec = np.linalg.eig(Mat)
        c = np.argmax(eigval)
        q = [eigvec[0, c].real, eigvec[1, c].real, eigvec[2, c].real, eigvec[3, c].real]
        Mat[:, :] = 0
        gMat[:, :] = 0
        norm = np.linalg.norm(q)

        qc = Quaternion(q[0] / (norm), q[1] / (norm), q[2] / (norm), q[3] / (norm))
        qcc = qc.conjugate()

        aq = Quaternion(0, data.loc[r, "ax"], data.loc[r, "ay"], data.loc[r, "az"])

        A = qc * aq * qcc
    else:
        activate_fut[r - 1, 1] = -0.1
        count = count

    # stroes rotation quaternion inot data table
    P.loc[r, "qw"] = round(qc.w, 3)
    P.loc[r, "qx"] = round(qc.x, 3)
    P.loc[r, "qy"] = round(qc.y, 3)
    P.loc[r, "qz"] = round(qc.z, 3)

    # converts roatation quaternian into euler angles to strore in data table
    [l, m, n] = qc.to_euler_angles()
    P.loc[r, "L"] = round(l, 6)
    P.loc[r, "M"] = round(m, 6)
    P.loc[r, "N"] = round(n, 6)

    # rotates accelerations into navigation frame

    # inputs linear accelerations into data frame taking acount for gravitational effects and with a minimum cut off of 2cm/ss
    P.loc[r, "az"] = round(A.z - gsize, 10)
    P.loc[r, "ax"] = round(A.x, 10)
    P.loc[r, "ay"] = round(A.y, 10)

    # intergrates navigation frame accelerations to find linear velocities
    P.loc[r, "VX"] = round(
        P.loc[r - 1, "VX"] + ((P.loc[r - 1, "ax"] + P.loc[r, "ax"]) * dt / 2), 10
    )
    P.loc[r, "VY"] = round(
        P.loc[r - 1, "VY"] + ((P.loc[r - 1, "ay"] + P.loc[r, "ay"]) * dt / 2), 10
    )
    P.loc[r, "VZ"] = round(
        P.loc[r - 1, "VZ"] + ((P.loc[r - 1, "az"] + P.loc[r, "az"]) * dt / 2), 10
    )

    # intergrates linea velocities to find linear displacments
    P.loc[r, "X"] = round(
        P.loc[r - 1, "X"]
        + ((P.loc[r - 1, "VX"]) * dt)
        + ((P.loc[r - 1, "ax"] + P.loc[r, "ax"]) * (dt**2) / 4),
        10,
    )
    P.loc[r, "Y"] = round(
        P.loc[r - 1, "Y"]
        + ((P.loc[r - 1, "VY"]) * dt)
        + ((P.loc[r - 1, "ax"] + P.loc[r, "ax"]) * (dt**2) / 4),
        10,
    )
    P.loc[r, "Z"] = round(
        P.loc[r - 1, "Z"]
        + ((P.loc[r - 1, "VZ"]) * dt)
        + ((P.loc[r - 1, "az"] + P.loc[r, "az"]) * (dt**2) / 4),
        10,
    )

print("gcal_count:")
print(count)
P.to_csv(
    "Temp_EKF_fut_Positional.csv", index=False
)  # saves data table of reconstructed trajectory into a csv file

print("Deadreckoning complete")
