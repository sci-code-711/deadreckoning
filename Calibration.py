# this code imports the raw readings from the IMU and calibrates all gyroscope readings for zero error
# by using the average of the first 1000 readings, for which the IMU remained stationary, and subtracting
# these averages from all gyroscope readings.

import pandas as pd
import numpy as np

with open("temporarily_filtered_file.csv") as file:
    data = pd.read_csv(
        file, delimiter=",", skiprows=0
    )  # opens file for use and enters it into array

length = data.shape[0]  # extracts parameter for length of data table

calib_length = 400
ro = calib_length / 2

cl = 0
cm = 0
cn = 0

# accelerometer calibration coefficients
a_coeff = np.array(
    [
        6.08032845e-03,
        -6.04269967e-03,
        -4.73899395e-03,
        9.77255708e00,
        9.82729876e00,
        9.81958972e00,
        -9.78688601e-03,
        -5.66163490e-04,
        -1.29535798e-02,
    ]
)
# accelerometer coefficients
g_coeff = np.array(
    [
        0.03702091,
        -0.02547652,
        0.01486421,
        0.00508759,
        0.00590621,
        -0.01795366,
        1.13553109,
        1.16152047,
        1.13504326,
    ]
)

### inserts coefficients into correct mathematical objects for accelerometer ###
c_a = a_coeff[6:]
Ta = np.array([[1, -a_coeff[0], a_coeff[1]], [0, 1, -a_coeff[2]], [0, 0, 1]])
Ka = np.array([[a_coeff[3], 0, 0], [0, a_coeff[4], 0], [0, 0, a_coeff[5]]])

### inserts coefficients into correct mathematical objects for gyroscope ###
cl = sum(data.iloc[0:calib_length, 4]) / calib_length
cm = sum(data.iloc[0:calib_length, 5]) / calib_length
cn = sum(data.iloc[0:calib_length, 6]) / calib_length

c_g = np.array([-cl, -cm, -cn])
Tg = np.array(
    [
        [1, -g_coeff[0], g_coeff[1]],
        [g_coeff[2], 1, -g_coeff[3]],
        [-g_coeff[4], g_coeff[5], 1],
    ]
)
Kg = np.array([[g_coeff[6], 0, 0], [0, g_coeff[7], 0], [0, 0, g_coeff[8]]])

to = data.loc[0, "t"]  # re=zeros the recorded time

for c in range(
    0, calib_length
):  # modifies all entries in data table by these calibration values, first 1000 entries should average to 0
    data.iloc[c, 0] = round((data.iloc[c, 0] - to) / 1000, 3)
    a_vec = np.array([data.iloc[c, 1], data.iloc[c, 2], data.iloc[c, 3]])
    a_cal = Ta @ Ka @ np.add(a_vec, c_a)
    data.iloc[c, 1] = a_cal[0]
    data.iloc[c, 2] = a_cal[1]
    data.iloc[c, 3] = a_cal[2]
    g_vec = np.array([data.iloc[c, 4], data.iloc[c, 5], data.iloc[c, 6]])
    g_cal = Tg @ Kg @ np.add(g_vec, c_g)
    data.iloc[c, 4] = g_cal[0]
    data.iloc[c, 5] = g_cal[1]
    data.iloc[c, 6] = g_cal[2]

data.to_csv(
    "Temp_calib_data.csv", index=False
)  # writes data with calibrated gyroscope readings to new temporary file

print("calibration complete")
