# Deadreckoning
"Trajectory reconstruction using Inertial Motion Units (IMUs) and the implications for Aerobots on Venus."

Included are:
3 working copies of the dead reckoning code used during this project
- "Deadreckoning.py"
- "EKF.py"
- "EKF_fut.py"

Required Packages:
- Pandas
- Numpy
- matplotlib
- pytest

with a working control space that integrates data processing capabilities "~Deadreckoning_interface.ipynb"

along with a set of example data "example_data.csv".

Additionally an implementation of a multi positional calibration procedure for a 6 axis IMU. "~calibration_notebook.ipynb"

The research was completed using a 6 axis LSM6SD3 embedded on a Arduino 33Iot board.

for more details read the full report saved as "A035_paper.pdf"


Bibliography:

[1] D. Tedaldi, A. Pretto, and E. Menegatti, A robust and easy to implement method for imu calibration without external equipments, 2014.
