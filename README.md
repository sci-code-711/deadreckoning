# Deadreckoning
A set of tools to reconstruct trajectorys from Inertial Motion Unit measurements (Gyroscope and Accelerometer). 

## Functionality:
- `quaternion.py` - An implimentation of quaternions for sue in attitude estimation see: [wiki:Quaternion](https://en.wikipedia.org/wiki/Quaternion)
- `Deadreckoning.py` - An implimentation of simple deadreckoning by directly intergating measurements
- `EKF.py` - A refined deadreckoning implimentation that uses prior gravity vector measurments to correct the attitude estimation via Extended Kalman Filtering (EKF)
- `EKF_fut.py` - An EKF implimentaion that uses prior and future gavity measurements to correct the attitude.

## Setup:

Dependencies are managed with [uv](https://docs.astral.sh/uv/) and declared
in `pyproject.toml`.

```bash
uv sync            # install dependencies into .venv
uv run ruff check . # lint
uv run pytest       # run tests
```

## Details:

The mathematical basis for the trajectory reconstruction methods used here cna be found in the [accompanying paper](https://github.com/sci-code-711/deadreckoning/blob/main/A035_paper.pdf).

The research for this paper was conducted using a 6 axis LSM6SD3 IMU embedded on a Arduino 33Iot board.

## Bibliography:

[1] D. Tedaldi, A. Pretto, and E. Menegatti, A robust and easy to implement method for imu calibration without external equipments, 2014.
