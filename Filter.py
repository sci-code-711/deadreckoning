#!/usr/bin/env python
"""
This script temporally filters a data file containing IMU measurements one
set per row. The time measurement should be the first item of each row.
"""

from argparse import ArgumentParser

import pandas as pd


def arg_parser():
    """Command line argument parser"""

    parser = ArgumentParser(
        "low pass filter an IMU file by taking moving averages of measurements."
    )
    parser.add_argument(
        "--in_file",
        required=True,
        help="The file containing the IMU data to be filtered",
    )
    parser.add_argument(
        "--out_file",
        default="./temporarily_filtered_file.csv",
        help="The file to output the filtered data to.",
    )
    parser.add_argument(
        "--filt_range",
        default=2,
        type=int,
        help=(
            f"The range over which to average eg. '--filt_range=2' corresponds "
            f"to each filtered measurement being the average of the bin itself "
            f"and the bins either side within a range of 2."
        ),
    )

    return parser.parse_args()


def main():
    """Main program."""

    # parse command line arguments
    args = arg_parser()

    data = []  # create array in which data will be stored
    with open(args.in_file) as csv_file:
        # opens file for use and enters it into array
        data = pd.read_csv(csv_file, delimiter=",", skiprows=[2])

    # extracts parameter for length of data table
    length = data.shape[0]

    # generates new dataframe to save filtered values to
    filt_data = data.copy()

    # specifies number of bins either side to average over
    print(
        f"Filtering will take a moving average of {2 * args.filt_range + 1} "
        f"measurements."
    )

    tot = 0
    for r in range(length):
        if r == 0:
            for a in range(args.filt_range + 1):
                tot = tot + data.iloc[a, 1:]
            filt_data.iloc[0, 1:] = tot / (args.filt_range + 1)

        elif r < args.filt_range + 1:
            tot = tot + data.iloc[r + args.filt_range, 1:]
            filt_data.iloc[r, 1:] = tot / (r + args.filt_range + 1)

        elif r > length - args.filt_range - 1:
            tot = tot - data.iloc[r - args.filt_range - 1, 1:]
            filt_data.iloc[r, 1:] = tot / (length - r + args.filt_range)

        else:
            tot += data.iloc[r + args.filt_range, 1:]
            tot -= data.iloc[r - args.filt_range - 1, 1:]
            filt_data.iloc[r, 1:] = tot / (2 * args.filt_range + 1)

    # writes data with calibrated gyroscope readings to new file.
    filt_data.to_csv(args.out_file, index=False)

    print(f"filtering complete output saved to: {args.out_file}")


if __name__ == "__main__":
    main()
