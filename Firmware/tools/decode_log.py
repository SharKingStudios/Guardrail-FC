#!/usr/bin/env python3
"""Convert a Guardrail .grf flight log to CSV on a desktop computer."""

import argparse
import csv
import struct
from pathlib import Path


MAGIC = b"GRFC"
FORMAT = "<IBB6h5iHBB10H10H"
SIZE = struct.calcsize(FORMAT)
INPUTS = [f"J{connector}_{side}" for connector in range(1, 6) for side in ("LEFT", "RIGHT")]
OUTPUTS = [f"J{connector}_{side}" for connector in range(6, 11) for side in ("LEFT", "RIGHT")]
BASE_HEADER = [
    "time_ms", "mode", "flags", "imu_ok", "barometer_ok", "gnss_ok", "receiver_ok",
    "failsafe", "auto_active", "auto_override", "roll_deg", "pitch_deg", "gyro_x_dps", "gyro_y_dps",
    "gyro_z_dps", "imu_temp_c", "pressure_pa", "baro_altitude_m", "latitude_deg",
    "longitude_deg", "gps_altitude_m", "speed_mps", "satellites", "fix_quality",
]


def rows(path):
    with path.open("rb") as source:
        if source.read(4) != MAGIC:
            raise ValueError(f"{path} is not a Guardrail log")
        version, record_size = source.read(2)
        if version != 1 or record_size != SIZE:
            raise ValueError(f"unsupported log version/record size: {version}/{record_size}")
        while data := source.read(SIZE):
            if len(data) != SIZE:
                print(f"warning: ignoring partial final record in {path}")
                break
            values = list(struct.unpack(FORMAT, data))
            flags = values[2]
            yield [
                values[0], ("PASSTHROUGH", "ASSIST", "AUTO")[values[1]] if values[1] < 3 else "UNKNOWN", flags,
                *((flags >> bit) & 1 for bit in range(7)),
                *(value / 100 for value in values[3:9]),
                values[9], values[10] / 100, values[11] / 10_000_000,
                values[12] / 10_000_000, values[13] / 100, values[14] / 100,
                values[15], values[16], *values[17:],
            ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", nargs="?", type=Path)
    args = parser.parse_args()
    output = args.output or args.input.with_suffix(".csv")
    with output.open("w", newline="", encoding="utf-8") as destination:
        writer = csv.writer(destination)
        writer.writerow(BASE_HEADER + [name + "_us" for name in INPUTS + OUTPUTS])
        writer.writerows(rows(args.input))
    print(output)


if __name__ == "__main__":
    main()
