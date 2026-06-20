"""Compact binary flight logger for the RP2040's limited onboard flash."""

import os
import struct
import time


MAGIC = b"GRFC"
VERSION = 1
RECORD_FORMAT = "<IBB6h5iHBB10H10H"
RECORD_SIZE = struct.calcsize(RECORD_FORMAT)


def _clamp_int(value, low, high):
    value = int(round(value))
    return low if value < low else high if value > high else value


class FlightLogger:
    def __init__(self, directory, max_bytes, flush_interval_ms, input_order, output_order):
        self.directory = directory
        self.max_bytes = max_bytes
        self.flush_interval_ms = flush_interval_ms
        self.input_order = input_order
        self.output_order = output_order
        self.file = None
        self.path = None
        self.size = 0
        self.last_flush = time.ticks_ms()
        self.enabled = False
        self._open_next()

    def _open_next(self):
        try:
            try:
                os.mkdir(self.directory)
            except OSError:
                pass
            existing = set(os.listdir(self.directory))
            index = 0
            while "flight_{:03d}.grf".format(index) in existing:
                index += 1
            self.path = "{}/flight_{:03d}.grf".format(self.directory, index)
            self.file = open(self.path, "wb")
            header = MAGIC + bytes((VERSION, RECORD_SIZE))
            self.file.write(header)
            self.size = len(header)
            self.enabled = True
            print("Logging to", self.path)
        except OSError as error:
            self.enabled = False
            print("Logging disabled:", error)

    def _rotate(self):
        if self.file:
            self.file.flush()
            self.file.close()
        self.file = None
        self.enabled = False
        self._open_next()

    def write(self, mode, flags, attitude, gyro, imu_temp_c, barometer, gnss, inputs, outputs):
        if not self.enabled:
            return
        if self.size + RECORD_SIZE > self.max_bytes:
            self._rotate()
            if not self.enabled:
                return

        latitude = gnss.latitude if gnss and gnss.latitude is not None else 0.0
        longitude = gnss.longitude if gnss and gnss.longitude is not None else 0.0
        gps_altitude = gnss.altitude_m if gnss and gnss.altitude_m is not None else 0.0
        speed = gnss.speed_mps if gnss else 0.0
        satellites = gnss.satellites if gnss else 0
        fix_quality = gnss.fix_quality if gnss else 0
        pressure = barometer.pressure_pa if barometer else 0.0
        baro_altitude = barometer.altitude_m if barometer else 0.0

        values = [
            time.ticks_ms() & 0xFFFFFFFF,
            2 if mode == "AUTO" else 1 if mode == "ASSIST" else 0,
            flags,
            _clamp_int(attitude[0] * 100, -32768, 32767),
            _clamp_int(attitude[1] * 100, -32768, 32767),
            _clamp_int(gyro[0] * 100, -32768, 32767),
            _clamp_int(gyro[1] * 100, -32768, 32767),
            _clamp_int(gyro[2] * 100, -32768, 32767),
            _clamp_int(imu_temp_c * 100, -32768, 32767),
            _clamp_int(pressure, -2147483648, 2147483647),
            _clamp_int(baro_altitude * 100, -2147483648, 2147483647),
            _clamp_int(latitude * 10000000, -2147483648, 2147483647),
            _clamp_int(longitude * 10000000, -2147483648, 2147483647),
            _clamp_int(gps_altitude * 100, -2147483648, 2147483647),
            _clamp_int(speed * 100, 0, 65535),
            _clamp_int(satellites, 0, 255),
            _clamp_int(fix_quality, 0, 255),
        ]
        values.extend(_clamp_int(inputs.get(name, 0), 0, 65535) for name in self.input_order)
        values.extend(_clamp_int(outputs.get(name, 0), 0, 65535) for name in self.output_order)

        try:
            data = struct.pack(RECORD_FORMAT, *values)
            self.file.write(data)
            self.size += len(data)
            now = time.ticks_ms()
            if time.ticks_diff(now, self.last_flush) >= self.flush_interval_ms:
                self.file.flush()
                try:
                    os.sync()
                except AttributeError:
                    pass
                self.last_flush = now
        except OSError as error:
            self.enabled = False
            print("Logging stopped:", error)
