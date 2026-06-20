"""Drivers for the sensors wired to the Guardrail FC PCB."""

import math
import struct
import time


def _signed(value, bits):
    sign = 1 << (bits - 1)
    return value - (1 << bits) if value & sign else value


def _axis_transform(values, mapping):
    return tuple(values[source] * sign for source, sign in mapping)


class ICM42688:
    ADDRESS = 0x68  # U5 SDO/SA0 is tied low.
    WHO_AM_I = 0x75
    EXPECTED_ID = 0x47

    def __init__(self, i2c, axis_map, alpha=0.98, bias=(0.0, 0.0, 0.0)):
        self.i2c = i2c
        self.axis_map = axis_map
        self.alpha = alpha
        self.bias = list(bias)
        self.roll = 0.0
        self.pitch = 0.0
        self.accel_g = (0.0, 0.0, 1.0)
        self.gyro_dps = (0.0, 0.0, 0.0)
        self.temperature_c = 0.0
        self.last_us = None
        self.ready = False
        self._configure()

    def _read(self, register, count=1):
        return self.i2c.readfrom_mem(self.ADDRESS, register, count)

    def _write(self, register, value):
        self.i2c.writeto_mem(self.ADDRESS, register, bytes((value,)))

    def _configure(self):
        if self._read(self.WHO_AM_I, 1)[0] != self.EXPECTED_ID:
            raise OSError("ICM-42688-P WHO_AM_I mismatch")
        self._write(0x11, 0x01)  # DEVICE_CONFIG: soft reset
        time.sleep_ms(10)
        self._write(0x4E, 0x0F)  # PWR_MGMT0: accel and gyro low-noise modes
        time.sleep_ms(50)
        self._write(0x4F, 0x47)  # +/-500 dps, 200 Hz
        self._write(0x50, 0x47)  # +/-4 g, 200 Hz
        self.ready = True

    def read_raw(self):
        # TEMP, ACCEL XYZ, GYRO XYZ are contiguous and big-endian.
        raw = struct.unpack(">7h", self._read(0x1D, 14))
        accel = _axis_transform(tuple(value / 8192.0 for value in raw[1:4]), self.axis_map)
        gyro = _axis_transform(tuple(value / 65.5 for value in raw[4:7]), self.axis_map)
        temperature = raw[0] / 132.48 + 25.0
        return accel, gyro, temperature

    def calibrate_gyro(self, samples=200):
        totals = [0.0, 0.0, 0.0]
        for _ in range(samples):
            _, gyro, _ = self.read_raw()
            for axis in range(3):
                totals[axis] += gyro[axis]
            time.sleep_ms(5)
        self.bias = [total / samples for total in totals]
        self.last_us = None

    def update(self):
        accel, gyro, temperature = self.read_raw()
        gyro = tuple(gyro[index] - self.bias[index] for index in range(3))
        now = time.ticks_us()
        if self.last_us is None:
            dt = 0.0
        else:
            dt = time.ticks_diff(now, self.last_us) / 1000000.0
        self.last_us = now

        ax, ay, az = accel
        accel_roll = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        if dt <= 0 or dt > 0.1:
            self.roll, self.pitch = accel_roll, accel_pitch
        else:
            self.roll = self.alpha * (self.roll + gyro[0] * dt) + (1.0 - self.alpha) * accel_roll
            self.pitch = self.alpha * (self.pitch + gyro[1] * dt) + (1.0 - self.alpha) * accel_pitch

        self.accel_g = accel
        self.gyro_dps = gyro
        self.temperature_c = temperature
        return (self.roll, self.pitch), gyro


class SPA06:
    ADDRESS = 0x76  # U4 SDO is tied low.

    def __init__(self, i2c, sea_level_pa=101325.0):
        self.i2c = i2c
        self.sea_level_pa = sea_level_pa
        self.temperature_c = 0.0
        self.pressure_pa = 0.0
        self.altitude_m = 0.0
        self.sample_revision = 0
        self.ready = False
        self._read_coefficients()
        self._configure()

    def _read(self, register, count=1):
        return self.i2c.readfrom_mem(self.ADDRESS, register, count)

    def _write(self, register, value):
        self.i2c.writeto_mem(self.ADDRESS, register, bytes((value,)))

    def _read_coefficients(self):
        data = self._read(0x10, 18)
        self.c0 = _signed((data[0] << 4) | (data[1] >> 4), 12)
        self.c1 = _signed(((data[1] & 0x0F) << 8) | data[2], 12)
        self.c00 = _signed((data[3] << 12) | (data[4] << 4) | (data[5] >> 4), 20)
        self.c10 = _signed(((data[5] & 0x0F) << 16) | (data[6] << 8) | data[7], 20)
        self.c01 = _signed((data[8] << 8) | data[9], 16)
        self.c11 = _signed((data[10] << 8) | data[11], 16)
        self.c20 = _signed((data[12] << 8) | data[13], 16)
        self.c21 = _signed((data[14] << 8) | data[15], 16)
        self.c30 = _signed((data[16] << 8) | data[17], 16)

    def _configure(self):
        # 32 samples/s, 16x oversampling. Both result scale factors are 253952.
        temp_source = self._read(0x28, 1)[0] & 0x80
        self._write(0x06, 0x54)
        self._write(0x07, temp_source | 0x54)
        self._write(0x09, 0x0C)  # Required result shifts when oversampling > 8x.
        self._write(0x08, 0x07)  # Continuous pressure and temperature.
        time.sleep_ms(50)
        self.ready = True

    @staticmethod
    def _raw24(data):
        return _signed((data[0] << 16) | (data[1] << 8) | data[2], 24)

    def update(self):
        raw = self._read(0x00, 6)
        pressure_scaled = self._raw24(raw[0:3]) / 253952.0
        temperature_scaled = self._raw24(raw[3:6]) / 253952.0
        self.temperature_c = self.c0 * 0.5 + self.c1 * temperature_scaled
        self.pressure_pa = (
            self.c00
            + pressure_scaled * (self.c10 + pressure_scaled * (self.c20 + pressure_scaled * self.c30))
            + temperature_scaled * self.c01
            + temperature_scaled * pressure_scaled * (self.c11 + pressure_scaled * self.c21)
        )
        if self.pressure_pa > 1000:
            self.altitude_m = 44330.0 * (1.0 - (self.pressure_pa / self.sea_level_pa) ** 0.190294957)
        self.sample_revision += 1
        return self.pressure_pa, self.altitude_m
