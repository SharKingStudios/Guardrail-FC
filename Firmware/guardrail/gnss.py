"""Small non-blocking NMEA parser for the SAM-M8Q UART."""

import time


class GNSS:
    def __init__(self, uart, fix_timeout_ms=2000):
        self.uart = uart
        self.fix_timeout_ms = fix_timeout_ms
        self.buffer = b""
        self.latitude = None
        self.longitude = None
        self.altitude_m = None
        self.speed_mps = 0.0
        self.satellites = 0
        self.fix_quality = 0
        self.valid = False
        self.altitude_revision = 0
        self._last_fix_ms = None

    @staticmethod
    def _coordinate(raw, hemisphere):
        if not raw:
            return None
        value = float(raw)
        degrees = int(value // 100)
        result = degrees + (value - degrees * 100) / 60.0
        return -result if hemisphere in ("S", "W") else result

    @staticmethod
    def _checksum_ok(sentence):
        if not sentence.startswith("$") or "*" not in sentence:
            return False
        body, expected = sentence[1:].split("*", 1)
        checksum = 0
        for character in body:
            checksum ^= ord(character)
        try:
            return checksum == int(expected[:2], 16)
        except ValueError:
            return False

    def _parse(self, sentence):
        if not self._checksum_ok(sentence):
            return
        fields = sentence.split("*")[0].split(",")
        kind = fields[0][-3:]
        try:
            if kind == "GGA" and len(fields) >= 10:
                self.fix_quality = int(fields[6] or 0)
                self.satellites = int(fields[7] or 0)
                self.latitude = self._coordinate(fields[2], fields[3])
                self.longitude = self._coordinate(fields[4], fields[5])
                self.altitude_m = float(fields[9]) if fields[9] else None
                self.altitude_revision += 1
                self.valid = self.fix_quality > 0 and self.latitude is not None
                if self.valid:
                    self._last_fix_ms = time.ticks_ms()
            elif kind == "RMC" and len(fields) >= 8:
                if fields[2] == "A":
                    self.latitude = self._coordinate(fields[3], fields[4])
                    self.longitude = self._coordinate(fields[5], fields[6])
                    self.speed_mps = float(fields[7] or 0.0) * 0.514444
                    self.valid = self.latitude is not None
                    if self.valid:
                        self._last_fix_ms = time.ticks_ms()
                else:
                    self.valid = False
        except (ValueError, IndexError):
            return

    def update(self):
        waiting = self.uart.any()
        if waiting:
            chunk = self.uart.read(min(waiting, 256))
            if chunk:
                self.buffer += chunk
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            try:
                self._parse(line.strip().decode("ascii"))
            except UnicodeError:
                pass
        if len(self.buffer) > 512:
            self.buffer = self.buffer[-256:]
        if (
            self.valid
            and self._last_fix_ms is not None
            and time.ticks_diff(time.ticks_ms(), self._last_fix_ms) > self.fix_timeout_ms
        ):
            self.valid = False
