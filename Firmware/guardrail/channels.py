"""Interrupt-driven RC PWM capture and hardware PWM output."""

import time
from machine import Pin, PWM


class Receiver:
    def __init__(self, pin_map, min_valid_us, max_valid_us, timeout_ms):
        self.names = tuple(pin_map)
        self._pins = []
        self._rises = [0] * len(self.names)
        self._widths = [0] * len(self.names)
        self._updates = [0] * len(self.names)
        self._seen = [False] * len(self.names)
        self._snapshot_values = {name: 0 for name in self.names}
        self._snapshot_fresh = {name: False for name in self.names}
        self._min = min_valid_us
        self._max = max_valid_us
        self._timeout = timeout_ms
        self._handlers = []  # Keep IRQ closures alive.

        for index, name in enumerate(self.names):
            pin = Pin(pin_map[name], Pin.IN, Pin.PULL_DOWN)

            def edge_handler(changed_pin, channel=index):
                now_us = time.ticks_us()
                if changed_pin.value():
                    self._rises[channel] = now_us
                    return
                width = time.ticks_diff(now_us, self._rises[channel])
                if self._min <= width <= self._max:
                    self._widths[channel] = width
                    self._updates[channel] = time.ticks_ms()
                    self._seen[channel] = True

            self._handlers.append(edge_handler)
            try:
                pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=edge_handler, hard=True)
            except TypeError:  # Compatibility with older RP2 MicroPython builds.
                pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=edge_handler)
            self._pins.append(pin)

    def snapshot(self):
        now = time.ticks_ms()
        for index, name in enumerate(self.names):
            self._snapshot_values[name] = self._widths[index]
            self._snapshot_fresh[name] = self._seen[index] and time.ticks_diff(now, self._updates[index]) <= self._timeout
        return self._snapshot_values, self._snapshot_fresh


class ServoOutputs:
    def __init__(self, pin_map, definitions, frame_us=20000):
        self.names = tuple(pin_map)
        self._frame_us = frame_us
        self._channels = {}
        self.last = {}
        self._duty_ns = True

        for name in self.names:
            channel = PWM(Pin(pin_map[name], Pin.OUT))
            channel.freq(round(1000000 / frame_us))
            self._channels[name] = channel
            self.write(name, definitions[name]["failsafe_us"])

    def write(self, name, pulse_us):
        pulse_us = int(pulse_us)
        channel = self._channels[name]
        if self._duty_ns:
            try:
                channel.duty_ns(pulse_us * 1000)
            except AttributeError:
                self._duty_ns = False
        if not self._duty_ns:
            channel.duty_u16((pulse_us * 65535) // self._frame_us)
        self.last[name] = pulse_us

    def write_all(self, values):
        for name in self.names:
            self.write(name, values[name])

    def failsafe_all(self, definitions):
        for name in self.names:
            self.write(name, definitions[name]["failsafe_us"])
