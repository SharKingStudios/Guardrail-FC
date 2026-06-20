# Guardrail FC firmware

The v1 firmware is a configurable, fail-safe signal router, flight-assist controller, and flight logger for the Guardrail FC PCB. It uses MicroPython on the RP2040 so all 10 receiver PWM channels can be captured concurrently with hardware interrupts. The RP2040 CircuitPython port cannot provide enough simultaneous pulse-capture channels for this board's 10 inputs.

The safe factory profile is intentionally boring: J1-J3 pass straight through to J6-J8, J4/J5 throttle passes straight through to J9/J10, and both flight assist and AUTO throttle stay unselected until the airframe mapping and correction directions have been bench-tested.


## Files

- `main.py` initializes hardware and runs the real-time loop.
- `config.py` contains every normal airframe-specific choice and tuning value.
- `guardrail/channels.py` captures receiver pulses and drives servo PWM.
- `guardrail/control.py` performs routing, surface mixing, roll/pitch PID control, and guarded automatic throttle.
- `guardrail/sensors.py` reads the ICM-42688-P and SPA06-003 directly over I2C.
- `guardrail/gnss.py` parses GGA/RMC messages from the SAM-M8Q.
- `guardrail/logger.py` writes compact `.grf` flight logs to onboard flash.
- `tools/decode_log.py` converts a downloaded `.grf` file to CSV.

No third-party modules are needed on the controller.

## Install

1. Install the current RP2040 build of MicroPython while holding BOOT and connecting
   USB. The generic Raspberry Pi Pico build matches the RP2040 and 2 MiB W25Q16 flash.
2. Install `mpremote` on the computer: `python -m pip install mpremote`.
3. Copy `main.py`, `config.py`, and the entire `guardrail/` directory to the board.
4. Reset the board and watch the serial console. It prints the I2C scan, sensor status,
   gyro calibration result, log filename, and active mode.

Keep the aircraft motionless for roughly one second after reset. Outputs stay at their configured failsafe values during gyro calibration. Set `AUTO_GYRO_CALIBRATION = False` and enter measured `GYRO_BIAS_DPS` values if that startup pause is undesirable.

## Configure an airframe

Only edit `config.py`. The names stay tied to physical connector sides, so an output can be reassigned without remembering GPIO numbers.

For a one-to-one route, use:

```python
"J6_LEFT": _direct("J2_RIGHT")
```

For a reversed servo with roll stabilization mixed in:

```python
"J6_RIGHT": _direct("J1_RIGHT", assist={"roll": -1.0}, reverse=True)
```

For an elevon or V-tail, replace `source` with weighted `sources` in that output:

```python
"J6_LEFT": {
    "sources": {"J1_LEFT": 1.0, "J2_LEFT": 1.0},
    "assist": {"roll": 1.0, "pitch": 1.0},
    "reverse": False,
    "trim_us": 0,
    "min_us": 1000,
    "center_us": 1500,
    "max_us": 2000,
    "failsafe_us": 1500,
    "passthrough": False,
}
```

This same weighted mapping handles dual ailerons, flaperons, V-tails, canards, and multiple motor sections. A receiver channel may feed any number of outputs, and an output may combine any number of non-throttle inputs. A J4/J5 throttle may feed any number of J9/J10 motor outputs. Those routes remain exact pass-through in PASSTHROUGH and ASSIST modes; only AUTO may override the configured motor outputs.

Set `CONTROL_AXES` to the receiver channels that command roll and pitch. Add `roll`
or `pitch` assist weights only to surfaces that should correct that axis. Verify every
sign with the propellers removed:

- Roll the right wing down; the commanded surfaces must drive it back up.
- Raise the nose; the elevator correction must command nose-down.
- Move every transmitter stick and confirm the expected output and direction.
- Turn the transmitter off and confirm control surfaces center and all throttles go low.

Only after those checks should `DEFAULT_MODE` be changed to `"ASSIST"`. Prefer assigning
a spare three-position receiver channel to `MODE_CHANNEL`: low selects PASSTHROUGH,
middle selects ASSIST, and high selects AUTO. Begin with low `kp`, leave `ki` near zero, increase `kp` until the
aircraft responds firmly, add `kd` to reduce oscillation, and add only enough `ki` to
remove steady bias. Change one axis and one gain at a time.

`IMU_AXIS_MAP` handles rotated or inverted board installations without changing sensor
code. Confirm the printed/logged roll and pitch signs before enabling assist.

## Automatic throttle

AUTO uses every useful onboard measurement without pretending the board has an
airspeed sensor:

- SPA06 pressure altitude is the primary feedback. AUTO captures the current altitude
  when selected unless `target_altitude_m` is configured explicitly.
- Vertical speed derived from pressure altitude damps throttle changes and reduces
  overshoot.
- ICM-42688 roll adds throttle in a bank, while positive pitch adds configurable climb
  feed-forward.
- SAM-M8Q ground speed can add a bounded correction when `groundspeed_target_mps` is
  configured. It is disabled by default because ground speed is not stall-protecting
  airspeed in wind.

All airframe values are in `AUTO_THROTTLE` in `config.py`. Tune `cruise_us`, `min_us`,
and `max_us` first with AUTO disabled. Then set a conservative `slew_rate_us_per_s`,
start with altitude `ki` at zero, and tune altitude `kp` at a safe height. Add vertical
speed damping, then only enough integral to remove a steady altitude error. Bank and
pitch compensation are feed-forward terms and should be tuned last.

The `outputs` map selects which J9/J10 motor sections AUTO controls and supplies a
per-motor trim. Outputs not listed remain pilot pass-through. AUTO will arm only on the
transition into AUTO when:

- every receiver channel feeding an automatic motor output is fresh;
- a valid barometric altitude is available (or explicitly enabled GNSS fallback);
- the IMU is healthy when `require_imu` is true; and
- GNSS is valid when GNSS ground-speed control is configured.

If any required input disappears after engagement, all automatic motor outputs go to
their configured low-throttle failsafe and AUTO will not re-arm until the mode switch is
toggled out and back. Moving any pilot throttle above `pilot_override_us` immediately
restores raw pilot throttle and latches that override until AUTO is toggled off/on.
Commands are bounded and slew-limited during normal automatic operation; receiver loss
still takes priority over every override.

Before fitting propellers, select AUTO and verify: throttle enters without a jump,
raising the board decreases the altitude-error command, banking adds only the expected
small compensation, pilot override restores every raw throttle channel, sensor failure
commands low throttle, and re-arming requires a deliberate mode-switch cycle.

## Logs

Logs contain mode (including AUTO), health flags, AUTO-active/pilot-override flags,
attitude, gyro and temperature, pressure altitude, GNSS position/speed/fix, all 10 raw
receiver pulses, and all 10 commanded output pulses. They rotate at 512 KiB; with the
default 10 Hz rate, each file holds roughly ten minutes. Logging stops safely if flash
fills; the control loop continues.

Copy a log from `/logs` with `mpremote`, then decode it on the computer:

```text
python tools/decode_log.py flight_000.grf
```

This produces `flight_000.csv`. Remove old `.grf` files from the controller between test sessions; the W25Q16 is also the firmware/filesystem flash, not a separate recorder.

## Safety status

This is unflown experimental firmware. The LED blinks quickly while any configured
output is in failsafe and slowly when all routed inputs are fresh. Always bench-test with propellers removed, verify PWM limits for every servo/ESC, and test receiver and sensor loss before flight. AUTO throttle is altitude hold, not stall protection or a complete navigation autopilot. A software pass-through cannot protect against total MCU or power failure; a production design that requires that behavior needs an external hardware bypass.
