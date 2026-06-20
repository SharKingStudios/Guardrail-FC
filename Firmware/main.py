"""Guardrail FC boot entry point (MicroPython on RP2040)."""

import time
import micropython
from machine import I2C, Pin, UART

import config
from guardrail.channels import Receiver, ServoOutputs
from guardrail.control import AttitudeAssist, AutoThrottle, OutputMixer
from guardrail.gnss import GNSS
from guardrail.logger import FlightLogger
from guardrail.sensors import ICM42688, SPA06
from guardrail.validation import validate


FLAG_IMU = 1 << 0
FLAG_BAROMETER = 1 << 1
FLAG_GNSS = 1 << 2
FLAG_RECEIVER = 1 << 3
FLAG_FAILSAFE = 1 << 4
FLAG_AUTO_ACTIVE = 1 << 5
FLAG_AUTO_OVERRIDE = 1 << 6


def due(now, deadline):
    return time.ticks_diff(now, deadline) >= 0


def selected_mode(inputs, fresh):
    if config.MODE_CHANNEL is None:
        return config.DEFAULT_MODE
    if not fresh.get(config.MODE_CHANNEL, False):
        return "PASSTHROUGH"
    pulse = inputs[config.MODE_CHANNEL]
    if pulse >= config.MODE_AUTO_THRESHOLD_US:
        return "AUTO"
    if pulse >= config.MODE_ASSIST_THRESHOLD_US:
        return "ASSIST"
    return "PASSTHROUGH"


def automatic_altitude(barometer, gnss):
    if barometer and barometer.sample_revision:
        return barometer.altitude_m, "BAROMETER", barometer.sample_revision
    if (
        config.AUTO_THROTTLE["allow_gnss_altitude_fallback"]
        and gnss
        and gnss.valid
        and gnss.altitude_m is not None
    ):
        return gnss.altitude_m, "GNSS", gnss.altitude_revision
    return None, None, None


def automatic_health(imu, gnss, altitude_m, throttle_sources, fresh):
    if not config.AUTO_THROTTLE["enabled"]:
        return "disabled in config"
    if any(not fresh.get(source, False) for source in throttle_sources):
        return "throttle receiver input stale"
    if altitude_m is None:
        return "no valid altitude"
    if config.AUTO_THROTTLE["require_imu"] and imu is None:
        return "IMU unavailable"
    if config.AUTO_THROTTLE["groundspeed_target_mps"] is not None and not (gnss and gnss.valid):
        return "GNSS groundspeed unavailable"
    return None


def initialize_sensor(label, constructor):
    try:
        sensor = constructor()
        print(label, "ready")
        return sensor
    except Exception as error:
        print(label, "unavailable:", error)
        return None


def main():
    micropython.alloc_emergency_exception_buf(100)
    validate(config)
    led = Pin(config.STATUS_LED_PIN, Pin.OUT, value=0)
    outputs = ServoOutputs(config.OUTPUT_PINS, config.OUTPUTS, config.PWM_FRAME_US)
    receiver = Receiver(
        config.INPUT_PINS,
        config.PWM_MIN_VALID_US,
        config.PWM_MAX_VALID_US,
        config.INPUT_TIMEOUT_MS,
    )
    mixer = OutputMixer(config.OUTPUTS, config.DEFAULT_INPUT_CALIBRATION, config.INPUT_CALIBRATION)
    assist = AttitudeAssist(config.CONTROL_AXES, config.PID, config.DEFAULT_INPUT_CALIBRATION, config.INPUT_CALIBRATION)
    automatic_throttle = AutoThrottle(config.AUTO_THROTTLE)
    automatic_outputs = tuple(config.AUTO_THROTTLE["outputs"])
    automatic_sources = tuple({config.OUTPUTS[output]["source"] for output in automatic_outputs})

    i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400000)
    print("I2C devices:", [hex(address) for address in i2c.scan()])
    imu = initialize_sensor(
        "ICM-42688-P",
        lambda: ICM42688(i2c, config.IMU_AXIS_MAP, config.COMPLEMENTARY_ALPHA, config.GYRO_BIAS_DPS),
    )
    barometer = initialize_sensor("SPA06-003", lambda: SPA06(i2c, config.SEA_LEVEL_PRESSURE_PA))
    gnss = initialize_sensor(
        "SAM-M8Q",
        lambda: GNSS(
            UART(0, baudrate=config.GNSS_BAUDRATE, tx=Pin(0), rx=Pin(1), timeout=0),
            config.GNSS_FIX_TIMEOUT_MS,
        ),
    )

    if imu and config.AUTO_GYRO_CALIBRATION:
        print("Keep aircraft still: calibrating gyro")
        try:
            imu.calibrate_gyro(config.GYRO_CALIBRATION_SAMPLES)
            print("Gyro bias:", imu.bias)
        except Exception as error:
            print("Gyro calibration failed:", error)
            imu = None

    logger = FlightLogger(
        config.LOG_DIRECTORY,
        config.MAX_LOG_BYTES,
        config.FLUSH_INTERVAL_MS,
        config.INPUT_ORDER,
        config.OUTPUT_ORDER,
    )

    attitude = (0.0, 0.0)
    gyro = (0.0, 0.0, 0.0)
    imu_temperature = 0.0
    corrections = {"roll": 0.0, "pitch": 0.0}
    input_values = {name: 0 for name in config.INPUT_ORDER}
    fresh = {name: False for name in config.INPUT_ORDER}
    failed_outputs = list(config.OUTPUT_ORDER)
    mode = "PASSTHROUGH"
    previous_mode = "PASSTHROUGH"
    last_automatic_reason = None

    now = time.ticks_ms()
    next_imu = now
    next_baro = now
    next_control = now
    next_log = now
    last_control = time.ticks_us()
    blink_deadline = now
    blink_state = False

    imu_period = max(1, 1000 // config.IMU_HZ)
    baro_period = max(1, 1000 // config.BAROMETER_HZ)
    control_period = max(1, 1000 // config.CONTROL_HZ)
    log_period = max(1, 1000 // config.LOG_HZ)

    print("Guardrail FC running; default mode:", config.DEFAULT_MODE)
    while True:
        now = time.ticks_ms()
        if gnss:
            gnss.update()

        if imu and due(now, next_imu):
            next_imu = time.ticks_add(next_imu, imu_period)
            try:
                attitude, gyro = imu.update()
                imu_temperature = imu.temperature_c
            except Exception as error:
                print("IMU stopped:", error)
                imu = None
                assist.reset()

        if barometer and due(now, next_baro):
            next_baro = time.ticks_add(next_baro, baro_period)
            try:
                barometer.update()
            except Exception as error:
                print("Barometer stopped:", error)
                barometer = None

        if due(now, next_control):
            next_control = time.ticks_add(next_control, control_period)
            input_values, fresh = receiver.snapshot()
            previous_mode = mode
            mode = selected_mode(input_values, fresh)
            control_now = time.ticks_us()
            dt = time.ticks_diff(control_now, last_control) / 1000000.0
            last_control = control_now
            if mode in ("ASSIST", "AUTO") and imu:
                corrections = assist.update(input_values, fresh, attitude, gyro, dt)
            else:
                assist.reset()
                corrections = {"roll": 0.0, "pitch": 0.0}
            throttle_overrides = {}
            forced_failsafe = ()
            if mode != "AUTO":
                if previous_mode == "AUTO":
                    automatic_throttle.disengage()
            else:
                altitude_m, altitude_source, altitude_revision = automatic_altitude(barometer, gnss)
                health_error = automatic_health(imu, gnss, altitude_m, automatic_sources, fresh)
                pilot_values = [input_values[source] for source in automatic_sources]
                pilot_average = sum(pilot_values) / len(pilot_values)
                pilot_highest = max(pilot_values)

                if previous_mode != "AUTO":
                    if health_error is None:
                        automatic_throttle.engage(
                            altitude_m,
                            altitude_source,
                            altitude_revision,
                            pilot_average,
                        )
                    else:
                        automatic_throttle.abort("entry blocked: " + health_error)

                if automatic_throttle.active:
                    if health_error is not None:
                        automatic_throttle.abort(health_error)
                    else:
                        ground_speed = gnss.speed_mps if gnss and gnss.valid else None
                        automatic_throttle.update(
                            altitude_m,
                            altitude_source,
                            altitude_revision,
                            attitude,
                            ground_speed,
                            pilot_highest,
                            dt,
                        )

                if automatic_throttle.active:
                    throttle_overrides = automatic_throttle.output_overrides(config.OUTPUTS)
                elif not automatic_throttle.override_latched:
                    throttle_overrides = automatic_throttle.failsafe_overrides(config.OUTPUTS)
                    forced_failsafe = automatic_outputs

                if automatic_throttle.reason != last_automatic_reason:
                    print("AUTO throttle:", automatic_throttle.reason)
                    last_automatic_reason = automatic_throttle.reason

            output_values, failed_outputs = mixer.mix(
                input_values,
                fresh,
                corrections,
                throttle_overrides,
                forced_failsafe,
            )
            outputs.write_all(output_values)

        if due(now, next_log):
            next_log = time.ticks_add(next_log, log_period)
            flags = 0
            if imu:
                flags |= FLAG_IMU
            if barometer:
                flags |= FLAG_BAROMETER
            if gnss and gnss.valid:
                flags |= FLAG_GNSS
            if any(fresh.values()):
                flags |= FLAG_RECEIVER
            if failed_outputs:
                flags |= FLAG_FAILSAFE
            if automatic_throttle.active:
                flags |= FLAG_AUTO_ACTIVE
            if automatic_throttle.override_latched:
                flags |= FLAG_AUTO_OVERRIDE
            logger.write(mode, flags, attitude, gyro, imu_temperature, barometer, gnss, input_values, outputs.last)

        # Fast blink means at least one output is in failsafe; slow blink means healthy.
        blink_period = 125 if failed_outputs else 500
        if due(now, blink_deadline):
            blink_state = not blink_state
            led.value(blink_state)
            blink_deadline = time.ticks_add(now, blink_period)

        time.sleep_ms(1)


main()
