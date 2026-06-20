"""Plane-independent surface mixing, attitude assist, and automatic throttle."""

import math


def clamp(value, low, high):
    return low if value < low else high if value > high else value


def normalized_input(pulse_us, calibration):
    center = calibration["center_us"]
    if pulse_us >= center:
        span = max(1, calibration["max_us"] - center)
    else:
        span = max(1, center - calibration["min_us"])
    return clamp((pulse_us - center) / span, -1.0, 1.0)


def pulse_from_normalized(value, definition):
    value = clamp(value, -1.0, 1.0)
    center = definition["center_us"]
    if value >= 0:
        pulse = center + value * (definition["max_us"] - center)
    else:
        pulse = center + value * (center - definition["min_us"])
    return int(clamp(pulse + definition.get("trim_us", 0), definition["min_us"], definition["max_us"]))


def calibration_for(name, default_calibration, calibrations):
    result = dict(default_calibration)
    result.update(calibrations.get(name, {}))
    return result


class PID:
    def __init__(self, settings):
        self.kp = settings["kp"]
        self.ki = settings["ki"]
        self.kd = settings["kd"]
        self.limit = settings["limit"]
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, error_deg, measured_rate_dps, dt):
        if dt <= 0 or dt > 0.25:
            self.reset()
            return 0.0
        self.integral = clamp(self.integral + error_deg * dt, -self.limit / max(self.ki, 0.000001), self.limit / max(self.ki, 0.000001))
        output = self.kp * error_deg + self.ki * self.integral - self.kd * measured_rate_dps
        return clamp(output, -self.limit, self.limit)


class AttitudeAssist:
    def __init__(self, axis_definitions, pid_definitions, default_calibration, calibrations):
        self.axes = axis_definitions
        self.pids = {axis: PID(pid_definitions[axis]) for axis in axis_definitions}
        self.default_calibration = default_calibration
        self.calibrations = calibrations
        self._axis_calibrations = {
            axis: calibration_for(definition["source"], default_calibration, calibrations)
            for axis, definition in axis_definitions.items()
        }

    def reset(self):
        for controller in self.pids.values():
            controller.reset()

    def update(self, inputs, fresh, attitude, gyro_dps, dt):
        correction = {"roll": 0.0, "pitch": 0.0}
        measurements = {"roll": attitude[0], "pitch": attitude[1]}
        rates = {"roll": gyro_dps[0], "pitch": gyro_dps[1]}
        for axis, definition in self.axes.items():
            source = definition["source"]
            if not fresh.get(source, False):
                self.pids[axis].reset()
                continue
            calibration = self._axis_calibrations[axis]
            command = normalized_input(inputs[source], calibration)
            desired_deg = command * definition["max_angle_deg"]
            correction[axis] = self.pids[axis].update(desired_deg - measurements[axis], rates[axis], dt)
        return correction


class AutoThrottle:
    """Guarded altitude-hold throttle controller.

    Pressure altitude is the primary feedback. IMU attitude adds turn/climb
    feed-forward and GNSS ground speed can add an optional, deliberately bounded term.
    """

    def __init__(self, settings):
        self.settings = settings
        self.active = False
        self.override_latched = False
        self.reason = "not selected"
        self.target_altitude_m = None
        self.command_us = None
        self.altitude_source = None
        self.integral_us = 0.0
        self.vertical_speed_mps = 0.0
        self._last_altitude_m = None
        self._last_altitude_revision = None
        self._sample_elapsed_s = 0.0

    def disengage(self, reason="not selected"):
        self.active = False
        self.override_latched = False
        self.reason = reason
        self.target_altitude_m = None
        self.command_us = None
        self.altitude_source = None
        self.integral_us = 0.0
        self.vertical_speed_mps = 0.0
        self._last_altitude_m = None
        self._last_altitude_revision = None
        self._sample_elapsed_s = 0.0

    def abort(self, reason):
        self.active = False
        self.command_us = None
        self.reason = reason

    def engage(self, altitude_m, altitude_source, altitude_revision, pilot_throttle_us):
        self.disengage()
        if not self.settings["enabled"]:
            self.reason = "disabled in config"
            return False
        self.target_altitude_m = self.settings["target_altitude_m"]
        if self.target_altitude_m is None:
            self.target_altitude_m = altitude_m
        self.altitude_source = altitude_source
        # Start at the live pilot pulse; normal AUTO limits are approached through the
        # configured slew rate so engagement does not create a throttle step.
        self.command_us = pilot_throttle_us
        self._last_altitude_m = altitude_m
        self._last_altitude_revision = altitude_revision
        self.active = True
        self.reason = "active"
        return True

    def update(
        self,
        altitude_m,
        altitude_source,
        altitude_revision,
        attitude,
        ground_speed_mps,
        pilot_throttle_us,
        dt,
    ):
        if not self.active:
            return None
        if pilot_throttle_us >= self.settings["pilot_override_us"]:
            self.active = False
            self.override_latched = True
            self.command_us = None
            self.reason = "pilot override latched"
            return None
        if altitude_source != self.altitude_source:
            self.abort("altitude source changed")
            return None
        if dt <= 0 or dt > 0.25:
            self.abort("control timing fault")
            return None

        self._sample_elapsed_s += dt
        if altitude_revision != self._last_altitude_revision and self._sample_elapsed_s > 0.005:
            measured_speed = (altitude_m - self._last_altitude_m) / self._sample_elapsed_s
            alpha = self.settings["vertical_speed_filter_alpha"]
            self.vertical_speed_mps = alpha * self.vertical_speed_mps + (1.0 - alpha) * measured_speed
            self._last_altitude_m = altitude_m
            self._last_altitude_revision = altitude_revision
            self._sample_elapsed_s = 0.0

        altitude_error = self.target_altitude_m - altitude_m
        self.integral_us += self.settings["altitude_ki_us_per_m_s"] * altitude_error * dt
        limit = self.settings["integral_limit_us"]
        self.integral_us = clamp(self.integral_us, -limit, limit)

        throttle = self.settings["cruise_us"]
        throttle += self.settings["altitude_kp_us_per_m"] * altitude_error
        throttle += self.integral_us
        throttle -= self.settings["vertical_speed_damping_us_per_mps"] * self.vertical_speed_mps

        roll_deg, pitch_deg = attitude
        limited_roll_rad = min(abs(roll_deg), 80.0) * 0.01745329252
        throttle += self.settings["bank_compensation_us"] * (1.0 - math.cos(limited_roll_rad))
        throttle += self.settings["pitch_compensation_us_per_deg"] * max(pitch_deg, 0.0)

        speed_target = self.settings["groundspeed_target_mps"]
        if speed_target is not None and ground_speed_mps is not None:
            throttle += self.settings["groundspeed_kp_us_per_mps"] * (speed_target - ground_speed_mps)

        throttle = clamp(throttle, self.settings["min_us"], self.settings["max_us"])
        max_step = self.settings["slew_rate_us_per_s"] * dt
        throttle = clamp(throttle, self.command_us - max_step, self.command_us + max_step)
        self.command_us = int(round(throttle))
        return self.command_us

    def output_overrides(self, definitions):
        if not self.active or self.command_us is None:
            return {}
        result = {}
        for output, trim_us in self.settings["outputs"].items():
            definition = definitions[output]
            result[output] = int(clamp(
                self.command_us + trim_us,
                definition["min_us"],
                min(self.settings["max_us"], definition["max_us"]),
            ))
        return result

    def failsafe_overrides(self, definitions):
        return {output: definitions[output]["failsafe_us"] for output in self.settings["outputs"]}


class OutputMixer:
    def __init__(self, definitions, default_calibration, calibrations):
        self.definitions = definitions
        self.default_calibration = default_calibration
        self.calibrations = calibrations
        self._sources_by_output = {name: self._sources(definition) for name, definition in definitions.items()}
        source_names = set()
        for sources in self._sources_by_output.values():
            source_names.update(sources)
        self._source_calibrations = {
            name: calibration_for(name, default_calibration, calibrations) for name in source_names
        }

    @staticmethod
    def _sources(definition):
        if "sources" in definition:
            return definition["sources"]
        return {definition["source"]: 1.0}

    def mix(self, inputs, fresh, assist, overrides=None, forced_failsafe=None):
        overrides = overrides or {}
        forced_failsafe = forced_failsafe or ()
        outputs = {}
        failed = []
        for output_name, definition in self.definitions.items():
            sources = self._sources_by_output[output_name]
            if any(not fresh.get(source, False) for source in sources):
                outputs[output_name] = definition["failsafe_us"]
                failed.append(output_name)
                continue

            if output_name in overrides:
                outputs[output_name] = int(overrides[output_name])
                if output_name in forced_failsafe:
                    failed.append(output_name)
                continue

            if definition.get("passthrough", False):
                source = definition["source"]
                outputs[output_name] = int(inputs[source])
                continue

            value = 0.0
            for source, weight in sources.items():
                calibration = self._source_calibrations[source]
                value += normalized_input(inputs[source], calibration) * weight
            for axis, weight in definition.get("assist", {}).items():
                value += assist.get(axis, 0.0) * weight
            if definition.get("reverse", False):
                value = -value
            outputs[output_name] = pulse_from_normalized(value, definition)
        return outputs, failed
