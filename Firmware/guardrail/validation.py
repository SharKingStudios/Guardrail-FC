"""Configuration checks that run before any outputs are enabled."""


def validate(config):
    errors = []
    input_names = set(config.INPUT_PINS)
    output_names = set(config.OUTPUT_PINS)

    if set(config.OUTPUTS) != output_names:
        errors.append("OUTPUTS must define every physical output exactly once")
    if len(set(config.INPUT_PINS.values())) != len(config.INPUT_PINS):
        errors.append("INPUT_PINS contains duplicate GPIOs")
    if len(set(config.OUTPUT_PINS.values())) != len(config.OUTPUT_PINS):
        errors.append("OUTPUT_PINS contains duplicate GPIOs")

    for name, definition in config.OUTPUTS.items():
        sources = definition.get("sources", {definition.get("source"): 1.0})
        for source in sources:
            if source not in input_names:
                errors.append("{} uses unknown input {}".format(name, source))
        if definition.get("passthrough") and ("source" not in definition or "sources" in definition):
            errors.append("{} pass-through must have exactly one source".format(name))
        if definition["min_us"] >= definition["max_us"]:
            errors.append("{} has invalid output limits".format(name))
        if not definition["min_us"] <= definition["failsafe_us"] <= definition["max_us"]:
            errors.append("{} failsafe is outside output limits".format(name))

    # Hardware contract: these routes pass through except for an explicit AUTO override.
    throttle_inputs = {"J4_LEFT", "J4_RIGHT", "J5_LEFT", "J5_RIGHT"}
    for output in ("J9_LEFT", "J9_RIGHT", "J10_LEFT", "J10_RIGHT"):
        definition = config.OUTPUTS.get(output, {})
        if definition.get("source") not in throttle_inputs or not definition.get("passthrough", False):
            errors.append("v1 requires {} to pass through one J4/J5 throttle input".format(output))

    if config.DEFAULT_MODE not in ("PASSTHROUGH", "ASSIST", "AUTO"):
        errors.append("DEFAULT_MODE must be PASSTHROUGH, ASSIST, or AUTO")
    if config.MODE_CHANNEL is not None and config.MODE_CHANNEL not in input_names:
        errors.append("MODE_CHANNEL names an unknown input")
    if not config.MODE_ASSIST_THRESHOLD_US < config.MODE_AUTO_THRESHOLD_US:
        errors.append("mode thresholds must increase from ASSIST to AUTO")
    if not config.PWM_MIN_VALID_US <= config.MODE_ASSIST_THRESHOLD_US < config.MODE_AUTO_THRESHOLD_US <= config.PWM_MAX_VALID_US:
        errors.append("mode thresholds must stay inside the valid receiver pulse range")
    for axis, definition in config.CONTROL_AXES.items():
        if axis not in ("roll", "pitch"):
            errors.append("unsupported control axis {}".format(axis))
        if definition["source"] not in input_names:
            errors.append("{} axis uses an unknown input".format(axis))

    auto = config.AUTO_THROTTLE
    auto_outputs = set(auto["outputs"])
    throttle_outputs = {"J9_LEFT", "J9_RIGHT", "J10_LEFT", "J10_RIGHT"}
    if not auto_outputs or not auto_outputs.issubset(throttle_outputs):
        errors.append("AUTO_THROTTLE outputs must select one or more J9/J10 outputs")
    for output in auto_outputs:
        if not config.OUTPUTS[output].get("passthrough", False):
            errors.append("AUTO output {} must remain a raw pass-through outside AUTO".format(output))
    if not auto["min_us"] < auto["cruise_us"] < auto["max_us"]:
        errors.append("AUTO throttle must satisfy min < cruise < max")
    if not auto["min_us"] <= auto["pilot_override_us"] <= 2250:
        errors.append("AUTO pilot override must be at or above minimum throttle")
    if auto["slew_rate_us_per_s"] <= 0:
        errors.append("AUTO throttle slew rate must be positive")
    if not 0.0 <= auto["vertical_speed_filter_alpha"] < 1.0:
        errors.append("AUTO vertical-speed filter alpha must be in [0, 1)")
    if auto["groundspeed_target_mps"] is not None and auto["groundspeed_target_mps"] <= 0:
        errors.append("AUTO GNSS groundspeed target must be positive or None")
    if auto["allow_gnss_altitude_fallback"] and auto["target_altitude_m"] is not None:
        errors.append("GNSS altitude fallback requires a captured (None) altitude target")
    if config.DEFAULT_MODE == "AUTO" and not auto["enabled"]:
        errors.append("DEFAULT_MODE cannot be AUTO while automatic throttle is disabled")

    if errors:
        raise ValueError("Invalid config:\n - " + "\n - ".join(errors))
