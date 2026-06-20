"""Plane-specific Guardrail FC configuration.

This is the only file that normally needs to change between airframes.  Connector
names describe the PCB as viewed from the component/silkscreen side: LEFT is the
odd-numbered signal pad (pad 1), RIGHT is the even-numbered signal pad (pad 2).
"""

# Receiver PWM limits and scheduling -------------------------------------------------
PWM_MIN_VALID_US = 750
PWM_MAX_VALID_US = 2250
PWM_FRAME_US = 20000
INPUT_TIMEOUT_MS = 120
CONTROL_HZ = 100
IMU_HZ = 200
BAROMETER_HZ = 25
LOG_HZ = 10

# Physical PCB pin map. Do not change this section for a different plane.
INPUT_PINS = {
    "J1_LEFT": 7,
    "J1_RIGHT": 8,
    "J2_LEFT": 9,
    "J2_RIGHT": 10,
    "J3_LEFT": 11,
    "J3_RIGHT": 12,
    "J4_LEFT": 13,
    "J4_RIGHT": 14,
    "J5_LEFT": 15,
    "J5_RIGHT": 16,
}

OUTPUT_PINS = {
    "J6_LEFT": 19,
    "J6_RIGHT": 20,
    "J7_LEFT": 21,
    "J7_RIGHT": 22,
    "J8_LEFT": 23,
    "J8_RIGHT": 24,
    "J9_LEFT": 25,
    "J9_RIGHT": 26,
    "J10_LEFT": 27,
    "J10_RIGHT": 28,
}

INPUT_ORDER = tuple(INPUT_PINS)
OUTPUT_ORDER = tuple(OUTPUT_PINS)

# Per-receiver calibration. Add only values that differ from these defaults.
DEFAULT_INPUT_CALIBRATION = {"min_us": 1000, "center_us": 1500, "max_us": 2000}
INPUT_CALIBRATION = {
    # "J1_LEFT": {"min_us": 988, "center_us": 1501, "max_us": 2012},
}

# Output routing ---------------------------------------------------------------------
#
# Every output may use either:
#   "source": "J1_LEFT"                         (one receiver channel), or
#   "sources": {"J1_LEFT": 1.0, "J2_LEFT": 1.0} (mixing/elevons/V-tail).
#
# "assist" mixes stabilization corrections into that output. Leave it empty for a
# pure pilot-command channel. "passthrough" copies the raw pulse exactly and ignores
# reversal, trim, mixing, and assist; v1 requires this on all J4/J5 throttle routes.
def _direct(source, assist=None, reverse=False, failsafe_us=1500):
    return {
        "source": source,
        "assist": assist or {},
        "reverse": reverse,
        "trim_us": 0,
        "min_us": 1000,
        "center_us": 1500,
        "max_us": 2000,
        "failsafe_us": failsafe_us,
        "passthrough": False,
    }


def _throttle(source):
    return {
        "source": source,
        "assist": {},
        "reverse": False,
        "trim_us": 0,
        "min_us": 1000,
        "center_us": 1500,
        "max_us": 2000,
        "failsafe_us": 1000,
        "passthrough": True,
    }


# Safe starter profile: each connector side controls the same side of its paired
# output connector. J4/J5 are raw pass-through except while guarded AUTO throttle is active.
OUTPUTS = {
    "J6_LEFT": _direct("J1_LEFT"),
    "J6_RIGHT": _direct("J1_RIGHT"),
    "J7_LEFT": _direct("J2_LEFT"),
    "J7_RIGHT": _direct("J2_RIGHT"),
    "J8_LEFT": _direct("J3_LEFT"),
    "J8_RIGHT": _direct("J3_RIGHT"),
    "J9_LEFT": _throttle("J4_LEFT"),
    "J9_RIGHT": _throttle("J4_RIGHT"),
    "J10_LEFT": _throttle("J5_LEFT"),
    "J10_RIGHT": _throttle("J5_RIGHT"),
}

# Flight assist ----------------------------------------------------------------------
# Start in PASSTHROUGH until directions and gains have been proven on the bench.
# To configure a conventional plane, assign the receiver channels below, add for
# example {"roll": 1.0} to each aileron output's "assist", reverse the sign on the
# opposite aileron, and then set DEFAULT_MODE to "ASSIST".
DEFAULT_MODE = "PASSTHROUGH"  # "PASSTHROUGH", "ASSIST", or "AUTO"

# Optional three-position mode channel. None means DEFAULT_MODE is fixed. With a mode
# channel: low=PASSTHROUGH, middle=ASSIST, high=AUTO.
MODE_CHANNEL = None
MODE_ASSIST_THRESHOLD_US = 1300
MODE_AUTO_THRESHOLD_US = 1700

CONTROL_AXES = {
    "roll": {"source": "J1_LEFT", "max_angle_deg": 45.0},
    "pitch": {"source": "J2_LEFT", "max_angle_deg": 30.0},
}

# PID output is a normalized surface correction (-1.0 to +1.0).
PID = {
    "roll": {"kp": 0.018, "ki": 0.002, "kd": 0.004, "limit": 0.45},
    "pitch": {"kp": 0.022, "ki": 0.003, "kd": 0.005, "limit": 0.45},
}

# Automatic throttle ----------------------------------------------------------------
# AUTO holds a pressure altitude using throttle, adds feed-forward for bank/pitch, and
# can optionally use GNSS ground speed. None captures the current altitude on entry;
# a numeric target is an absolute barometric altitude based on SEA_LEVEL_PRESSURE_PA.
#
# AUTO only arms when every configured throttle receiver channel is fresh and the
# required sensors are healthy. Moving any pilot throttle above pilot_override_us
# immediately restores raw pass-through and latches it until AUTO is toggled off/on.
AUTO_THROTTLE = {
    "enabled": True,
    "outputs": {
        "J9_LEFT": 0,    # Per-motor trim in microseconds.
        "J9_RIGHT": 0,
        "J10_LEFT": 0,
        "J10_RIGHT": 0,
    },
    "target_altitude_m": None,
    "cruise_us": 1500,
    "min_us": 1100,
    "max_us": 1900,
    "pilot_override_us": 1850,
    "slew_rate_us_per_s": 250.0,
    "altitude_kp_us_per_m": 18.0,
    "altitude_ki_us_per_m_s": 1.5,
    "integral_limit_us": 180.0,
    "vertical_speed_damping_us_per_mps": 35.0,
    "vertical_speed_filter_alpha": 0.80,
    "bank_compensation_us": 140.0,
    "pitch_compensation_us_per_deg": 2.0,
    "require_imu": True,
    "allow_gnss_altitude_fallback": False,
    # Ground speed is not airspeed. Leave disabled unless conservative values have
    # been flight-tested for this exact aircraft and wind conditions.
    "groundspeed_target_mps": None,
    "groundspeed_kp_us_per_mps": 12.0,
}

# IMU mounting transform. Each tuple is (source axis index, sign), where source axes
# are 0=X, 1=Y, 2=Z. The defaults mean the sensor axes match aircraft forward/right/down.
IMU_AXIS_MAP = ((0, 1), (1, 1), (2, 1))
COMPLEMENTARY_ALPHA = 0.98
AUTO_GYRO_CALIBRATION = True
GYRO_CALIBRATION_SAMPLES = 200
GYRO_BIAS_DPS = (0.0, 0.0, 0.0)

# Telemetry and logging --------------------------------------------------------------
SEA_LEVEL_PRESSURE_PA = 101325.0
GNSS_BAUDRATE = 9600
GNSS_FIX_TIMEOUT_MS = 2000
LOG_DIRECTORY = "/logs"
MAX_LOG_BYTES = 512 * 1024
FLUSH_INTERVAL_MS = 1000
STATUS_LED_PIN = 17
