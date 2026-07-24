#!/usr/bin/env python3
"""Host-side checks for JY61P fixed-point, time, and yaw-zero behavior."""

UINT32_MASK = 0xFFFFFFFF
MAX_VALID_AGE = 0x7FFFFFFF
AGE_UNKNOWN = None


def trunc_div(numerator: int, denominator: int) -> int:
    magnitude = abs(numerator) // denominator
    return -magnitude if numerator < 0 else magnitude


def raw_to_angle_x100(raw: int) -> int:
    return trunc_div(raw * 18000, 32768)


def raw_to_gyro_x10(raw: int) -> int:
    return trunc_div(raw * 20000, 32768)


def normalize_yaw_x100(value: int) -> int:
    while value > 18000:
        value -= 36000
    while value < -18000:
        value += 36000
    return value


def calculate_age(now: int, timestamp: int, has_frame: bool):
    if not has_frame:
        return AGE_UNKNOWN
    elapsed = (now - timestamp) & UINT32_MASK
    return elapsed if elapsed <= MAX_VALID_AGE else AGE_UNKNOWN


def format_age(age) -> str:
    return "na" if age is AGE_UNKNOWN else str(age)


class DriverState:
    def __init__(self):
        self.yaw = 0
        self.relative_yaw = 0
        self.yaw_zero_valid = False
        self.yaw_zero_offset = 0
        self.online = False
        self.last_valid = 0
        self.last_angle = 0
        self.angle_frames = 0
        self.gyro_frames = 0
        self.unsupported_frames = 0
        self.checksum_errors = 0
        self.sync_errors = 0
        self.rx_overflow = 0
        self.timebase_faults = 0
        self.yaw_state_faults = 0

    def accept_angle(self, yaw: int, now: int):
        self.yaw = yaw
        self.relative_yaw = normalize_yaw_x100(
            yaw - self.yaw_zero_offset if self.yaw_zero_valid else yaw
        )
        self.last_valid = now
        self.last_angle = now
        self.online = True
        self.angle_frames += 1

    def reset_yaw(self):
        assert self.online
        self.yaw_zero_offset = self.yaw
        self.yaw_zero_valid = True
        self.relative_yaw = 0

    def clear_statistics(self):
        self.angle_frames = 0
        self.gyro_frames = 0
        self.unsupported_frames = 0
        self.checksum_errors = 0
        self.sync_errors = 0
        self.rx_overflow = 0
        self.timebase_faults = 0
        self.yaw_state_faults = 0


def check(actual, expected, label: str):
    if actual != expected:
        raise AssertionError(f"{label}: expected {expected!r}, got {actual!r}")


def main():
    for raw, expected in (
        (0, 0),
        (16384, 9000),
        (-16384, -9000),
        (32767, 17999),
        (-32768, -18000),
    ):
        check(raw_to_angle_x100(raw), expected, f"angle raw={raw}")

    for raw, expected in (
        (0, 0),
        (16384, 10000),
        (-16384, -10000),
        (32767, 19999),
        (-32768, -20000),
    ):
        check(raw_to_gyro_x10(raw), expected, f"gyro raw={raw}")

    check(normalize_yaw_x100(-9661 - (-9683)), 22, "negative yaw zero")
    check(normalize_yaw_x100(-17000 - 17000), 2000, "positive wrap")
    check(normalize_yaw_x100(17000 - (-17000)), -2000, "negative wrap")

    check(calculate_age(20, 0xFFFFFFF0, True), 36, "uint32 time wrap")
    unknown_age = calculate_age(20, 0, False)
    check(unknown_age, AGE_UNKNOWN, "never received frame")
    check(format_age(unknown_age), "na", "unknown age formatting")
    check(calculate_age(1000, 1598, True), AGE_UNKNOWN, "future timestamp guard")

    state = DriverState()
    state.accept_angle(-9683, 100)
    state.reset_yaw()
    state.accept_angle(-9661, 120)
    check(state.relative_yaw, 22, "post-zero relative yaw")
    state.angle_frames = 10
    state.gyro_frames = 11
    state.unsupported_frames = 12
    state.checksum_errors = 13
    state.sync_errors = 14
    state.rx_overflow = 15
    state.timebase_faults = 16
    state.yaw_state_faults = 17

    before_clear = (
        state.yaw,
        state.yaw_zero_valid,
        state.yaw_zero_offset,
        state.last_valid,
        state.last_angle,
        state.online,
        state.relative_yaw,
    )
    state.clear_statistics()
    check(
        (
            state.angle_frames,
            state.gyro_frames,
            state.unsupported_frames,
            state.checksum_errors,
            state.sync_errors,
            state.rx_overflow,
            state.timebase_faults,
            state.yaw_state_faults,
        ),
        (0, 0, 0, 0, 0, 0, 0, 0),
        "clear statistics counters",
    )
    check(
        (
            state.yaw,
            state.yaw_zero_valid,
            state.yaw_zero_offset,
            state.last_valid,
            state.last_angle,
            state.online,
            state.relative_yaw,
        ),
        before_clear,
        "clear statistics preserves live state",
    )

    state.online = False
    check(state.yaw_zero_offset, -9683, "offline preserves yaw zero")
    state.accept_angle(-9650, 2000)
    check(state.yaw_zero_offset, -9683, "recovery preserves yaw zero")
    check(state.relative_yaw, 33, "recovery relative yaw")

    print("JY61P host tests: PASS")


if __name__ == "__main__":
    main()
