import math
import random

from navigation import Navigator
from robot import Robot
from sensors import GPSSensor, Gyroscope, Magnetometer
from localization import Localization


# =========================================================
# LOCKED CONTROLLER
# =========================================================

CROSS_TRACK_GAIN = 0.70
LOOKAHEAD_DISTANCE = 4.50
TURN_SPEED = 0.50
TURN_ZONE_DISTANCE = 5.00

CRUISE_SPEED = 2.50
MINIMUM_SPEED = 1.20
APPROACH_DISTANCE = 8.00

DT = 0.10

TRIALS = 25

GPS_NOISE_LEVELS = [
    0.25,
    0.30,
    0.35,
    0.40,
    0.45,
    0.50,
]


WAYPOINTS = [
    (10.0, 10.0),
    (30.0, 10.0),
    (30.0, 30.0),
    (10.0, 30.0),
    (10.0, 10.0),
]

WAYPOINT_TOLERANCE = 0.50


# =========================================================
# GEOMETRY
# =========================================================

def distance(x1, y1, x2, y2):
    return math.hypot(
        x2 - x1,
        y2 - y1,
    )


def point_to_segment_distance(
    px,
    py,
    x1,
    y1,
    x2,
    y2,
):
    dx = x2 - x1
    dy = y2 - y1

    length_squared = (
        dx * dx
        + dy * dy
    )

    if length_squared <= 1e-12:
        return distance(
            px,
            py,
            x1,
            y1,
        )

    t = (
        (px - x1) * dx
        + (py - y1) * dy
    ) / length_squared

    t = max(
        0.0,
        min(1.0, t),
    )

    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    return distance(
        px,
        py,
        closest_x,
        closest_y,
    )


def active_segment_error(
    x,
    y,
    segment,
):
    x1, y1 = WAYPOINTS[segment]
    x2, y2 = WAYPOINTS[segment + 1]

    return point_to_segment_distance(
        x,
        y,
        x1,
        y1,
        x2,
        y2,
    )


# =========================================================
# RUN ONE TRIAL
# =========================================================

def run_trial(
    gps_noise,
    trial_number,
):
    robot = Robot()

    gps = GPSSensor()
    gyro = Gyroscope()
    magnetometer = Magnetometer()

    localization = Localization()

    navigator = Navigator(
        WAYPOINTS
    )

    # LOCKED CONTROLLER

    navigator.cross_track_gain = (
        CROSS_TRACK_GAIN
    )

    navigator.lookahead_distance = (
        LOOKAHEAD_DISTANCE
    )

    navigator.turn_speed = (
        TURN_SPEED
    )

    navigator.turn_zone_distance = (
        TURN_ZONE_DISTANCE
    )

    navigator.cruise_speed = (
        CRUISE_SPEED
    )

    navigator.minimum_speed = (
        MINIMUM_SPEED
    )

    navigator.approach_distance = (
        APPROACH_DISTANCE
    )

    straight_errors = []
    turn_errors = []

    waypoint_errors = []

    waypoint_index = 1

    max_error = 0.0
    max_error_step = 0

    max_distance_from_path = 0.0

    total_distance = 0.0

    previous_x = robot.x
    previous_y = robot.y

    max_steps = 10000

    failure_reason = None

    # -----------------------------------------------------
    # Deterministic trial seed
    # -----------------------------------------------------

    random.seed(
        20000
        + trial_number
    )

    # -----------------------------------------------------
    # SIMULATION
    # -----------------------------------------------------

    for step in range(max_steps):

        # TRUE POSITION
        true_x = robot.x
        true_y = robot.y

        true_heading = robot.heading

        # GPS MEASUREMENT WITH NOISE
        measured_x = (
            true_x
            +
            random.gauss(
                0.0,
                gps_noise,
            )
        )

        measured_y = (
            true_y
            +
            random.gauss(
                0.0,
                gps_noise,
            )
        )

        # HEADING SENSOR
        gyro_reading = gyro.read(
            0.0
        )

        magnetometer_heading = (
            magnetometer.read(
                true_heading
            )
        )

        # LOCALIZATION
        localization.update(
            measured_x,
            measured_y,
            gyro_reading,
            magnetometer_heading,
            DT,
        )

        # NAVIGATION
        (
            desired_heading,
            desired_speed,
            complete,
        ) = navigator.update(
            localization.x,
            localization.y,
        )

        # ROBOT
        robot.update(
            desired_heading,
            desired_speed,
            DT,
        )

        movement = distance(
            previous_x,
            previous_y,
            robot.x,
            robot.y,
        )

        total_distance += movement

        previous_x = robot.x
        previous_y = robot.y

        # -------------------------------------------------
        # WAYPOINT DETECTION
        # -------------------------------------------------

        if (
            waypoint_index
            < len(WAYPOINTS)
        ):
            target_x, target_y = (
                WAYPOINTS[
                    waypoint_index
                ]
            )

            waypoint_distance = distance(
                robot.x,
                robot.y,
                target_x,
                target_y,
            )

            if (
                waypoint_distance
                <= WAYPOINT_TOLERANCE
            ):
                waypoint_errors.append(
                    waypoint_distance
                )

                waypoint_index += 1

        # -------------------------------------------------
        # ACTIVE SEGMENT
        # -------------------------------------------------

        active_segment = (
            waypoint_index - 1
        )

        if (
            0 <= active_segment
            < len(WAYPOINTS) - 1
        ):

            error = active_segment_error(
                robot.x,
                robot.y,
                active_segment,
            )

            if error > max_error:
                max_error = error
                max_error_step = step

            max_distance_from_path = max(
                max_distance_from_path,
                error,
            )

            x1, y1 = WAYPOINTS[
                active_segment
            ]

            x2, y2 = WAYPOINTS[
                active_segment + 1
            ]

            distance_from_start = distance(
                robot.x,
                robot.y,
                x1,
                y1,
            )

            distance_from_end = distance(
                robot.x,
                robot.y,
                x2,
                y2,
            )

            if (
                distance_from_start
                <= TURN_ZONE_DISTANCE
                or
                distance_from_end
                <= TURN_ZONE_DISTANCE
            ):
                turn_errors.append(
                    error
                )
            else:
                straight_errors.append(
                    error
                )

        # -------------------------------------------------
        # CATASTROPHIC DIVERGENCE CHECK
        #
        # This is only diagnostic. It does not stop the
        # normal test at reasonable tracking errors.
        # -------------------------------------------------

        if error > 100.0:
            failure_reason = (
                "PATH DIVERGENCE"
            )
            break

        if abs(robot.x) > 1000.0:
            failure_reason = (
                "X POSITION DIVERGENCE"
            )
            break

        if abs(robot.y) > 1000.0:
            failure_reason = (
                "Y POSITION DIVERGENCE"
            )
            break

        # -------------------------------------------------
        # MISSION COMPLETE
        # -------------------------------------------------

        if (
            waypoint_index
            >= len(WAYPOINTS)
        ):
            break

    completed = (
        waypoint_index
        >= len(WAYPOINTS)
    )

    if not completed and failure_reason is None:
        failure_reason = (
            "MISSION TIMEOUT"
        )

    # -----------------------------------------------------
    # Statistics
    # -----------------------------------------------------

    if straight_errors:
        straight_rms = math.sqrt(
            sum(
                e * e
                for e in straight_errors
            )
            /
            len(straight_errors)
        )
    else:
        straight_rms = float("inf")

    if turn_errors:
        turn_rms = math.sqrt(
            sum(
                e * e
                for e in turn_errors
            )
            /
            len(turn_errors)
        )
    else:
        turn_rms = float("inf")

    return {
        "trial": trial_number,
        "completed": completed,
        "failure_reason": failure_reason,

        "straight_rms": straight_rms,

        "straight_max": (
            max(straight_errors)
            if straight_errors
            else float("inf")
        ),

        "turn_rms": turn_rms,

        "turn_max": (
            max(turn_errors)
            if turn_errors
            else float("inf")
        ),

        "waypoint_count": (
            waypoint_index
        ),

        "waypoint_max": (
            max(waypoint_errors)
            if waypoint_errors
            else float("inf")
        ),

        "max_error": max_error,

        "max_error_step": max_error_step,

        "total_distance": total_distance,

        "final_x": robot.x,
        "final_y": robot.y,

        "final_heading": robot.heading,
    }


# =========================================================
# TEST ONE NOISE LEVEL
# =========================================================

def test_noise_level(
    gps_noise,
):
    results = []

    for trial in range(TRIALS):

        result = run_trial(
            gps_noise,
            trial,
        )

        results.append(
            result
        )

    return results


# =========================================================
# PRINT SUMMARY
# =========================================================

def print_summary(
    gps_noise,
    results,
):

    completed = sum(
        1
        for r in results
        if r["completed"]
    )

    valid_straight = [
        r["straight_rms"]
        for r in results
        if math.isfinite(
            r["straight_rms"]
        )
    ]

    valid_turn = [
        r["turn_rms"]
        for r in results
        if math.isfinite(
            r["turn_rms"]
        )
    ]

    max_straight = max(
        (
            r["straight_max"]
            for r in results
        ),
        default=float("inf"),
    )

    max_turn = max(
        (
            r["turn_max"]
            for r in results
        ),
        default=float("inf"),
    )

    max_path_error = max(
        r["max_error"]
        for r in results
    )

    print()
    print("=" * 80)
    print(
        f"GPS NOISE = ±{gps_noise:.2f} m"
    )
    print("=" * 80)

    print(
        f"Completed trials : "
        f"{completed}/{TRIALS}"
    )

    if valid_straight:
        print(
            f"Average straight RMS : "
            f"{sum(valid_straight) / len(valid_straight):.3f} m"
        )
    else:
        print(
            "Average straight RMS : inf"
        )

    print(
        f"Straight MAX      : "
        f"{max_straight:.3f} m"
    )

    if valid_turn:
        print(
            f"Average turn RMS     : "
            f"{sum(valid_turn) / len(valid_turn):.3f} m"
        )
    else:
        print(
            "Average turn RMS     : inf"
        )

    print(
        f"Turn MAX          : "
        f"{max_turn:.3f} m"
    )

    print(
        f"Worst path error  : "
        f"{max_path_error:.3f} m"
    )

    # -----------------------------------------------------
    # PRINT FAILED TRIALS
    # -----------------------------------------------------

    failed = [
        r
        for r in results
        if not r["completed"]
    ]

    if failed:

        print()
        print("FAILED TRIALS")
        print("-" * 80)

        for r in failed:

            print(
                f"Trial {r['trial']:02d} | "
                f"Reason: {r['failure_reason']} | "
                f"Waypoints: {r['waypoint_count']}/"
                f"{len(WAYPOINTS)} | "
                f"Max error: "
                f"{r['max_error']:.3f} m | "
                f"Step: "
                f"{r['max_error_step']} | "
                f"Final: "
                f"({r['final_x']:.2f}, "
                f"{r['final_y']:.2f})"
            )

    else:

        print()
        print(
            "No failed trials."
        )


# =========================================================
# MAIN
# =========================================================

def main():

    print()
    print("=" * 80)
    print("GPS NOISE BOUNDARY TEST")
    print("=" * 80)

    print()
    print("LOCKED CONTROLLER")
    print(
        f"Gain       : {CROSS_TRACK_GAIN:.2f}"
    )
    print(
        f"Lookahead  : {LOOKAHEAD_DISTANCE:.2f} m"
    )
    print(
        f"Turn speed : {TURN_SPEED:.2f} m/s"
    )
    print(
        f"Turn zone  : {TURN_ZONE_DISTANCE:.2f} m"
    )

    print()
    print(
        "No navigation parameters will be changed."
    )

    for gps_noise in GPS_NOISE_LEVELS:

        results = test_noise_level(
            gps_noise
        )

        print_summary(
            gps_noise,
            results,
        )

    print()
    print("=" * 80)
    print("GPS BOUNDARY TEST COMPLETE")
    print("=" * 80)


if __name__ == "__main__":
    main()