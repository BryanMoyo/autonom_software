import math
import random

from navigation import Navigator
from robot import Robot
from sensors import GPSSensor, Gyroscope, Magnetometer
from localization import Localization


# =========================================================
# LOCKED VALIDATED CONTROLLER
# =========================================================

CROSS_TRACK_GAIN = 0.70
LOOKAHEAD_DISTANCE = 4.50
TURN_SPEED = 0.50
TURN_ZONE_DISTANCE = 5.00

CRUISE_SPEED = 2.50
MINIMUM_SPEED = 1.20
APPROACH_DISTANCE = 8.00

DT = 0.10

TRIALS_PER_CASE = 25


# =========================================================
# MISSION
# =========================================================

WAYPOINTS = [
    (10.0, 10.0),
    (30.0, 10.0),
    (30.0, 30.0),
    (10.0, 30.0),
    (10.0, 10.0),
]


# =========================================================
# TARGETS
# =========================================================

WAYPOINT_TOLERANCE = 0.50


# =========================================================
# SENSOR NOISE LEVELS
# =========================================================

GPS_NOISE_LEVELS = [
    0.00,
    0.05,
    0.10,
    0.25,
    0.50,
    1.00,
]

HEADING_NOISE_LEVELS = [
    0.0,
    0.5,
    1.0,
    2.0,
    5.0,
    10.0,
]


# =========================================================
# GEOMETRY
# =========================================================

def distance(
    x1,
    y1,
    x2,
    y2,
):
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
        +
        dy * dy
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
        +
        (py - y1) * dy
    ) / length_squared

    t = max(
        0.0,
        min(
            1.0,
            t,
        ),
    )

    closest_x = (
        x1
        +
        t * dx
    )

    closest_y = (
        y1
        +
        t * dy
    )

    return distance(
        px,
        py,
        closest_x,
        closest_y,
    )


def active_segment_error(
    x,
    y,
    segment_index,
):
    if segment_index < 0:
        return 0.0

    if (
        segment_index
        >= len(WAYPOINTS) - 1
    ):
        segment_index = (
            len(WAYPOINTS) - 2
        )

    x1, y1 = WAYPOINTS[
        segment_index
    ]

    x2, y2 = WAYPOINTS[
        segment_index + 1
    ]

    return point_to_segment_distance(
        x,
        y,
        x1,
        y1,
        x2,
        y2,
    )


# =========================================================
# STATISTICS
# =========================================================

def rms(values):

    if not values:
        return float("inf")

    return math.sqrt(
        sum(
            value * value
            for value in values
        )
        / len(values)
    )


def average(values):

    if not values:
        return float("inf")

    return (
        sum(values)
        /
        len(values)
    )


# =========================================================
# SYSTEM
# =========================================================

def create_system():

    robot = Robot()

    gps = GPSSensor()
    gyro = Gyroscope()
    magnetometer = Magnetometer()

    localization = Localization()

    navigator = Navigator(
        WAYPOINTS
    )

    # -----------------------------------------------------
    # LOCKED CONTROLLER
    # -----------------------------------------------------

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

    return (
        robot,
        gps,
        gyro,
        magnetometer,
        localization,
        navigator,
    )


# =========================================================
# RUN ONE TRIAL
# =========================================================

def run_trial(
    gps_noise=0.0,
    heading_noise=0.0,
):

    (
        robot,
        gps,
        gyro,
        magnetometer,
        localization,
        navigator,
    ) = create_system()


    # =====================================================
    # METRICS
    # =====================================================

    straight_errors = []
    turn_errors = []

    waypoint_errors = []

    reached_waypoints = [
        WAYPOINTS[0]
    ]

    waypoint_index = 1

    max_steps = 10000


    # =====================================================
    # SIMULATION
    # =====================================================

    for step in range(max_steps):

        # -------------------------------------------------
        # TRUE SENSOR VALUES
        # -------------------------------------------------

        true_gps_x = robot.x
        true_gps_y = robot.y

        true_heading = robot.heading


        # -------------------------------------------------
        # ADD EXTERNAL GPS NOISE
        #
        # Noise is zero-mean Gaussian.
        # -------------------------------------------------

        measured_gps_x = (
            true_gps_x
            +
            random.gauss(
                0.0,
                gps_noise,
            )
        )

        measured_gps_y = (
            true_gps_y
            +
            random.gauss(
                0.0,
                gps_noise,
            )
        )


        # -------------------------------------------------
        # GYROSCOPE
        #
        # Keep the existing gyro model unchanged.
        # -------------------------------------------------

        gyro_reading = gyro.read(
            0.0
        )


        # -------------------------------------------------
        # MAGNETOMETER
        #
        # Add heading measurement noise externally.
        # -------------------------------------------------

        measured_heading = (
            true_heading
            +
            random.gauss(
                0.0,
                heading_noise,
            )
        )

        measured_heading %= 360.0

        magnetometer_heading = (
            magnetometer.read(
                measured_heading
            )
        )


        # -------------------------------------------------
        # LOCALIZATION
        # -------------------------------------------------

        localization.update(
            measured_gps_x,
            measured_gps_y,
            gyro_reading,
            magnetometer_heading,
            DT,
        )


        # -------------------------------------------------
        # NAVIGATION
        # -------------------------------------------------

        (
            desired_heading,
            desired_speed,
            complete,
        ) = navigator.update(
            localization.x,
            localization.y,
        )


        # -------------------------------------------------
        # ROBOT
        # -------------------------------------------------

        robot.update(
            desired_heading,
            desired_speed,
            DT,
        )


        # =================================================
        # WAYPOINT ARRIVAL
        # =================================================

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

                reached_waypoints.append(
                    WAYPOINTS[
                        waypoint_index
                    ]
                )

                waypoint_index += 1


        # =================================================
        # ACTIVE SEGMENT
        # =================================================

        active_segment = (
            waypoint_index - 1
        )

        if (
            0 <= active_segment
            < len(WAYPOINTS) - 1
        ):

            error = (
                active_segment_error(
                    robot.x,
                    robot.y,
                    active_segment,
                )
            )


            # -------------------------------------------------
            # DISTANCE FROM END OF CURRENT ROW
            # -------------------------------------------------

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


            # -------------------------------------------------
            # TURN CLASSIFICATION
            # -------------------------------------------------

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


        # =================================================
        # COMPLETE
        # =================================================

        if (
            waypoint_index
            >= len(WAYPOINTS)
        ):
            break


    # =====================================================
    # RESULT
    # =====================================================

    completed = (
        waypoint_index
        >= len(WAYPOINTS)
    )

    return {

        "straight_rms": rms(
            straight_errors
        ),

        "straight_max": (
            max(straight_errors)
            if straight_errors
            else float("inf")
        ),

        "turn_rms": rms(
            turn_errors
        ),

        "turn_max": (
            max(turn_errors)
            if turn_errors
            else float("inf")
        ),

        "waypoint_average": average(
            waypoint_errors
        ),

        "waypoint_max": (
            max(waypoint_errors)
            if waypoint_errors
            else float("inf")
        ),

        "waypoint_count": len(
            reached_waypoints
        ),

        "completed": completed,
    }


# =========================================================
# MULTIPLE TRIALS
# =========================================================

def run_trials(
    gps_noise=0.0,
    heading_noise=0.0,
):

    results = []

    for trial in range(
        TRIALS_PER_CASE
    ):

        random.seed(
            10000
            +
            trial
        )

        result = run_trial(
            gps_noise=gps_noise,
            heading_noise=heading_noise,
        )

        results.append(
            result
        )

    return results


# =========================================================
# AGGREGATE
# =========================================================

def aggregate(results):

    return {

        "straight_rms": average([
            r["straight_rms"]
            for r in results
        ]),

        "straight_max": max([
            r["straight_max"]
            for r in results
        ]),

        "turn_rms": average([
            r["turn_rms"]
            for r in results
        ]),

        "turn_max": max([
            r["turn_max"]
            for r in results
        ]),

        "waypoint_average": average([
            r["waypoint_average"]
            for r in results
        ]),

        "waypoint_max": max([
            r["waypoint_max"]
            for r in results
        ]),

        "waypoint_count": average([
            r["waypoint_count"]
            for r in results
        ]),

        "completed_trials": sum(
            1
            for r in results
            if r["completed"]
        ),
    }


# =========================================================
# PRINT RESULT
# =========================================================

def print_result(
    label,
    result,
):

    print()
    print("=" * 80)
    print(label)
    print("=" * 80)

    print(
        f"Straight RMS       : "
        f"{result['straight_rms']:.3f} m"
    )

    print(
        f"Straight MAX       : "
        f"{result['straight_max']:.3f} m"
    )

    print(
        f"Turn RMS           : "
        f"{result['turn_rms']:.3f} m"
    )

    print(
        f"Turn MAX           : "
        f"{result['turn_max']:.3f} m"
    )

    print(
        f"Waypoint Average   : "
        f"{result['waypoint_average']:.3f} m"
    )

    print(
        f"Waypoint MAX       : "
        f"{result['waypoint_max']:.3f} m"
    )

    print(
        f"Waypoints reached  : "
        f"{result['waypoint_count']:.2f}"
    )

    print(
        f"Completed trials   : "
        f"{result['completed_trials']}/"
        f"{TRIALS_PER_CASE}"
    )


# =========================================================
# MAIN
# =========================================================

def main():

    print()
    print("=" * 80)
    print("SENSOR NOISE ROBUSTNESS TEST")
    print("=" * 80)

    print()
    print("LOCKED CONTROLLER")
    print()

    print(
        f"Cross-track gain : "
        f"{CROSS_TRACK_GAIN:.2f}"
    )

    print(
        f"Lookahead        : "
        f"{LOOKAHEAD_DISTANCE:.2f} m"
    )

    print(
        f"Turn speed       : "
        f"{TURN_SPEED:.2f} m/s"
    )

    print(
        f"Turn zone        : "
        f"{TURN_ZONE_DISTANCE:.2f} m"
    )

    print()
    print(
        "Controller parameters will NOT be changed."
    )


    # =====================================================
    # TEST 1 — BASELINE
    # =====================================================

    result = aggregate(
        run_trials(
            gps_noise=0.0,
            heading_noise=0.0,
        )
    )

    print_result(
        "TEST 1 - ZERO ADDITIONAL SENSOR NOISE",
        result,
    )


    # =====================================================
    # TEST 2 — GPS NOISE
    # =====================================================

    print()
    print("=" * 80)
    print("TEST 2 - GPS POSITION NOISE")
    print("=" * 80)

    for noise in GPS_NOISE_LEVELS:

        result = aggregate(
            run_trials(
                gps_noise=noise,
                heading_noise=0.0,
            )
        )

        print_result(
            f"GPS NOISE = ±{noise:.2f} m",
            result,
        )


    # =====================================================
    # TEST 3 — HEADING NOISE
    # =====================================================

    print()
    print("=" * 80)
    print("TEST 3 - HEADING SENSOR NOISE")
    print("=" * 80)

    for noise in HEADING_NOISE_LEVELS:

        result = aggregate(
            run_trials(
                gps_noise=0.0,
                heading_noise=noise,
            )
        )

        print_result(
            f"HEADING NOISE = ±{noise:.1f} deg",
            result,
        )


    # =====================================================
    # TEST 4 — COMBINED NOISE
    # =====================================================

    print()
    print("=" * 80)
    print("TEST 4 - COMBINED GPS + HEADING NOISE")
    print("=" * 80)

    combined_cases = [

        (
            "GPS ±0.10 m + Heading ±1°",
            0.10,
            1.0,
        ),

        (
            "GPS ±0.25 m + Heading ±2°",
            0.25,
            2.0,
        ),

        (
            "GPS ±0.50 m + Heading ±5°",
            0.50,
            5.0,
        ),

        (
            "GPS ±1.00 m + Heading ±10°",
            1.00,
            10.0,
        ),
    ]

    for (
        label,
        gps_noise,
        heading_noise,
    ) in combined_cases:

        result = aggregate(
            run_trials(
                gps_noise=gps_noise,
                heading_noise=heading_noise,
            )
        )

        print_result(
            label,
            result,
        )


    # =====================================================
    # COMPLETE
    # =====================================================

    print()
    print("=" * 80)
    print("SENSOR NOISE TEST COMPLETE")
    print("=" * 80)

    print()
    print(
        "No navigation parameters were optimized."
    )

    print(
        "The purpose of this test is to identify "
        "the noise level at which the locked controller "
        "begins to degrade."
    )

    print("=" * 80)


if __name__ == "__main__":
    main()