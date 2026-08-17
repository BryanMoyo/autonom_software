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

STRAIGHT_MAX_TARGET = 0.75
TURN_MAX_TARGET = 1.50
WAYPOINT_MAX_TARGET = 0.50


# =========================================================
# WAYPOINT TOLERANCE
# =========================================================

WAYPOINT_TOLERANCE = 0.50


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
        dx * dx +
        dy * dy
    )

    if length_squared <= 1e-12:
        return math.hypot(
            px - x1,
            py - y1,
        )

    t = (
        (px - x1) * dx +
        (py - y1) * dy
    ) / length_squared

    t = max(
        0.0,
        min(
            1.0,
            t,
        ),
    )

    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    return math.hypot(
        px - closest_x,
        py - closest_y,
    )


def active_segment_error(
    x,
    y,
    segment_index,
):
    if segment_index < 0:
        return 0.0

    if segment_index >= len(WAYPOINTS) - 1:
        segment_index = len(WAYPOINTS) - 2

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

    return sum(values) / len(values)


def percentile(
    values,
    p,
):

    if not values:
        return float("inf")

    ordered = sorted(values)

    index = (
        p / 100.0
        *
        (len(ordered) - 1)
    )

    lower = math.floor(index)
    upper = math.ceil(index)

    if lower == upper:
        return ordered[lower]

    fraction = index - lower

    return (
        ordered[lower]
        +
        (
            ordered[upper]
            -
            ordered[lower]
        )
        *
        fraction
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
    # LOCKED PARAMETERS
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
    disturbance=None,
    initial_offset=None,
    initial_heading_offset=0.0,
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
    # INITIAL DISTURBANCE
    # =====================================================

    if initial_offset is not None:

        robot.x += initial_offset[0]
        robot.y += initial_offset[1]


    robot.heading = (
        robot.heading
        +
        initial_heading_offset
    ) % 360.0


    # =====================================================
    # METRICS
    # =====================================================

    straight_errors = []
    turn_errors = []

    waypoint_errors = []

    recovery_errors = []

    total_distance = 0.0

    disturbance_applied = False

    recovery_start_distance = None
    recovery_distance = None

    consecutive_recovered = 0

    max_recovery_error = 0.0


    # =====================================================
    # ROBUST WAYPOINT TRACKING
    #
    # We maintain our own waypoint index.
    #
    # This is deliberately independent of Navigator's
    # internal waypoint bookkeeping.
    # =====================================================

    test_waypoint_index = 1

    reached_waypoints = []

    # The first waypoint is the starting location.
    reached_waypoints.append(
        WAYPOINTS[0]
    )


    # =====================================================
    # SIMULATION
    # =====================================================

    max_steps = 10000

    for step in range(max_steps):


        # -------------------------------------------------
        # SENSORS
        # -------------------------------------------------

        gps_x, gps_y = gps.read(
            robot.x,
            robot.y,
        )

        gyro_reading = gyro.read(
            0.0
        )

        magnetometer_heading = (
            magnetometer.read(
                robot.heading
            )
        )


        # -------------------------------------------------
        # LOCALIZATION
        # -------------------------------------------------

        localization.update(
            gps_x,
            gps_y,
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

        previous_x = robot.x
        previous_y = robot.y

        robot.update(
            desired_heading,
            desired_speed,
            DT,
        )

        movement = math.hypot(
            robot.x - previous_x,
            robot.y - previous_y,
        )

        total_distance += movement


        # =================================================
        # MID-ROW DISTURBANCE
        # =================================================

        if (
            disturbance is not None
            and not disturbance_applied
            and test_waypoint_index == 1
            and robot.x >= 15.0
        ):

            robot.x += disturbance[0]
            robot.y += disturbance[1]

            disturbance_applied = True

            recovery_start_distance = (
                total_distance
            )


        # =================================================
        # WAYPOINT DETECTION
        #
        # IMPORTANT:
        # Check the actual robot position against the
        # actual target waypoint.
        # =================================================

        if (
            test_waypoint_index
            < len(WAYPOINTS)
        ):

            target_x, target_y = (
                WAYPOINTS[
                    test_waypoint_index
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
                        test_waypoint_index
                    ]
                )

                test_waypoint_index += 1


        # =================================================
        # ACTIVE SEGMENT
        # =================================================

        active_segment = (
            test_waypoint_index - 1
        )


        # =================================================
        # CROSS-TRACK ERROR
        # =================================================

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
            # Determine whether this is the end/turn area.
            # -------------------------------------------------

            x1, y1 = WAYPOINTS[
                active_segment
            ]

            x2, y2 = WAYPOINTS[
                active_segment + 1
            ]

            distance_from_end = distance(
                robot.x,
                robot.y,
                x2,
                y2,
            )

            distance_from_start = distance(
                robot.x,
                robot.y,
                x1,
                y1,
            )

            if (
                distance_from_end
                <= TURN_ZONE_DISTANCE
                or
                distance_from_start
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
            # Recovery measurement
            # -------------------------------------------------

            if disturbance_applied:

                recovery_errors.append(
                    error
                )

                max_recovery_error = max(
                    max_recovery_error,
                    error,
                )

                if error <= 0.50:

                    consecutive_recovered += 1

                else:

                    consecutive_recovered = 0


                if (
                    recovery_distance is None
                    and consecutive_recovered >= 10
                ):

                    recovery_distance = (
                        total_distance
                        -
                        recovery_start_distance
                    )


        # =================================================
        # MISSION COMPLETE
        # =================================================

        if (
            test_waypoint_index
            >= len(WAYPOINTS)
        ):

            break


    # =====================================================
    # FINAL RESULT
    # =====================================================

    completed = (
        test_waypoint_index
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

        "waypoint_p95": percentile(
            waypoint_errors,
            95,
        ),

        "waypoint_count": len(
            reached_waypoints
        ),

        "completed": completed,

        "recovery_max_error": (
            max_recovery_error
        ),

        "recovery_distance": (
            recovery_distance
            if recovery_distance is not None
            else float("inf")
        ),
    }


# =========================================================
# MULTIPLE TRIALS
# =========================================================

def run_trials(
    disturbance=None,
    initial_offset=None,
    initial_heading_offset=0.0,
):

    results = []

    for trial in range(
        TRIALS_PER_CASE
    ):

        random.seed(
            5000 + trial
        )

        results.append(
            run_trial(
                disturbance=disturbance,
                initial_offset=initial_offset,
                initial_heading_offset=initial_heading_offset,
            )
        )

    return results


# =========================================================
# AGGREGATION
# =========================================================

def aggregate(results):

    recovery_distances = [
        r["recovery_distance"]
        for r in results
        if math.isfinite(
            r["recovery_distance"]
        )
    ]

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

        "waypoint_max": max([
            r["waypoint_max"]
            for r in results
        ]),

        "waypoint_average": average([
            r["waypoint_average"]
            for r in results
        ]),

        "waypoint_p95": max([
            r["waypoint_p95"]
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

        "recovery_max_error": max([
            r["recovery_max_error"]
            for r in results
        ]),

        "recovery_distance": (
            average(
                recovery_distances
            )
            if recovery_distances
            else float("inf")
        ),

        "recovery_successes": sum(
            1
            for r in results
            if math.isfinite(
                r["recovery_distance"]
            )
        ),
    }


# =========================================================
# PRINT
# =========================================================

def print_case(
    name,
    result,
):

    print()
    print("=" * 80)
    print(name)
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
        f"Waypoint 95%      : "
        f"{result['waypoint_p95']:.3f} m"
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

    if result["recovery_max_error"] > 0:

        print(
            f"Recovery max error : "
            f"{result['recovery_max_error']:.3f} m"
        )

        if math.isfinite(
            result["recovery_distance"]
        ):

            print(
                f"Recovery distance  : "
                f"{result['recovery_distance']:.2f} m"
            )

        else:

            print(
                "Recovery distance  : "
                "NOT RECOVERED"
            )

        print(
            f"Recovery successes : "
            f"{result['recovery_successes']}/"
            f"{TRIALS_PER_CASE}"
        )


# =========================================================
# MAIN
# =========================================================

def main():

    print()
    print("=" * 80)
    print("NAVIGATION ROBUSTNESS TEST")
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
        "No controller optimization will be performed."
    )


    # =====================================================
    # TEST 1
    # =====================================================

    print()
    print("=" * 80)
    print("TEST 1 - BASELINE")
    print("=" * 80)

    result = aggregate(
        run_trials()
    )

    print_case(
        "BASELINE RESULT",
        result,
    )


    # =====================================================
    # TEST 2
    # =====================================================

    print()
    print("=" * 80)
    print("TEST 2 - INITIAL POSITION ROBUSTNESS")
    print("=" * 80)

    initial_cases = [

        ("START +0.5 m Y", (0.0, 0.5)),

        ("START -0.5 m Y", (0.0, -0.5)),

        ("START +1.0 m Y", (0.0, 1.0)),

        ("START -1.0 m Y", (0.0, -1.0)),

        ("START +2.0 m Y", (0.0, 2.0)),

        ("START -2.0 m Y", (0.0, -2.0)),
    ]

    for name, offset in initial_cases:

        result = aggregate(
            run_trials(
                initial_offset=offset
            )
        )

        print_case(
            name,
            result,
        )


    # =====================================================
    # TEST 3
    # =====================================================

    print()
    print("=" * 80)
    print("TEST 3 - MID-ROW CROSS-TRACK RECOVERY")
    print("=" * 80)

    disturbance_cases = [

        ("DISTURB +0.5 m", (0.0, 0.5)),

        ("DISTURB -0.5 m", (0.0, -0.5)),

        ("DISTURB +1.0 m", (0.0, 1.0)),

        ("DISTURB -1.0 m", (0.0, -1.0)),

        ("DISTURB +2.0 m", (0.0, 2.0)),

        ("DISTURB -2.0 m", (0.0, -2.0)),
    ]

    for name, disturbance in disturbance_cases:

        result = aggregate(
            run_trials(
                disturbance=disturbance
            )
        )

        print_case(
            name,
            result,
        )


    # =====================================================
    # TEST 4
    # =====================================================

    print()
    print("=" * 80)
    print("TEST 4 - INITIAL HEADING ROBUSTNESS")
    print("=" * 80)

    heading_cases = [

        ("HEADING +5 deg", 5.0),

        ("HEADING -5 deg", -5.0),

        ("HEADING +10 deg", 10.0),

        ("HEADING -10 deg", -10.0),

        ("HEADING +20 deg", 20.0),

        ("HEADING -20 deg", -20.0),
    ]

    for name, heading in heading_cases:

        result = aggregate(
            run_trials(
                initial_heading_offset=heading
            )
        )

        print_case(
            name,
            result,
        )


    # =====================================================
    # COMPLETE
    # =====================================================

    print()
    print("=" * 80)
    print("ROBUSTNESS TEST COMPLETE")
    print("=" * 80)

    print()
    print(
        "The controller parameters were NOT changed."
    )

    print(
        "Waypoint completion is now measured "
        "independently from Navigator bookkeeping."
    )

    print("=" * 80)


if __name__ == "__main__":
    main()