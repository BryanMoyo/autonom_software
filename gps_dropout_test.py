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

WAYPOINT_TOLERANCE = 0.50


# =========================================================
# GPS DROPOUT DURATIONS
# =========================================================

DROPOUT_DURATIONS = [
    0.0,
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


def segment_error(
    x,
    y,
    segment,
):
    segment = max(
        0,
        min(
            segment,
            len(WAYPOINTS) - 2,
        ),
    )

    x1, y1 = WAYPOINTS[
        segment
    ]

    x2, y2 = WAYPOINTS[
        segment + 1
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
# CREATE SYSTEM
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
    # LOCKED NAVIGATION PARAMETERS
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
    dropout_duration,
    dropout_start,
    trial_number,
):

    (
        robot,
        gps,
        gyro,
        magnetometer,
        localization,
        navigator,
    ) = create_system()

    # -----------------------------------------------------
    # Deterministic trial
    # -----------------------------------------------------

    random.seed(
        30000
        + trial_number
    )

    # -----------------------------------------------------
    # Dropout timing
    # -----------------------------------------------------

    dropout_start_step = int(
        dropout_start / DT
    )

    dropout_steps = int(
        dropout_duration / DT
    )

    dropout_end_step = (
        dropout_start_step
        +
        dropout_steps
    )

    dropout_started = False
    dropout_recovered = False

    # -----------------------------------------------------
    # Metrics
    # -----------------------------------------------------

    straight_errors = []
    turn_errors = []

    waypoint_errors = []

    waypoint_index = 1

    max_error = 0.0
    max_error_step = 0

    error_at_dropout = None
    error_at_recovery = None

    max_error_during_dropout = 0.0

    max_localization_error = 0.0

    recovery_time = None

    recovery_start_step = None

    consecutive_good_steps = 0

    total_distance = 0.0

    previous_x = robot.x
    previous_y = robot.y

    max_steps = 10000

    failure_reason = None


    # =====================================================
    # SIMULATION LOOP
    # =====================================================

    for step in range(max_steps):

        # -------------------------------------------------
        # TRUE ROBOT STATE
        # -------------------------------------------------

        true_x = robot.x
        true_y = robot.y
        true_heading = robot.heading


        # -------------------------------------------------
        # GPS AVAILABILITY
        # -------------------------------------------------

        gps_available = not (
            dropout_start_step
            <= step
            <
            dropout_end_step
        )


        # -------------------------------------------------
        # DROPOUT START
        # -------------------------------------------------

        if (
            dropout_duration > 0
            and
            step == dropout_start_step
        ):

            dropout_started = True

            error_at_dropout = (
                segment_error(
                    robot.x,
                    robot.y,
                    waypoint_index - 1,
                )
            )

            print(
                f"    GPS DROPOUT START "
                f"at step {step}"
            )


        # -------------------------------------------------
        # GPS RECOVERY
        # -------------------------------------------------

        if (
            dropout_duration > 0
            and
            step == dropout_end_step
        ):

            dropout_recovered = True

            recovery_start_step = step

            error_at_recovery = (
                segment_error(
                    robot.x,
                    robot.y,
                    waypoint_index - 1,
                )
            )

            print(
                f"    GPS RECOVERED "
                f"at step {step}"
            )


        # =================================================
        # GPS MEASUREMENT
        # =================================================

        if gps_available:

            # GPS available:
            # use the true robot position as the
            # simulated GPS measurement.

            measured_x = robot.x
            measured_y = robot.y

        else:

            # GPS unavailable:
            #
            # Hold the last localization estimate.
            #
            # Localization will now propagate its position
            # using dead reckoning because we pass robot.speed
            # into Localization.update() below.

            measured_x = localization.x
            measured_y = localization.y


        # =================================================
        # GYROSCOPE
        # =================================================

        gyro_reading = gyro.read(
            0.0
        )


        # =================================================
        # MAGNETOMETER
        # =================================================

        magnetometer_heading = (
            magnetometer.read(
                true_heading
            )
        )


        # =================================================
        # LOCALIZATION
        # =================================================

        # IMPORTANT:
        #
        # robot.speed is explicitly passed here.
        #
        # This allows Localization to execute:
        #
        #     distance = speed * dt
        #
        # during GPS dropout.

        localization.update(
            measured_x,
            measured_y,
            gyro_reading,
            magnetometer_heading,
            DT,
            robot.speed,
        )


        # =================================================
        # LOCALIZATION ERROR
        # =================================================

        localization_error = distance(
            robot.x,
            robot.y,
            localization.x,
            localization.y,
        )

        max_localization_error = max(
            max_localization_error,
            localization_error,
        )


        # =================================================
        # NAVIGATION
        # =================================================

        (
            desired_heading,
            desired_speed,
            complete,
        ) = navigator.update(
            localization.x,
            localization.y,
        )


        # =================================================
        # ROBOT
        # =================================================

        robot.update(
            desired_heading,
            desired_speed,
            DT,
        )


        # =================================================
        # DISTANCE TRAVELLED
        # =================================================

        movement = distance(
            previous_x,
            previous_y,
            robot.x,
            robot.y,
        )

        total_distance += movement

        previous_x = robot.x
        previous_y = robot.y


        # =================================================
        # WAYPOINT DETECTION
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

                waypoint_index += 1


        # =================================================
        # CROSS-TRACK ERROR
        # =================================================

        active_segment = (
            waypoint_index - 1
        )

        if (
            0
            <= active_segment
            <
            len(WAYPOINTS) - 1
        ):

            error = segment_error(
                robot.x,
                robot.y,
                active_segment,
            )


            # -------------------------------------------------
            # MAX ERROR
            # -------------------------------------------------

            if error > max_error:

                max_error = error
                max_error_step = step


            # -------------------------------------------------
            # DISTANCE FROM ROW ENDS
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
            # STRAIGHT VS TURN
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


            # -------------------------------------------------
            # ERROR DURING GPS DROPOUT
            # -------------------------------------------------

            if (
                dropout_started
                and
                not dropout_recovered
            ):

                max_error_during_dropout = max(
                    max_error_during_dropout,
                    error,
                )


            # -------------------------------------------------
            # RECOVERY DETECTION
            #
            # Ten consecutive steps below 0.50 m
            # is considered recovered.
            # -------------------------------------------------

            if dropout_recovered:

                if error <= 0.50:

                    consecutive_good_steps += 1

                else:

                    consecutive_good_steps = 0


                if (
                    recovery_time is None
                    and
                    consecutive_good_steps
                    >= 10
                ):

                    recovery_time = (
                        step
                        -
                        recovery_start_step
                    ) * DT


        # =================================================
        # SAFETY DIVERGENCE
        # =================================================

        if error > 100.0:

            failure_reason = (
                "PATH DIVERGENCE"
            )

            break


        if (
            abs(robot.x) > 1000.0
            or
            abs(robot.y) > 1000.0
        ):

            failure_reason = (
                "POSITION DIVERGENCE"
            )

            break


        # =================================================
        # MISSION COMPLETE
        # =================================================

        if (
            waypoint_index
            >= len(WAYPOINTS)
        ):

            break


    # =====================================================
    # FINAL RESULT
    # =====================================================

    completed = (
        waypoint_index
        >= len(WAYPOINTS)
    )

    if (
        not completed
        and
        failure_reason is None
    ):

        failure_reason = (
            "MISSION TIMEOUT"
        )


    # =====================================================
    # RMS
    # =====================================================

    if straight_errors:

        straight_rms = math.sqrt(
            sum(
                value * value
                for value in straight_errors
            )
            /
            len(straight_errors)
        )

    else:

        straight_rms = float("inf")


    if turn_errors:

        turn_rms = math.sqrt(
            sum(
                value * value
                for value in turn_errors
            )
            /
            len(turn_errors)
        )

    else:

        turn_rms = float("inf")


    # =====================================================
    # RETURN
    # =====================================================

    return {

        "trial": trial_number,

        "completed": completed,

        "failure_reason": failure_reason,

        "waypoints": waypoint_index,

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

        "waypoint_max": (
            max(waypoint_errors)
            if waypoint_errors
            else float("inf")
        ),

        "max_error": max_error,

        "max_error_step": max_error_step,

        "error_at_dropout": (
            error_at_dropout
        ),

        "max_error_during_dropout": (
            max_error_during_dropout
        ),

        "error_at_recovery": (
            error_at_recovery
        ),

        "recovery_time": (
            recovery_time
        ),

        "max_localization_error": (
            max_localization_error
        ),

        "total_distance": total_distance,

        "final_x": robot.x,

        "final_y": robot.y,

        "final_heading": robot.heading,
    }


# =========================================================
# RUN CASE
# =========================================================

def run_case(
    label,
    dropout_duration,
    dropout_start,
):

    print()
    print("=" * 80)
    print(label)
    print("=" * 80)

    results = []

    for trial in range(TRIALS):

        result = run_trial(
            dropout_duration,
            dropout_start,
            trial,
        )

        results.append(
            result
        )


    # =====================================================
    # SUMMARY
    # =====================================================

    completed = sum(
        1
        for result in results
        if result["completed"]
    )

    max_error = max(
        result["max_error"]
        for result in results
    )

    max_localization_error = max(
        result["max_localization_error"]
        for result in results
    )

    recovery_times = [
        result["recovery_time"]
        for result in results
        if result["recovery_time"]
        is not None
    ]


    print()

    print(
        f"Completed trials       : "
        f"{completed}/{TRIALS}"
    )

    print(
        f"Worst path error       : "
        f"{max_error:.3f} m"
    )

    print(
        f"Worst localization err : "
        f"{max_localization_error:.3f} m"
    )


    if recovery_times:

        print(
            f"Average recovery time  : "
            f"{sum(recovery_times) / len(recovery_times):.2f} s"
        )

    elif dropout_duration > 0:

        print(
            "Average recovery time  : "
            "NOT RECOVERED"
        )


    # =====================================================
    # FAILED TRIALS
    # =====================================================

    failed = [
        result
        for result in results
        if not result["completed"]
    ]

    if failed:

        print()
        print("FAILED TRIALS")
        print("-" * 80)

        for result in failed:

            print(
                f"Trial "
                f"{result['trial']:02d} | "
                f"Reason="
                f"{result['failure_reason']} | "
                f"Waypoints="
                f"{result['waypoints']}/5 | "
                f"MaxError="
                f"{result['max_error']:.3f} m | "
                f"Final=("
                f"{result['final_x']:.2f}, "
                f"{result['final_y']:.2f})"
            )

    else:

        print(
            "Failed trials         : NONE"
        )


# =========================================================
# MAIN
# =========================================================

def main():

    print()
    print("=" * 80)
    print("GPS DROPOUT AND RECOVERY TEST")
    print("=" * 80)

    print()
    print("LOCKED CONTROLLER")

    print(
        f"Gain       : "
        f"{CROSS_TRACK_GAIN:.2f}"
    )

    print(
        f"Lookahead  : "
        f"{LOOKAHEAD_DISTANCE:.2f} m"
    )

    print(
        f"Turn speed : "
        f"{TURN_SPEED:.2f} m/s"
    )

    print(
        f"Turn zone  : "
        f"{TURN_ZONE_DISTANCE:.2f} m"
    )

    print()
    print(
        "No navigation parameters will be changed."
    )

    print()
    print(
        "DEAD-RECKONING ENABLED"
    )

    print(
        "Localization receives robot.speed."
    )


    # =====================================================
    # BASELINE
    # =====================================================

    run_case(
        "TEST 1 - GPS ALWAYS AVAILABLE",
        0.0,
        15.0,
    )


    # =====================================================
    # DROPOUT DURING FIRST ROW
    # =====================================================

    for duration in DROPOUT_DURATIONS[1:]:

        run_case(
            (
                f"TEST - {duration:.1f}s GPS DROPOUT "
                f"DURING FIRST ROW"
            ),
            duration,
            6.0,
        )


    # =====================================================
    # DROPOUT NEAR FIRST WAYPOINT
    # =====================================================

    for duration in [
        1.0,
        2.0,
        5.0,
    ]:

        run_case(
            (
                f"TEST - {duration:.1f}s GPS DROPOUT "
                f"NEAR FIRST WAYPOINT"
            ),
            duration,
            7.8,
        )


    # =====================================================
    # COMPLETE
    # =====================================================

    print()
    print("=" * 80)
    print("GPS DROPOUT TEST COMPLETE")
    print("=" * 80)


if __name__ == "__main__":
    main()