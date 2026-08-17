import math
import random

from navigation import Navigator
from robot import Robot
from sensors import GPSSensor, Gyroscope, Magnetometer
from localization import Localization


# =========================================================
# LOCKED NAVIGATION PARAMETERS
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
# TEST SCENARIOS
# =========================================================

SCENARIOS = [

    {
        "name": "BASELINE",
        "gps_noise": 0.0,
        "dropout": 0.0,
        "heading_noise": 0.0,
        "dropout_start": 15.0,
    },

    {
        "name": "GPS ±0.25m + 5s DROPOUT + ±5° HEADING",
        "gps_noise": 0.25,
        "dropout": 5.0,
        "heading_noise": 5.0,
        "dropout_start": 6.0,
    },

    {
        "name": "GPS ±0.50m + 5s DROPOUT + ±5° HEADING",
        "gps_noise": 0.50,
        "dropout": 5.0,
        "heading_noise": 5.0,
        "dropout_start": 6.0,
    },

    {
        "name": "GPS ±0.25m + 5s DROPOUT + ±10° HEADING",
        "gps_noise": 0.25,
        "dropout": 5.0,
        "heading_noise": 10.0,
        "dropout_start": 6.0,
    },

    {
        "name": "GPS ±0.50m + 5s DROPOUT + ±10° HEADING",
        "gps_noise": 0.50,
        "dropout": 5.0,
        "heading_noise": 10.0,
        "dropout_start": 6.0,
    },

    {
        "name": "GPS ±0.50m + 10s DROPOUT + ±10° HEADING",
        "gps_noise": 0.50,
        "dropout": 10.0,
        "heading_noise": 10.0,
        "dropout_start": 6.0,
    },
]


# =========================================================
# TEST CLASSIFICATION LIMITS
# =========================================================

# If the robot reaches the final area but the waypoint counter
# does not advance because of the test's strict 0.50 m check,
# classify it separately rather than calling it navigation
# divergence.

NEAR_FINAL_DISTANCE = 1.50

# A true navigation failure is a large departure from the
# intended path.

DIVERGENCE_ERROR = 10.0


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
# CLASSIFY FINAL STATE
# =========================================================

def classify_result(
    robot,
    waypoint_index,
    max_error,
):
    final_x = robot.x
    final_y = robot.y

    final_x_target, final_y_target = WAYPOINTS[-1]

    final_distance = distance(
        final_x,
        final_y,
        final_x_target,
        final_y_target,
    )

    # -----------------------------------------------------
    # TRUE MISSION COMPLETION
    # -----------------------------------------------------

    if (
        waypoint_index
        >= len(WAYPOINTS)
    ):
        return (
            "MISSION COMPLETE",
            final_distance,
        )

    # -----------------------------------------------------
    # GENUINE DIVERGENCE
    #
    # If the robot accumulated a very large path error,
    # this is a navigation failure, regardless of where it
    # eventually stopped.
    # -----------------------------------------------------

    if max_error >= DIVERGENCE_ERROR:
        return (
            "NAVIGATION DIVERGENCE",
            final_distance,
        )

    # -----------------------------------------------------
    # NEAR-FINAL COMPLETION
    #
    # Robot is physically close to the final waypoint but
    # the strict waypoint counter did not register it.
    # -----------------------------------------------------

    if final_distance <= NEAR_FINAL_DISTANCE:
        return (
            "NEAR-FINAL COMPLETION",
            final_distance,
        )

    # -----------------------------------------------------
    # OTHER FAILURE
    # -----------------------------------------------------

    return (
        "MISSION FAILURE",
        final_distance,
    )


# =========================================================
# RUN ONE TRIAL
# =========================================================

def run_trial(
    scenario,
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

    random.seed(
        40000
        +
        trial_number
    )

    gps_noise = scenario[
        "gps_noise"
    ]

    dropout_duration = scenario[
        "dropout"
    ]

    heading_noise = scenario[
        "heading_noise"
    ]

    dropout_start = scenario[
        "dropout_start"
    ]

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

    waypoint_index = 1

    waypoint_errors = []

    straight_errors = []

    turn_errors = []

    max_error = 0.0

    max_localization_error = 0.0

    max_dropout_error = 0.0

    recovery_time = None

    recovery_start_step = None

    consecutive_good_steps = 0

    total_distance = 0.0

    previous_x = robot.x
    previous_y = robot.y

    max_steps = 10000

    failure_reason = None


    # =====================================================
    # SIMULATION
    # =====================================================

    for step in range(max_steps):

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
        # GPS
        # -------------------------------------------------

        if gps_available:

            gps_x = (
                robot.x
                +
                random.uniform(
                    -gps_noise,
                    gps_noise,
                )
            )

            gps_y = (
                robot.y
                +
                random.uniform(
                    -gps_noise,
                    gps_noise,
                )
            )

        else:

            gps_x = localization.x
            gps_y = localization.y


        # -------------------------------------------------
        # HEADING
        # -------------------------------------------------

        measured_heading = (
            robot.heading
            +
            random.uniform(
                -heading_noise,
                heading_noise,
            )
        )

        measured_heading %= 360.0


        # -------------------------------------------------
        # GYRO
        # -------------------------------------------------

        gyro_reading = gyro.read(
            0.0
        )


        # -------------------------------------------------
        # LOCALIZATION
        # -------------------------------------------------

        localization.update(
            gps_x,
            gps_y,
            gyro_reading,
            measured_heading,
            DT,
            robot.speed,
        )


        # -------------------------------------------------
        # LOCALIZATION ERROR
        # -------------------------------------------------

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


        # -------------------------------------------------
        # DISTANCE
        # -------------------------------------------------

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
        # WAYPOINT TEST COUNTER
        # -------------------------------------------------

        if (
            waypoint_index
            <
            len(WAYPOINTS)
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

        if active_segment < 0:
            active_segment = 0

        if (
            active_segment
            >= len(WAYPOINTS) - 1
        ):
            active_segment = (
                len(WAYPOINTS) - 2
            )


        # -------------------------------------------------
        # PATH ERROR
        # -------------------------------------------------

        error = segment_error(
            robot.x,
            robot.y,
            active_segment,
        )

        max_error = max(
            max_error,
            error,
        )


        # -------------------------------------------------
        # START / END DISTANCE
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
        # STRAIGHT / TURN
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
        # DROPOUT ERROR
        # -------------------------------------------------

        if (
            dropout_start_step
            <= step
            <
            dropout_end_step
        ):

            max_dropout_error = max(
                max_dropout_error,
                error,
            )


        # -------------------------------------------------
        # RECOVERY
        # -------------------------------------------------

        if (
            dropout_duration > 0
            and
            step >= dropout_end_step
        ):

            if error <= 0.50:

                consecutive_good_steps += 1

            else:

                consecutive_good_steps = 0


            if (
                recovery_time is None
                and
                consecutive_good_steps >= 10
            ):

                recovery_start_step = (
                    dropout_end_step
                )

                recovery_time = (
                    step
                    -
                    recovery_start_step
                ) * DT


        # -------------------------------------------------
        # HARD DIVERGENCE
        # -------------------------------------------------

        if error >= DIVERGENCE_ERROR:

            failure_reason = (
                "NAVIGATION DIVERGENCE"
            )

            break


        # -------------------------------------------------
        # NAVIGATOR COMPLETION
        # -------------------------------------------------

        if navigator.complete:

            break


    # =====================================================
    # CLASSIFICATION
    # =====================================================

    classification, final_distance = (
        classify_result(
            robot,
            waypoint_index,
            max_error,
        )
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

        straight_rms = 0.0


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

        turn_rms = 0.0


    return {

        "trial": trial_number,

        "classification": classification,

        "completed": (
            classification
            == "MISSION COMPLETE"
        ),

        "waypoints": waypoint_index,

        "straight_rms": straight_rms,

        "straight_max": (
            max(straight_errors)
            if straight_errors
            else 0.0
        ),

        "turn_rms": turn_rms,

        "turn_max": (
            max(turn_errors)
            if turn_errors
            else 0.0
        ),

        "waypoint_max": (
            max(waypoint_errors)
            if waypoint_errors
            else 0.0
        ),

        "max_error": max_error,

        "max_localization_error": (
            max_localization_error
        ),

        "max_dropout_error": (
            max_dropout_error
        ),

        "recovery_time": (
            recovery_time
        ),

        "final_distance": (
            final_distance
        ),

        "final_x": robot.x,

        "final_y": robot.y,

        "final_heading": robot.heading,

        "total_distance": total_distance,
    }


# =========================================================
# RUN SCENARIO
# =========================================================

def run_scenario(
    scenario,
):

    print()
    print("=" * 80)
    print(
        scenario["name"]
    )
    print("=" * 80)

    results = []

    for trial in range(
        1,
        TRIALS + 1,
    ):

        result = run_trial(
            scenario,
            trial,
        )

        results.append(
            result
        )


    # =====================================================
    # COUNTS
    # =====================================================

    complete_count = sum(
        result["classification"]
        == "MISSION COMPLETE"
        for result in results
    )

    near_final_count = sum(
        result["classification"]
        == "NEAR-FINAL COMPLETION"
        for result in results
    )

    divergence_count = sum(
        result["classification"]
        == "NAVIGATION DIVERGENCE"
        for result in results
    )

    other_failure_count = sum(
        result["classification"]
        == "MISSION FAILURE"
        for result in results
    )


    # =====================================================
    # MAXIMUMS
    # =====================================================

    max_path_error = max(
        result["max_error"]
        for result in results
    )

    max_localization_error = max(
        result["max_localization_error"]
        for result in results
    )

    max_dropout_error = max(
        result["max_dropout_error"]
        for result in results
    )

    recovery_times = [
        result["recovery_time"]
        for result in results
        if result["recovery_time"]
        is not None
    ]


    # =====================================================
    # SUMMARY
    # =====================================================

    print()

    print(
        f"MISSION COMPLETE       : "
        f"{complete_count}/{TRIALS}"
    )

    print(
        f"NEAR-FINAL COMPLETION  : "
        f"{near_final_count}/{TRIALS}"
    )

    print(
        f"NAVIGATION DIVERGENCE  : "
        f"{divergence_count}/{TRIALS}"
    )

    print(
        f"OTHER FAILURE          : "
        f"{other_failure_count}/{TRIALS}"
    )

    print()

    print(
        f"Worst path error       : "
        f"{max_path_error:.3f} m"
    )

    print(
        f"Worst localization err : "
        f"{max_localization_error:.3f} m"
    )

    if scenario["dropout"] > 0:

        print(
            f"Worst dropout error    : "
            f"{max_dropout_error:.3f} m"
        )

    if recovery_times:

        print(
            f"Average recovery time  : "
            f"{sum(recovery_times) / len(recovery_times):.2f} s"
        )


    # =====================================================
    # TRIAL DETAILS
    # =====================================================

    print()
    print(
        "TRIAL RESULTS"
    )
    print("-" * 80)

    for result in results:

        print(
            f"Trial {result['trial']:02d} | "
            f"{result['classification']:<22} | "
            f"WP={result['waypoints']}/5 | "
            f"MaxError="
            f"{result['max_error']:.3f} m | "
            f"Final="
            f"({result['final_x']:.2f}, "
            f"{result['final_y']:.2f}) | "
            f"FinalDist="
            f"{result['final_distance']:.3f} m"
        )


# =========================================================
# MAIN
# =========================================================

def main():

    print()
    print("=" * 80)
    print(
        "CORRECTED COMBINED SENSOR TEST"
    )
    print("=" * 80)

    print()
    print(
        "Navigation parameters are LOCKED."
    )

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
        "Test classification:"
    )

    print(
        "  MISSION COMPLETE       = all waypoints registered"
    )

    print(
        "  NEAR-FINAL COMPLETION  = within 1.50 m of final waypoint"
    )

    print(
        "  NAVIGATION DIVERGENCE  = path error >= 10.00 m"
    )

    print(
        "  MISSION FAILURE        = neither condition"
    )

    print()


    for scenario in SCENARIOS:

        run_scenario(
            scenario
        )


    print()
    print("=" * 80)
    print(
        "CORRECTED COMBINED SENSOR TEST COMPLETE"
    )
    print("=" * 80)


if __name__ == "__main__":

    main()