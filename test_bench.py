import math
import random

from navigation import Navigator
from robot import Robot
from sensors import GPSSensor, Gyroscope, Magnetometer
from localization import Localization


# =========================================================
# GENERAL SETTINGS
# =========================================================

DT = 0.1

# Quick screening
SCREENING_TRIALS = 10

# Confirmation testing for the best candidates
CONFIRMATION_TRIALS = 100

# Number of candidates carried into confirmation testing
TOP_CANDIDATES = 10


# =========================================================
# ENGINEERING TARGETS
# =========================================================

STRAIGHT_RMS_TARGET = 0.50

STRAIGHT_MAX_TARGET = 0.75

TURN_MAX_TARGET = 1.50

WAYPOINT_TARGET = 0.50


# =========================================================
# WAYPOINTS
# =========================================================

WAYPOINTS = [
    (10.0, 10.0),
    (30.0, 10.0),
    (30.0, 30.0),
    (10.0, 30.0),
    (10.0, 10.0),
]


# =========================================================
# FIXED PARAMETERS
#
# These are NOT being optimized.
# =========================================================

CRUISE_SPEED = 2.50

MINIMUM_SPEED = 1.20

APPROACH_DISTANCE = 8.00


# =========================================================
# AUTOMATIC SEARCH SPACE
#
# The computer will test these combinations.
# =========================================================

CROSS_TRACK_GAINS = [
    0.20,
    0.30,
    0.40,
    0.50,
    0.60,
    0.70,
    0.80,
]


LOOKAHEAD_DISTANCES = [
    1.50,
    2.00,
    2.50,
    3.00,
    3.50,
    4.00,
    4.50,
]


TURN_SPEEDS = [
    0.50,
    0.60,
    0.70,
    0.80,
    0.90,
    1.00,
    1.10,
]


TURN_ZONE_DISTANCES = [
    2.00,
    3.00,
    4.00,
    5.00,
]


# =========================================================
# GEOMETRY
# =========================================================

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

    if length_squared == 0.0:
        return math.sqrt(
            (px - x1) ** 2
            + (py - y1) ** 2
        )

    t = (
        (px - x1) * dx
        + (py - y1) * dy
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
        + t * dx
    )

    closest_y = (
        y1
        + t * dy
    )

    return math.sqrt(
        (px - closest_x) ** 2
        + (py - closest_y) ** 2
    )


# =========================================================
# ACTIVE ROW ERROR
# =========================================================

def calculate_cross_track_error(
    x,
    y,
    segment_index,
):
    if segment_index < 0:
        return 0.0

    if segment_index >= len(WAYPOINTS) - 1:
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
# TURN-ZONE CLASSIFICATION
# =========================================================

def is_turn_zone(
    x,
    y,
    segment_index,
    turn_zone_distance,
):
    if segment_index < 0:
        return True

    if segment_index >= len(WAYPOINTS) - 1:
        return True

    start_x, start_y = WAYPOINTS[
        segment_index
    ]

    end_x, end_y = WAYPOINTS[
        segment_index + 1
    ]

    distance_from_start = math.sqrt(
        (x - start_x) ** 2
        + (y - start_y) ** 2
    )

    distance_from_end = math.sqrt(
        (x - end_x) ** 2
        + (y - end_y) ** 2
    )

    return (
        distance_from_start
        <= turn_zone_distance
        or
        distance_from_end
        <= turn_zone_distance
    )


# =========================================================
# RMS
# =========================================================

def calculate_rms(values):
    if not values:
        return float("inf")

    return math.sqrt(
        sum(
            value * value
            for value in values
        )
        / len(values)
    )


# =========================================================
# PERCENTILE
# =========================================================

def calculate_percentile(
    values,
    percentile,
):
    if not values:
        return float("inf")

    values = sorted(values)

    position = (
        percentile
        / 100.0
        * (len(values) - 1)
    )

    lower = math.floor(position)

    upper = math.ceil(position)

    if lower == upper:
        return values[lower]

    fraction = position - lower

    return (
        values[lower]
        + (
            values[upper]
            - values[lower]
        )
        * fraction
    )


# =========================================================
# RUN ONE MISSION
# =========================================================

def run_mission(
    cross_track_gain,
    lookahead_distance,
    turn_speed,
    turn_zone_distance,
):
    robot = Robot()

    gps = GPSSensor()

    gyro = Gyroscope()

    magnetometer = Magnetometer()

    localization = Localization()

    navigator = Navigator(
        WAYPOINTS
    )

    # -----------------------------------------------------
    # APPLY TEST PARAMETERS
    # -----------------------------------------------------

    navigator.cruise_speed = (
        CRUISE_SPEED
    )

    navigator.minimum_speed = (
        MINIMUM_SPEED
    )

    navigator.approach_distance = (
        APPROACH_DISTANCE
    )

    navigator.cross_track_gain = (
        cross_track_gain
    )

    navigator.lookahead_distance = (
        lookahead_distance
    )

    navigator.turn_speed = (
        turn_speed
    )

    navigator.turn_zone_distance = (
        turn_zone_distance
    )

    # -----------------------------------------------------
    # ERROR STORAGE
    # -----------------------------------------------------

    straight_errors = []

    turn_errors = []

    overall_errors = []

    # -----------------------------------------------------
    # MISSION STATE
    # -----------------------------------------------------

    previous_waypoint = (
        navigator.current_waypoint
    )

    max_steps = 10000

    # =====================================================
    # SIMULATION LOOP
    # =====================================================

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
        # ROBOT SPEED
        # -------------------------------------------------

        desired_speed = min(
            desired_speed,
            CRUISE_SPEED,
        )

        # -------------------------------------------------
        # ROBOT UPDATE
        # -------------------------------------------------

        robot.update(
            desired_heading,
            desired_speed,
            DT,
        )

        # -------------------------------------------------
        # CURRENT PATH SEGMENT
        # -------------------------------------------------

        current_waypoint = (
            navigator.current_waypoint
        )

        active_segment = (
            current_waypoint - 1
        )

        # -------------------------------------------------
        # CROSS-TRACK ERROR
        # -------------------------------------------------

        if (
            0 <= active_segment
            < len(WAYPOINTS) - 1
        ):
            error = (
                calculate_cross_track_error(
                    robot.x,
                    robot.y,
                    active_segment,
                )
            )

            overall_errors.append(
                error
            )

            if is_turn_zone(
                robot.x,
                robot.y,
                active_segment,
                turn_zone_distance,
            ):
                turn_errors.append(
                    error
                )
            else:
                straight_errors.append(
                    error
                )

        # -------------------------------------------------
        # WAYPOINT CHANGE
        # -------------------------------------------------

        if (
            current_waypoint
            != previous_waypoint
        ):
            previous_waypoint = (
                current_waypoint
            )

        # -------------------------------------------------
        # COMPLETE
        # -------------------------------------------------

        if complete:
            break

    # =====================================================
    # WAYPOINT RESULTS
    # =====================================================

    waypoint_errors = []

    if hasattr(
        navigator,
        "waypoint_errors",
    ):
        waypoint_errors = list(
            navigator.waypoint_errors
        )

    waypoint_count = (
        len(waypoint_errors)
    )

    expected_waypoints = (
        len(WAYPOINTS)
    )

    all_waypoints_reached = (
        waypoint_count
        >= expected_waypoints
    )

    if waypoint_errors:

        waypoint_average = (
            sum(waypoint_errors)
            / len(waypoint_errors)
        )

        waypoint_max = max(
            waypoint_errors
        )

        waypoint_p95 = (
            calculate_percentile(
                waypoint_errors,
                95,
            )
        )

    else:

        waypoint_average = float("inf")

        waypoint_max = float("inf")

        waypoint_p95 = float("inf")

    # =====================================================
    # METRICS
    # =====================================================

    overall_rms = calculate_rms(
        overall_errors
    )

    straight_rms = calculate_rms(
        straight_errors
    )

    turn_rms = calculate_rms(
        turn_errors
    )

    if straight_errors:

        straight_max = max(
            straight_errors
        )

        straight_p95 = (
            calculate_percentile(
                straight_errors,
                95,
            )
        )

    else:

        straight_max = float("inf")

        straight_p95 = float("inf")

    if turn_errors:

        turn_max = max(
            turn_errors
        )

        turn_p95 = (
            calculate_percentile(
                turn_errors,
                95,
            )
        )

    else:

        turn_max = float("inf")

        turn_p95 = float("inf")

    return {
        "cross_track_gain": cross_track_gain,

        "lookahead_distance": (
            lookahead_distance
        ),

        "turn_speed": turn_speed,

        "turn_zone_distance": (
            turn_zone_distance
        ),

        "overall_rms": overall_rms,

        "straight_rms": straight_rms,

        "straight_max": straight_max,

        "straight_p95": straight_p95,

        "turn_rms": turn_rms,

        "turn_max": turn_max,

        "turn_p95": turn_p95,

        "waypoint_average": (
            waypoint_average
        ),

        "waypoint_max": waypoint_max,

        "waypoint_p95": waypoint_p95,

        "waypoint_count": waypoint_count,

        "all_waypoints_reached": (
            all_waypoints_reached
        ),
    }


# =========================================================
# RUN MULTIPLE TRIALS
# =========================================================

def evaluate_configuration(
    cross_track_gain,
    lookahead_distance,
    turn_speed,
    turn_zone_distance,
    trials,
):
    results = []

    for trial in range(trials):

        # Same random conditions for every
        # configuration.
        random.seed(
            1000 + trial
        )

        result = run_mission(
            cross_track_gain,
            lookahead_distance,
            turn_speed,
            turn_zone_distance,
        )

        results.append(
            result
        )

    # -----------------------------------------------------
    # Average metrics across trials
    # -----------------------------------------------------

    return {
        "cross_track_gain": (
            cross_track_gain
        ),

        "lookahead_distance": (
            lookahead_distance
        ),

        "turn_speed": turn_speed,

        "turn_zone_distance": (
            turn_zone_distance
        ),

        "overall_rms": average(
            results,
            "overall_rms",
        ),

        "straight_rms": average(
            results,
            "straight_rms",
        ),

        "straight_max": average(
            results,
            "straight_max",
        ),

        "straight_p95": average(
            results,
            "straight_p95",
        ),

        "turn_rms": average(
            results,
            "turn_rms",
        ),

        "turn_max": average(
            results,
            "turn_max",
        ),

        "turn_p95": average(
            results,
            "turn_p95",
        ),

        "waypoint_average": average(
            results,
            "waypoint_average",
        ),

        "waypoint_max": average(
            results,
            "waypoint_max",
        ),

        "waypoint_p95": average(
            results,
            "waypoint_p95",
        ),

        "waypoint_count": average(
            results,
            "waypoint_count",
        ),

        "all_waypoints_reached": all(
            result[
                "all_waypoints_reached"
            ]
            for result in results
        ),
    }


# =========================================================
# AVERAGE HELPER
# =========================================================

def average(
    results,
    key,
):
    values = [
        result[key]
        for result in results
    ]

    if not values:
        return float("inf")

    return (
        sum(values)
        / len(values)
    )


# =========================================================
# PASS / FAIL
# =========================================================

def passes_all_targets(
    result,
):
    return (
        result["straight_rms"]
        <= STRAIGHT_RMS_TARGET

        and

        result["straight_max"]
        <= STRAIGHT_MAX_TARGET

        and

        result["turn_max"]
        <= TURN_MAX_TARGET

        and

        result["waypoint_max"]
        <= WAYPOINT_TARGET

        and

        result["all_waypoints_reached"]
    )


# =========================================================
# DISTANCE FROM TARGET
#
# Used when no configuration passes everything.
# =========================================================

def failure_score(
    result,
):
    straight_rms_penalty = max(
        0.0,
        result["straight_rms"]
        - STRAIGHT_RMS_TARGET,
    )

    straight_max_penalty = max(
        0.0,
        result["straight_max"]
        - STRAIGHT_MAX_TARGET,
    )

    turn_penalty = max(
        0.0,
        result["turn_max"]
        - TURN_MAX_TARGET,
    )

    waypoint_penalty = max(
        0.0,
        result["waypoint_max"]
        - WAYPOINT_TARGET,
    )

    waypoint_completion_penalty = (
        max(
            0,
            len(WAYPOINTS)
            - result["waypoint_count"],
        )
        * 2.0
    )

    return (
        straight_rms_penalty * 2.0
        +
        straight_max_penalty * 3.0
        +
        turn_penalty * 3.0
        +
        waypoint_penalty * 3.0
        +
        waypoint_completion_penalty
    )


# =========================================================
# RANK CONFIGURATIONS
# =========================================================

def rank_key(
    result,
):
    # -----------------------------------------------------
    # Passing configurations always beat failing ones.
    # -----------------------------------------------------

    if passes_all_targets(
        result
    ):
        pass_group = 0
    else:
        pass_group = 1

    return (
        pass_group,

        # For passing configurations:
        # minimize worst turn error first.
        result["turn_max"],

        result["straight_max"],

        result["waypoint_max"],

        result["straight_rms"],
    )


# =========================================================
# PRINT CONFIGURATION
# =========================================================

def print_configuration(
    result,
):
    print(
        f"Gain={result['cross_track_gain']:.2f} | "
        f"Lookahead={result['lookahead_distance']:.2f} | "
        f"TurnSpeed={result['turn_speed']:.2f} | "
        f"TurnZone={result['turn_zone_distance']:.2f}"
    )


# =========================================================
# PRINT RESULT
# =========================================================

def print_result(
    result,
):
    status = (
        "PASS"
        if passes_all_targets(
            result
        )
        else
        "FAIL"
    )

    print(
        f"{result['cross_track_gain']:<6.2f}"
        f"{result['lookahead_distance']:<12.2f}"
        f"{result['turn_speed']:<11.2f}"
        f"{result['turn_zone_distance']:<11.2f}"
        f"{result['straight_rms']:<13.3f}"
        f"{result['straight_max']:<13.3f}"
        f"{result['turn_max']:<11.3f}"
        f"{result['waypoint_max']:<13.3f}"
        f"{result['waypoint_count']:<9.2f}"
        f"{status}"
    )


# =========================================================
# GENERATE ALL CONFIGURATIONS
# =========================================================

def generate_configurations():
    configurations = []

    for gain in CROSS_TRACK_GAINS:

        for lookahead in LOOKAHEAD_DISTANCES:

            for turn_speed in TURN_SPEEDS:

                for turn_zone in TURN_ZONE_DISTANCES:

                    configurations.append(
                        (
                            gain,
                            lookahead,
                            turn_speed,
                            turn_zone,
                        )
                    )

    return configurations


# =========================================================
# MAIN
# =========================================================

def main():

    configurations = (
        generate_configurations()
    )

    total_configurations = len(
        configurations
    )

    print()

    print("=" * 100)

    print(
        "AUTOMATED PATH-FOLLOWING OPTIMIZER"
    )

    print("=" * 100)

    print()

    print(
        "The computer will automatically search "
        "the controller parameter space."
    )

    print()

    print(
        f"Configurations: "
        f"{total_configurations}"
    )

    print(
        f"Screening trials/configuration: "
        f"{SCREENING_TRIALS}"
    )

    print(
        f"Confirmation trials: "
        f"{CONFIRMATION_TRIALS}"
    )

    print()

    print(
        "FIXED PARAMETERS"
    )

    print("-" * 100)

    print(
        f"Cruise speed:       "
        f"{CRUISE_SPEED:.2f} m/s"
    )

    print(
        f"Minimum speed:      "
        f"{MINIMUM_SPEED:.2f} m/s"
    )

    print(
        f"Approach distance:  "
        f"{APPROACH_DISTANCE:.2f} m"
    )

    print()

    print(
        "TARGETS"
    )

    print("-" * 100)

    print(
        f"Straight RMS <= "
        f"{STRAIGHT_RMS_TARGET:.2f} m"
    )

    print(
        f"Straight MAX <= "
        f"{STRAIGHT_MAX_TARGET:.2f} m"
    )

    print(
        f"Turn MAX <= "
        f"{TURN_MAX_TARGET:.2f} m"
    )

    print(
        f"Waypoint MAX <= "
        f"{WAYPOINT_TARGET:.2f} m"
    )

    print(
        "All waypoints must be reached."
    )

    # =====================================================
    # SCREENING
    # =====================================================

    print()

    print("=" * 100)

    print(
        "STAGE 1 — AUTOMATED SCREENING"
    )

    print("=" * 100)

    print()

    screening_results = []

    for index, configuration in enumerate(
        configurations,
        start=1,
    ):

        (
            gain,
            lookahead,
            turn_speed,
            turn_zone,
        ) = configuration

        result = evaluate_configuration(
            gain,
            lookahead,
            turn_speed,
            turn_zone,
            SCREENING_TRIALS,
        )

        screening_results.append(
            result
        )

        if (
            index == 1
            or index % 25 == 0
            or passes_all_targets(
                result
            )
        ):

            print(
                f"Screened "
                f"{index}/"
                f"{total_configurations}"
            )

            if passes_all_targets(
                result
            ):
                print(
                    "  >>> PASSING CONFIGURATION FOUND"
                )

                print_configuration(
                    result
                )

    # =====================================================
    # RANK SCREENING
    # =====================================================

    screening_results.sort(
        key=rank_key
    )

    top_candidates = (
        screening_results[
            :TOP_CANDIDATES
        ]
    )

    # =====================================================
    # SHOW TOP SCREENING RESULTS
    # =====================================================

    print()

    print("=" * 100)

    print(
        "TOP SCREENING CONFIGURATIONS"
    )

    print("=" * 100)

    print()

    print(
        f"{'GAIN':<6}"
        f"{'LOOK':<12}"
        f"{'TURN SPD':<11}"
        f"{'TURN ZONE':<11}"
        f"{'STRAIGHT RMS':<13}"
        f"{'STRAIGHT MAX':<13}"
        f"{'TURN MAX':<11}"
        f"{'WAYPOINT':<13}"
        f"{'WP COUNT':<9}"
        f"STATUS"
    )

    print("-" * 100)

    for result in top_candidates:

        print_result(
            result
        )

    # =====================================================
    # CONFIRMATION
    # =====================================================

    print()

    print("=" * 100)

    print(
        "STAGE 2 — CONFIRMATION"
    )

    print("=" * 100)

    print()

    confirmed_results = []

    for index, candidate in enumerate(
        top_candidates,
        start=1,
    ):

        print(
            f"Confirming candidate "
            f"{index}/"
            f"{len(top_candidates)}:"
        )

        print_configuration(
            candidate
        )

        confirmed = evaluate_configuration(
            candidate[
                "cross_track_gain"
            ],

            candidate[
                "lookahead_distance"
            ],

            candidate[
                "turn_speed"
            ],

            candidate[
                "turn_zone_distance"
            ],

            CONFIRMATION_TRIALS,
        )

        confirmed_results.append(
            confirmed
        )

        print(
            f"  Straight RMS: "
            f"{confirmed['straight_rms']:.3f} m"
        )

        print(
            f"  Straight MAX: "
            f"{confirmed['straight_max']:.3f} m"
        )

        print(
            f"  Turn MAX:     "
            f"{confirmed['turn_max']:.3f} m"
        )

        print(
            f"  Waypoint MAX: "
            f"{confirmed['waypoint_max']:.3f} m"
        )

        print(
            f"  Waypoints:    "
            f"{confirmed['waypoint_count']:.2f}"
        )

        if passes_all_targets(
            confirmed
        ):
            print(
                "  >>> ALL TARGETS PASS"
            )
        else:
            print(
                "  >>> FAIL"
            )

        print()

    # =====================================================
    # FINAL RANKING
    # =====================================================

    confirmed_results.sort(
        key=rank_key
    )

    best = confirmed_results[0]

    # =====================================================
    # FINAL RESULTS
    # =====================================================

    print()

    print("=" * 100)

    print(
        "FINAL OPTIMIZATION RESULTS"
    )

    print("=" * 100)

    print()

    print(
        f"{'GAIN':<6}"
        f"{'LOOK':<12}"
        f"{'TURN SPD':<11}"
        f"{'TURN ZONE':<11}"
        f"{'STRAIGHT RMS':<13}"
        f"{'STRAIGHT MAX':<13}"
        f"{'TURN MAX':<11}"
        f"{'WAYPOINT':<13}"
        f"{'WP COUNT':<9}"
        f"STATUS"
    )

    print("-" * 100)

    for result in confirmed_results:

        print_result(
            result
        )

    # =====================================================
    # WINNER
    # =====================================================

    print()

    print("=" * 100)

    print(
        "BEST CONFIGURATION"
    )

    print("=" * 100)

    print()

    print_configuration(
        best
    )

    print()

    print(
        f"Straight RMS: "
        f"{best['straight_rms']:.3f} m"
    )

    print(
        f"Straight MAX: "
        f"{best['straight_max']:.3f} m"
    )

    print(
        f"Turn RMS:     "
        f"{best['turn_rms']:.3f} m"
    )

    print(
        f"Turn MAX:     "
        f"{best['turn_max']:.3f} m"
    )

    print(
        f"Waypoint Avg: "
        f"{best['waypoint_average']:.3f} m"
    )

    print(
        f"Waypoint MAX: "
        f"{best['waypoint_max']:.3f} m"
    )

    print(
        f"Waypoint 95%: "
        f"{best['waypoint_p95']:.3f} m"
    )

    print(
        f"Waypoints reached: "
        f"{best['waypoint_count']:.2f}"
    )

    print()

    # =====================================================
    # FINAL PASS/FAIL
    # =====================================================

    if passes_all_targets(
        best
    ):

        print("=" * 100)

        print(
            "ALL TARGETS PASSED"
        )

        print("=" * 100)

        print()

        print(
            "Use these parameters in navigation.py:"
        )

        print()

        print(
            f"self.cross_track_gain = "
            f"{best['cross_track_gain']:.2f}"
        )

        print(
            f"self.lookahead_distance = "
            f"{best['lookahead_distance']:.2f}"
        )

        print(
            f"self.turn_speed = "
            f"{best['turn_speed']:.2f}"
        )

        print(
            f"self.turn_zone_distance = "
            f"{best['turn_zone_distance']:.2f}"
        )

        print()

        print(
            "Do NOT change the controller manually."
        )

    else:

        print("=" * 100)

        print(
            "NO CONFIGURATION PASSED ALL TARGETS"
        )

        print("=" * 100)

        print()

        print(
            "Best configuration found:"
        )

        print_configuration(
            best
        )

        print()

        print(
            "The controller architecture needs "
            "improvement rather than simply more "
            "parameter tuning."
        )

    print()

    print("=" * 100)

    print(
        "OPTIMIZATION COMPLETE"
    )

    print("=" * 100)


# =========================================================
# ENTRY POINT
# =========================================================

if __name__ == "__main__":
    main()