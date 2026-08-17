import math
import random

from navigation import Navigator
from robot import Robot
from sensors import GPSSensor, Gyroscope, Magnetometer
from localization import Localization


# =========================================================
# EXACT LOCKED CONFIGURATION
# =========================================================

CROSS_TRACK_GAIN = 0.70
LOOKAHEAD_DISTANCE = 4.50
TURN_SPEED = 0.50
TURN_ZONE_DISTANCE = 5.00

CRUISE_SPEED = 2.50
MINIMUM_SPEED = 1.20
APPROACH_DISTANCE = 8.00

DT = 0.10


# =========================================================
# EXACT TRIAL BEING DEBUGGED
# =========================================================

TRIAL_NUMBER = 4

GPS_NOISE = 0.25
DROPOUT_DURATION = 5.0
HEADING_NOISE = 5.0
DROPOUT_START = 6.0

DIVERGENCE_ERROR = 10.0
WAYPOINT_TOLERANCE = 0.50


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

    return distance(
        px,
        py,
        closest_x,
        closest_y,
    )


def segment_error(
    x,
    y,
    segment_index,
):
    if segment_index < 0:
        segment_index = 0

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
    # EXACT LOCKED NAVIGATION PARAMETERS
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
# RUN EXACT TRIAL 04
# =========================================================

def run_trial():

    (
        robot,
        gps,
        gyro,
        magnetometer,
        localization,
        navigator,
    ) = create_system()


    # -----------------------------------------------------
    # EXACT RANDOM SEED FROM ORIGINAL TEST
    # Trial 04 -> 40000 + 4 = 40004
    # -----------------------------------------------------

    random.seed(
        40000
        +
        TRIAL_NUMBER
    )


    # -----------------------------------------------------
    # DROPOUT STEPS
    # -----------------------------------------------------

    dropout_start_step = int(
        DROPOUT_START / DT
    )

    dropout_steps = int(
        DROPOUT_DURATION / DT
    )

    dropout_end_step = (
        dropout_start_step
        +
        dropout_steps
    )


    # -----------------------------------------------------
    # TEST STATE
    # -----------------------------------------------------

    waypoint_index = 1

    max_error = 0.0

    max_localization_error = 0.0

    max_dropout_error = 0.0

    previous_x = robot.x

    previous_y = robot.y

    total_distance = 0.0

    first_error_above_150 = None

    first_divergence = None

    previous_waypoint = (
        waypoint_index
    )


    # =====================================================
    # HEADER
    # =====================================================

    print()

    print("=" * 100)

    print(
        "EXACT TRIAL 04 DIAGNOSTIC"
    )

    print("=" * 100)

    print()

    print(
        "GPS noise       : ±0.25 m"
    )

    print(
        "GPS dropout     : 5.0 s"
    )

    print(
        "Dropout start   : 6.0 s"
    )

    print(
        "Heading noise   : ±5.0°"
    )

    print(
        "Trial           : 04"
    )

    print(
        "Random seed     : 40004"
    )

    print()

    print(
        "Diagnostic window: 4.0 s - 13.0 s"
    )

    print()

    print("=" * 100)


    # =====================================================
    # SIMULATION
    # =====================================================

    max_steps = 10000

    for step in range(max_steps):

        time = step * DT


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
        #
        # SAME RANDOM ORDER AS ORIGINAL TEST
        # -------------------------------------------------

        if gps_available:

            gps_x = (
                robot.x
                +
                random.uniform(
                    -GPS_NOISE,
                    GPS_NOISE,
                )
            )

            gps_y = (
                robot.y
                +
                random.uniform(
                    -GPS_NOISE,
                    GPS_NOISE,
                )
            )

        else:

            gps_x = localization.x

            gps_y = localization.y


        # -------------------------------------------------
        # HEADING
        #
        # SAME RANDOM CALL AS ORIGINAL TEST
        # -------------------------------------------------

        measured_heading = (
            robot.heading
            +
            random.uniform(
                -HEADING_NOISE,
                HEADING_NOISE,
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
        # DISTANCE TRAVELLED
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
        # WAYPOINT COUNTER
        # -------------------------------------------------

        if waypoint_index < len(WAYPOINTS):

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

                old_waypoint = (
                    waypoint_index
                )

                waypoint_index += 1

                print()

                print(
                    "-" * 100
                )

                print(
                    "WAYPOINT TRANSITION"
                )

                print(
                    "-" * 100
                )

                print(
                    f"Time              : "
                    f"{time:.2f} s"
                )

                print(
                    f"Waypoint changed  : "
                    f"{old_waypoint} -> "
                    f"{waypoint_index}"
                )

                print(
                    f"GPS available     : "
                    f"{gps_available}"
                )

                print()

                print(
                    f"True position     : "
                    f"({robot.x:.3f}, "
                    f"{robot.y:.3f})"
                )

                print(
                    f"Localized position: "
                    f"({localization.x:.3f}, "
                    f"{localization.y:.3f})"
                )

                print(
                    f"GPS position      : "
                    f"({gps_x:.3f}, "
                    f"{gps_y:.3f})"
                )

                print()

                print(
                    f"True heading      : "
                    f"{robot.heading:.3f}°"
                )

                print(
                    f"Localized heading : "
                    f"{localization.heading:.3f}°"
                )

                print(
                    f"Measured heading  : "
                    f"{measured_heading:.3f}°"
                )

                print(
                    f"Desired heading   : "
                    f"{desired_heading:.3f}°"
                )

                print()

                print(
                    f"Localization err  : "
                    f"{localization_error:.3f} m"
                )

                print(
                    "-" * 100
                )


        # -------------------------------------------------
        # ACTIVE PATH SEGMENT
        # -------------------------------------------------

        active_segment = (
            waypoint_index - 1
        )

        if active_segment < 0:

            active_segment = 0

        if (
            active_segment
            >=
            len(WAYPOINTS) - 1
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
        # DIAGNOSTIC WINDOW
        #
        # 4s -> 13s captures:
        #   4-6s  = pre-dropout
        #   6-11s = GPS dropout
        #   11-13 = recovery
        # -------------------------------------------------

        if (
            4.0
            <= time
            <= 13.0
        ):

            gps_state = (
                "ON "
                if gps_available
                else "OFF"
            )

            print(
                f"T={time:6.2f}s | "
                f"GPS={gps_state} | "
                f"WP={waypoint_index} | "
                f"TRUE=("
                f"{robot.x:7.2f},"
                f"{robot.y:7.2f}) | "
                f"LOC=("
                f"{localization.x:7.2f},"
                f"{localization.y:7.2f}) | "
                f"LOC_ERR="
                f"{localization_error:6.3f} | "
                f"PATH_ERR="
                f"{error:6.3f} | "
                f"TRUE_HEAD="
                f"{robot.heading:7.2f} | "
                f"LOC_HEAD="
                f"{localization.heading:7.2f} | "
                f"MEAS_HEAD="
                f"{measured_heading:7.2f} | "
                f"CMD_HEAD="
                f"{desired_heading:7.2f} | "
                f"CMD_SPEED="
                f"{desired_speed:6.2f}"
            )


        # -------------------------------------------------
        # FIRST ERROR ABOVE 1.50m
        # -------------------------------------------------

        if (
            first_error_above_150 is None
            and
            error >= 1.50
        ):

            first_error_above_150 = (
                step
            )

            print()

            print(
                "=" * 100
            )

            print(
                "FIRST PATH ERROR ABOVE 1.50 m"
            )

            print(
                "=" * 100
            )

            print(
                f"Time              : "
                f"{time:.2f} s"
            )

            print(
                f"GPS available     : "
                f"{gps_available}"
            )

            print(
                f"Waypoint          : "
                f"{waypoint_index}"
            )

            print(
                f"Active segment    : "
                f"{active_segment}"
            )

            print()

            print(
                f"True position     : "
                f"({robot.x:.3f}, "
                f"{robot.y:.3f})"
            )

            print(
                f"Localized position: "
                f"({localization.x:.3f}, "
                f"{localization.y:.3f})"
            )

            print(
                f"GPS position      : "
                f"({gps_x:.3f}, "
                f"{gps_y:.3f})"
            )

            print()

            print(
                f"True heading      : "
                f"{robot.heading:.3f}°"
            )

            print(
                f"Localized heading : "
                f"{localization.heading:.3f}°"
            )

            print(
                f"Measured heading  : "
                f"{measured_heading:.3f}°"
            )

            print(
                f"Desired heading   : "
                f"{desired_heading:.3f}°"
            )

            print()

            print(
                f"Localization err  : "
                f"{localization_error:.3f} m"
            )

            print(
                f"Path error        : "
                f"{error:.3f} m"
            )

            print(
                f"Robot speed       : "
                f"{robot.speed:.3f} m/s"
            )

            print(
                f"Command speed     : "
                f"{desired_speed:.3f} m/s"
            )

            print(
                "=" * 100
            )


        # -------------------------------------------------
        # FIRST TRUE DIVERGENCE
        # -------------------------------------------------

        if (
            first_divergence is None
            and
            error >= DIVERGENCE_ERROR
        ):

            first_divergence = step

            print()

            print(
                "=" * 100
            )

            print(
                "FIRST TRUE NAVIGATION DIVERGENCE"
            )

            print(
                "=" * 100
            )

            print(
                f"Time              : "
                f"{time:.2f} s"
            )

            print(
                f"Step              : "
                f"{step}"
            )

            print(
                f"GPS available     : "
                f"{gps_available}"
            )

            print(
                f"Waypoint          : "
                f"{waypoint_index}"
            )

            print(
                f"Active segment    : "
                f"{active_segment}"
            )

            print()

            print(
                f"True position     : "
                f"({robot.x:.3f}, "
                f"{robot.y:.3f})"
            )

            print(
                f"Localized position: "
                f"({localization.x:.3f}, "
                f"{localization.y:.3f})"
            )

            print(
                f"GPS position      : "
                f"({gps_x:.3f}, "
                f"{gps_y:.3f})"
            )

            print()

            print(
                f"True heading      : "
                f"{robot.heading:.3f}°"
            )

            print(
                f"Localized heading : "
                f"{localization.heading:.3f}°"
            )

            print(
                f"Measured heading  : "
                f"{measured_heading:.3f}°"
            )

            print(
                f"Desired heading   : "
                f"{desired_heading:.3f}°"
            )

            print()

            print(
                f"Localization err  : "
                f"{localization_error:.3f} m"
            )

            print(
                f"Path error        : "
                f"{error:.3f} m"
            )

            print(
                f"Maximum path error: "
                f"{max_error:.3f} m"
            )

            print()

            print(
                f"Robot speed       : "
                f"{robot.speed:.3f} m/s"
            )

            print(
                f"Command speed     : "
                f"{desired_speed:.3f} m/s"
            )

            print(
                "=" * 100
            )

            break


        # -------------------------------------------------
        # MISSION COMPLETE
        # -------------------------------------------------

        if complete or navigator.complete:

            print()

            print(
                "=" * 100
            )

            print(
                "MISSION COMPLETE"
            )

            print(
                "=" * 100
            )

            break


    # =====================================================
    # FINAL RESULTS
    # =====================================================

    print()

    print(
        "=" * 100
    )

    print(
        "TRIAL 04 DIAGNOSTIC COMPLETE"
    )

    print(
        "=" * 100
    )

    print()

    print(
        f"Final position     : "
        f"({robot.x:.3f}, "
        f"{robot.y:.3f})"
    )

    print(
        f"Localized position: "
        f"({localization.x:.3f}, "
        f"{localization.y:.3f})"
    )

    print(
        f"Final waypoint     : "
        f"{waypoint_index}/"
        f"{len(WAYPOINTS)}"
    )

    print(
        f"Maximum path error : "
        f"{max_error:.3f} m"
    )

    print(
        f"Maximum loc error  : "
        f"{max_localization_error:.3f} m"
    )

    print(
        f"Maximum dropout    : "
        f"{max_dropout_error:.3f} m"
    )

    print(
        f"Total distance     : "
        f"{total_distance:.3f} m"
    )

    if first_error_above_150 is None:

        print(
            "First >1.50 m      : "
            "NOT REACHED"
        )

    else:

        print(
            "First >1.50 m      : "
            f"{first_error_above_150 * DT:.2f} s"
        )

    if first_divergence is None:

        print(
            "First >=10.00 m    : "
            "NOT REACHED"
        )

    else:

        print(
            "First >=10.00 m    : "
            f"{first_divergence * DT:.2f} s"
        )

    print()

    print(
        "=" * 100
    )


# =========================================================
# START
# =========================================================

if __name__ == "__main__":
    run_trial()