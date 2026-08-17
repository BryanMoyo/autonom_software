import math
import random

from navigation import Navigator
from robot import Robot
from sensors import GPSSensor, Gyroscope, Magnetometer
from localization import Localization


# =========================================================
# TEST CONFIGURATION
# =========================================================

GPS_NOISE = 0.25
GPS_DROPOUT = 5.0
HEADING_NOISE = 5.0

TRIAL_NUMBER = 4

DROPOUT_START = 6.0

DT = 0.10


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
# RUN DIAGNOSTIC
# =========================================================

def run_diagnostic():

    (
        robot,
        gps,
        gyro,
        magnetometer,
        localization,
        navigator,
    ) = create_system()

    # Reproduce the same deterministic trial
    # from the combined-sensor test.
    random.seed(
        40000 + TRIAL_NUMBER
    )

    dropout_start_step = int(
        DROPOUT_START / DT
    )

    dropout_end_step = (
        dropout_start_step
        +
        int(
            GPS_DROPOUT / DT
        )
    )

    waypoint_index = 1

    previous_waypoint = (
        navigator.current_waypoint
    )

    max_error = 0.0

    max_steps = 10000

    print()
    print("=" * 100)
    print("WAYPOINT TRANSITION DIAGNOSTIC")
    print("=" * 100)

    print()
    print(
        f"GPS noise     : ±{GPS_NOISE:.2f} m"
    )

    print(
        f"GPS dropout   : {GPS_DROPOUT:.1f} s"
    )

    print(
        f"Heading noise : ±{HEADING_NOISE:.1f}°"
    )

    print(
        f"Trial         : {TRIAL_NUMBER}"
    )

    print()
    print(
        "Watching waypoint transitions..."
    )

    print()


    # =====================================================
    # SIMULATION
    # =====================================================

    for step in range(max_steps):

        time = step * DT

        true_x = robot.x
        true_y = robot.y
        true_heading = robot.heading


        # -------------------------------------------------
        # GPS STATE
        # -------------------------------------------------

        gps_available = not (
            dropout_start_step
            <= step
            <
            dropout_end_step
        )


        # -------------------------------------------------
        # GPS MEASUREMENT
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
        # HEADING MEASUREMENT
        # -------------------------------------------------

        measured_heading = (
            true_heading
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
        # CURRENT NAVIGATION STATE
        # -------------------------------------------------

        current_waypoint = (
            navigator.current_waypoint
        )

        active_segment = (
            current_waypoint - 1
        )

        active_segment = max(
            0,
            min(
                active_segment,
                len(WAYPOINTS) - 2,
            ),
        )


        # -------------------------------------------------
        # ERRORS
        # -------------------------------------------------

        localization_error = distance(
            robot.x,
            robot.y,
            localization.x,
            localization.y,
        )

        true_path_error = (
            segment_error(
                robot.x,
                robot.y,
                active_segment,
            )
        )

        estimated_path_error = (
            segment_error(
                localization.x,
                localization.y,
                active_segment,
            )
        )

        heading_error = (
            desired_heading
            -
            true_heading
            +
            180
        ) % 360 - 180


        max_error = max(
            max_error,
            true_path_error,
        )


        # =================================================
        # WAYPOINT TRANSITION DETECTION
        # =================================================

        if (
            current_waypoint
            != previous_waypoint
        ):

            print()
            print("=" * 100)
            print("WAYPOINT TRANSITION")
            print("=" * 100)

            print(
                f"Time              : "
                f"{time:.2f} s"
            )

            print(
                f"Waypoint changed  : "
                f"{previous_waypoint}"
                f" -> "
                f"{current_waypoint}"
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
                f"{true_heading:.3f}°"
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

            print(
                f"Heading error     : "
                f"{heading_error:.3f}°"
            )

            print()

            print(
                f"True path error   : "
                f"{true_path_error:.3f} m"
            )

            print(
                f"Estimated path    : "
                f"{estimated_path_error:.3f} m"
            )

            print(
                f"Localization err  : "
                f"{localization_error:.3f} m"
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

            print("=" * 100)
            print()

            previous_waypoint = (
                current_waypoint
            )


        # =================================================
        # PRINT AROUND WAYPOINT 2
        # =================================================

        if (
            current_waypoint == 2
            and
            28.0 <= robot.x <= 31.0
            and
            28.0 <= robot.y <= 31.0
        ):

            print(
                f"T={time:6.2f}s | "
                f"GPS={'ON ' if gps_available else 'OFF'} | "
                f"WP={current_waypoint} | "
                f"TRUE=({robot.x:6.2f},"
                f"{robot.y:6.2f}) | "
                f"LOC=({localization.x:6.2f},"
                f"{localization.y:6.2f}) | "
                f"TRUE_ERR={true_path_error:6.2f} | "
                f"LOC_ERR={localization_error:6.2f} | "
                f"HEAD={true_heading:6.1f} | "
                f"CMD={desired_heading:6.1f}"
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
        # STOP ON LARGE DIVERGENCE
        # -------------------------------------------------

        if true_path_error >= 10.0:

            print()
            print("=" * 100)
            print("FIRST DIVERGENCE DETECTED")
            print("=" * 100)

            print(
                f"Time              : "
                f"{time:.2f} s"
            )

            print(
                f"Waypoint          : "
                f"{current_waypoint}"
            )

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
                f"Localization err  : "
                f"{localization_error:.3f} m"
            )

            print(
                f"True path error   : "
                f"{true_path_error:.3f} m"
            )

            print(
                f"Estimated path    : "
                f"{estimated_path_error:.3f} m"
            )

            print(
                f"True heading      : "
                f"{true_heading:.3f}°"
            )

            print(
                f"Localized heading : "
                f"{localization.heading:.3f}°"
            )

            print(
                f"Desired heading   : "
                f"{desired_heading:.3f}°"
            )

            print(
                f"Heading error     : "
                f"{heading_error:.3f}°"
            )

            print("=" * 100)

            break


        # -------------------------------------------------
        # MISSION COMPLETE
        # -------------------------------------------------

        if navigator.complete:

            print()
            print(
                "MISSION COMPLETE"
            )

            break


    # =====================================================
    # FINAL SUMMARY
    # =====================================================

    print()
    print("=" * 100)
    print("DIAGNOSTIC COMPLETE")
    print("=" * 100)

    print(
        f"Final position     : "
        f"({robot.x:.3f}, "
        f"{robot.y:.3f})"
    )

    print(
        f"Final waypoint     : "
        f"{navigator.current_waypoint}"
    )

    print(
        f"Maximum path error : "
        f"{max_error:.3f} m"
    )

    print("=" * 100)


# =========================================================
# RUN
# =========================================================

if __name__ == "__main__":

    run_diagnostic()