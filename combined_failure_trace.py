import math
import random
import csv

from navigation import Navigator
from robot import Robot
from sensors import GPSSensor, Gyroscope, Magnetometer
from localization import Localization


# =========================================================
# LOCKED PARAMETERS
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
# FAILURE CASE
# =========================================================

GPS_NOISE = 0.25
GPS_DROPOUT = 5.0
HEADING_NOISE = 5.0

TRIAL_NUMBER = 3

DROPOUT_START = 6.0


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
# MAIN TRACE
# =========================================================

def main():

    print()
    print("=" * 100)
    print("COMBINED SENSOR FAILURE TRACE")
    print("=" * 100)

    print()
    print(
        "GPS noise      : ±"
        f"{GPS_NOISE:.2f} m"
    )

    print(
        "GPS dropout    : "
        f"{GPS_DROPOUT:.1f} s"
    )

    print(
        "Heading noise   : ±"
        f"{HEADING_NOISE:.1f}°"
    )

    print(
        "Trial           : "
        f"{TRIAL_NUMBER}"
    )

    print(
        "Dropout starts  : "
        f"{DROPOUT_START:.1f} s"
    )

    print()
    print("LOCKED NAVIGATION")

    print(
        f"Gain            : "
        f"{CROSS_TRACK_GAIN:.2f}"
    )

    print(
        f"Lookahead       : "
        f"{LOOKAHEAD_DISTANCE:.2f} m"
    )

    print(
        f"Turn speed      : "
        f"{TURN_SPEED:.2f} m/s"
    )

    print(
        f"Turn zone       : "
        f"{TURN_ZONE_DISTANCE:.2f} m"
    )

    print()
    print(
        "NO controller parameters will be changed."
    )


    # =====================================================
    # CREATE SYSTEM
    # =====================================================

    (
        robot,
        gps,
        gyro,
        magnetometer,
        localization,
        navigator,
    ) = create_system()


    # =====================================================
    # RANDOM SEED
    # =====================================================

    random.seed(
        40000
        +
        TRIAL_NUMBER
    )


    # =====================================================
    # DROPOUT TIMING
    # =====================================================

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


    # =====================================================
    # TRACE
    # =====================================================

    trace = []

    waypoint_index = 1

    previous_x = robot.x
    previous_y = robot.y

    total_distance = 0.0

    max_steps = 10000

    divergence_step = None


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
        # LOCALIZATION ERROR
        # -------------------------------------------------

        localization_error = distance(
            robot.x,
            robot.y,
            localization.x,
            localization.y,
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
        # NAVIGATION STATE
        # -------------------------------------------------

        active_segment = (
            navigator.current_waypoint - 1
        )

        if (
            active_segment < 0
        ):

            active_segment = 0

        if (
            active_segment
            >= len(WAYPOINTS) - 1
        ):

            active_segment = (
                len(WAYPOINTS) - 2
            )


        # -------------------------------------------------
        # TRUE CROSS-TRACK ERROR
        # -------------------------------------------------

        true_cross_track = (
            segment_error(
                robot.x,
                robot.y,
                active_segment,
            )
        )


        # -------------------------------------------------
        # LOCALIZED CROSS-TRACK ERROR
        # -------------------------------------------------

        estimated_cross_track = (
            segment_error(
                localization.x,
                localization.y,
                active_segment,
            )
        )


        # -------------------------------------------------
        # HEADING ERROR
        # -------------------------------------------------

        heading_error = (
            desired_heading
            -
            true_heading
            +
            180.0
        ) % 360.0 - 180.0


        # -------------------------------------------------
        # WAYPOINT DISTANCE
        # -------------------------------------------------

        if (
            navigator.current_waypoint
            <
            len(WAYPOINTS)
        ):

            target_x, target_y = (
                WAYPOINTS[
                    navigator.current_waypoint
                ]
            )

            true_waypoint_distance = (
                distance(
                    robot.x,
                    robot.y,
                    target_x,
                    target_y,
                )
            )

            estimated_waypoint_distance = (
                distance(
                    localization.x,
                    localization.y,
                    target_x,
                    target_y,
                )
            )

        else:

            true_waypoint_distance = 0.0

            estimated_waypoint_distance = 0.0


        # -------------------------------------------------
        # RECORD
        # -------------------------------------------------

        row = {

            "step": step,

            "time": time,

            "gps_available": (
                gps_available
            ),

            "true_x": robot.x,
            "true_y": robot.y,

            "gps_x": gps_x,
            "gps_y": gps_y,

            "localized_x": (
                localization.x
            ),

            "localized_y": (
                localization.y
            ),

            "localization_error": (
                localization_error
            ),

            "true_heading": (
                true_heading
            ),

            "measured_heading": (
                measured_heading
            ),

            "localized_heading": (
                localization.heading
            ),

            "desired_heading": (
                desired_heading
            ),

            "heading_error": (
                heading_error
            ),

            "robot_speed": (
                robot.speed
            ),

            "desired_speed": (
                desired_speed
            ),

            "current_waypoint": (
                navigator.current_waypoint
            ),

            "active_segment": (
                active_segment
            ),

            "true_cross_track": (
                true_cross_track
            ),

            "estimated_cross_track": (
                estimated_cross_track
            ),

            "true_waypoint_distance": (
                true_waypoint_distance
            ),

            "estimated_waypoint_distance": (
                estimated_waypoint_distance
            ),
        }

        trace.append(row)


        # =================================================
        # DIAGNOSTIC OUTPUT
        # =================================================

        important = (

            not gps_available

            or

            localization_error > 0.75

            or

            true_cross_track > 2.0

            or

            estimated_cross_track > 2.0

            or

            abs(heading_error) > 20.0
        )


        if important:

            print(
                f"STEP {step:04d} "
                f"T={time:6.2f}s | "
                f"GPS={'ON ' if gps_available else 'OFF'} | "
                f"WP={navigator.current_waypoint} | "
                f"TRUE=({robot.x:7.2f},"
                f"{robot.y:7.2f}) | "
                f"LOC=({localization.x:7.2f},"
                f"{localization.y:7.2f}) | "
                f"LOC_ERR={localization_error:6.2f} | "
                f"TRUE_ERR={true_cross_track:6.2f} | "
                f"LOC_ERR_ROW={estimated_cross_track:6.2f} | "
                f"TRUE_HEAD={true_heading:7.2f} | "
                f"LOC_HEAD={localization.heading:7.2f} | "
                f"CMD_HEAD={desired_heading:7.2f} | "
                f"HEAD_ERR={heading_error:7.2f}"
            )


        # =================================================
        # ROBOT UPDATE
        # =================================================

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


        # =================================================
        # WAYPOINT CHECK
        # =================================================

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

                print()
                print(
                    f">>> TRUE WAYPOINT "
                    f"{waypoint_index} "
                    f"REACHED"
                )

                print(
                    f"    True position: "
                    f"({robot.x:.3f}, "
                    f"{robot.y:.3f})"
                )

                print(
                    f"    Localized: "
                    f"({localization.x:.3f}, "
                    f"{localization.y:.3f})"
                )

                print()

                waypoint_index += 1


        # =================================================
        # DIVERGENCE DETECTION
        # =================================================

        if (
            true_cross_track
            > 15.0
        ):

            divergence_step = step

            print()
            print("=" * 100)
            print(">>> FIRST MAJOR DIVERGENCE")
            print("=" * 100)

            print(
                f"Step              : "
                f"{step}"
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
                f"Current waypoint  : "
                f"{navigator.current_waypoint}"
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
                f"True cross-track  : "
                f"{true_cross_track:.3f} m"
            )

            print(
                f"Estimated cross   : "
                f"{estimated_cross_track:.3f} m"
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

            print(
                f"Robot speed       : "
                f"{robot.speed:.3f} m/s"
            )

            print()

            break


        # =================================================
        # MISSION COMPLETE
        # =================================================

        if navigator.complete:

            print()
            print(
                ">>> NAVIGATOR REPORTS "
                "MISSION COMPLETE"
            )

            break


    # =====================================================
    # SAVE TRACE
    # =====================================================

    with open(
        "combined_failure_trace.csv",
        "w",
        newline="",
    ) as file:

        writer = csv.DictWriter(
            file,
            fieldnames=trace[0].keys(),
        )

        writer.writeheader()

        writer.writerows(
            trace
        )


    # =====================================================
    # FINAL SUMMARY
    # =====================================================

    print()
    print("=" * 100)
    print("TRACE COMPLETE")
    print("=" * 100)

    print(
        f"Steps recorded : "
        f"{len(trace)}"
    )

    print(
        f"Final position : "
        f"({robot.x:.3f}, "
        f"{robot.y:.3f})"
    )

    print(
        f"Final heading  : "
        f"{robot.heading:.3f}°"
    )

    print(
        f"Waypoints      : "
        f"{navigator.current_waypoint - 1}/"
        f"{len(WAYPOINTS) - 1}"
    )

    print(
        f"Total distance : "
        f"{total_distance:.3f} m"
    )

    print(
        f"Divergence     : "
        f"{divergence_step}"
    )

    print()
    print(
        "Full trace saved to:"
    )

    print(
        "combined_failure_trace.csv"
    )

    print("=" * 100)


if __name__ == "__main__":
    main()