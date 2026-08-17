import math
import random
import csv

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


# =========================================================
# FAILURE CASE
# =========================================================

GPS_NOISE = 0.45

TRIAL_NUMBER = 23

WAYPOINT_TOLERANCE = 0.50

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
        dx * dx +
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
    if segment < 0:
        segment = 0

    if segment >= len(WAYPOINTS) - 1:
        segment = len(WAYPOINTS) - 2

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
# TRACE
# =========================================================

def main():

    print()
    print("=" * 90)
    print("GPS FAILURE TRACE")
    print("=" * 90)

    print()
    print(
        f"GPS noise      : ±{GPS_NOISE:.2f} m"
    )

    print(
        f"Trial          : {TRIAL_NUMBER}"
    )

    print()
    print("LOCKED CONTROLLER")
    print(
        f"Gain           : {CROSS_TRACK_GAIN:.2f}"
    )
    print(
        f"Lookahead      : {LOOKAHEAD_DISTANCE:.2f} m"
    )
    print(
        f"Turn speed     : {TURN_SPEED:.2f} m/s"
    )
    print(
        f"Turn zone      : {TURN_ZONE_DISTANCE:.2f} m"
    )

    print()
    print(
        "No controller parameters will be changed."
    )

    (
        robot,
        gps,
        gyro,
        magnetometer,
        localization,
        navigator,
    ) = create_system()


    # =====================================================
    # CRITICAL:
    # SAME RANDOM SEED AS BOUNDARY TEST
    # =====================================================

    random.seed(
        20000 + TRIAL_NUMBER
    )


    waypoint_index = 1

    total_distance = 0.0

    previous_x = robot.x
    previous_y = robot.y

    trace = []

    max_steps = 10000

    failure_step = None


    # =====================================================
    # SIMULATION
    # =====================================================

    for step in range(max_steps):

        true_x = robot.x
        true_y = robot.y
        true_heading = robot.heading


        # -------------------------------------------------
        # GPS NOISE
        # -------------------------------------------------

        gps_x = (
            true_x
            +
            random.gauss(
                0.0,
                GPS_NOISE,
            )
        )

        gps_y = (
            true_y
            +
            random.gauss(
                0.0,
                GPS_NOISE,
            )
        )


        # -------------------------------------------------
        # OTHER SENSORS
        # -------------------------------------------------

        gyro_reading = gyro.read(
            0.0
        )

        magnetometer_heading = (
            magnetometer.read(
                true_heading
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
        # CURRENT SEGMENT
        # -------------------------------------------------

        active_segment = (
            waypoint_index - 1
        )

        true_cross_track = (
            segment_error(
                robot.x,
                robot.y,
                active_segment,
            )
        )

        estimated_cross_track = (
            segment_error(
                localization.x,
                localization.y,
                active_segment,
            )
        )


        # -------------------------------------------------
        # WAYPOINT DISTANCE
        # -------------------------------------------------

        if waypoint_index < len(WAYPOINTS):

            target_x, target_y = (
                WAYPOINTS[
                    waypoint_index
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
        # RECORD BEFORE ROBOT MOVES
        # -------------------------------------------------

        row = {
            "step": step,

            "true_x": robot.x,
            "true_y": robot.y,

            "gps_x": gps_x,
            "gps_y": gps_y,

            "localized_x": localization.x,
            "localized_y": localization.y,

            "true_heading": true_heading,

            "desired_heading": (
                desired_heading
            ),

            "heading_error": (
                heading_error
            ),

            "desired_speed": (
                desired_speed
            ),

            "active_segment": (
                active_segment
            ),

            "waypoint_index": (
                waypoint_index
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

            "localization_position_error": (
                distance(
                    robot.x,
                    robot.y,
                    localization.x,
                    localization.y,
                )
            ),
        }

        trace.append(row)


        # -------------------------------------------------
        # PRINT IMPORTANT REGION
        #
        # Normally quiet.
        # Once error begins growing, print every step.
        # -------------------------------------------------

        if (
            true_cross_track > 2.0
            or
            estimated_cross_track > 2.0
            or
            abs(heading_error) > 45.0
            or
            row["localization_position_error"] > 1.5
        ):

            print(
                f"STEP {step:04d} | "
                f"TRUE=({robot.x:7.2f},{robot.y:7.2f}) | "
                f"GPS=({gps_x:7.2f},{gps_y:7.2f}) | "
                f"LOC=({localization.x:7.2f},{localization.y:7.2f}) | "
                f"SEG={active_segment} | "
                f"TRUE_ERR={true_cross_track:7.2f} | "
                f"LOC_ERR={estimated_cross_track:7.2f} | "
                f"HEAD_ERR={heading_error:7.2f} | "
                f"CMD={desired_heading:7.2f}"
            )


        # -------------------------------------------------
        # ROBOT UPDATE
        # -------------------------------------------------

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
        # WAYPOINT ARRIVAL
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

                print()
                print(
                    f">>> WAYPOINT "
                    f"{waypoint_index} "
                    f"REACHED"
                )

                print(
                    f"    Position = "
                    f"({robot.x:.3f}, "
                    f"{robot.y:.3f})"
                )

                print(
                    f"    Error = "
                    f"{waypoint_distance:.3f} m"
                )

                print()

                waypoint_index += 1


        # -------------------------------------------------
        # FAILURE DETECTION
        # -------------------------------------------------

        if true_cross_track > 100.0:

            failure_step = step

            print()
            print("=" * 90)
            print(
                ">>> DIVERGENCE DETECTED"
            )
            print("=" * 90)

            print(
                f"Step              : {step}"
            )

            print(
                f"True position     : "
                f"({robot.x:.3f}, "
                f"{robot.y:.3f})"
            )

            print(
                f"GPS position      : "
                f"({gps_x:.3f}, "
                f"{gps_y:.3f})"
            )

            print(
                f"Localized position: "
                f"({localization.x:.3f}, "
                f"{localization.y:.3f})"
            )

            print(
                f"True cross-track  : "
                f"{true_cross_track:.3f} m"
            )

            print(
                f"Estimated error   : "
                f"{estimated_cross_track:.3f} m"
            )

            print(
                f"Desired heading   : "
                f"{desired_heading:.3f}°"
            )

            print(
                f"Actual heading    : "
                f"{robot.heading:.3f}°"
            )

            print(
                f"Heading error     : "
                f"{heading_error:.3f}°"
            )

            print(
                f"Waypoint          : "
                f"{waypoint_index}/"
                f"{len(WAYPOINTS) - 1}"
            )

            break


        # -------------------------------------------------
        # COMPLETE
        # -------------------------------------------------

        if (
            waypoint_index
            >= len(WAYPOINTS)
        ):

            print()
            print(
                ">>> MISSION COMPLETED"
            )

            break


    # =====================================================
    # SAVE COMPLETE TRACE
    # =====================================================

    with open(
        "gps_failure_trace.csv",
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
    # SUMMARY
    # =====================================================

    print()
    print("=" * 90)
    print("TRACE COMPLETE")
    print("=" * 90)

    print(
        f"Steps recorded : {len(trace)}"
    )

    print(
        f"Waypoints      : "
        f"{waypoint_index - 1}/"
        f"{len(WAYPOINTS) - 1}"
    )

    print(
        f"Final position  : "
        f"({robot.x:.3f}, "
        f"{robot.y:.3f})"
    )

    print(
        f"Final heading   : "
        f"{robot.heading:.3f}°"
    )

    print(
        f"Total distance  : "
        f"{total_distance:.3f} m"
    )

    if failure_step is not None:

        print(
            f"Failure step    : "
            f"{failure_step}"
        )

    else:

        print(
            "Failure step    : none"
        )

    print()
    print(
        "Full trace saved to:"
    )

    print(
        "gps_failure_trace.csv"
    )

    print("=" * 90)


if __name__ == "__main__":
    main()