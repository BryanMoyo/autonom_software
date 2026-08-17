import pygame
import math

from robot import Robot
from sensors import GPSSensor, Gyroscope, Magnetometer
from localization import Localization
from navigation import Navigator
from controller import Controller


# =================================================
# INITIALIZE
# =================================================

pygame.init()

WIDTH = 1100
HEIGHT = 750

screen = pygame.display.set_mode((WIDTH, HEIGHT))

pygame.display.set_caption(
    "Autonom Navigation Simulator"
)

title_font = pygame.font.Font(None, 30)
section_font = pygame.font.Font(None, 24)
data_font = pygame.font.Font(None, 19)
small_font = pygame.font.Font(None, 16)


# =================================================
# FIELD
# =================================================

FIELD_WIDTH_METERS = 100
FIELD_HEIGHT_METERS = 100

SCALE = 5

FIELD_WIDTH = FIELD_WIDTH_METERS * SCALE
FIELD_HEIGHT = FIELD_HEIGHT_METERS * SCALE

FIELD_X = 40
FIELD_Y = 40


# =================================================
# WAYPOINTS
# =================================================

waypoints = [
    (10.0, 10.0),
    (80.0, 10.0),
    (80.0, 40.0),
    (20.0, 40.0),
    (20.0, 80.0),
]


# =================================================
# SYSTEM COMPONENTS
# =================================================

robot = Robot()

gps = GPSSensor()

gyro = Gyroscope()

magnetometer = Magnetometer()

localization = Localization(
    initial_x=robot.x,
    initial_y=robot.y,
)

navigator = Navigator(
    waypoints
)

controller = Controller()


# =================================================
# GRAPH DATA
# =================================================

true_distance_history = []

estimated_distance_history = []

MAX_GRAPH_POINTS = 300


# =================================================
# MISSION PERFORMANCE
# =================================================

waypoint_errors = []

previous_waypoint = navigator.current_waypoint


# =================================================
# SIMULATION
# =================================================

running = True

previous_heading = robot.heading


while running:

    dt = 1 / 60


    # =================================================
    # EVENTS
    # =================================================

    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False


    # =================================================
    # GPS
    # =================================================

    gps_x, gps_y = gps.read(
        robot.x,
        robot.y,
    )


    # =================================================
    # GYROSCOPE
    # =================================================

    heading_change = (
        robot.heading
        - previous_heading
    )

    if heading_change > 180:
        heading_change -= 360

    if heading_change < -180:
        heading_change += 360

    true_rotation_rate = (
        heading_change / dt
    )

    gyro_reading = gyro.read(
        true_rotation_rate
    )


    # =================================================
    # MAGNETOMETER
    # =================================================

    magnetometer_reading = magnetometer.read(
        robot.heading
    )


    # =================================================
    # LOCALIZATION
    # =================================================

    localization.update(
        gps_x,
        gps_y,
        gyro_reading,
        magnetometer_reading,
        dt,
        robot.speed,
    )


    # =================================================
    # NAVIGATION
    # =================================================

    (
        desired_heading,
        desired_speed,
        mission_complete,
    ) = navigator.update(
        localization.x,
        localization.y,
    )


    # =================================================
    # WAYPOINT CHANGE DETECTION
    # =================================================

    current_waypoint = navigator.current_waypoint

    if current_waypoint != previous_waypoint:

        completed_waypoint = previous_waypoint

        if completed_waypoint < len(waypoints):

            completed_x, completed_y = waypoints[
                completed_waypoint
            ]

            true_dx = (
                completed_x
                - robot.x
            )

            true_dy = (
                completed_y
                - robot.y
            )

            true_error = math.sqrt(
                true_dx ** 2
                + true_dy ** 2
            )

            waypoint_errors.append(
                true_error
            )

        true_distance_history.clear()

        estimated_distance_history.clear()

        previous_waypoint = current_waypoint


    # =================================================
    # CURRENT TARGET
    # =================================================

    if current_waypoint < len(waypoints):

        target_x, target_y = waypoints[
            current_waypoint
        ]

        # TRUE DISTANCE

        true_dx = (
            target_x
            - robot.x
        )

        true_dy = (
            target_y
            - robot.y
        )

        true_distance_to_target = math.sqrt(
            true_dx ** 2
            + true_dy ** 2
        )

        # ESTIMATED DISTANCE

        estimated_dx = (
            target_x
            - localization.x
        )

        estimated_dy = (
            target_y
            - localization.y
        )

        estimated_distance_to_target = math.sqrt(
            estimated_dx ** 2
            + estimated_dy ** 2
        )

    else:

        true_distance_to_target = 0.0

        estimated_distance_to_target = 0.0


    # =================================================
    # DISTANCE GRAPH DATA
    # =================================================

    true_distance_history.append(
        true_distance_to_target
    )

    estimated_distance_history.append(
        estimated_distance_to_target
    )


    if len(true_distance_history) > MAX_GRAPH_POINTS:

        true_distance_history.pop(0)


    if len(estimated_distance_history) > MAX_GRAPH_POINTS:

        estimated_distance_history.pop(0)


    # =================================================
    # POSITION ERROR
    # =================================================

    position_error_x = (
        robot.x
        - localization.x
    )

    position_error_y = (
        robot.y
        - localization.y
    )

    position_error = math.sqrt(
        position_error_x ** 2
        + position_error_y ** 2
    )


    # =================================================
    # PERFORMANCE STATISTICS
    # =================================================

    if waypoint_errors:

        average_error = (
            sum(waypoint_errors)
            / len(waypoint_errors)
        )

        maximum_error = max(
            waypoint_errors
        )

    else:

        average_error = 0.0

        maximum_error = 0.0


    # =================================================
    # CONTROLLER
    # =================================================

    controller.set_command(
        desired_heading,
        desired_speed,
    )

    (
        commanded_heading,
        commanded_speed,
    ) = controller.get_command()


    # =================================================
    # ROBOT MOVEMENT
    # =================================================

    robot.update(
        commanded_heading,
        commanded_speed,
        dt,
    )

    previous_heading = robot.heading


    # =================================================
    # DRAW BACKGROUND
    # =================================================

    screen.fill("white")


    # =================================================
    # LEFT SIDE — FIELD
    # =================================================

    field_rect = pygame.Rect(
        FIELD_X,
        FIELD_Y,
        FIELD_WIDTH,
        FIELD_HEIGHT,
    )

    pygame.draw.rect(
        screen,
        "black",
        field_rect,
        2,
    )


    # =================================================
    # FIELD AXIS LABELS
    # =================================================

    x_label = small_font.render(
        "[m]",
        True,
        "black",
    )

    screen.blit(
        x_label,
        (
            FIELD_X,
            FIELD_Y + FIELD_HEIGHT + 5,
        ),
    )

    y_label = small_font.render(
        "[m]",
        True,
        "black",
    )

    screen.blit(
        y_label,
        (
            FIELD_X - 25,
            FIELD_Y - 5,
        ),
    )


    # =================================================
    # WAYPOINT ROUTE
    # =================================================

    for index, waypoint in enumerate(waypoints):

        waypoint_x, waypoint_y = waypoint

        screen_x = (
            FIELD_X
            + waypoint_x * SCALE
        )

        screen_y = (
            FIELD_Y
            + waypoint_y * SCALE
        )

        pygame.draw.circle(
            screen,
            "green",
            (
                int(screen_x),
                int(screen_y),
            ),
            7,
        )

        number_text = small_font.render(
            str(index + 1),
            True,
            "black",
        )

        screen.blit(
            number_text,
            (
                int(screen_x) + 10,
                int(screen_y) - 15,
            ),
        )

        if index > 0:

            previous_x, previous_y = (
                waypoints[index - 1]
            )

            previous_screen_x = (
                FIELD_X
                + previous_x * SCALE
            )

            previous_screen_y = (
                FIELD_Y
                + previous_y * SCALE
            )

            pygame.draw.line(
                screen,
                "green",
                (
                    int(previous_screen_x),
                    int(previous_screen_y),
                ),
                (
                    int(screen_x),
                    int(screen_y),
                ),
                2,
            )


    # =================================================
    # GPS POSITION
    # =================================================

    gps_screen_x = (
        FIELD_X
        + gps_x * SCALE
    )

    gps_screen_y = (
        FIELD_Y
        + gps_y * SCALE
    )

    pygame.draw.circle(
        screen,
        "blue",
        (
            int(gps_screen_x),
            int(gps_screen_y),
        ),
        5,
    )


    # =================================================
    # ESTIMATED POSITION
    # =================================================

    estimated_screen_x = (
        FIELD_X
        + localization.x * SCALE
    )

    estimated_screen_y = (
        FIELD_Y
        + localization.y * SCALE
    )

    pygame.draw.circle(
        screen,
        "purple",
        (
            int(estimated_screen_x),
            int(estimated_screen_y),
        ),
        5,
    )


    # =================================================
    # TRUE ROBOT
    # =================================================

    robot_screen_x = (
        FIELD_X
        + robot.x * SCALE
    )

    robot_screen_y = (
        FIELD_Y
        + robot.y * SCALE
    )

    pygame.draw.circle(
        screen,
        "red",
        (
            int(robot_screen_x),
            int(robot_screen_y),
        ),
        9,
    )


    # =================================================
    # TRUE HEADING
    # =================================================

    heading_length = 30

    true_heading_end_x = (
        robot_screen_x
        + math.cos(
            math.radians(
                robot.heading
            )
        )
        * heading_length
    )

    true_heading_end_y = (
        robot_screen_y
        + math.sin(
            math.radians(
                robot.heading
            )
        )
        * heading_length
    )

    pygame.draw.line(
        screen,
        "red",
        (
            int(robot_screen_x),
            int(robot_screen_y),
        ),
        (
            int(true_heading_end_x),
            int(true_heading_end_y),
        ),
        3,
    )


    # =================================================
    # ESTIMATED HEADING
    # =================================================

    estimated_heading_length = 38

    estimated_heading_end_x = (
        robot_screen_x
        + math.cos(
            math.radians(
                localization.heading
            )
        )
        * estimated_heading_length
    )

    estimated_heading_end_y = (
        robot_screen_y
        + math.sin(
            math.radians(
                localization.heading
            )
        )
        * estimated_heading_length
    )

    pygame.draw.line(
        screen,
        "purple",
        (
            int(robot_screen_x),
            int(robot_screen_y),
        ),
        (
            int(estimated_heading_end_x),
            int(estimated_heading_end_y),
        ),
        2,
    )


    # =================================================
    # LEGEND UNDER FIELD
    # =================================================

    legend_y = (
        FIELD_Y
        + FIELD_HEIGHT
        + 25
    )

    legend_items = [
        ("Robot (True)", "red"),
        ("GPS", "blue"),
        ("Estimated", "purple"),
    ]

    legend_x = FIELD_X + 80

    for label, color in legend_items:

        pygame.draw.circle(
            screen,
            color,
            (
                legend_x,
                legend_y,
            ),
            6,
        )

        legend_text = data_font.render(
            label,
            True,
            "black",
        )

        screen.blit(
            legend_text,
            (
                legend_x + 12,
                legend_y - 8,
            ),
        )

        legend_x += 120


    # =================================================
    # DISTANCE GRAPH UNDER FIELD
    # =================================================

    graph_x = 40
    graph_y = 570

    graph_width = 490
    graph_height = 100

    graph_rect = pygame.Rect(
        graph_x,
        graph_y,
        graph_width,
        graph_height,
    )

    pygame.draw.rect(
        screen,
        "black",
        graph_rect,
        1,
    )


    graph_title = section_font.render(
        "DISTANCE TO TARGET",
        True,
        "black",
    )

    screen.blit(
        graph_title,
        (
            graph_x + 10,
            graph_y + 5,
        ),
    )


    # =================================================
    # GRAPH SCALE
    # =================================================

    max_distance = 1.0

    if true_distance_history:

        max_distance = max(
            max_distance,
            max(true_distance_history),
        )

    if estimated_distance_history:

        max_distance = max(
            max_distance,
            max(estimated_distance_history),
        )

    max_distance *= 1.1


    # =================================================
    # GRAPH AREA
    # =================================================

    inner_left = graph_x + 10

    inner_right = (
        graph_x
        + graph_width
        - 10
    )

    inner_top = graph_y + 27

    inner_bottom = (
        graph_y
        + graph_height
        - 22
    )


    # =================================================
    # ZERO LINE
    # =================================================

    pygame.draw.line(
        screen,
        "black",
        (
            inner_left,
            inner_bottom,
        ),
        (
            inner_right,
            inner_bottom,
        ),
        1,
    )


    # =================================================
    # TRUE DISTANCE LINE
    # =================================================

    if len(true_distance_history) > 1:

        true_points = []

        for index, distance in enumerate(
            true_distance_history
        ):

            x = (
                inner_left
                + (
                    index
                    / (
                        MAX_GRAPH_POINTS - 1
                    )
                )
                * (
                    inner_right
                    - inner_left
                )
            )

            y_position = (
                inner_bottom
                - (
                    distance
                    / max_distance
                )
                * (
                    inner_bottom
                    - inner_top
                )
            )

            true_points.append(
                (
                    int(x),
                    int(y_position),
                )
            )

        pygame.draw.lines(
            screen,
            "red",
            False,
            true_points,
            2,
        )


    # =================================================
    # ESTIMATED DISTANCE LINE
    # =================================================

    if len(estimated_distance_history) > 1:

        estimated_points = []

        for index, distance in enumerate(
            estimated_distance_history
        ):

            x = (
                inner_left
                + (
                    index
                    / (
                        MAX_GRAPH_POINTS - 1
                    )
                )
                * (
                    inner_right
                    - inner_left
                )
            )

            y_position = (
                inner_bottom
                - (
                    distance
                    / max_distance
                )
                * (
                    inner_bottom
                    - inner_top
                )
            )

            estimated_points.append(
                (
                    int(x),
                    int(y_position),
                )
            )

        pygame.draw.lines(
            screen,
            "purple",
            False,
            estimated_points,
            2,
        )


    # =================================================
    # GRAPH LEGEND
    # =================================================

    red_label = small_font.render(
        "RED = TRUE",
        True,
        "black",
    )

    purple_label = small_font.render(
        "PURPLE = ESTIMATED",
        True,
        "black",
    )

    screen.blit(
        red_label,
        (
            graph_x + 10,
            graph_y + graph_height - 15,
        ),
    )

    screen.blit(
        purple_label,
        (
            graph_x + 100,
            graph_y + graph_height - 15,
        ),
    )


    # =================================================
    # RIGHT DASHBOARD
    # =================================================

    dashboard_x = 575

    title = title_font.render(
        "AUTON NAVIGATION",
        True,
        "black",
    )

    screen.blit(
        title,
        (
            dashboard_x,
            35,
        ),
    )


    # =================================================
    # DASHBOARD DIVIDER
    # =================================================

    divider_x = 815

    pygame.draw.line(
        screen,
        "black",
        (
            divider_x,
            85,
        ),
        (
            divider_x,
            650,
        ),
        1,
    )


    # =================================================
    # LEFT DASHBOARD COLUMN
    # =================================================

    left_x = dashboard_x

    y_left = 85


    # -------------------------------------------------
    # ROBOT
    # -------------------------------------------------

    section = section_font.render(
        "ROBOT",
        True,
        "black",
    )

    screen.blit(
        section,
        (
            left_x,
            y_left,
        ),
    )

    y_left += 28

    lines = [
        f"X {robot.x:.2f} m    Y {robot.y:.2f} m",
        f"H {robot.heading:.1f}°    V {robot.speed:.2f} m/s",
    ]

    for line in lines:

        text = data_font.render(
            line,
            True,
            "black",
        )

        screen.blit(
            text,
            (
                left_x,
                y_left,
            ),
        )

        y_left += 22


    # -------------------------------------------------
    # GPS
    # -------------------------------------------------

    y_left += 12

    section = section_font.render(
        "GPS",
        True,
        "black",
    )

    screen.blit(
        section,
        (
            left_x,
            y_left,
        ),
    )

    y_left += 28

    text = data_font.render(
        f"X {gps_x:.2f} m    Y {gps_y:.2f} m",
        True,
        "black",
    )

    screen.blit(
        text,
        (
            left_x,
            y_left,
        ),
    )

    y_left += 22


    # -------------------------------------------------
    # LOCALIZATION
    # -------------------------------------------------

    y_left += 12

    section = section_font.render(
        "LOCALIZATION",
        True,
        "black",
    )

    screen.blit(
        section,
        (
            left_x,
            y_left,
        ),
    )

    y_left += 28

    lines = [
        f"X {localization.x:.2f} m    Y {localization.y:.2f} m",
        f"H {localization.heading:.1f}°",
        f"Position error {position_error:.2f} m",
    ]

    for line in lines:

        text = data_font.render(
            line,
            True,
            "black",
        )

        screen.blit(
            text,
            (
                left_x,
                y_left,
            ),
        )

        y_left += 22


    # -------------------------------------------------
    # SENSORS
    # -------------------------------------------------

    y_left += 12

    section = section_font.render(
        "SENSORS",
        True,
        "black",
    )

    screen.blit(
        section,
        (
            left_x,
            y_left,
        ),
    )

    y_left += 28

    lines = [
        f"Gyro {gyro_reading:.2f} °/s",
        f"Mag {magnetometer_reading:.1f}°",
    ]

    for line in lines:

        text = data_font.render(
            line,
            True,
            "black",
        )

        screen.blit(
            text,
            (
                left_x,
                y_left,
            ),
        )

        y_left += 22


    # =================================================
    # RIGHT DASHBOARD COLUMN
    # =================================================

    right_x = 840

    y_right = 85


    # -------------------------------------------------
    # NAVIGATION
    # -------------------------------------------------

    section = section_font.render(
        "NAVIGATION",
        True,
        "black",
    )

    screen.blit(
        section,
        (
            right_x,
            y_right,
        ),
    )

    y_right += 28

    waypoint_display = min(
        navigator.current_waypoint + 1,
        len(waypoints),
    )

    lines = [
        f"Waypoint {waypoint_display}/{len(waypoints)}",
        f"True distance {true_distance_to_target:.2f} m",
        f"Estimated distance {estimated_distance_to_target:.2f} m",
        f"Desired H {desired_heading:.1f}°",
        f"Desired V {desired_speed:.2f} m/s",
    ]

    for line in lines:

        text = data_font.render(
            line,
            True,
            "black",
        )

        screen.blit(
            text,
            (
                right_x,
                y_right,
            ),
        )

        y_right += 22


    # -------------------------------------------------
    # MISSION PERFORMANCE
    # -------------------------------------------------

    y_right += 18

    section = section_font.render(
        "MISSION PERFORMANCE",
        True,
        "black",
    )

    screen.blit(
        section,
        (
            right_x,
            y_right,
        ),
    )

    y_right += 28


    if waypoint_errors:

        for index, error in enumerate(
            waypoint_errors
        ):

            line = (
                f"WP {index + 1}    "
                f"{error:.2f} m"
            )

            text = data_font.render(
                line,
                True,
                "black",
            )

            screen.blit(
                text,
                (
                    right_x,
                    y_right,
                ),
            )

            y_right += 20

    else:

        text = data_font.render(
            "No completed waypoints",
            True,
            "black",
        )

        screen.blit(
            text,
            (
                right_x,
                y_right,
            ),
        )

        y_right += 20


    # -------------------------------------------------
    # PERFORMANCE SUMMARY
    # -------------------------------------------------

    y_right += 8

    pygame.draw.line(
        screen,
        "black",
        (
            right_x,
            y_right,
        ),
        (
            right_x + 190,
            y_right,
        ),
        1,
    )

    y_right += 15

    summary_lines = [
        f"AVG     {average_error:.2f} m",
        f"MAX     {maximum_error:.2f} m",
    ]

    for line in summary_lines:

        text = data_font.render(
            line,
            True,
            "black",
        )

        screen.blit(
            text,
            (
                right_x,
                y_right,
            ),
        )

        y_right += 22


    # =================================================
    # STATUS
    # =================================================

    status = (
        "MISSION COMPLETE"
        if mission_complete
        else "NAVIGATING"
    )

    status_text = section_font.render(
        status,
        True,
        "black",
    )

    screen.blit(
        status_text,
        (
            840,
            620,
        ),
    )


    # =================================================
    # DISPLAY
    # =================================================

    pygame.display.flip()


# =================================================
# SHUTDOWN
# =================================================

pygame.quit()