import math


class Robot:

    def __init__(self):
        # True physical state
        self.x = 10.0
        self.y = 10.0
        self.heading = 0.0
        self.speed = 0.0

        # Physical limits
        self.max_speed = 5.0
        self.max_turn_rate = 90.0

    def update(self, desired_heading, desired_speed, dt):

        # Calculate heading error
        heading_error = (
            desired_heading
            - self.heading
            + 180
        ) % 360 - 180

        # Maximum turn allowed this frame
        max_turn = self.max_turn_rate * dt

        # Turn toward desired heading
        if abs(heading_error) <= max_turn:

            self.heading = desired_heading

        elif heading_error > 0:

            self.heading += max_turn

        else:

            self.heading -= max_turn

        # Keep heading between 0 and 360 degrees
        self.heading %= 360

        # Limit speed
        self.speed = min(
            desired_speed,
            self.max_speed,
        )

        # Calculate movement
        distance = self.speed * dt

        self.x += (
            math.cos(
                math.radians(self.heading)
            )
            * distance
        )

        self.y += (
            math.sin(
                math.radians(self.heading)
            )
            * distance
        )