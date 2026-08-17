import math


class Localization:

    def __init__(
        self,
        initial_x=10.0,
        initial_y=10.0,
    ):

        self.x = initial_x
        self.y = initial_y

        self.heading = 0.0

        # GPS position correction
        self.position_correction = 0.2

        # Magnetometer heading correction
        self.heading_correction = 0.60


    def update(
        self,
        gps_x,
        gps_y,
        gyro_reading,
        magnetometer_heading,
        dt,
        speed=0.0,
    ):

        # -------------------------------------------------
        # HEADING PREDICTION
        # -------------------------------------------------

        self.heading += (
            gyro_reading * dt
        )

        self.heading %= 360


        # -------------------------------------------------
        # MAGNETOMETER HEADING CORRECTION
        # -------------------------------------------------

        heading_error = (
            magnetometer_heading
            - self.heading
            + 180
        ) % 360 - 180

        self.heading += (
            heading_error
            * self.heading_correction
        )

        self.heading %= 360


        # -------------------------------------------------
        # DEAD-RECKONING POSITION PREDICTION
        # -------------------------------------------------

        distance = speed * dt

        self.x += (
            math.cos(
                math.radians(
                    self.heading
                )
            )
            * distance
        )

        self.y += (
            math.sin(
                math.radians(
                    self.heading
                )
            )
            * distance
        )


        # -------------------------------------------------
        # GPS POSITION CORRECTION
        # -------------------------------------------------

        position_error_x = (
            gps_x - self.x
        )

        position_error_y = (
            gps_y - self.y
        )

        self.x += (
            position_error_x
            * self.position_correction
        )

        self.y += (
            position_error_y
            * self.position_correction
        )