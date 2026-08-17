import random


class GPSSensor:

    def __init__(self):
        self.error_x = 0.0
        self.error_y = 0.0

        self.drift_rate = 0.02
        self.max_error = 3.0

    def read(self, true_x, true_y):

        self.error_x += random.uniform(
            -self.drift_rate,
            self.drift_rate,
        )

        self.error_y += random.uniform(
            -self.drift_rate,
            self.drift_rate,
        )

        self.error_x = max(
            -self.max_error,
            min(self.max_error, self.error_x),
        )

        self.error_y = max(
            -self.max_error,
            min(self.max_error, self.error_y),
        )

        gps_x = true_x + self.error_x
        gps_y = true_y + self.error_y

        return gps_x, gps_y


class Gyroscope:

    def __init__(self):
        self.noise = 0.5

    def read(self, true_rotation_rate):

        reading = (
            true_rotation_rate
            + random.uniform(
                -self.noise,
                self.noise,
            )
        )

        return reading


class Magnetometer:

    def __init__(self):
        self.noise = 2.0

    def read(self, true_heading):

        # Simulate compass measurement error
        reading = (
            true_heading
            + random.uniform(
                -self.noise,
                self.noise,
            )
        )

        return reading % 360