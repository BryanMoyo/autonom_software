class Controller:

    def __init__(self):
        self.desired_heading = 0.0
        self.desired_speed = 0.0

    def set_command(
        self,
        desired_heading,
        desired_speed,
    ):

        self.desired_heading = desired_heading
        self.desired_speed = desired_speed

    def get_command(self):

        return (
            self.desired_heading,
            self.desired_speed,
        )