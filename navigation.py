import math


class Navigator:

    def __init__(self, waypoints):

        self.waypoints = list(waypoints)

        # -------------------------------------------------
        # MISSION STATE
        # -------------------------------------------------

        # Waypoint 0 is the starting position.
        # Waypoint 1 is therefore the first target.
        self.current_waypoint = 1

        self.waypoint_errors = [0.0]

        self.complete = False

        # -------------------------------------------------
        # NAVIGATION PARAMETERS
        # -------------------------------------------------

        self.cruise_speed = 2.50
        self.minimum_speed = 1.20
        self.approach_distance = 8.00

        self.cross_track_gain = 0.40
        self.lookahead_distance = 3.00

        self.turn_speed = 0.50
        self.turn_zone_distance = 2.00

        self.waypoint_tolerance = 0.50

        # Maximum cross-track steering correction.
        self.max_cross_track_correction = 45.0

        # Final transition zone.
        self.heading_transition_distance = 1.00

    # =====================================================
    # ANGLE UTILITIES
    # =====================================================

    @staticmethod
    def normalize_angle(angle):

        return (
            angle + 180.0
        ) % 360.0 - 180.0

    # =====================================================
    # DISTANCE
    # =====================================================

    @staticmethod
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

    # =====================================================
    # SEGMENT GEOMETRY
    # =====================================================

    @staticmethod
    def segment_geometry(
        x,
        y,
        x1,
        y1,
        x2,
        y2,
    ):

        dx = x2 - x1
        dy = y2 - y1

        length = math.hypot(
            dx,
            dy,
        )

        if length < 1e-9:

            return (
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )

        ux = dx / length
        uy = dy / length

        # Position along the segment.
        along = (
            (x - x1) * ux
            +
            (y - y1) * uy
        )

        # Signed cross-track error.
        cross_track = (
            ux * (y - y1)
            -
            uy * (x - x1)
        )

        # Projection clamped to the physical segment.
        projection = max(
            0.0,
            min(
                length,
                along,
            ),
        )

        closest_x = (
            x1
            +
            ux * projection
        )

        closest_y = (
            y1
            +
            uy * projection
        )

        distance_to_segment = math.hypot(
            x - closest_x,
            y - closest_y,
        )

        return (
            length,
            ux,
            uy,
            along,
            cross_track,
            distance_to_segment,
        )

    # =====================================================
    # WAYPOINT CHECK
    # =====================================================

    def check_waypoint(
        self,
        x,
        y,
    ):

        if self.complete:
            return

        if (
            self.current_waypoint
            >= len(self.waypoints)
        ):

            self.complete = True

            return

        # -------------------------------------------------
        # Current target
        # -------------------------------------------------

        target_x, target_y = self.waypoints[
            self.current_waypoint
        ]

        # -------------------------------------------------
        # Start of current segment
        # -------------------------------------------------

        start_x, start_y = self.waypoints[
            self.current_waypoint - 1
        ]

        # -------------------------------------------------
        # Segment geometry
        # -------------------------------------------------

        (
            segment_length,
            ux,
            uy,
            along,
            cross_track,
            distance_to_segment,
        ) = self.segment_geometry(
            x,
            y,
            start_x,
            start_y,
            target_x,
            target_y,
        )

        # -------------------------------------------------
        # Distance to waypoint
        # -------------------------------------------------

        distance_to_waypoint = self.distance(
            x,
            y,
            target_x,
            target_y,
        )

        # -------------------------------------------------
        # METHOD 1:
        # Normal waypoint-radius detection.
        # -------------------------------------------------

        reached_by_distance = (
            distance_to_waypoint
            <= self.waypoint_tolerance
        )

        # -------------------------------------------------
        # METHOD 2:
        # Endpoint-plane crossing.
        #
        # This is the important correction.
        #
        # Instead of asking:
        #
        #     "Am I within 1 m of the waypoint?"
        #
        # we ask:
        #
        #     "Have I reached or crossed the endpoint
        #      in the direction of this segment?"
        #
        # For a horizontal row:
        #
        #     (30,30) -> (10,30)
        #
        # the endpoint plane is x = 10.
        #
        # For a vertical row:
        #
        #     (10,30) -> (10,10)
        #
        # the endpoint plane is y = 10.
        #
        # `along` represents position along the segment,
        # so reaching the endpoint means:
        #
        #     along >= segment_length
        #
        # A small tolerance is included to avoid floating
        # point problems.
        # -------------------------------------------------

        endpoint_tolerance = 0.10

        reached_by_endpoint = (
            along
            >=
            segment_length
            -
            endpoint_tolerance
        )

        # -------------------------------------------------
        # IMPORTANT SAFETY CONDITION
        #
        # Do not switch rows simply because the projection
        # along the segment is beyond the endpoint while the
        # robot is extremely far away laterally.
        #
        # Allow a generous 3 m lateral window because GPS
        # dropout can temporarily produce >1 m error.
        # -------------------------------------------------

        endpoint_lateral_limit = 3.00

        reached_by_crossing = (
            reached_by_endpoint
            and
            abs(cross_track)
            <= endpoint_lateral_limit
        )

        # -------------------------------------------------
        # Advance waypoint
        # -------------------------------------------------

        if (
            reached_by_distance
            or
            reached_by_crossing
        ):

            self.waypoint_errors.append(
                distance_to_waypoint
            )

            self.current_waypoint += 1

            # -------------------------------------------------
            # Mission complete
            # -------------------------------------------------

            if (
                self.current_waypoint
                >= len(self.waypoints)
            ):

                self.complete = True

    # =====================================================
    # ACTIVE SEGMENT
    # =====================================================

    def get_active_segment(self):

        if self.complete:
            return None

        segment_index = (
            self.current_waypoint - 1
        )

        if segment_index < 0:

            segment_index = 0

        if (
            segment_index
            >= len(self.waypoints) - 1
        ):

            return None

        return segment_index

    # =====================================================
    # ROW HEADING
    # =====================================================

    def get_row_heading(
        self,
        segment_index,
    ):

        x1, y1 = self.waypoints[
            segment_index
        ]

        x2, y2 = self.waypoints[
            segment_index + 1
        ]

        return math.degrees(
            math.atan2(
                y2 - y1,
                x2 - x1,
            )
        )

    # =====================================================
    # CROSS-TRACK HEADING
    # =====================================================

    def get_cross_track_heading(
        self,
        row_heading,
        cross_track_error,
    ):

        lookahead = max(
            self.lookahead_distance,
            0.10,
        )

        correction = math.degrees(
            math.atan(
                (
                    self.cross_track_gain
                    *
                    cross_track_error
                )
                /
                lookahead
            )
        )

        correction = max(
            -self.max_cross_track_correction,
            min(
                self.max_cross_track_correction,
                correction,
            ),
        )

        corrected_heading = (
            row_heading
            -
            correction
        )

        return corrected_heading

    # =====================================================
    # ENDPOINT HEADING TRANSITION
    # =====================================================

    def get_transition_heading(
        self,
        row_heading,
        next_heading,
        distance_to_waypoint,
    ):

        transition_distance = max(
            self.heading_transition_distance,
            0.10,
        )

        # Outside transition zone:
        # stay aligned with current row.
        if (
            distance_to_waypoint
            >= transition_distance
        ):

            return row_heading

        # -------------------------------------------------
        # Inside transition zone.
        # -------------------------------------------------

        progress = (
            transition_distance
            -
            distance_to_waypoint
        ) / transition_distance

        progress = max(
            0.0,
            min(
                1.0,
                progress,
            ),
        )

        heading_difference = (
            self.normalize_angle(
                next_heading
                -
                row_heading
            )
        )

        return (
            row_heading
            +
            heading_difference
            *
            progress
        )

    # =====================================================
    # SPEED CONTROL
    # =====================================================

    def get_speed(
        self,
        distance_to_waypoint,
    ):

        # -------------------------------------------------
        # Normal cruise.
        # -------------------------------------------------

        if (
            distance_to_waypoint
            >
            self.turn_zone_distance
        ):

            return self.cruise_speed

        # -------------------------------------------------
        # Transition / turn zone.
        # -------------------------------------------------

        if (
            self.turn_zone_distance
            <=
            self.heading_transition_distance
        ):

            return max(
                self.turn_speed,
                self.minimum_speed,
            )

        progress = (
            distance_to_waypoint
            /
            self.turn_zone_distance
        )

        progress = max(
            0.0,
            min(
                1.0,
                progress,
            ),
        )

        speed = (
            self.turn_speed
            +
            (
                self.cruise_speed
                -
                self.turn_speed
            )
            *
            progress
        )

        return max(
            self.minimum_speed,
            min(
                self.cruise_speed,
                speed,
            ),
        )

    # =====================================================
    # MAIN UPDATE
    # =====================================================

    def update(
        self,
        x,
        y,
    ):

        # -------------------------------------------------
        # Check waypoint before calculating the new command.
        # -------------------------------------------------

        self.check_waypoint(
            x,
            y,
        )

        # -------------------------------------------------
        # Mission complete.
        # -------------------------------------------------

        if self.complete:

            return (
                0.0,
                0.0,
                True,
            )

        # -------------------------------------------------
        # Active segment.
        # -------------------------------------------------

        segment_index = (
            self.get_active_segment()
        )

        if segment_index is None:

            self.complete = True

            return (
                0.0,
                0.0,
                True,
            )

        # -------------------------------------------------
        # Current segment endpoints.
        # -------------------------------------------------

        x1, y1 = self.waypoints[
            segment_index
        ]

        x2, y2 = self.waypoints[
            segment_index + 1
        ]

        # -------------------------------------------------
        # Segment geometry.
        # -------------------------------------------------

        (
            segment_length,
            ux,
            uy,
            along_track,
            cross_track_error,
            distance_to_segment,
        ) = self.segment_geometry(
            x,
            y,
            x1,
            y1,
            x2,
            y2,
        )

        # -------------------------------------------------
        # Current target waypoint.
        # -------------------------------------------------

        waypoint_x, waypoint_y = self.waypoints[
            self.current_waypoint
        ]

        distance_to_waypoint = self.distance(
            x,
            y,
            waypoint_x,
            waypoint_y,
        )

        # =================================================
        # CURRENT ROW HEADING
        # =================================================

        row_heading = self.get_row_heading(
            segment_index
        )

        # =================================================
        # CROSS-TRACK CORRECTION
        # =================================================

        corrected_heading = (
            self.get_cross_track_heading(
                row_heading,
                cross_track_error,
            )
        )

        desired_heading = corrected_heading

        # =================================================
        # NEXT ROW TRANSITION
        # =================================================

        if (
            self.current_waypoint
            <
            len(self.waypoints) - 1
        ):

            next_segment = (
                segment_index + 1
            )

            next_heading = (
                self.get_row_heading(
                    next_segment
                )
            )

            desired_heading = (
                self.get_transition_heading(
                    corrected_heading,
                    next_heading,
                    distance_to_waypoint,
                )
            )

        # =================================================
        # SPEED
        # =================================================

        desired_speed = self.get_speed(
            distance_to_waypoint
        )

        # =================================================
        # FINAL SAFETY LIMIT
        # =================================================

        desired_speed = max(
            0.0,
            min(
                self.cruise_speed,
                desired_speed,
            ),
        )

        desired_heading %= 360.0

        return (
            desired_heading,
            desired_speed,
            False,
        )