from test_bench import (
    evaluate_configuration,
    passes_all_targets,
)


# =========================================================
# CONFIRMED WINNING CONFIGURATION
# =========================================================

GAIN = 0.70
LOOKAHEAD = 4.50
TURN_SPEED = 0.50
TURN_ZONE = 5.00

TRIALS = 100


# =========================================================
# RUN CONFIRMATION
# =========================================================

def main():

    print()
    print("=" * 80)
    print("FINAL CONTROLLER CONFIRMATION")
    print("=" * 80)

    print()
    print("Configuration:")
    print(f"Cross-track gain : {GAIN:.2f}")
    print(f"Lookahead        : {LOOKAHEAD:.2f} m")
    print(f"Turn speed       : {TURN_SPEED:.2f} m/s")
    print(f"Turn zone        : {TURN_ZONE:.2f} m")
    print(f"Trials           : {TRIALS}")

    print()
    print("=" * 80)
    print("RUNNING 100 INDEPENDENT TRIALS")
    print("=" * 80)

    result = evaluate_configuration(
        GAIN,
        LOOKAHEAD,
        TURN_SPEED,
        TURN_ZONE,
        TRIALS,
    )

    print()
    print("=" * 80)
    print("CONFIRMATION RESULTS")
    print("=" * 80)

    print()
    print(
        f"Straight RMS : "
        f"{result['straight_rms']:.3f} m"
    )

    print(
        f"Straight MAX : "
        f"{result['straight_max']:.3f} m"
    )

    print(
        f"Turn RMS     : "
        f"{result['turn_rms']:.3f} m"
    )

    print(
        f"Turn MAX     : "
        f"{result['turn_max']:.3f} m"
    )

    print(
        f"Waypoint Avg : "
        f"{result['waypoint_average']:.3f} m"
    )

    print(
        f"Waypoint MAX : "
        f"{result['waypoint_max']:.3f} m"
    )

    print(
        f"Waypoint 95%: "
        f"{result['waypoint_p95']:.3f} m"
    )

    print(
        f"Waypoints    : "
        f"{result['waypoint_count']:.2f}"
    )

    print()
    print("=" * 80)
    print("TARGET ASSESSMENT")
    print("=" * 80)

    print()

    straight_rms_pass = (
        result["straight_rms"] <= 0.50
    )

    straight_max_pass = (
        result["straight_max"] <= 0.75
    )

    turn_max_pass = (
        result["turn_max"] <= 1.50
    )

    waypoint_pass = (
        result["waypoint_max"] <= 0.50
    )

    print(
        "Straight RMS target "
        "(<= 0.50 m): "
        f"{'PASS' if straight_rms_pass else 'FAIL'}"
    )

    print(
        "Straight MAX target "
        "(<= 0.75 m): "
        f"{'PASS' if straight_max_pass else 'FAIL'}"
    )

    print(
        "Turn MAX target "
        "(<= 1.50 m): "
        f"{'PASS' if turn_max_pass else 'FAIL'}"
    )

    print(
        "Waypoint target "
        "(<= 0.50 m): "
        f"{'PASS' if waypoint_pass else 'FAIL'}"
    )

    print()

    if passes_all_targets(result):

        print("=" * 80)
        print("FINAL CONFIRMATION: ALL TARGETS PASSED")
        print("=" * 80)

        print()
        print("Validated configuration:")
        print()
        print(
            f"cross_track_gain = {GAIN:.2f}"
        )
        print(
            f"lookahead_distance = {LOOKAHEAD:.2f}"
        )
        print(
            f"turn_speed = {TURN_SPEED:.2f}"
        )
        print(
            f"turn_zone_distance = {TURN_ZONE:.2f}"
        )

    else:

        print("=" * 80)
        print("FINAL CONFIRMATION: FAILED")
        print("=" * 80)

        print()
        print(
            "The optimizer result was not reproduced "
            "over the 100-trial confirmation run."
        )

    print()
    print("=" * 80)
    print("CONFIRMATION COMPLETE")
    print("=" * 80)


if __name__ == "__main__":
    main()