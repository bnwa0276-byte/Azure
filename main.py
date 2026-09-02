"""Demo entry point for the finite-state-machine drone platform."""

from __future__ import annotations

from drone import Drone, FlightMode, TransitionError


def main() -> None:
    """Show the drone transitioning through a valid flight-mode sequence."""
    drone = Drone()

    print("Initial drone state:")
    print(drone.status_report())

    for mode in (FlightMode.INITIALIZING, FlightMode.IDLE, FlightMode.ARMED):
        drone.change_mode(mode)
        print(f"Transitioned to {mode.value}.")
        print(drone.status_report())

    print("\nTaking off...")
    drone.takeoff()
    print(drone.status_report())

    print("\nLanding...")
    drone.land()
    print(drone.status_report())

    print("\nAttempting an invalid transition...")
    try:
        drone.change_mode(FlightMode.HOVER)
    except TransitionError as exc:
        print(exc)


if __name__ == "__main__":
    main()
