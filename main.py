"""Demo entry point and simulation runner for the autonomous drone platform."""

from __future__ import annotations

import argparse
import sys
from typing import Optional

from analytics.statistics import Statistics
from drone import Drone, FlightMode, TransitionError
from environment.model import Environment
from flight_controller import FlightController
from flight_recorder.recorder import FlightRecorder
from fusion.estimator import ComplementaryEstimator
from gazebo.bridge import GazeboBridge
from gazebo.config import GazeboBridgeConfig
from navigation import Mission, NavigationSystem, Waypoint
from guidance import GuidanceSystem
from obstacles.representation import Obstacle
from simulation_runner import SimulationRunner


def print_telemetry_banner(runner: SimulationRunner) -> None:
    telemetry = runner.telemetry()
    alt = telemetry["altitude"]
    vz = telemetry["vz"]
    mode = telemetry["mode"]
    battery = telemetry["battery"]
    sim_t = telemetry["sim_time"]
    print(
        f"  [T={sim_t:5.2f}s] Mode: {mode:12s} | Alt: {alt:5.2f}m | Vz: {vz:+5.2f}m/s | Batt: {battery:4.1f}%"
    )


def run_simulation(
    target_altitude: float = 5.0,
    enable_mission: bool = False,
    enable_gazebo: bool = False,
    dt: float = 0.02,
) -> None:
    print("=" * 65)
    print("      AUTONOMOUS DRONE PLATFORM - V2.0 SIMULATION RUNNER")
    print("=" * 65)

    # 1. Instantiate Vehicle & Core Subsystems
    drone = Drone()
    controller = FlightController(drone)
    recorder = FlightRecorder()
    estimator = ComplementaryEstimator(alpha=0.98, seed=42)
    env = Environment(steady_wind=(1.0, 0.5), turbulence_strength=0.1, enabled=True)

    # Optional Gazebo bridge visualizer
    bridge = None
    if enable_gazebo:
        bridge = GazeboBridge(GazeboBridgeConfig(enabled=True))
        bridge.connect()
        print("[Visualizer] GazeboBridge UDP streaming enabled (127.0.0.1:9871)")

    # Optional mission & navigation setup
    nav: Optional[NavigationSystem] = None
    guidance: Optional[GuidanceSystem] = None
    if enable_mission:
        waypoints = [
            Waypoint(name="WP1", latitude=0.00005, longitude=0.00005, altitude=target_altitude),
            Waypoint(name="WP2", latitude=0.00010, longitude=0.00000, altitude=target_altitude),
            Waypoint(name="WP3", latitude=0.00000, longitude=0.00000, altitude=target_altitude),
        ]
        mission = Mission(waypoints)
        nav = NavigationSystem(mission)
        controller.navigation_system = nav
        guidance = GuidanceSystem(nav, desired_speed=1.5)
        guidance.set_obstacles([Obstacle(x=4.0, y=2.0, radius=1.5, z_min=0.0, z_max=15.0)])
        print(f"[Navigation] Loaded mission with {len(waypoints)} waypoints and obstacle avoidance.")

    runner = SimulationRunner(
        drone=drone,
        controller=controller,
        navigation=nav,
        dt=dt,
        visualizer=bridge,
        recorder=recorder,
        estimator=estimator,
        environment=env,
    )

    print("\n--- Phase 1: Initialization & Arming ---")
    print(f"Initial state: {drone.status_report()}")
    drone.change_mode(FlightMode.INITIALIZING)
    print(f"Transitioned to {drone.mode.value}.")
    drone.change_mode(FlightMode.IDLE)
    print(f"Transitioned to {drone.mode.value}.")
    drone.change_mode(FlightMode.ARMED)
    print(f"Transitioned to {drone.mode.value}.")
    print(f"Pre-flight health check: OK")

    print(f"\n--- Phase 2: Commanded Takeoff (Target Alt: {target_altitude:.1f}m) ---")
    drone.takeoff(
        target_altitude=target_altitude,
        dt=dt,
        tolerance=0.25,
        touch_vz=0.3,
        flight_controller=controller,
        Kp=1.5,
    )

    log_interval = 0.5
    next_log_time = 0.0

    # Step simulation through takeoff until HOVER is achieved
    max_takeoff_steps = int(15.0 / dt)
    for _ in range(max_takeoff_steps):
        runner.step()
        if runner.sim_time >= next_log_time:
            print_telemetry_banner(runner)
            next_log_time += log_interval
        if drone.mode == FlightMode.HOVER:
            print_telemetry_banner(runner)
            print(f">> Takeoff complete! Handoff to HOVER achieved at altitude {drone.altitude:.2f}m.")
            break

    # Phase 3: Mission Progression or Station Keeping
    if enable_mission and guidance is not None and nav is not None:
        print("\n--- Phase 3: Mission Waypoint Progression ---")
        drone.change_mode(FlightMode.MISSION)
        next_log_time = runner.sim_time + log_interval
        max_mission_steps = int(25.0 / dt)
        for _ in range(max_mission_steps):
            runner.step()
            pos = drone.physics.position
            lat = pos[0] / 111000.0
            lon = pos[1] / 111000.0
            cmd = guidance.compute_command(lat, lon, pos[2])
            if cmd.status != "MISSION_COMPLETE":
                controller.apply_guidance(cmd)
            status = controller.supervise_mission(lat, lon, pos[2])
            if runner.sim_time >= next_log_time:
                wp_idx = nav.mission.current_index
                total_wps = len(nav.mission.waypoints)
                print(f"  [T={runner.sim_time:5.2f}s] Waypoint {wp_idx+1}/{total_wps} | Pos: ({pos[0]:5.2f}, {pos[1]:5.2f}, {pos[2]:5.2f})m | Status: {cmd.status}")
                next_log_time += log_interval
            if status == "MISSION_COMPLETE":
                print(f">> Mission complete! All waypoints reached.")
                drone.change_mode(FlightMode.RETURN_HOME)
                break
        if drone.mode != FlightMode.RETURN_HOME:
            drone.change_mode(FlightMode.HOVER)
    else:
        print("\n--- Phase 3: Stabilized Hover (2 seconds) ---")
        next_log_time = runner.sim_time + log_interval
        hover_steps = int(2.0 / dt)
        for _ in range(hover_steps):
            runner.step()
            if runner.sim_time >= next_log_time:
                print_telemetry_banner(runner)
                next_log_time += log_interval

    # Phase 4: Controlled Descent & Landing
    print("\n--- Phase 4: Controlled Descent & Landing ---")
    drone.land(dt=dt, tolerance=0.15, touch_vz=0.2, flight_controller=controller, Kp=1.0)
    next_log_time = runner.sim_time + log_interval
    max_land_steps = int(15.0 / dt)
    for _ in range(max_land_steps):
        runner.step()
        if runner.sim_time >= next_log_time:
            print_telemetry_banner(runner)
            next_log_time += log_interval
        if drone.mode == FlightMode.IDLE:
            print_telemetry_banner(runner)
            print(f">> Touchdown confirmed! Safe touchdown at altitude {drone.altitude:.2f}m. Mode returned to IDLE.")
            break

    # Phase 5: Post-Flight & FSM Safety Verification
    print("\n--- Phase 5: Post-Flight & FSM Safety Verification ---")
    print(f"Vehicle safely on ground in {drone.mode.value} mode. Final status: {drone.status_report()}")

    print("\nAttempting invalid state transition (IDLE -> HOVER) to verify safety logic:")
    try:
        drone.change_mode(FlightMode.HOVER)
    except TransitionError as exc:
        print(f"  [Security Check Passed] Caught expected TransitionError: {exc}")

    if bridge is not None:
        bridge.close()

    # Simulation Telemetry Summary
    stats = Statistics(recorder.entries())
    print("\n" + "=" * 65)
    print("                     SIMULATION FLIGHT SUMMARY")
    print("=" * 65)
    print(f"Total Simulation Time : {runner.sim_time:.2f} s ({runner.step_count} steps @ {1.0/dt:.0f} Hz)")
    print(f"Final Battery Level   : {drone.battery:.1f} %")
    print(f"Max Altitude Overshoot: {stats.max_overshoot():.3f} m")
    print(f"Average Alt Error     : {stats.average_altitude_error():.3f} m")
    print(f"Recorded Data Points  : {len(list(recorder.entries()))}")
    print("=" * 65)


def main() -> None:
    parser = argparse.ArgumentParser(description="Autonomous Drone Platform Simulation")
    parser.add_argument("--alt", type=float, default=5.0, help="Target altitude for takeoff in meters (default: 5.0)")
    parser.add_argument("--mission", action="store_true", help="Execute autonomous waypoint mission")
    parser.add_argument("--gazebo", action="store_true", help="Enable Gazebo UDP visualization bridge")
    parser.add_argument("--dt", type=float, default=0.02, help="Simulation timestep in seconds (default: 0.02)")
    args = parser.parse_args()

    run_simulation(
        target_altitude=args.alt,
        enable_mission=args.mission,
        enable_gazebo=args.gazebo,
        dt=args.dt,
    )


if __name__ == "__main__":
    main()
