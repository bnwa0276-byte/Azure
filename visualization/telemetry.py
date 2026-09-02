"""Telemetry formatting utilities for the Ground Control Station."""
from typing import Dict, Any

def format_telemetry(telemetry: Dict[str, Any]) -> Dict[str, str]:
    return {
        'sim_time': f"{telemetry.get('sim_time', 0.0):.2f}s",
        'mode': str(telemetry.get('mode', 'UNKNOWN')),
        'controller': str(getattr(telemetry.get('controller'), '__class__', 'None')) if telemetry.get('controller') is not None else 'None',
        'altitude': f"{telemetry.get('altitude', 0.0):.2f} m",
        'vz': f"{telemetry.get('vz', 0.0):.2f} m/s",
        'battery': f"{telemetry.get('battery', 0.0):.1f}%",
        'target_altitude': f"{telemetry.get('target_altitude', None)}",
        'current_waypoint': str(telemetry.get('current_waypoint', 'None')),
        'mission_status': str(telemetry.get('mission_status', 'UNKNOWN')),
        'est_altitude': f"{telemetry.get('estimated', {}).get('position', (0.0,0.0,0.0))[2]:.2f} m",
        'est_vz': f"{telemetry.get('estimated', {}).get('velocity', (0.0,0.0,0.0))[2]:.2f} m/s",
        'est_confidence': f"{telemetry.get('estimated', {}).get('confidence', 0.0):.2f}",
    }
