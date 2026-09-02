"""Renderer protocol and a Matplotlib-based visualizer.

The visualizer reads telemetry and positions but never mutates simulation state.
It is written to be replaceable by other backends (e.g., a 3D renderer).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Sequence, Tuple, Protocol

from .telemetry import format_telemetry as format_telemetry_fn
from .world import map_world_to_screen
from .ui_helpers import telemetry_text_block

try:
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    _HAS_MATPLOTLIB = True
except Exception:
    plt = None
    _HAS_MATPLOTLIB = False


class VisualizerProtocol(Protocol):
    def update(self, *, telemetry: Dict[str, Any], position: Tuple[float, float, float], waypoints: Sequence[Any], path: Sequence[Tuple[float, float, float]]) -> None:
        ...


@dataclass
class MatplotlibVisualizer:
    width: int = 800
    height: int = 600
    figsize: Tuple[float, float] = (8.0, 6.0)
    fig: Any = field(default=None, init=False)
    ax: Any = field(default=None, init=False)
    telemetry_panel: Any = field(default=None, init=False)
    path_history: List[Tuple[float, float]] = field(default_factory=list, init=False)

    def __post_init__(self):
        if not _HAS_MATPLOTLIB:
            # fallback: simple headless visualizer state, no drawing
            self.fig = None
            self.ax = None
            self._last_drawn = None
            return
        self.fig, self.ax = plt.subplots(figsize=self.figsize)
        self.ax.set_aspect('equal', 'box')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self._last_drawn = None

    def _map_coord(self, pos: Tuple[float, float, float]) -> Tuple[float, float]:
        # Map using world utilities to allow future projection changes
        return map_world_to_screen(pos)

    def format_telemetry(self, telemetry: Dict[str, Any]) -> Dict[str, str]:
        return {
            'sim_time': f"{telemetry.get('sim_time', 0.0):.2f}s",
            'mode': str(telemetry.get('mode', 'UNKNOWN')),
            'altitude': f"{telemetry.get('altitude', 0.0):.2f} m",
            'vz': f"{telemetry.get('vz', 0.0):.2f} m/s",
            'battery': f"{telemetry.get('battery', 0.0):.1f}%",
            'target_altitude': f"{telemetry.get('target_altitude', None)}",
        }

    def update(self, *, telemetry: Dict[str, Any], position: Tuple[float, float, float], waypoints: Sequence[Any], path: Sequence[Tuple[float, float, float]]) -> None:
        # read-only: do not modify telemetry/position
        px, py = self._map_coord(position)

        # update path history (2D)
        if len(path) <= 1:
            # treat as incremental update: append current position
            self.path_history.append((px, py))
        else:
            # full path snapshot provided
            self.path_history = [self._map_coord(p) for p in path]

        # redraw (if matplotlib available)
        if _HAS_MATPLOTLIB:
            self.ax.clear()
            self.ax.set_aspect('equal', 'box')
            self.ax.set_xlabel('X (m)')
            self.ax.set_ylabel('Y (m)')

            # draw path
            if self.path_history:
                xs, ys = zip(*self.path_history)
                self.ax.plot(xs, ys, linestyle='-', color='blue', linewidth=1)

            # draw waypoints
            if waypoints:
                wx = []
                wy = []
                for w in waypoints:
                    if isinstance(w, (list, tuple)) and len(w) >= 2:
                        wx.append(float(w[0]))
                        wy.append(float(w[1]))
                    else:
                        try:
                            # support Waypoint-like objects with latitude/longitude
                            if hasattr(w, "latitude") and hasattr(w, "longitude"):
                                # convert degrees to meters using same simplification
                                meters_per_degree = 111_000.0
                                wx.append((float(w.longitude)) * meters_per_degree)
                                wy.append((float(w.latitude)) * meters_per_degree)
                            else:
                                wx.append(float(w.x))
                                wy.append(float(w.y))
                        except Exception:
                            continue
                if wx:
                    self.ax.scatter(wx, wy, marker='x', color='red')

            # draw guidance path if guidance info present in telemetry
            guidance = telemetry.get("guidance")
            if guidance is not None:
                try:
                    # guidance may include avoidance path or target
                    gx = guidance.get("target_x")
                    gy = guidance.get("target_y")
                    if gx is not None and gy is not None:
                        self.ax.plot([px, gx], [py, gy], linestyle='--', color='orange')
                        self.ax.scatter([gx], [gy], marker='D', color='orange')
                    # obstacles list
                    obstacles = guidance.get("obstacles")
                    if obstacles:
                        ox = [float(o[0]) for o in obstacles]
                        oy = [float(o[1]) for o in obstacles]
                        self.ax.scatter(ox, oy, marker='s', color='black')
                except Exception:
                    pass

            # draw drone
            self.ax.scatter([px], [py], marker='o', color='green')

            # telemetry panel text
            info = format_telemetry_fn(dict(telemetry, controller=getattr(telemetry.get('controller'), '__class__', None)))
            txt = telemetry_text_block(info)
            self.ax.text(0.02, 0.98, txt, transform=self.ax.transAxes, va='top', fontsize=8, bbox=dict(facecolor='white', alpha=0.7))

            # avoid blocking; just draw to buffer
            self.fig.canvas.draw()
        else:
            # headless fallback: nothing to draw, but methods must behave
            return
