import ast
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent


class ArchitectureEnforcementTests(unittest.TestCase):
    def _module_ast(self, relative_path: str) -> ast.Module:
        return ast.parse((ROOT / relative_path).read_text(encoding="utf-8"), filename=str(ROOT / relative_path))

    def _class_names(self, relative_path: str) -> set[str]:
        tree = self._module_ast(relative_path)
        names: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef):
                names.add(node.name)
        return names

    def _module_names(self, relative_path: str) -> set[str]:
        tree = self._module_ast(relative_path)
        names: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Name):
                names.add(node.id)
        return names

    def _imported_modules(self, relative_path: str) -> set[str]:
        tree = self._module_ast(relative_path)
        modules: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    modules.add(alias.name.split(".")[0])
            elif isinstance(node, ast.ImportFrom) and node.module:
                modules.add(node.module.split(".")[0])
        return modules

    def test_single_flight_controller_implementation(self) -> None:
        matches: list[str] = []
        for file_path in sorted(ROOT.rglob("*.py")):
            if any(part in {".venv", "__pycache__"} for part in file_path.parts):
                continue
            try:
                tree = ast.parse(file_path.read_text(encoding="utf-8"), filename=str(file_path))
            except SyntaxError:
                continue
            for node in tree.body:
                if isinstance(node, ast.ClassDef) and node.name == "FlightController":
                    matches.append(str(file_path.relative_to(ROOT)).replace("\\", "/"))

        self.assertEqual(matches, ["flight_controller.py"], msg=f"Unexpected FlightController ownership: {matches}")
        self.assertNotIn("drone.py", matches)

    def test_flight_controller_owns_decisions_and_not_navigation_geometry(self) -> None:
        fc_tree = self._module_ast("flight_controller.py")
        fc_class_names = {node.name for node in fc_tree.body if isinstance(node, ast.ClassDef)}
        fc_names = self._module_names("flight_controller.py")

        self.assertIn("FlightController", fc_class_names)
        self.assertNotIn("distance_to_active_waypoint", fc_names)
        self.assertNotIn("Waypoint", fc_names)
        self.assertNotIn("Mission", fc_names)

        imported = self._imported_modules("flight_controller.py")
        self.assertIn("navigation", imported)
        self.assertIn("sensors", imported)

        self.assertNotIn("set_thrust_acceleration", fc_names)
        self.assertNotIn("update_position", fc_names)

    def test_navigation_owns_mission_state_and_not_safety_policy(self) -> None:
        nav_classes = self._class_names("navigation.py")
        nav_names = self._module_names("navigation.py")

        self.assertIn("NavigationSystem", nav_classes)
        self.assertIn("Mission", nav_classes)
        self.assertIn("Waypoint", nav_classes)
        self.assertNotIn("FlightMode", nav_names)
        self.assertNotIn("SensorStatus", nav_names)
        self.assertNotIn("Drone", nav_names)

        imported = self._imported_modules("navigation.py")
        self.assertNotIn("sensors", imported)
        self.assertNotIn("drone", imported)
        self.assertNotIn("flight_controller", imported)

    def test_guidance_is_horizontal_guidance_and_not_vehicle_logic(self) -> None:
        guidance_classes = self._class_names("guidance.py")
        guidance_names = self._module_names("guidance.py")
        imported = self._imported_modules("guidance.py")

        self.assertIn("GuidanceSystem", guidance_classes)
        self.assertIn("GuidanceCommand", guidance_classes)
        self.assertIn("navigation", imported)
        self.assertNotIn("Drone", guidance_names)
        self.assertNotIn("FlightMode", guidance_names)

    def test_read_only_subsystems_do_not_depend_on_simulation_mutators(self) -> None:
        analytics_imports = self._imported_modules("analytics/analyzer.py")
        visualization_imports = self._imported_modules("visualization/renderer.py")

        self.assertNotIn("drone", analytics_imports)
        self.assertNotIn("physics", analytics_imports)
        self.assertNotIn("flight_controller", analytics_imports)

        self.assertNotIn("drone", visualization_imports)
        self.assertNotIn("physics", visualization_imports)
        self.assertNotIn("flight_controller", visualization_imports)

    def test_flight_recorder_and_fusion_remain_boundary_clean(self) -> None:
        recorder_imports = self._imported_modules("flight_recorder/recorder.py")
        fusion_imports = self._imported_modules("fusion/estimator.py")

        self.assertNotIn("drone", recorder_imports)
        self.assertNotIn("physics", recorder_imports)
        self.assertNotIn("flight_controller", recorder_imports)

        self.assertNotIn("drone", fusion_imports)
        self.assertNotIn("flight_controller", fusion_imports)

    def test_hal_and_hil_remain_separated_from_physical_transport_logic(self) -> None:
        hal_imports = self._imported_modules("hal/interfaces.py")
        hil_imports = self._imported_modules("hal/hil.py")

        self.assertIn("abc", hal_imports)
        self.assertIn("hal", hil_imports)
        self.assertNotIn("drone", hil_imports)
        self.assertNotIn("physics", hil_imports)


if __name__ == "__main__":
    unittest.main()
