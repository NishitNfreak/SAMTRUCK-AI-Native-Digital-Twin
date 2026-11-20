import inspect
import unittest

import samtruck.digital_twin as digital_twin
import samtruck.twin_runtime as twin_runtime
from samtruck.digital_twin import VehicleState
from samtruck.twin_runtime import PlatoonTwin


class GroundTruthBoundaryTests(unittest.TestCase):
    def test_twin_modules_have_no_simulator_imports(self):
        for module in (digital_twin, twin_runtime):
            source = inspect.getsource(module)
            self.assertNotIn("import traci", source)
            self.assertNotIn("import sumolib", source)

    def test_twin_rejects_legacy_raw_truth_state(self):
        raw = VehicleState(1.0, "samtruck_leader", 10.0, 20.0, 0.0, 0, None, None, None)
        with self.assertRaises(TypeError):
            PlatoonTwin(("samtruck_leader",)).update(1.0, [raw], [])
