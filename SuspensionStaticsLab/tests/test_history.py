import csv
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from SuspensionStaticsLab.history import acceleration_history, lap_history, solve_history
from SuspensionStaticsLab.load_cases import ACCEL, STATIC, example_loads
from SuspensionStaticsLab.model import CORNERS, EXAMPLE, build_model, read_hardpoints
from SuspensionStaticsLab.solver import Load, Section, solve


class HistoryTests(unittest.TestCase):
    def setUp(self):
        self.model = build_model(read_hardpoints(EXAMPLE))
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / "lap_result.csv"

    def write_lap(self, fields, rows):
        with self.path.open("w", encoding="utf-8-sig", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(fields)
            writer.writerows(rows)

    def test_linear_rise_from_gravity_baseline(self):
        target = (3., -1., 2.)
        history = acceleration_history(self.model, 2.5, target, mass_kg=321., include_gravity=True)
        self.assertEqual(history.time_s[[0, 50, -1]].tolist(), [0., 1.25, 2.5])
        np.testing.assert_allclose(history.acceleration_mps2[[0, 50, -1]], np.outer((0., .5, 1.), target))
        np.testing.assert_allclose(history.loads[0][0].xyz, example_loads(self.model, STATIC)[0].xyz)
        startup = solve(self.model, example_loads(self.model, STATIC))
        final = solve(self.model, example_loads(self.model, ACCEL, acceleration_mps2=target))
        for name, values in history.force_n.items():
            np.testing.assert_allclose(values[[0, 50, -1]],
                                       [startup.forces[name], (startup.forces[name] + final.forces[name]) / 2.,
                                        final.forces[name]], atol=1e-8)
        sections = dict.fromkeys(history.force_n, Section())
        self.assertAlmostEqual(history.peak_stress(sections),
                               max(np.max(np.abs(values)) for values in history.force_n.values()) / Section().area_mm2())

    def test_downforce_is_constant_during_acceleration_rise(self):
        mass = 321.
        downforce = 800.
        history = acceleration_history(self.model, 2.5, (1.5, -1., 2.), mass_kg=mass,
                                       include_gravity=False, downforce_n=downforce)
        expected = downforce + mass * history.acceleration_mps2[:, 2]
        contacts = {wheel.contact for wheel in self.model.wheels}
        cg_force_z = [sum(load.xyz[2] for load in frame if load.target == self.model.cg)
                      for frame in history.loads]
        contact_force_z = [sum(load.xyz[2] for load in frame if load.target in contacts)
                           for frame in history.loads]
        np.testing.assert_allclose(cg_force_z, -expected)
        np.testing.assert_allclose(contact_force_z, expected)

    def test_actual_corner_forces_override_acceleration_columns(self):
        forces = [(120., -450., 820.), (230., -170., 700.), (85., -430., 720.), (21., -140., 520.)]
        flat = [value for vector in forces for value in vector]
        fields = ["time_s", "ax_g", "ay_g", *(f"f{axis}_{corner}_N" for corner in CORNERS for axis in "xyz")]
        self.write_lap(fields, [(1.2, 100., 100., *flat), (2.7, 200., 200., *(2. * value for value in flat))])
        history = lap_history(self.model, self.path, mass_kg="", include_gravity=False)
        self.assertEqual(history.time_s.tolist(), [1.2, 2.7])
        np.testing.assert_allclose(history.acceleration_mps2[:, :2], [(981., 981.), (1962., 1962.)])
        self.assertTrue(np.isnan(history.acceleration_mps2[:, 2]).all())
        expected = [Load(wheel.contact, xyz) for wheel, xyz in zip(self.model.wheels, forces)]
        self.assertEqual(history.loads[0], expected)
        single = solve(self.model, expected)
        for name, force in single.forces.items():
            np.testing.assert_allclose(history.force_n[name], (force, 2. * force), atol=1e-8)

    def test_lap_acceleration_fallback_and_duplicate_time_validation(self):
        fields = ("time_s", "long_g", "lat_g")
        self.write_lap(fields, [(0., 0., 0.), (.2, .25, -.1)])
        history = lap_history(self.model, self.path, mass_kg=280., include_gravity=True)
        expected = example_loads(self.model, ACCEL, mass_kg=280., acceleration_mps2=(.25 * 9.81, -.1 * 9.81, 0.))
        for recorded, load in zip(history.loads[-1], expected):
            self.assertEqual(recorded.target, load.target)
            np.testing.assert_allclose(recorded.xyz, load.xyz)
        self.write_lap(fields, [(0., 0., 0.), (0., .25, -.1)])
        with self.assertRaisesRegex(ValueError, "time_s 必須大於"):
            lap_history(self.model, self.path, mass_kg=280., include_gravity=True)

    def test_governing_rod_and_time_weighted_statistics(self):
        points = {"O": (0., 0., 0.), "X": (1., 0., 0.), "Y": (0., 1., 0.)}
        topology = {"fixed": ["X", "Y"], "rods": [{"name": axis, "a": "O", "b": axis} for axis in "XY"]}
        model = build_model(points, topology)
        times = np.array([0., 1., 4.])
        loads = [[Load("O", (-fx, -fy, 0.))] for fx, fy in ((8., -1.), (-4., 10.), (-12., 2.))]
        history = solve_history(model, times, loads, np.zeros((3, 3)), "test")
        names, forces = history.largest_loads
        self.assertEqual(names.tolist(), ["X", "Y", "X"])
        self.assertEqual(forces.tolist(), [8., 10., -12.])
        stats = history.statistics("X")
        self.assertEqual((stats.peak_n, stats.peak_time_s), (-12., 4.))
        self.assertAlmostEqual(stats.mean_n, -5.5)
        self.assertAlmostEqual(stats.mean_absolute_n, 41. / 6.)
