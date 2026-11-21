import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from SuspensionStaticsLab import project
from SuspensionStaticsLab.history import DEFAULT_RISE_S
from SuspensionStaticsLab.load_cases import ACCEL, DEFAULT_ACCELERATION, MASS_KG, STATIC, example_loads
from SuspensionStaticsLab.model import EXAMPLE, build_model, read_hardpoints
from SuspensionStaticsLab.solver import Load, Section, member_results, solve


class ProjectTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / "四輪專案.json"

    def test_round_trip_preserves_settings_and_solve(self):
        points = read_hardpoints(EXAMPLE)
        points["CG"] = (-.74, .01, .33)
        model = build_model(points)
        acceleration = (2., -1., 4.)
        downforce = 650.
        loads = example_loads(model, ACCEL, mass_kg=302., acceleration_mps2=acceleration,
                              include_gravity=False, downforce_n=downforce)
        loads.append(Load("FL_TIE_ROD", (5., -7., 2.), "B"))
        sections = {rod.name: Section() for rod in model.rods}
        sections["FL_TIE_ROD"] = Section(24., 14., 195.)
        sections["RR_PUSH_ROD"] = Section(29., 0., 72.)
        saved = project.Project(model, None, loads, sections, ACCEL, 302., acceleration, False, 2.5, downforce,
                                tire_mu=.85)

        project.save_project(self.path, saved)
        restored = project.load_project(self.path)

        self.assertEqual(restored, saved)
        before = member_results(solve(model, loads), sections)
        after = member_results(solve(restored.model, restored.loads), restored.sections)
        np.testing.assert_allclose([item.stress_mpa for item in before.values()],
                                   [item.stress_mpa for item in after.values()])

    def test_legacy_static_cases_convert_and_preserve_saved_loads(self):
        points = read_hardpoints(EXAMPLE)
        points["COP"] = (-.84, 0., .31)
        model = build_model(points)
        loads = [*example_loads(model, STATIC), Load("COP", (0., 0., -800.))]
        sections = dict.fromkeys((rod.name for rod in model.rods), Section())
        for case in (STATIC, "空力下壓"):
            with self.subTest(case=case):
                legacy = project.Project(model, None, loads, sections, case, 321., (6.3765, 0., 0.), True)
                project.save_project(self.path, legacy)
                restored = project.load_project(self.path)
                self.assertEqual(restored.case, ACCEL)
                self.assertEqual(restored.acceleration_mps2, (0., 0., 0.))
                self.assertEqual(restored.loads, loads)
                self.assertEqual(solve(restored.model, restored.loads), solve(model, loads))

    def test_round_trip_preserves_shared_damper_mounts(self):
        points = read_hardpoints(EXAMPLE)
        points["DAMPER_1_A"] = (0., -.092, .617)
        points["DAMPER_1_B"] = (0., .092, .536)
        model = build_model(points, member_mounts={"DAMPER_1": ("FR", "FL")})
        sections = {rod.name: Section() for rod in model.rods}
        sections["DAMPER_1"] = Section(24., 14., 195.)
        loads = [Load("DAMPER_1_A", (0., 0., 100.))]
        saved = project.Project(model, None, loads, sections, ACCEL, 321., (0., 0., 0.), True)

        project.save_project(self.path, saved)
        restored = project.load_project(self.path)

        self.assertEqual(restored, saved)
        self.assertEqual(restored.model.owners["DAMPER_1_A"], "FR_ROCKER")
        self.assertEqual(restored.model.owners["DAMPER_1_B"], "FL_ROCKER")

    def test_kinematics_project_import_and_native_save(self):
        points = read_hardpoints(EXAMPLE)
        outboard = points.pop("FL_PUSH_OUT_UCA")
        points["FL_PUSH_OUT"] = outboard
        dampers = {
            "Roll_Damper": ((0., -.2, .39), (0., .2, .33)),
            "Heave_Damper": ((0., -.2, .38), (0., .2, .38)),
        }
        kinematics = {
            "version": 1,
            "points": points,
            "pushrod_mounts": {"FL": "UCA"},
            "motion": {"duration": .5, "samples": 21},
            "deformable_members": [
                {"name": name, "a": {"xyz": a, "mount": "FR"}, "b": {"xyz": b, "mount": "FL"}}
                for name, (a, b) in dampers.items()
            ],
        }
        original = json.dumps(kinematics)
        self.path.write_text(original, encoding="utf-8-sig")

        imported = project.load_project(self.path)

        self.assertEqual(imported.source_format, project.KINEMATICS_FORMAT)
        self.assertIsNone(imported.topology)
        self.assertEqual(imported.loads, [])
        self.assertEqual(imported.case, ACCEL)
        self.assertEqual(imported.mass_kg, MASS_KG)
        self.assertEqual(imported.acceleration_mps2, DEFAULT_ACCELERATION)
        self.assertTrue(imported.include_gravity)
        self.assertEqual(imported.rise_time_s, DEFAULT_RISE_S)
        self.assertIsNone(imported.downforce_n)

        model = imported.model
        points["FL_PUSH_OUT_UCA"] = points.pop("FL_PUSH_OUT")
        self.assertEqual({name: model.points[name] for name in points}, points)
        self.assertNotIn("FL_PUSH_OUT", model.points)
        self.assertEqual(model.owners["FL_PUSH_OUT_UCA"], "FL_UCA")
        self.assertEqual(model.member_mounts, dict.fromkeys(dampers, ("FR", "FL")))
        for name, (a, b) in dampers.items():
            self.assertEqual(model.points[f"{name}_A"], a)
            self.assertEqual(model.points[f"{name}_B"], b)
            self.assertEqual(model.owners[f"{name}_A"], "FR_ROCKER")
            self.assertEqual(model.owners[f"{name}_B"], "FL_ROCKER")
        self.assertEqual(imported.sections, dict.fromkeys((rod.name for rod in model.rods), Section()))

        equilibrium = solve(model, example_loads(model, STATIC))
        self.assertEqual(equilibrium.rank, equilibrium.equations)
        self.assertEqual(len(equilibrium.forces), 18)
        self.assertAlmostEqual(equilibrium.forces["Roll_Damper"], 0., places=6)
        self.assertLess(equilibrium.forces["Heave_Damper"], 0.)

        native_path = self.path.with_name("statics.json")
        project.save_project(native_path, imported)
        self.assertEqual(self.path.read_text(encoding="utf-8-sig"), original)
        payload = json.loads(native_path.read_text(encoding="utf-8"))
        self.assertEqual(payload["format"], project.FORMAT)
        self.assertNotIn("source_format", payload)
        restored = project.load_project(native_path)
        self.assertEqual(restored.source_format, project.FORMAT)
        self.assertEqual(restored.model, imported.model)
        self.assertEqual(restored.sections, imported.sections)

    def test_custom_topology_and_invalid_section(self):
        topology = {"fixed": ["車身"], "rods": [{"name": "推桿", "a": "球頭", "b": "車身"}]}
        model = build_model({"球頭": (0., 0., 0.), "車身": (.5, 0., 0.)}, topology)
        saved = project.Project(model, topology, [Load("球頭", (50., 0., 0.))], {"推桿": Section(20., 0., 200.)},
                                ACCEL, 321., (0., 0., 0.), True)
        project.save_project(self.path, saved)

        self.assertEqual(project.load_project(self.path), saved)
        data = json.loads(self.path.read_text(encoding="utf-8"))
        del data["case_settings"]["rise_time_s"]
        del data["case_settings"]["downforce_n"]
        del data["case_settings"]["tire_mu"]
        self.path.write_text(json.dumps(data), encoding="utf-8")
        self.assertEqual(project.load_project(self.path), saved)
        data["sections"]["推桿"]["inner_mm"] = 21.
        self.path.write_text(json.dumps(data), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "外徑"):
            project.load_project(self.path)

    def test_failed_save_keeps_existing_file(self):
        model = build_model({"A": (0., 0., 0.)})
        saved = project.Project(model, None, [], {}, ACCEL, 321., (0., 0., 0.), True)
        project.save_project(self.path, saved)
        previous = self.path.read_bytes()
        with patch.object(project, "replace", side_effect=PermissionError("Locked file")):
            with self.assertRaises(PermissionError):
                project.save_project(self.path, saved)
        self.assertEqual(self.path.read_bytes(), previous)
        self.assertEqual(list(self.path.parent.iterdir()), [self.path])
