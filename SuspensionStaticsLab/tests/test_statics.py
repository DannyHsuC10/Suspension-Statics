import csv
import json
import tempfile
import unittest
from math import pi, sqrt
from pathlib import Path

import numpy as np

from SuspensionStaticsLab.interference import find_interferences
from SuspensionStaticsLab.load_cases import ACCEL, BUMP, CASES, GRAVITY, GRIP, STATIC, example_loads, grip_acceleration_limit
from SuspensionStaticsLab.model import (
    CORNERS, EXAMPLE, build_model, load_model_csv, read_hardpoint_table, read_hardpoints, write_hardpoint_csv,
)
from SuspensionStaticsLab.solver import Load, Section, member_results, solve


def rod(name, a, b):
    return {"name": name, "a": a, "b": b}


class StaticsTests(unittest.TestCase):
    def test_pin_forces_sign_and_original_length_postprocessing(self):
        points = {"O": (0, 0, 0), "X": (.5, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 2)}
        model = build_model(points, {"fixed": list("XYZ"), "rods": [rod(axis, "O", axis) for axis in "XYZ"]})
        result = solve(model, [Load("O", (1000, -400, 0))])
        self.assertEqual(result.forces, {"X": -1000., "Y": 400., "Z": 0.})
        self.assertEqual(result.support_force, (-1000., 400., 0.))
        sections = dict.fromkeys(result.forces, Section(20, 10, 200))
        member = member_results(result, sections)["X"]
        self.assertAlmostEqual(member.area_mm2, 75 * pi)
        self.assertAlmostEqual(member.stress_mpa, -1000 / (75 * pi))
        self.assertAlmostEqual(member.strain, -1000 / (75 * pi * 200_000))
        self.assertAlmostEqual(member.change_mm, -1000 * 500 / (75 * pi * 200_000))
        self.assertEqual(model.points, points)

    def test_couple_is_balanced_by_axial_forces_with_lever_arms(self):
        points = {
            "O": (0, 0, 0), "X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1),
            "AX": (1, 0, 0), "AY": (0, 1, 0), "AZ": (0, 0, 1),
            "MX": (0, 1, 1), "MY": (1, 0, 1), "MZ": (1, 1, 0),
        }
        members = [rod(f"F{axis}", "O", f"A{axis}") for axis in "XYZ"]
        members += [rod(f"M{axis}", point, f"M{axis}") for axis, point in zip("XYZ", "YZX")]
        topology = {"fixed": ["AX", "AY", "AZ", "MX", "MY", "MZ"],
                    "bodies": {"upright": list("OXYZ")}, "rods": members}
        result = solve(build_model(points, topology), [Load("FX", (30, -40, 50), "A")])
        np.testing.assert_allclose(list(result.forces.values()), [-40, 50, 30, -30, 40, -50])
        np.testing.assert_allclose(result.support_moment, [-30, 40, -50])

    def test_insufficient_and_redundant_support(self):
        points = {"O": (0, 0, 0), "X": (1, 0, 0)}
        members = [rod("first", "O", "X")]
        model = build_model(points, {"fixed": ["X"], "rods": members})
        with self.assertRaisesRegex(ValueError, "無法平衡"):
            solve(model, [Load("O", (0, 50, 0))])
        with self.assertRaisesRegex(ValueError, "只傳力"):
            solve(model, [Load("first", (0, 0, 10), "A")])
        model = build_model(points, {"fixed": ["X"], "rods": members + [rod("second", "O", "X")]})
        with self.assertRaisesRegex(ValueError, "無唯一解"):
            solve(model, [Load("O", (50, 0, 0))])

    def test_example_cases_balance_all_corners(self):
        model = build_model(read_hardpoints(EXAMPLE))
        baseloads = example_loads(model, STATIC)
        self.assertNotIn(STATIC, CASES)
        self.assertEqual(example_loads(model, ACCEL), baseloads)
        baseline = np.array([load.xyz[2] for load in baseloads[1:5]])
        for case in CASES:
            with self.subTest(case=case):
                loads = example_loads(model, case, acceleration_mps2=(.65 * GRAVITY, 0., 0.))
                result = solve(model, loads)
                self.assertEqual(len(result.forces), 16)
                self.assertEqual(result.rank, result.equations)
                if case == BUMP:
                    self.assertEqual(result.support_force, (0., 0., -1800.))
                    np.testing.assert_allclose([v for n, v in result.forces.items() if not n.startswith("FL_")], 0., atol=1e-9)
                else:
                    np.testing.assert_allclose(result.support_force, 0., atol=1e-9)
                    np.testing.assert_allclose(result.support_moment, 0., atol=1e-9)
                    tire_z = np.array([load.xyz[2] for load in loads[1:5]])
                    if case in (ACCEL, GRIP):
                        self.assertLess(tire_z[:2].sum(), baseline[:2].sum())

    def test_cg_acceleration_combines_transfer_and_apparent_weight(self):
        points = read_hardpoints(EXAMPLE)
        for corner, height in zip(CORNERS, (.015, -.030, .022, .004)):
            x, y, _ = points[f"{corner}_WC"]
            points[f"{corner}_CONTACT"] = (x, y, height)
        model = build_model(points)
        mass = 280.
        ax, ay, az = 3., 4., 1.5
        weight = mass * (GRAVITY + az)
        lateral_ratio = mass * ay / weight

        def contact_loads(components):
            loads = example_loads(model, ACCEL, mass_kg=mass, acceleration_mps2=components)
            return np.array([load.xyz for load in loads[1:]])

        baseline = contact_loads((0., 0., 0.))
        longitudinal = contact_loads((ax, 0., 0.))
        lateral = contact_loads((0., ay, 0.))
        combined = contact_loads((ax, ay, az))
        self.assertLess(longitudinal[:2, 2].sum(), baseline[:2, 2].sum())
        self.assertGreater(lateral[1::2, 2].sum(), baseline[1::2, 2].sum())
        np.testing.assert_allclose(lateral.sum(axis=0), (0., mass * ay, mass * GRAVITY))
        np.testing.assert_array_equal(combined[:, 0], longitudinal[:, 0])
        np.testing.assert_allclose(combined[:, 1], lateral_ratio * combined[:, 2], rtol=1e-12)
        np.testing.assert_allclose(combined.sum(axis=0), (mass * ax, mass * ay, weight))
        relative = np.array([points[wheel.contact] for wheel in model.wheels]) - model.points[model.cg]
        moment = np.cross(relative, combined).sum(axis=0)
        np.testing.assert_allclose(moment[:2], 0., atol=1e-9)
        self.assertGreater(abs(moment[2]), 1.)
        loads = example_loads(model, ACCEL, mass_kg=mass, acceleration_mps2=(ax, ay, az))
        np.testing.assert_allclose(loads[0].xyz, (-mass * ax, -mass * ay, -weight))
        solution = solve(model, loads)
        np.testing.assert_allclose(solution.support_force, 0., atol=1e-9)
        np.testing.assert_allclose(solution.support_moment, (0., 0., -moment[2]), atol=1e-9)
        with self.assertRaisesRegex(ValueError, "離地"):
            contact_loads((0., 0., -2. * GRAVITY))
        with self.assertRaisesRegex(ValueError, "質量"):
            example_loads(model, ACCEL, mass_kg=0.)

    def test_circular_section_properties_about_centroid(self):
        for inner in (0., 10.):
            with self.subTest(inner=inner):
                outer = 20.
                properties = Section(outer, inner).properties()
                area = pi / 4. * (outer ** 2 - inner ** 2)
                second_moment = pi / 64. * (outer ** 4 - inner ** 4)
                self.assertAlmostEqual(properties.area_mm2, area)
                self.assertAlmostEqual(properties.second_moment_mm4, second_moment)
                self.assertAlmostEqual(properties.modulus_mm3, second_moment / (outer / 2.))
                self.assertAlmostEqual(properties.gyration_mm, sqrt(second_moment / area))
                self.assertAlmostEqual(properties.polar_moment_mm4, 2. * second_moment)
                self.assertAlmostEqual(properties.polar_modulus_mm3, 2. * second_moment / (outer / 2.))
                self.assertAlmostEqual(properties.polar_gyration_mm, sqrt(2. * second_moment / area))
        with self.assertRaisesRegex(ValueError, "外徑"):
            Section(19., 19.).properties()

    def test_cg_gravity_toggle_includes_weight_once(self):
        model = build_model(read_hardpoints(EXAMPLE))
        static = example_loads(model, STATIC)
        with_gravity = example_loads(model, ACCEL, acceleration_mps2=(0., 0., 0.), include_gravity=True)
        self.assertEqual(with_gravity, static)
        self.assertGreater(max(map(abs, solve(model, with_gravity).forces.values())), 1000.)
        zero = example_loads(model, ACCEL, acceleration_mps2=(0., 0., 0.), include_gravity=False)
        self.assertTrue(all(load.xyz == (0., 0., 0.) for load in zero))
        self.assertEqual(max(map(abs, solve(model, zero).forces.values())), 0.)
        vertical = example_loads(model, ACCEL, mass_kg=200., acceleration_mps2=(0., 0., 2.), include_gravity=False)
        self.assertAlmostEqual(vertical[0].xyz[2], -400.)
        self.assertAlmostEqual(sum(load.xyz[2] for load in vertical[1:]), 400.)

    def test_cg_downforce_preserves_lateral_demand_with_or_without_gravity(self):
        model = build_model(read_hardpoints(EXAMPLE))
        mass, downforce = 280., 800.
        ax, ay, az = 1.2, -1., 1.5
        contact_z = model.points[model.wheels[0].contact][2]
        cg_height = model.points[model.cg][2] - contact_z
        for gravity in (False, True):
            with self.subTest(gravity=gravity):
                loads = example_loads(model, ACCEL, mass_kg=mass, acceleration_mps2=(ax, ay, az),
                                      include_gravity=gravity, downforce_n=downforce)
                expected_z = mass * (az + (GRAVITY if gravity else 0.)) + downforce
                lateral_ratio = mass * ay / expected_z
                self.assertEqual(loads[0].target, model.cg)
                self.assertEqual([load.target for load in loads[1:]], [wheel.contact for wheel in model.wheels])
                np.testing.assert_allclose(loads[0].xyz, (-mass * ax, -mass * ay, -expected_z))
                contacts = np.array([load.xyz for load in loads[1:]])
                np.testing.assert_allclose(contacts[:, 1], lateral_ratio * contacts[:, 2])
                np.testing.assert_allclose(contacts.sum(axis=0), (mass * ax, mass * ay, expected_z))
                result = solve(model, loads)
                np.testing.assert_allclose(result.support_force, 0., atol=1e-9)
                np.testing.assert_allclose(result.support_moment, (0., 0., cg_height * mass * ax * lateral_ratio), atol=1e-9)

    def test_cg_downforce_can_prevent_lateral_traction_failure_and_wheel_lift(self):
        model = build_model(read_hardpoints(EXAMPLE))
        mass = 321.
        for acceleration_g, coefficient, error in ((1.2, 1., "側向抓地力不足"), (2.1, 3., "離地")):
            with self.subTest(acceleration_g=acceleration_g, coefficient=coefficient):
                ay = acceleration_g * GRAVITY
                settings = {"mass_kg": mass, "acceleration_mps2": (0., ay, 0.), "tire_mu": coefficient}
                with self.assertRaisesRegex(ValueError, error):
                    example_loads(model, ACCEL, **settings)
                loads = example_loads(model, ACCEL, downforce_n=1000., **settings)
                tires = np.array([load.xyz for load in loads[1:]])
                weight = mass * GRAVITY + 1000.
                self.assertTrue(np.all(tires[:, 2] > 0.))
                self.assertTrue(np.all(np.abs(tires[:, 1]) <= coefficient * tires[:, 2]))
                np.testing.assert_allclose(tires.sum(axis=0), (0., mass * ay, weight))
                np.testing.assert_allclose(tires[:, 1], mass * ay / weight * tires[:, 2])
                np.testing.assert_allclose(solve(model, loads).support_moment, 0., atol=1e-9)

    def test_cg_tire_mu_requires_finite_nonnegative_coefficient(self):
        model = build_model(read_hardpoints(EXAMPLE))
        for value in (-1., float("nan"), float("inf"), True, ""):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "輪胎摩擦係數"):
                example_loads(model, ACCEL, tire_mu=value)
        with self.assertRaisesRegex(ValueError, "側向抓地力不足"):
            example_loads(model, ACCEL, acceleration_mps2=(0., 1., 0.), tire_mu=0.)
        with self.assertRaisesRegex(ValueError, "側向抓地力不足"):
            example_loads(model, ACCEL, acceleration_mps2=(0., 1., 0.), include_gravity=False, tire_mu=4.)
        boundary = example_loads(model, ACCEL, acceleration_mps2=(0., -.3 * GRAVITY, 0.), tire_mu=.3)
        tires = np.array([load.xyz for load in boundary[1:]])
        np.testing.assert_allclose(tires[:, 1], -.3 * tires[:, 2])

    def test_grip_case_limits_combined_force_and_balances_both_lever_arms(self):
        model = build_model(read_hardpoints(EXAMPLE))
        mass = 280.
        acceleration = np.array((.6 * GRAVITY, -.6 * GRAVITY, 0.))
        settings = {"mass_kg": mass, "acceleration_mps2": acceleration, "tire_mu": .8}
        self.assertTrue(example_loads(model, ACCEL, **settings))
        with self.assertRaisesRegex(ValueError, "超出摩擦圓"):
            example_loads(model, GRIP, **settings)
        with self.assertRaisesRegex(ValueError, "超出摩擦圓"):
            example_loads(model, GRIP, acceleration_mps2=(1., 0., 0.), include_gravity=False)

        downforce = 500.
        loads = example_loads(model, GRIP, downforce_n=downforce, **settings)
        tires = np.array([load.xyz for load in loads[1:]])
        weight = mass * GRAVITY + downforce
        np.testing.assert_allclose(loads[0].xyz, (*(-mass * acceleration[:2]), -weight))
        np.testing.assert_allclose(tires.sum(axis=0), (*(mass * acceleration[:2]), weight))
        np.testing.assert_allclose(tires[:, :2], tires[:, 2:] * mass * acceleration[:2] / weight)
        self.assertTrue(np.all(np.linalg.norm(tires[:, :2], axis=1) <= .8 * tires[:, 2]))
        result = solve(model, loads)
        np.testing.assert_allclose(result.support_force, 0., atol=1e-9)
        np.testing.assert_allclose(result.support_moment, 0., atol=1e-9)

    def test_grip_acceleration_limit_tracks_normal_load(self):
        self.assertAlmostEqual(grip_acceleration_limit(100., 2., tire_mu=.8), .8 * (GRAVITY + 2.))
        self.assertAlmostEqual(grip_acceleration_limit(100., 2., downforce_n=300., tire_mu=.8),
                               .8 * (GRAVITY + 5.))
        self.assertAlmostEqual(grip_acceleration_limit(100., 2., include_gravity=False,
                                                       downforce_n=300., tire_mu=.8), 4.)
        self.assertEqual(grip_acceleration_limit(100., 0., tire_mu=0.), 0.)
        with self.assertRaisesRegex(ValueError, "離地"):
            grip_acceleration_limit(100., -2., include_gravity=False)

    def test_cg_downforce_zero_validation_and_other_cases(self):
        model = build_model(read_hardpoints(EXAMPLE))
        self.assertEqual(example_loads(model, ACCEL, downforce_n=0.), example_loads(model, ACCEL))
        for case in (STATIC, BUMP):
            self.assertEqual(example_loads(model, case, downforce_n=800.), example_loads(model, case))
        for value in (-1., float("nan"), float("inf"), True, ""):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "額外下壓力"):
                example_loads(model, ACCEL, downforce_n=value)

    def test_csv_mm_and_midspan_pushrod(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "points.csv"
            path.write_text("corner,point,x_mm,y_mm,z_mm\nFL,UFI,12,34,56\n", encoding="utf-8-sig")
            self.assertEqual(read_hardpoints(path), {"FL_UFI": (.012, .034, .056)})
        damper_example = Path(__file__).resolve().parents[1] / "examples" / "hardpoints.csv"
        points = read_hardpoints(damper_example)
        points["FL_PUSH_OUT"] = points.pop("FL_PUSH_OUT_UPRIGHT")
        model = build_model(points)
        result = solve(model, example_loads(model, STATIC))
        self.assertEqual(result.rank, result.equations)
        self.assertIn("FL_PUSH_ROD", result.forces)
        self.assertIn("FL_DAMPER", result.forces)
        self.assertNotIn("FL_LCA_F-LBJ", result.forces)

    def test_kinematics_csv_imports_and_exports_a_shared_rocker_damper(self):
        endpoint_a, endpoint_b = (.01, -.16, .42), (.02, .17, .35)
        metadata = {"version": 1, "deformable_members": [
            {"name": "DAMPER_1", "a": {"xyz": endpoint_a, "mount": "FR"},
             "b": {"xyz": endpoint_b, "mount": "FL"}},
        ]}
        with EXAMPLE.open(encoding="utf-8-sig", newline="") as handle:
            reader = csv.reader(handle)
            header = next(reader)
            rows = [row for row in reader if row[0].startswith(("FL_", "FR_"))]
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "kinematics.csv"
            with source.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow([*header, "jacobian_project"])
                writer.writerow([*rows[0], json.dumps(metadata)])
                writer.writerows([*row, ""] for row in rows[1:])
            model = load_model_csv(source)
            point_a, point_b = "DAMPER_1_A", "DAMPER_1_B"
            self.assertEqual(model.member_mounts, {"DAMPER_1": ("FR", "FL")})
            self.assertEqual((model.points[point_a], model.points[point_b]), (endpoint_a, endpoint_b))
            self.assertEqual((model.owners[point_a], model.owners[point_b]), ("FR_ROCKER", "FL_ROCKER"))
            self.assertEqual((model.owners["FR_PUSH_IN"], model.owners["FL_PUSH_IN"]), ("FR_ROCKER", "FL_ROCKER"))
            dampers = [(member.name, member.a, member.b) for member in model.rods if "DAMPER" in member.name]
            self.assertEqual(dampers, [("DAMPER_1", point_a, point_b)])

            axis = np.subtract(endpoint_b, endpoint_a)
            force = 250. * axis / np.linalg.norm(axis)
            result = solve(model, [Load(point_a, tuple(-force)), Load(point_b, tuple(force))])
            self.assertAlmostEqual(result.forces["DAMPER_1"], 250.)
            np.testing.assert_allclose(result.support_force, 0., atol=1e-9)
            np.testing.assert_allclose(result.support_moment, 0., atol=1e-9)

            points, mounts = read_hardpoint_table(source)
            points[point_b] = (.02, .18, .36)
            output = Path(directory) / "edited.csv"
            write_hardpoint_csv(output, points, mounts)
            restored = load_model_csv(output)
            self.assertEqual(restored.member_mounts, model.member_mounts)
            self.assertEqual(restored.owners, model.owners)
            self.assertEqual(restored.rods, model.rods)
            self.assertEqual(restored.points[point_b], points[point_b])

    def test_single_corner_without_prefix_survives_geometry_edit(self):
        points = {name[3:]: xyz for name, xyz in read_hardpoints(EXAMPLE).items() if name.startswith("FL_")}
        original = build_model(points)
        self.assertIn("FL_CONTACT", original.points)
        edited = build_model({**original.points, "WC": (.005, .625, .196)})
        self.assertEqual(edited.wheels, original.wheels)
        self.assertEqual(edited.rods, original.rods)
        self.assertEqual(solve(edited, example_loads(edited, BUMP)).rank, 12)

    def test_rod_interference_uses_finite_3d_segments_and_outside_diameters(self):
        points = {"A": (-.1, 0, 0), "B": (.1, 0, 0), "C": (0, -.1, 0), "D": (0, .1, 0)}
        topology = {"rods": [rod("x", "A", "B"), rod("y", "C", "D")]}
        sections = {"x": Section(20, 15), "y": Section(20, 15)}
        hit, = find_interferences(build_model(points, topology), sections)
        self.assertEqual((hit.rod_a, hit.rod_b), ("x", "y"))
        self.assertAlmostEqual(hit.distance_mm, 0.)
        self.assertAlmostEqual(hit.overlap_mm, 20.)

        raised = build_model({**points, "C": (0, -.1, .025), "D": (0, .1, .025)}, topology)
        self.assertFalse(find_interferences(raised, sections))
        sections["y"] = Section(30, 15)
        self.assertFalse(find_interferences(raised, sections))  # Exactly tangent.
        sections["y"] = Section(32, 15)
        hit, = find_interferences(raised, sections)
        self.assertAlmostEqual(hit.distance_mm, 25.)
        self.assertAlmostEqual(hit.required_mm, 26.)
        self.assertAlmostEqual(hit.overlap_mm, 1.)

        shortened = build_model({**points, "C": (0, .05, 0)}, topology)
        self.assertFalse(find_interferences(shortened, sections))

    def test_rod_interference_excludes_the_shared_joint_but_checks_the_shafts(self):
        points = {
            "origin": (0., 0., 0.), "same": (.5e-9, 0., 0.),
            "x_end": (.15, 0., 0.), "y_end": (0., .15, 0.), "shallow_end": (.15, .015, 0.),
        }
        topology = {"rods": [rod("x", "origin", "x_end"), rod("y", "y_end", "same"),
                             rod("shallow", "same", "shallow_end")]}
        sections = dict.fromkeys(("x", "y", "shallow"), Section(20, 15))
        hit, = find_interferences(build_model(points, topology), sections)
        self.assertEqual((hit.rod_a, hit.rod_b), ("shallow", "x"))
        self.assertGreater(hit.distance_mm, 0.)  # The intersecting joint area was removed.
        self.assertGreater(hit.overlap_mm, 0.)

    def test_rod_interference_includes_same_and_reverse_coincident_segments(self):
        points = {"A": (0., 0., 0.), "B": (.15, 0., 0.)}
        topology = {"rods": [rod("second", "A", "B"), rod("first", "A", "B"), rod("reverse", "B", "A")]}
        sections = {"first": Section(30, 15), "second": Section(20, 15), "reverse": Section(35, 15)}
        hits = find_interferences(build_model(points, topology), sections)
        self.assertEqual([(hit.rod_a, hit.rod_b) for hit in hits],
                         [("first", "reverse"), ("reverse", "second"), ("first", "second")])
        self.assertTrue(all(hit.distance_mm == 0. for hit in hits))


if __name__ == "__main__":
    unittest.main()
