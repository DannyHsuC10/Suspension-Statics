"""Build and solve quasi-static force histories on fixed hardpoints."""

import csv
from dataclasses import dataclass
from functools import cached_property
from pathlib import Path

import numpy as np

from .load_cases import ACCEL, DEFAULT_TIRE_MU, GRAVITY, GRIP, example_loads
from .model import CORNERS, number
from .solver import FrameError, Load, solve_frames


DEFAULT_RISE_S = 1.
RISE_SAMPLES = 101


@dataclass(frozen=True)
class RodStatistics:
    peak_n: float
    peak_time_s: float
    mean_n: float
    mean_absolute_n: float


@dataclass(frozen=True)
class History:
    time_s: np.ndarray
    loads: tuple
    equilibria: tuple
    force_n: dict[str, np.ndarray]
    acceleration_mps2: np.ndarray
    source: str

    @cached_property
    def largest_loads(self):
        """Return the governing rod and its signed force at each time."""
        names = np.array(tuple(self.force_n))
        forces = np.stack(tuple(self.force_n.values()))
        indices = np.argmax(np.abs(forces), axis=0)
        return names[indices], forces[indices, np.arange(len(self.time_s))]

    def statistics(self, name):
        force = self.force_n[name]
        peak = np.argmax(np.abs(force))
        weights = np.diff(self.time_s) / (self.time_s[-1] - self.time_s[0])
        left, right = force[:-1], force[1:]
        mean = np.dot(weights, .5 * left + .5 * right)
        a, b = np.abs(left), np.abs(right)
        absolute = .5 * a + .5 * b
        # Integrate the two triangles separately when signed force crosses zero.
        crossing = (np.signbit(left) != np.signbit(right)) & (a > 0.) & (b > 0.)
        size = np.maximum(a[crossing], b[crossing])
        scaled_a, scaled_b = a[crossing] / size, b[crossing] / size
        fraction = scaled_a / (scaled_a + scaled_b)
        absolute[crossing] = (.5 * a[crossing]) * fraction + (.5 * b[crossing]) * (1. - fraction)
        return RodStatistics(float(force[peak]), float(self.time_s[peak]), float(mean), float(np.dot(weights, absolute)))

    def peak_stress(self, sections):
        peak = max(float(np.max(np.abs(values))) / sections[name].area_mm2()
                   for name, values in self.force_n.items())
        if not np.isfinite(peak):
            raise ValueError("歷程應力超出可計算的範圍。")
        return peak


def rise_seconds(value):
    duration = number(value, "加速度上升時間")
    if duration <= 0.:
        raise ValueError("加速度上升時間必須大於 0 秒。")
    return duration


def solve_history(model, times, loads, acceleration, source):
    try:
        equilibria = solve_frames(model, loads)
    except FrameError as error:
        raise ValueError(f"t = {times[error.index]:.6g} s：{error}") from error
    names = equilibria[0].forces
    values = {name: np.array([frame.forces[name] for frame in equilibria]) for name in names}
    return History(times, tuple(loads), equilibria, values, acceleration, source)


def acceleration_frames(model, times, accelerations, mass_kg, include_gravity, *, downforce_n=0.,
                        tire_mu=DEFAULT_TIRE_MU, case=ACCEL):
    if case not in (ACCEL, GRIP):
        raise ValueError("加速度歷程需要重心加速或摩擦圓工況。")
    frames = []
    for at, acceleration in zip(times, accelerations):
        try:
            loads = example_loads(model, case, mass_kg=mass_kg, acceleration_mps2=acceleration,
                                  include_gravity=include_gravity, downforce_n=downforce_n, tire_mu=tire_mu)
        except ValueError as error:
            raise ValueError(f"t = {at:.6g} s：{error}") from error
        frames.append(loads)
    return frames


def acceleration_history(model, duration, acceleration, *, mass_kg, include_gravity, downforce_n=0.,
                         tire_mu=DEFAULT_TIRE_MU, case=ACCEL):
    duration = rise_seconds(duration)
    times = np.linspace(0., duration, RISE_SAMPLES)
    ratios = np.linspace(0., 1., RISE_SAMPLES)
    accelerations = np.outer(ratios, acceleration)
    frames = acceleration_frames(model, times, accelerations, mass_kg, include_gravity,
                                 downforce_n=downforce_n, tire_mu=tire_mu, case=case)
    return solve_history(model, times, frames, accelerations, "重心加速上升歷程")


def read_lap_csv(path):
    """Read time, optional wheel forces, and acceleration channels in m/s²."""
    path = Path(path)
    with path.open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle, strict=True)
        names = [name.strip().lower() for name in (reader.fieldnames or ())]
        fields = set(names)
        if len(names) != len(fields) or "time_s" not in fields:
            raise ValueError("圈速 CSV 需要唯一欄名及 time_s 時間欄位。")
        reader.fieldnames = names
        wheel_forces = [f"f{axis}_{corner.lower()}_n" for corner in CORNERS for axis in "xyz"]
        direct = set(wheel_forces) <= fields
        columns = []
        for axis, alias in zip("xyz", ("long_g", "lat_g", "")):
            options = ((f"a{axis}_m_s2", 1.), (f"a{axis}_mps2", 1.), (f"a{axis}_g", GRAVITY), (alias, GRAVITY))
            found = next(((name, scale) for name, scale in options if name and name in fields), None)
            if found is None and not direct and axis != "z":
                raise ValueError("圈速 CSV 需要四輪 Fx/Fy/Fz，或提供縱向與側向加速度欄位。")
            columns.append(found or (None, 1.))

        times, force_rows, accel_rows = [], [], []
        for row in reader:
            label = f"CSV 第 {reader.line_num} 列"
            if None in row or None in row.values():
                raise ValueError(f"{label}的欄數不一致。")
            time = number(row["time_s"], f"{label} time_s")
            if times and time <= times[-1]:
                raise ValueError(f"{label}的 time_s 必須大於前一筆。")
            times.append(time)
            if direct:
                force_rows.append([number(row[name], f"{label} {name}") for name in wheel_forces])
            accel = []
            for name, scale in columns:
                if not name or (direct and not row[name].strip()):
                    accel.append(np.nan if direct else 0.)
                    continue
                value = number(row[name], f"{label} {name}") * scale
                if not np.isfinite(value):
                    raise ValueError(f"{label} {name} 換算後超出數值範圍。")
                accel.append(value)
            accel_rows.append(accel)
    if len(times) < 2:
        raise ValueError("圈速 CSV 需要至少兩筆有效的時間資料。")
    if not np.isfinite(times[-1] - times[0]):
        raise ValueError("圈速 CSV 的時間範圍超出數值範圍。")
    return np.asarray(times), np.asarray(force_rows) if direct else None, np.asarray(accel_rows)


def lap_history(model, path, *, mass_kg, include_gravity, downforce_n=0., tire_mu=DEFAULT_TIRE_MU, case=ACCEL):
    times, forces, accelerations = read_lap_csv(path)
    if forces is not None:
        wheels = {wheel.corner: wheel.contact for wheel in model.wheels}
        if set(wheels) != set(CORNERS):
            raise ValueError("使用四輪接地力需要 FL、FR、RL、RR 四個輪位。")
        frames = [[Load(wheels[corner], tuple(xyz)) for corner, xyz in zip(CORNERS, row)]
                  for row in forces.reshape(-1, 4, 3)]
        source = "CSV 四輪接地力"
    else:
        frames = acceleration_frames(model, times, accelerations, mass_kg, include_gravity,
                                     downforce_n=downforce_n, tire_mu=tire_mu, case=case)
        source = "CSV 加速度換算輪荷"
    return solve_history(model, times, frames, accelerations, f"{Path(path).name} · {source}")
