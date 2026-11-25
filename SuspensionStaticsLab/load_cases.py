"""Editable example load cases resolved into forces at the CG and tire contacts."""

import math

import numpy as np

from .model import number
from .solver import Load, solve_frames


STATIC = "靜態負載"
ACCEL = "重心加速"
GRIP = "摩擦圓工況"
BUMP = "單輪受衝擊"
CASES = (ACCEL, GRIP, BUMP)
MASS_KG = 321.
GRAVITY = 9.81
DEFAULT_TIRE_MU = 1.
DEFAULT_ACCELERATION = (0., 0., 0.)
DESCRIPTIONS = {
    ACCEL: "+X 前、+Y 左、+Z 上。變更後按「套用範例」更新載荷。",
    GRIP: "以輪胎摩擦係數和有效正向負載限制縱向與側向合成加速度。",
    BUMP: "第一個可用輪位承受 +Z 峰值 1800 N",
}


class WheelLiftError(ValueError):
    pass


def downforce_newtons(value):
    downforce = number(value, "額外下壓力")
    if downforce < 0.:
        raise ValueError("額外下壓力必須大於或等於 0 N。")
    return downforce


def tire_mu_coefficient(value):
    coefficient = number(value, "輪胎摩擦係數 μ")
    if coefficient < 0.:
        raise ValueError("輪胎摩擦係數 μ 必須大於或等於 0。")
    return coefficient


def _vehicle_mass(value):
    mass = number(value, "重心質量")
    if mass <= 0.:
        raise ValueError("重心質量必須大於零。")
    return mass


def _normal_load(mass, az, include_gravity, downforce):
    gravity = GRAVITY if include_gravity else 0.
    total = mass * gravity + mass * az + downforce
    if not math.isfinite(total):
        raise ValueError("重心質量、垂直加速度或額外下壓力超出可計算的範圍。")
    if total < 0.:
        raise WheelLiftError("目前工況的合計正向負載為負，車輪會離地。請調整垂直加速度或下壓力。")
    return total


def grip_acceleration_limit(mass_kg, az, *, include_gravity=True, downforce_n=0., tire_mu=DEFAULT_TIRE_MU):
    mass = _vehicle_mass(mass_kg)
    normal = _normal_load(mass, number(az, "Z 軸加速度"), include_gravity, downforce_newtons(downforce_n))
    coefficient = tire_mu_coefficient(tire_mu)
    if not coefficient or not normal:
        return 0.
    limit = coefficient * (normal / mass)
    if not math.isfinite(limit):
        raise ValueError("摩擦圓加速度上限超出可計算的範圍。")
    return limit


def example_loads(model, kind, *, mass_kg=MASS_KG, acceleration_mps2=DEFAULT_ACCELERATION, include_gravity=True,
                  downforce_n=0., tire_mu=DEFAULT_TIRE_MU):
    if kind != STATIC and kind not in CASES:
        raise ValueError("請選擇有效的範例工況。")
    if kind == BUMP:
        if not model.wheels:
            raise ValueError("單輪衝擊範例需要至少一個輪心 WC。")
        return [Load(model.wheels[0].contact, (0., 0., 1800.))]
    if len(model.wheels) != 4 or model.cg is None:
        raise ValueError("這個工況需要 FL、FR、RL、RR 四個輪心及重心。")
    required = [model.cg, *(wheel.contact for wheel in model.wheels)]
    if any(point not in model.owners for point in required):
        raise ValueError("範例需要把 CG 和接地點指派到剛體或車身。")

    contacts = np.asarray([model.points[wheel.contact] for wheel in model.wheels])
    cg = np.asarray(model.points[model.cg])
    relative = contacts - cg
    balance = np.vstack((np.ones(4), relative[:, 1], -relative[:, 0]))
    mass_kg = _vehicle_mass(mass_kg)
    acceleration = np.zeros(3)
    downforce = 0.
    if kind in (ACCEL, GRIP):
        downforce = downforce_newtons(downforce_n)
        tire_mu = tire_mu_coefficient(tire_mu)
        acceleration = np.asarray(acceleration_mps2, dtype=float)
        if acceleration.shape != (3,) or not np.isfinite(acceleration).all():
            raise ValueError("重心加速度需要三個有限的 XYZ 分量。")
    apparent_weight = _normal_load(mass_kg, float(acceleration[2]), kind == STATIC or include_gravity, downforce)
    longitudinal_demand = mass_kg * float(acceleration[0])
    lateral_demand = mass_kg * float(acceleration[1])
    if not math.isfinite(longitudinal_demand) or not math.isfinite(lateral_demand):
        raise ValueError("重心質量或加速度超出可計算的範圍。")
    circle = kind == GRIP
    longitudinal_ratio = lateral_ratio = 0.
    demand = math.hypot(longitudinal_demand, lateral_demand) if circle else abs(lateral_demand)
    if demand:
        if apparent_weight:
            lateral_ratio = lateral_demand / apparent_weight
            if circle:
                longitudinal_ratio = longitudinal_demand / apparent_weight
            required_mu = math.hypot(longitudinal_ratio, lateral_ratio)
        else:
            required_mu = math.inf
        # Comparing coefficients avoids overflow when μ × W exceeds the numeric range.
        tolerance = 1e-12 * max(required_mu, tire_mu)
        if not math.isfinite(required_mu) or required_mu - tire_mu > tolerance:
            limit = tire_mu * apparent_weight
            reason = "輪胎抓地力不足，超出摩擦圓" if circle else "側向抓地力不足"
            raise ValueError(f"{reason}（需求 {demand:.6g} N，上限 {limit:.6g} N）。"
                             "請調整加速度、輪胎摩擦係數 μ 或下壓力。")
    rear_x, front_x = relative[2:, 0].mean(), relative[:2, 0].mean()
    if abs(front_x - rear_x) < 1e-6:
        raise ValueError("範例需要不同 X 座標的前後軸。")
    front_share = -rear_x / (front_x - rear_x)
    nominal = np.repeat([front_share, 1. - front_share], 2) / 2.
    correction, _, rank, _ = np.linalg.lstsq(balance, (1., 0., 0.) - balance @ nominal, rcond=1e-10)
    if rank < 3:
        raise ValueError("四輪接地點在 XY 平面上沒有足夠的幾何範圍。")
    shares = nominal + correction
    reactions = np.zeros_like(relative, dtype=float)
    if not circle:
        reactions[:, 0] = shares * longitudinal_demand
    vertical_balance = balance
    if lateral_ratio or longitudinal_ratio:
        # Tire shear proportional to Fz changes the roll and pitch lever arms.
        vertical_balance = balance.copy()
        vertical_balance[1] -= lateral_ratio * relative[:, 2]
        vertical_balance[2] += longitudinal_ratio * relative[:, 2]
    initial = shares * apparent_weight
    target = (apparent_weight, 0., -np.dot(relative[:, 2], reactions[:, 0]))
    residual = target - vertical_balance @ initial
    if not np.isfinite(vertical_balance).all() or not np.isfinite(residual).all():
        raise ValueError("輪荷或力矩超出可計算的範圍。")
    increment, _, rank, _ = np.linalg.lstsq(vertical_balance, residual, rcond=1e-10)
    if rank < 3:
        raise ValueError("四輪接地點無法在指定的水平力分配下平衡垂直力、側傾及俯仰力矩。")
    reactions[:, 2] = initial + increment
    reactions[:, 1] = lateral_ratio * reactions[:, 2]
    if circle:
        reactions[:, 0] = longitudinal_ratio * reactions[:, 2]
    if not np.isfinite(reactions).all():
        raise ValueError("輪荷超出可計算的範圍。")
    if min(reactions[:, 2]) < -1e-6:
        raise WheelLiftError("目前工況會使至少一個車輪離地，請調整重心或加速度，或自行設定輪荷。")

    loads = [Load(model.cg, (-longitudinal_demand, -lateral_demand, -apparent_weight))]
    loads.extend(Load(wheel.contact, tuple(force)) for wheel, force in zip(model.wheels, reactions))
    return loads


def _grip_sample(model, settings, acceleration):
    try:
        return acceleration, example_loads(model, GRIP, acceleration_mps2=acceleration, **settings)
    except WheelLiftError:
        return None


def _refine_grip_boundary(sample, low, high, low_sample, high_sample=None):
    inside = low_sample is not None
    best = low_sample or high_sample
    for _ in range(24):
        middle = (low + high) / 2.
        candidate = sample(middle)
        if candidate is not None:
            best = candidate
        if (candidate is not None) == inside:
            low = middle
        else:
            high = middle
    return best


def fit_grip_acceleration(model, acceleration, **settings):
    ax, ay, az = acceleration

    def sample(factor):
        return _grip_sample(model, settings, (ax * factor, ay * factor, az))

    full = sample(1.)
    if full is not None:
        return *full, False
    center = sample(0.)
    if center is None:
        raise WheelLiftError("水平加速度為零時仍會輪胎離地。請檢查重心、垂直加速度及下壓力。")
    for step in range(23, 0, -1):
        candidate = sample(step / 24.)
        if candidate is not None:
            return *_refine_grip_boundary(sample, step / 24., (step + 1) / 24., candidate), True
    return *_refine_grip_boundary(sample, 0., 1. / 24., center), True


def scan_grip_region(model, az, *, mass_kg=MASS_KG, **settings):
    limit = grip_acceleration_limit(mass_kg, az, **settings)
    settings = dict(settings, mass_kg=mass_kg)
    center = _grip_sample(model, settings, (0., 0., az))
    samples = [center] if center is not None else []
    attempted = 1
    directions, rings = 120, 24
    if limit > 0.:
        for direction in range(directions):
            angle = math.tau * direction / directions
            ax, ay = limit * math.cos(angle), limit * math.sin(angle)

            def sample(radius):
                return _grip_sample(model, settings, (ax * radius, ay * radius, az))

            previous = center
            for ring in range(1, rings + 1):
                current = sample(ring / rings)
                attempted += 1
                if current is not None:
                    samples.append(current)
                if (previous is None) != (current is None):
                    samples.append(_refine_grip_boundary(sample, (ring - 1) / rings, ring / rings, previous, current))
                    attempted += 1
                previous = current
    if not samples:
        raise ValueError("掃描範圍內找不到符合輪胎接地條件的工況。")
    equilibria = solve_frames(model, tuple(loads for _, loads in samples), skip_unbalanced=True)
    peaks = {}
    feasible = 0
    for (acceleration, _), equilibrium in zip(samples, equilibria):
        if equilibrium is None:
            continue
        feasible += 1
        for member, force in equilibrium.forces.items():
            if member not in peaks or abs(force) > abs(peaks[member][1]):
                peaks[member] = member, force, acceleration
    if not feasible:
        raise ValueError("掃描範圍內找不到可平衡的工況。請檢查幾何及約束。")
    return sorted(peaks.values(), key=lambda peak: -abs(peak[1])), feasible, attempted
