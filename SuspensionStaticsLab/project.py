"""Load suspension projects and save self-contained statics project JSON."""

from dataclasses import asdict, dataclass
import json
from os import replace
from pathlib import Path
from tempfile import NamedTemporaryFile

from .history import DEFAULT_RISE_S, rise_seconds
from .load_cases import (ACCEL, CASES, DEFAULT_ACCELERATION, DEFAULT_TIRE_MU, MASS_KG, STATIC,
                         downforce_newtons, tire_mu_coefficient)
from .model import Model, build_kinematics_model, build_model, number
from .solver import Load, Section


FORMAT = "SuspensionStaticsLab"
KINEMATICS_FORMAT = "JacobianKinematicsLab"
SCHEMA_VERSION = 1


@dataclass(frozen=True)
class Project:
    model: Model
    topology: dict | None
    loads: list[Load]
    sections: dict[str, Section]
    case: str
    mass_kg: float
    acceleration_mps2: tuple[float, float, float]
    include_gravity: bool
    rise_time_s: float = DEFAULT_RISE_S
    downforce_n: float | None = None
    source_format: str = FORMAT
    tire_mu: float = DEFAULT_TIRE_MU


def vector(values, label):
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(f"{label}需要三個 XYZ 分量。")
    return tuple(number(value, label) for value in values)


def is_kinematics_project(data):
    return isinstance(data, dict) and "format" not in data and "points" in data


def load_project(path):
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if is_kinematics_project(data):
        model = build_kinematics_model(data)
        sections = {rod.name: Section() for rod in model.rods}
        return Project(model, None, [], sections, ACCEL, MASS_KG, DEFAULT_ACCELERATION, True,
                       source_format=KINEMATICS_FORMAT)
    if not isinstance(data, dict) or data.get("format") != FORMAT:
        raise ValueError("請選擇 SuspensionStaticsLab 或 JacobianKinematicsLab 專案 JSON。")
    if type(data.get("schema_version")) is not int or data["schema_version"] != SCHEMA_VERSION:
        raise ValueError("此專案的檔案版本目前無法開啟。")

    raw_points = data.get("hardpoints_m")
    if not isinstance(raw_points, dict) or not raw_points or any(not name.strip() for name in raw_points):
        raise ValueError("專案需要具名的硬點座標。")
    points = {name: vector(xyz, f"硬點 {name}") for name, xyz in raw_points.items()}
    topology = data.get("topology")
    model = build_model(points, topology, member_mounts=data.get("member_mounts"))
    rods = {rod.name: rod for rod in model.rods}

    raw_sections = data.get("sections")
    if not isinstance(raw_sections, dict) or raw_sections.keys() != rods.keys():
        raise ValueError("專案截面名稱必須與桿件清單相符。")
    sections = {}
    for name, values in raw_sections.items():
        if not isinstance(values, dict):
            raise ValueError(f"{name} 的截面設定格式有誤。")
        section = Section(*(number(values.get(field), f"{name} 的{label}") for field, label in
                            (("outer_mm", "外徑"), ("inner_mm", "內徑"), ("young_gpa", "楊氏模數"))))
        section.properties()
        section.young_mpa()
        sections[name] = section

    raw_loads = data.get("loads")
    if not isinstance(raw_loads, list):
        raise ValueError("專案需要外載清單。")
    loads = []
    load_points = model.owners.keys() | {point for rod in model.rods for point in (rod.a, rod.b)}
    for index, values in enumerate(raw_loads, 1):
        if not isinstance(values, dict):
            raise ValueError(f"第 {index} 筆外載格式有誤。")
        target, end = values.get("target"), values.get("end")
        if not isinstance(target, str) or end not in ("", "A", "B"):
            raise ValueError(f"第 {index} 筆外載需要有效的目標和型態。")
        if end:
            rod = rods.get(target)
            point = (rod.a if end == "A" else rod.b) if rod else None
            if point not in model.owners:
                raise ValueError(f"第 {index} 筆力矩需要連到剛體的有效桿端。")
        elif target not in load_points:
            raise ValueError(f"第 {index} 筆外力需要有效的硬點。")
        loads.append(Load(target, vector(values.get("xyz"), f"第 {index} 筆外載"), end))

    settings = data.get("case_settings")
    case = settings.get("case") if isinstance(settings, dict) else None
    legacy_static = case in (STATIC, "static", "空力下壓")
    if legacy_static:
        case = ACCEL
    if case not in CASES:
        raise ValueError("專案的工況設定有誤。")
    mass = number(settings.get("mass_kg"), "重心質量")
    acceleration = DEFAULT_ACCELERATION if legacy_static else vector(settings.get("acceleration_mps2"), "重心加速度")
    gravity = settings.get("include_gravity")
    if type(gravity) is not bool:
        raise ValueError("專案需要有效的重力勾選狀態。")
    rise = rise_seconds(settings.get("rise_time_s", DEFAULT_RISE_S))
    downforce = settings.get("downforce_n")
    if downforce is not None:
        downforce = downforce_newtons(downforce)
    tire_mu = tire_mu_coefficient(settings.get("tire_mu", DEFAULT_TIRE_MU))
    return Project(model, topology, loads, sections, case, mass, acceleration, gravity, rise, downforce, tire_mu=tire_mu)


def save_project(path, project):
    path = Path(path)
    data = {
        "format": FORMAT,
        "schema_version": SCHEMA_VERSION,
        "hardpoints_m": project.model.points,
        "topology": project.topology,
        "loads": [asdict(load) for load in project.loads],
        "sections": {name: asdict(section) for name, section in project.sections.items()},
        "case_settings": {
            "case": project.case,
            "mass_kg": project.mass_kg,
            "acceleration_mps2": project.acceleration_mps2,
            "include_gravity": project.include_gravity,
            "rise_time_s": project.rise_time_s,
            "downforce_n": project.downforce_n,
            "tire_mu": tire_mu_coefficient(project.tire_mu),
        },
    }
    if project.model.member_mounts:
        data["member_mounts"] = project.model.member_mounts
    payload = json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    temporary = None
    try:
        with NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent, delete=False) as handle:
            temporary = Path(handle.name)
            handle.write(payload)
        replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
