"""Read hardpoints and describe axial load paths through rigid assemblies."""

import csv
import json
import math
from dataclasses import dataclass, field
from itertools import combinations
from pathlib import Path

import numpy as np


EXAMPLE = Path(__file__).resolve().parent / "examples" / "default_hardpoints.csv"
CORNERS = ("FL", "FR", "RL", "RR")
CHASSIS = "CHASSIS"
PROJECT_COLUMN = "jacobian_project"
ALIASES = {
    "UFI": "UCA_F", "URI": "UCA_R", "LFI": "LCA_F", "LRI": "LCA_R",
    "WHEEL_CENTER": "WC", "ROD_IN": "RACK", "ROD_OUT": "TOE",
    "TIE_IN": "RACK", "TIE_OUT": "TOE", "TIE_ROD_IN": "RACK", "TIE_ROD_OUT": "TOE",
    "PUSH_OUT_LCA": "PUSH_OUT", "PUSHROD_IN": "PUSH_IN", "PUSHROD_OUT": "PUSH_OUT",
}
PUSHROD_SUFFIXES = {"LCA": "PUSH_OUT", "UCA": "PUSH_OUT_UCA", "UPRIGHT": "PUSH_OUT_UPRIGHT"}
ARM_POINTS = {"UCA_F", "UCA_R", "LCA_F", "LCA_R", "UBJ", "LBJ"}
CORNER_POINTS = ARM_POINTS | {
    "RACK", "TOE", "WC", "CONTACT", "CONSTRUCTION_CONTACT", "FORWARD",
    "PUSH_IN", *PUSHROD_SUFFIXES.values(),
    "ROCKER_A", "ROCKER_B", "DAMPER_IN", "DAMPER_BODY",
}


@dataclass(frozen=True)
class Rod:
    name: str
    a: str
    b: str


@dataclass(frozen=True)
class Joint:
    body: str
    base: str
    point: str
    axis_end: str | None = None


@dataclass(frozen=True)
class Wheel:
    corner: str
    center: str
    contact: str
    heading: str | None


@dataclass(frozen=True)
class Model:
    points: dict
    owners: dict
    bodies: tuple[str, ...]
    rods: tuple[Rod, ...]
    joints: tuple[Joint, ...] = ()
    rigid_edges: tuple[tuple[str, str], ...] = ()
    wheels: tuple[Wheel, ...] = ()
    cg: str | None = None
    cop: str | None = None
    notes: tuple[str, ...] = ()
    member_mounts: dict[str, tuple[str, str]] = field(default_factory=dict)


def number(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label}需要有限數值。") from exc
    if isinstance(value, bool) or not math.isfinite(result):
        raise ValueError(f"{label}需要有限數值。")
    return result


def _coordinates(values, label):
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError(f"{label} 需要三個 XYZ 分量。")
    return tuple(number(value, label) for value in values)


def read_hardpoints(path):
    """Read the named points without building a suspension model."""
    return read_hardpoint_table(path)[0]


def read_hardpoint_table(path):
    """Read named coordinates and axial members from a kinematics CSV."""
    with Path(path).open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle, strict=True)
        reader.fieldnames = [field.strip().lower() for field in (reader.fieldnames or ())]
        headers = set(reader.fieldnames)
        units = [unit for unit in ("m", "mm") if {f"{axis}_{unit}" for axis in "xyz"} <= headers]
        if len(headers) != len(reader.fieldnames) or len(units) != 1 or not headers & {"name", "point"}:
            raise ValueError("硬點 CSV 需要 name 或 point，以及完整的 x_m/y_m/z_m 或 x_mm/y_mm/z_mm。")
        unit = units[0]
        scale = .001 if unit == "mm" else 1.
        field = "name" if "name" in headers else "point"
        points = {}
        payload = ""
        for row in reader:
            if None in row or None in row.values():
                raise ValueError(f"CSV 第 {reader.line_num} 列的欄數不一致。")
            name = row[field].strip()
            if not name:
                raise ValueError(f"CSV 第 {reader.line_num} 列缺少硬點名稱。")
            if field == "point" and row.get("corner", "").strip():
                name = f"{row['corner'].strip()}_{name}"
            if name in points:
                raise ValueError(f"CSV 有重複硬點 {name}。")
            points[name] = tuple(number(row[f"{axis}_{unit}"], f"{name} 的 {axis.upper()}") * scale for axis in "xyz")
            settings = row.get(PROJECT_COLUMN, "").strip()
            if settings:
                if payload:
                    raise ValueError("硬點 CSV 只能包含一筆 Jacobian 專案設定。")
                payload = settings
    if not points:
        raise ValueError("CSV 沒有硬點資料。")
    try:
        settings = json.loads(payload) if payload else {}
    except json.JSONDecodeError as exc:
        raise ValueError("硬點 CSV 的 Jacobian 專案設定無法解析。") from exc
    return points, _read_deformable_members(settings, points)


def _read_deformable_members(settings, points):
    members = settings.get("deformable_members", []) if isinstance(settings, dict) else None
    if not isinstance(members, list):
        raise ValueError("deformable_members 需要桿件清單。")

    point_names = {name.casefold() for name in points}
    rod_names = set()
    member_mounts = {}
    for member in members:
        name = member.get("name") if isinstance(member, dict) else None
        if not isinstance(name, str) or not name.strip():
            raise ValueError("變形元件需要桿件名稱。")
        name = name.strip()
        if name.casefold() in rod_names:
            raise ValueError(f"有重複桿件 {name}。")
        rod_names.add(name.casefold())
        mounts = []
        for label in "AB":
            point = f"{name}_{label}"
            end = member.get(label.lower())
            xyz = end.get("xyz") if isinstance(end, dict) else None
            if point.casefold() in point_names:
                raise ValueError(f"有重複硬點 {point}。")
            point_names.add(point.casefold())
            # Kinematics metadata stores endpoint coordinates in meters.
            points[point] = _coordinates(xyz, point)
            mounts.append(end.get("mount"))
        member_mounts[name] = tuple(mounts)
    return member_mounts


def load_model_csv(path):
    points, member_mounts = read_hardpoint_table(path)
    return build_model(points, member_mounts=member_mounts)


def build_kinematics_model(data):
    """Translate standard Jacobian Kinematics Lab project geometry to statics."""
    if not isinstance(data, dict):
        raise ValueError("運動學專案需要 JSON 物件。")
    version = data.get("version", 1)
    if isinstance(version, bool) or version != 1:
        raise ValueError("運動學專案 JSON 需要 version 1。")
    if data.get("mechanism") is not None:
        raise ValueError("請使用標準懸吊幾何的運動學專案。自訂運動學機構的約束不同。")
    source_points = data.get("points")
    if (not isinstance(source_points, dict) or not source_points
            or any(not isinstance(name, str) or not name.strip() for name in source_points)):
        raise ValueError("運動學專案需要以公尺表示的具名硬點。")
    points = {name: _coordinates(values, name) for name, values in source_points.items()}
    overrides = data.get("pushrod_mounts", {})
    if not isinstance(overrides, dict):
        raise ValueError("運動學專案的 pushrod_mounts 需要 JSON 物件。")
    for corner, mount in overrides.items():
        if corner not in CORNERS or not isinstance(mount, str) or mount not in PUSHROD_SUFFIXES:
            raise ValueError(f"{corner} 的推桿安裝位置無效。輪位須為 FL、FR、RL、RR，安裝位置須為 UCA、LCA、UPRIGHT。")
        candidates = []
        for name in points:
            prefix, _, key = name.upper().partition("_")
            suffix = ALIASES.get(key, key)
            if prefix == corner and suffix in PUSHROD_SUFFIXES.values():
                candidates.append((name, suffix))
        if len(candidates) != 1:
            raise ValueError(f"{corner} 的推桿安裝設定需要一個推桿外端硬點。")
        name, current_suffix = candidates[0]
        expected_suffix = PUSHROD_SUFFIXES[mount]
        if current_suffix != expected_suffix:
            points[f"{corner}_{expected_suffix}"] = points.pop(name)

    member_mounts = _read_deformable_members(data, points)
    for name in member_mounts:
        length = math.dist(points[f"{name}_A"], points[f"{name}_B"])
        if not math.isfinite(length) or length <= 1e-9:
            raise ValueError(f"{name} 的兩端太近，或座標超出數值範圍。")
    model = build_model(points, member_mounts=member_mounts)
    rods = {rod.name: rod for rod in model.rods}
    for corner, mount in overrides.items():
        pushrod = rods.get(f"{corner}_PUSH_ROD")
        body = f"{corner}_{mount}"
        if pushrod is None or body not in model.bodies or model.owners.get(pushrod.a) != body:
            raise ValueError(f"{corner} 需要完整的 {mount} 與推桿幾何才能使用指定的安裝位置。")
    return model


def write_hardpoint_csv(path, points, member_mounts):
    headers = ("name", "x_m", "y_m", "z_m")
    members, endpoints = [], set()
    for name, mounts in member_mounts.items():
        member = {"name": name}
        for label, mount in zip("AB", mounts):
            point = f"{name}_{label}"
            endpoints.add(point)
            member[label.lower()] = {"xyz": points[point], "mount": mount}
        members.append(member)
    payload = ""
    if members:
        payload = json.dumps({"version": 1, "deformable_members": members}, ensure_ascii=False, allow_nan=False)
    with Path(path).open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow((*headers, PROJECT_COLUMN) if members else headers)
        for name, point in points.items():
            if name in endpoints:
                continue
            row = (name, *point)
            writer.writerow((*row, payload) if members else row)
            payload = ""


def wheel_points(points):
    local = {corner: {} for corner in CORNERS}
    prefixes = [name.upper().partition("_") for name in points]
    prefixed = any(corner in CORNERS and ALIASES.get(key, key) in ARM_POINTS | {"WC"}
                   for corner, _, key in prefixes)
    for name, (corner, _, key) in zip(points, prefixes):
        if corner not in local and not prefixed:
            corner, key = "FL", name.upper()
        key = ALIASES.get(key, key)
        if corner not in local or key not in CORNER_POINTS:
            continue
        if key in local[corner]:
            raise ValueError(f"{corner} 的 {key} 有重複定義。")
        local[corner][key] = name
    wheels = []
    for corner, parts in local.items():
        if "WC" not in parts:
            continue
        center = parts["WC"]
        contact = parts.get("CONTACT") or parts.get("CONSTRUCTION_CONTACT")
        if contact is None:
            x, y, z = points[center]
            contact = f"{corner}_CONTACT"
            points[contact] = (x, y, 0. if .1 <= z <= .45 else z - .196)
        wheels.append(Wheel(corner, center, contact, parts.get("FORWARD")))
    return local, tuple(wheels)


def _upper_arm_pushrod_corners(points):
    corners, _ = wheel_points(dict(points))
    return [local for local in corners.values() if {"PUSH_IN", "PUSH_OUT_UCA"} <= local.keys()
            and not {"PUSH_OUT", "PUSH_OUT_UPRIGHT"} & local.keys()]


def equivalent_pushrod_joints(points):
    corners = _upper_arm_pushrod_corners(points)
    return tuple(joint for joint in ("UBJ", "TOE") if any(joint in local for local in corners))


def relocate_upper_arm_pushrods(points, joint):
    if joint not in ("UBJ", "TOE"):
        raise ValueError("請選擇上 A 臂球接頭或轉向拉桿外接點。")
    renamed = {}
    for local in _upper_arm_pushrod_corners(points):
        if joint not in local:
            continue
        original = local["PUSH_OUT_UCA"]
        replacement = original[:-len("PUSH_OUT_UCA")] + "PUSH_OUT_UPRIGHT"
        renamed[original] = replacement
        points[replacement] = points[local[joint]]
        del points[original]
    if not renamed:
        raise ValueError(f"找不到具備內端與 {joint} 接點的上 A 臂推桿。")
    return renamed


def build_model(hardpoints, topology=None, *, member_mounts=None):
    points = dict(hardpoints)
    corners, wheels = wheel_points(points)
    cg = next((name for name in points if name.upper() == "CG"), None)
    if cg is None and wheels:
        centers = np.array([points[wheel.center] for wheel in wheels])
        cg = "CG"
        points[cg] = (*centers.mean(axis=0)[:2], min(points[wheel.contact][2] for wheel in wheels) + .31)
    if topology is not None:
        return custom_model(points, topology, wheels, cg)

    if member_mounts is None:
        member_mounts = {}
    if not isinstance(member_mounts, dict):
        raise ValueError("桿件安裝設定需要名稱及 A、B 端安裝位置。")
    added_rods, mounted_points = [], {mount: [] for mount in (CHASSIS, *CORNERS)}
    member_names = set()
    for name, mounts in member_mounts.items():
        if not isinstance(name, str) or not name.strip() or name.casefold() in member_names:
            raise ValueError("桿件安裝設定需要唯一的桿件名稱。")
        member_names.add(name.casefold())
        if not isinstance(mounts, (list, tuple)) or len(mounts) != 2:
            raise ValueError(f"{name} 需要 A、B 端安裝位置。")
        a, b = f"{name}_A", f"{name}_B"
        for point, mount in zip((a, b), mounts):
            if point not in points:
                raise ValueError(f"找不到桿件端點 {point}。")
            if not isinstance(mount, str) or mount not in mounted_points:
                raise ValueError(f"{point} 的安裝位置須為 CHASSIS 或 FL、FR、RL、RR 搖臂。")
            mounted_points[mount].append(point)
        added_rods.append(Rod(name, a, b))

    owners = {cg: CHASSIS} if cg else {}
    owners.update(dict.fromkeys(mounted_points[CHASSIS], CHASSIS))
    cop = next((name for name in points if name.upper() == "COP"), None)
    if cop:
        owners[cop] = CHASSIS
    rods, bodies, joints, edges, notes = [], [], [], [], []
    if added_rods:
        notes.append(f"已匯入 {len(added_rods)} 支軸力桿件。請設定外載後求解。")
    contacts = {wheel.corner: wheel.contact for wheel in wheels}
    for corner, local in corners.items():
        if not (ARM_POINTS & local.keys() or corner in contacts):
            continue
        upright = f"{corner}_UPRIGHT"
        assign(owners, upright, local, ("UBJ", "LBJ", "TOE", "WC", "FORWARD"))
        if corner in contacts:
            owners[contacts[corner]] = upright
        if upright in owners.values():
            bodies.append(upright)
        assign(owners, CHASSIS, local, ("UCA_F", "UCA_R", "LCA_F", "LCA_R", "RACK", "ROCKER_A", "ROCKER_B", "DAMPER_BODY"))
        draw_edges(edges, local, ("UBJ", "LBJ", "TOE", "WC"))
        if "WC" in local:
            edges.append((local["WC"], contacts[corner]))

        outboard = [key for key in PUSHROD_SUFFIXES.values() if key in local]
        if len(outboard) > 1:
            raise ValueError(f"{corner} 有多個推桿外端，請保留一個。")
        push = outboard[0] if outboard else None
        push_connected = "PUSH_IN" in local and push is not None
        for arm, ball, push_key in (("UCA", "UBJ", "PUSH_OUT_UCA"), ("LCA", "LBJ", "PUSH_OUT")):
            a, b = f"{arm}_F", f"{arm}_R"
            if push_connected and push == push_key and {a, b, ball} <= local.keys():
                body = f"{corner}_{arm}"
                bodies.append(body)
                owners[local[push]] = body
                joints.extend((Joint(body, CHASSIS, local[a], local[b]), Joint(body, upright, local[ball])))
                draw_edges(edges, local, (a, b, ball, push))
                notes.append(f"{body} 的推桿接在 A 臂，已用剛性 A 臂與鉸鏈表示。")
            else:
                for mount in (a, b):
                    add_rod(rods, corner, local, f"{mount}-{ball}", mount, ball)

        add_rod(rods, corner, local, "TIE_ROD", "RACK", "TOE")
        if push_connected and push == "PUSH_OUT_UPRIGHT":
            owners[local[push]] = upright
            draw_edges(edges, local, ("LBJ", "WC", push))
        if push_connected and local[push] in owners:
            add_rod(rods, corner, local, "PUSH_ROD", push, "PUSH_IN")
            axes = ("ROCKER_A", "ROCKER_B")
            legacy_damper = {"DAMPER_IN", "DAMPER_BODY"} <= local.keys()
            member_ends = mounted_points[corner]
            if set(axes) <= local.keys() and (legacy_damper or member_ends):
                body = f"{corner}_ROCKER"
                bodies.append(body)
                rocker_points = []
                if legacy_damper:
                    rocker_points.append(local["DAMPER_IN"])
                    add_rod(rods, corner, local, "DAMPER", "DAMPER_IN", "DAMPER_BODY")
                rocker_points.extend((local["PUSH_IN"], *member_ends))
                owners.update(dict.fromkeys(rocker_points, body))
                joints.append(Joint(body, CHASSIS, local["ROCKER_A"], local["ROCKER_B"]))
                edges.extend(combinations([local[key] for key in axes] + rocker_points, 2))
            else:
                owners[local["PUSH_IN"]] = CHASSIS
                draw_edges(edges, local, ("ROCKER_A", "ROCKER_B", "PUSH_IN"))
                notes.append(f"{corner} 的推桿內端固定於車身。提供搖臂軸與阻尼器兩端可改用鉸接搖臂。")
        else:
            notes.append(f"{corner} 無完整推桿受力路徑。求解時會檢查平衡與約束。")

    for corner in CORNERS:
        if mounted_points[corner] and f"{corner}_ROCKER" not in bodies:
            raise ValueError(f"{corner} 的匯入桿件需要 ROCKER_A、ROCKER_B 與完整的推桿連接。")
    if member_names.intersection(rod.name.casefold() for rod in rods):
        raise ValueError("匯入的軸力桿件名稱與懸吊桿件重複。")
    rods.extend(added_rods)
    if not rods:
        notes.append("目前只有硬點座標。請使用標準懸吊點名，或載入自訂拓撲 JSON。")
    return Model(points, owners, tuple(bodies), tuple(rods), tuple(joints), tuple(dict.fromkeys(edges)), wheels, cg, cop,
                 tuple(notes), {name: tuple(mounts) for name, mounts in member_mounts.items()})


def assign(owners, body, local, keys):
    for key in keys:
        if key in local:
            owners[local[key]] = body


def add_rod(rods, corner, local, label, a, b):
    if a in local and b in local:
        rods.append(Rod(f"{corner}_{label}", local[a], local[b]))


def draw_edges(edges, local, keys):
    edges.extend(combinations((local[key] for key in keys if key in local), 2))


def read_topology(path):
    with Path(path).open(encoding="utf-8-sig") as handle:
        return json.load(handle)


def custom_model(points, data, wheels, cg):
    if not isinstance(data, dict):
        raise ValueError("拓撲 JSON 需要一個物件。")
    fixed, groups, members = data.get("fixed", []), data.get("bodies", {}), data.get("rods", [])
    if not isinstance(fixed, list) or not isinstance(groups, dict) or not isinstance(members, list):
        raise ValueError("拓撲請使用 fixed 陣列、bodies 物件及 rods 陣列。")
    if CHASSIS in groups or any(not name.strip() for name in groups):
        raise ValueError("自訂剛體須有名稱，CHASSIS 為保留名稱。")
    owners, edges = {}, []
    for body, names in ((CHASSIS, fixed), *groups.items()):
        if not isinstance(names, list) or (body != CHASSIS and not names):
            raise ValueError(f"{body} 需要硬點名稱陣列。")
        for name in names:
            if not isinstance(name, str) or name not in points or name in owners:
                raise ValueError(f"{body} 使用未知或重複指派的硬點 {name}。")
            owners[name] = body
        if body != CHASSIS:
            edges.extend(combinations(names, 2))
    rods = []
    labels = set()
    for member in members:
        if not isinstance(member, dict):
            raise ValueError("每支桿件需要 name、a、b 欄位。")
        label, a, b = (member.get(key) for key in ("name", "a", "b"))
        if not all(isinstance(value, str) and value.strip() for value in (label, a, b)):
            raise ValueError("每支桿件需要 name、a、b 欄位。")
        if label in labels or a == b or a not in points or b not in points:
            raise ValueError(f"桿件 {label} 使用重複名稱或無效端點。")
        rods.append(Rod(label, a, b))
        labels.add(label)
    cop = next((name for name in points if name.upper() == "COP"), None)
    return Model(points, owners, tuple(groups), tuple(rods), rigid_edges=tuple(edges), wheels=wheels, cg=cg, cop=cop)
