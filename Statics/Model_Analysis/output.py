import csv
from datetime import datetime
from pathlib import Path

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "output"


def save_report(report, filename="static_report"):# 靜態報告存檔
    # 紀錄時間
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename_with_time = f"{filename}_{timestamp}.txt"

    # 完整路徑
    OUTPUT_DIR.mkdir(exist_ok=True)
    file_path = OUTPUT_DIR / filename_with_time


    # 寫入文字
    with open(file_path, "w", encoding="utf-8") as f:
        f.write(report)

    print(f"Report saved to: {file_path}")

    return file_path

def save_rod_forces_csv(rods, filename="rod_forces"):

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename_with_time = f"{filename}_{timestamp}.csv"

    OUTPUT_DIR.mkdir(exist_ok=True)
    file_path = OUTPUT_DIR / filename_with_time

    with open(file_path, "w", newline="", encoding="utf-8") as f:

        writer = csv.writer(f)

        # =========================
        # header（工程完整版）
        # =========================
        writer.writerow([
            "rod_name",
            "force_N",
            "state",

            # geometry
            "length_m",
            "radius_m",
            "area_m2",

            # material
            "E_Pa",
            "max_stress_Pa",

            # mechanics
            "stress_Pa",
            "strain",
            "delta_L_m",
            "stiffness_Nm",

            # energy
            "energy_J",

            # safety
            "safety_ratio",
            "fail"
        ])

        # =========================
        # data
        # =========================
        for r in rods.values():

            force = r.force
            # force state
            if force > 0.01:
                state = "tension"
            elif force < -0.01:
                state = "compression"
            else:
                state = "zero"
                force = 0

            # safety check (保險再算一次，避免 state 沒更新)
            stress = r.stress if r.stress is not None else 0
            max_s = r.max_stress if hasattr(r, "max_stress") else 0

            safety_ratio = abs(stress) / max_s if max_s else None
            fail = (abs(stress) > max_s) if max_s else False

            writer.writerow([
                r.name,
                force,
                state,

                # geometry
                r.L0,
                r.radius,
                r.area,

                # material
                r.E,
                max_s,

                # mechanics
                r.stress,
                r.strain,
                r.delta_L,
                r.k,

                # energy
                r.energy,

                # safety
                safety_ratio,
                fail
            ])

def build_report(state):

    lines = []

    # ==================================================
    # Rod Forces
    # ==================================================
    lines.append("")
    lines.append("==================================================")
    lines.append("Rod Forces")
    lines.append("==================================================")

    for r in state.active_rods.values():

        lines.append(
            f"{r.name:20s} | "
            f"F = {r.force:12.3f} N"
        )

    # ==================================================
    # Material Results
    # ==================================================
    lines.append("")
    lines.append("==================================================")
    lines.append("Material Results")
    lines.append("==================================================")

    for r in state.active_rods.values():

        if r.max_stress and r.max_stress > 0:
            status = "FAIL" if r.fail else "OK"
            sf = r.safety_ratio
            sf_str = f"{sf:6.2f}" if sf is not None else "  -  "
        else:
            status = "N/A"
            sf_str = "  -  "

        lines.append(
            f"{r.name:20s} | "
            f"σ = {r.stress:10.3f} MPa | "
            f"ε = {r.strain:12.6e} | "
            f"ΔL = {r.delta_L:10.6f} mm | "
            f"U = {r.energy:10.3f} | "
            f"max = {r.max_stress:8.2f} | "
            f"S.F. = {sf_str} | "
            f"{status}"
        )

    # ==================================================
    # Node Forces
    # ==================================================
    lines.append("")
    lines.append("==================================================")
    lines.append("Node Equilibrium")
    lines.append("==================================================")

    for name, F in state.node_forces.items():

        node = state.nodes[name]

        tag = "Reaction" if node.fixed else "Residual"

        lines.append(
            f"{name:20s} | "
            f"{tag:10s} | "
            f"Fx = {F[0]:10.3f} | "
            f"Fy = {F[1]:10.3f} | "
            f"Fz = {F[2]:10.3f}"
        )

    report = "\n".join(lines)
    print(report)

    return report
