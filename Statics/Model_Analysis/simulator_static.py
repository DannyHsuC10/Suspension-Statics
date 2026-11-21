import json
from pathlib import Path

import numpy as np

from . import output
from .Inverse_dynamics import Material_engine as mater
from .Inverse_dynamics import Static_engine as se


def solve_static(nodes, rods):
    state = mater.AnalysisState(nodes, rods)
    sa = se.StaticAnalyzer(state)
    sa.solve()
    state.node_forces = sa.compute_node_forces()
    material_engine = mater.MaterialEngine()
    material_engine.compute(state)
    return state


def run_static(config_path):
    """Run the static case in a JSON file and return its analysis state."""
    config_path = Path(config_path)
    with config_path.open(encoding="utf-8-sig") as config_file:
        config = json.load(config_file)["static"]

    files = config["files"]
    nodes = mater.load_nodes(config_path.parent / files["nodes_path"])
    rod_table = mater.load_rod_table(config_path.parent / files["rods_path"])
    rods = mater.initialize_rod_properties(nodes, rod_table)

    for load in config.get("forces", []):
        node_name = load["node"]
        if node_name in nodes:
            nodes[node_name].force = np.array(load["value"])

    state = solve_static(nodes, rods)
    report = output.build_report(state)
    export = config.get("output", {})
    if export.get("report", False):
        output.save_report(report)
    if export.get("csv", False):
        output.save_rod_forces_csv(rods)

    return state
