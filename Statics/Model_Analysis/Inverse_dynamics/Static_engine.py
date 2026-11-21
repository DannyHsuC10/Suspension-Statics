import numpy as np


class StaticAnalyzer:

    def __init__(self, state):

        self.state = state
        self.nodes = state.nodes

        self.rods = state.active_rods  # dict

    # =========================
    # Solve static equilibrium
    # =========================
    def solve(self):

        nodes = self.nodes
        rods = list(self.rods.values())

        A = []
        b = []

        for node in nodes.values():

            if node.fixed:
                continue

            for axis in range(3):

                row = np.zeros(len(rods))

                for i, r in enumerate(rods):

                    d = r.b.pos - r.a.pos
                    L = np.linalg.norm(d)

                    if L < 1e-12:
                        continue

                    u = d / L

                    if r.a == node:
                        sign = 1
                    elif r.b == node:
                        sign = -1
                    else:
                        continue

                    row[i] += sign * u[axis]

                A.append(row)
                b.append(-node.force[axis])

        A = np.array(A)
        b = np.array(b)

        x, *_ = np.linalg.lstsq(A, b, rcond=None)

        # write back
        for i, r in enumerate(rods):
            r.force = x[i]

        return x

        # Compute node force balance
    def compute_node_forces(self):

        results = {}

        for node_name, node in self.nodes.items():

            total = np.zeros(3)

            for r in self.rods.values():

                d = r.b.pos - r.a.pos
                L = np.linalg.norm(d)

                if L < 1e-12:
                    continue

                u = d / L

                if r.a == node:
                    total += r.force * u

                elif r.b == node:
                    total -= r.force * u

            total += node.force

            results[node_name] = total

        return results
