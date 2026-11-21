# Suspension Statics Lab

[Open the web workbench](https://dannyhsuc10.github.io/Suspension-Statics/) · [Calculation method](https://dannyhsuc10.github.io/Suspension-Statics/method.html)

Solve member axial forces and chassis support reactions on the imported suspension geometry. The workbench includes an interactive 3D view, editable tube or solid-round sections, and quasi-static load histories.

## Import and export

| File | Input |
| --- | --- |
| Hardpoints CSV | `name,x_m,y_m,z_m` or `name,x_mm,y_mm,z_mm`. Supports JacobianKinematicsLab's embedded deformable-member endpoints and mounts. |
| Project JSON | The desktop lab's version 1 format. Includes hardpoints, topology, mounted members, applied loads and section assignments. |
| Custom topology JSON | `fixed` lists chassis points. `bodies` maps rigid body names to their points. `rods` lists objects with unique `name` and hardpoint references `a` and `b`. |
| External loads CSV | `target,end,x,y,z`. Leave `end` blank for forces (N), or use `A` or `B` for a member-end force couple (N·m). |
| Lap CSV | Increasing `time_s` with either XYZ tire force channels for each wheel or longitudinal and lateral acceleration. Vertical acceleration is optional for acceleration-based laps. |

Export hardpoints, external loads, member results and load histories as CSV, or save the work in a project JSON. The browser processes the input files locally.

## Run locally

Serve the static assets with Python and open `http://localhost:8000` in a browser.

```sh
python -m http.server 8000 --directory web
```

GitHub Actions publishes the `web` directory to GitHub Pages on pushes to `main`. The repository's Pages publishing source is **GitHub Actions**.
