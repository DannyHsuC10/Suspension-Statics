import {
  accelerationHistory, buildModel, DEFAULT_SECTION, exampleLoads, formatHardpointCsv,
  historyStatistics, lapHistory, memberResults, parseHardpointCsv, sectionProperties, solve,
} from "./engine.js";
import {
  formatHistoryCsv, formatLoadsCsv, formatProject, formatResultsCsv, parseLoadsCsv, parseProject,
  validateLoads,
} from "./data.js";
import { drawSection, HistoryPlot, Viewport } from "./views.js";

const $ = id => document.getElementById(id);
const state = {
  model: null,
  topology: null,
  source: "",
  loads: [],
  sections: {},
  selected: null,
  checked: new Set(),
  loadIndex: -1,
  equilibrium: null,
  results: {},
  stressLimit: 0,
  history: null,
  historyPeak: 0,
  historyStats: new Map(),
  frame: 0,
  playback: null,
  tab: "loads",
  revision: 0,
};

const presetHints = {
  static: "321 kg vehicle weight and four vertical tire reactions.",
  acceleration: "+X forward, +Y left, +Z up. Apply the preset after changing its settings.",
  bump: "An independent +1,800 N vertical load at the first available tire contact.",
};
const views = new Viewport($("geometry-view"), selectGeometry);
const plot = new HistoryPlot($("history-plot"), index => seek(index, true));

function display(value, digits = 5) {
  if (!Number.isFinite(value)) return "—";
  return String(Math.abs(value) < 1e-9 ? 0 : Number(value.toPrecision(digits)));
}

function xyzText(xyz) {
  return `(${xyz.map(value => display(value)).join(", ")})`;
}

function numeric(input, label) {
  if (input.value.trim() === "" || !Number.isFinite(Number(input.value))) {
    throw new Error(`${label} must be a finite number.`);
  }
  return Number(input.value);
}

function status(message, error = false) {
  $("status").textContent = message;
  $("status").dataset.error = String(error);
}

function perform(action) {
  try {
    return action();
  } catch (error) {
    status(error.message, true);
    return undefined;
  }
}

function options(select, values) {
  const previous = select.value;
  select.replaceChildren(...values.map(value => new Option(value, value)));
  if (values.includes(previous)) select.value = previous;
  select.disabled = values.length === 0;
}

function td(value, className = "") {
  const cell = document.createElement("td");
  if (value instanceof Node) cell.append(value);
  else cell.textContent = value;
  if (className) cell.className = className;
  return cell;
}

function row(cells) {
  const tr = document.createElement("tr");
  tr.append(...cells);
  return tr;
}

function emptyRow(body, columns, message) {
  const cell = td(message, "empty-cell");
  cell.colSpan = columns;
  body.replaceChildren(row([cell]));
}

function nameButton(name) {
  const button = document.createElement("button");
  button.type = "button";
  button.textContent = name;
  button.className = "table-button";
  return button;
}

function metric(label, value) {
  const box = document.createElement("div");
  const heading = document.createElement("dt");
  const detail = document.createElement("dd");
  heading.textContent = label;
  detail.textContent = value;
  box.append(heading, detail);
  return box;
}

function readSettings() {
  const massKg = numeric($("mass"), "Mass");
  if (massKg <= 0) throw new Error("Mass must be greater than zero.");
  const downforceN = $("enable-downforce").checked ? numeric($("downforce"), "Downforce") : null;
  if (downforceN !== null && downforceN < 0) throw new Error("Downforce must be zero or positive.");
  return {
    massKg,
    accelerationMps2: ["ax", "ay", "az"].map(axis => numeric($(axis), axis)),
    includeGravity: $("gravity").checked,
    downforceN,
  };
}

function readRise() {
  const duration = numeric($("rise-time"), "Rise time");
  if (duration <= 0) throw new Error("Rise time must be greater than zero.");
  return duration;
}

function renderPreset() {
  const preset = $("preset").value;
  $("acceleration-settings").hidden = preset !== "acceleration";
  $("preset-hint").textContent = presetHints[preset];
  $("downforce").disabled = !$("enable-downforce").checked;
  const acceleration = ["ax", "ay", "az"].map(axis => $(axis).valueAsNumber);
  $("acceleration-magnitude").textContent = `Input acceleration magnitude ${display(Math.hypot(...acceleration))} m/s²`;
}

function install(model, source, { topology = null, loads = [], sections = {}, resetView = true } = {}) {
  stopPlayback();
  state.model = model;
  state.source = source;
  state.topology = topology;
  state.loads = loads;
  state.sections = Object.fromEntries(model.rods.map(({ name }) => [
    name, Object.hasOwn(sections, name) ? sections[name] : { ...DEFAULT_SECTION },
  ]));
  state.loadIndex = -1;
  state.history = null;
  state.historyPeak = 0;
  state.historyStats.clear();
  state.frame = 0;
  const names = model.rods.map(({ name }) => name);
  if (resetView) {
    state.selected = null;
    state.checked.clear();
    for (const button of document.querySelectorAll("[data-view]")) {
      button.setAttribute("aria-pressed", String(button.dataset.view === "iso"));
    }
  } else {
    state.checked = new Set([...state.checked].filter(name => names.includes(name)));
  }
  $("source-name").textContent = source;
  $("project-metrics").textContent = `${Object.keys(model.points).length} hardpoints · ${names.length} axial members · ${model.wheels.length} wheels`;
  $("edit-points").disabled = false;
  views.setModel(model, resetView);
  if (resetView) views.setView("iso");
  options($("section-member"), names);
  options($("history-member"), names);
  inspectSection($("section-member").value, false);
  renderSections();
  renderLoads();
  updateLoadTargets();
  clearStaticResults();
  renderHistory();
  syncButtons();
  status(model.memberMounts && Object.keys(model.memberMounts).length
    ? `Imported ${Object.keys(model.memberMounts).length} axial member(s) with their endpoint mounts. Set loads and solve.`
    : "Geometry ready. Apply a preset or enter external loads.");
}

async function loadExample(name) {
  const revision = ++state.revision;
  const response = await fetch(`./examples/${name}`);
  if (!response.ok) throw new Error("Could not load the example geometry.");
  const { points, memberMounts } = parseHardpointCsv(await response.text());
  const model = buildModel(points, null, memberMounts);
  const loads = exampleLoads(model, "static");
  if (revision !== state.revision) return;
  const title = name === "hardpoints.csv" ? "Four-wheel example with dampers" : "Four-wheel example";
  install(model, title, { loads });
  $("preset").value = "static";
  renderPreset();
  switchTab("loads");
  solveCurrent(false);
}

function syncButtons() {
  const model = Boolean(state.model);
  const members = Boolean(state.model?.rods.length);
  $("apply-preset").disabled = !model;
  for (const id of ["solve", "apply-sections", "apply-all-sections", "generate-history"]) $(id).disabled = !members;
  $("clear-loads").disabled = !state.loads.length;
  $("add-load").disabled = !$("load-target").value;
  for (const id of ["update-load", "delete-load"]) $(id).disabled = state.loadIndex < 0;
  for (const button of document.querySelectorAll("[data-export]")) {
    const available = { geometry: model, project: model, loads: model, results: Boolean(state.equilibrium), history: Boolean(state.history) };
    button.disabled = !available[button.dataset.export];
  }
  for (const button of document.querySelectorAll("[data-file]")) {
    if (["topology", "loads", "lap"].includes(button.dataset.file)) button.disabled = !model;
  }
}

function switchTab(name, focus = false) {
  state.tab = name;
  for (const key of ["loads", "sections", "results", "history"]) {
    const active = name === key;
    const tab = $(`tab-${key}`);
    tab.setAttribute("aria-selected", String(active));
    tab.tabIndex = active ? 0 : -1;
    $(`panel-${key}`).hidden = !active;
    if (active && focus) tab.focus();
  }
  if (name !== "history") stopPlayback();
  if (name === "sections") renderSectionPreview();
  if (name === "history") renderHistory();
  renderViewport();
}

function updateLoadTargets() {
  const end = $("load-type").value;
  const model = state.model;
  const used = new Set(model ? Object.keys(model.owners) : []);
  for (const rod of model?.rods ?? []) {
    used.add(rod.a);
    used.add(rod.b);
  }
  const targets = end ? model?.rods.filter(rod => Object.hasOwn(model.owners, end === "A" ? rod.a : rod.b)).map(rod => rod.name)
    : Object.keys(model?.points ?? {}).filter(name => used.has(name));
  options($("load-target"), targets ?? []);
  for (const axis of "xyz") {
    $(`load-${axis}-label`).textContent = end ? `M${axis} (N·m)` : `F${axis} (N)`;
  }
  renderLoadHint();
  syncButtons();
}

function renderLoadHint() {
  const name = $("load-target").value;
  const end = $("load-type").value;
  const vector = ["load-x", "load-y", "load-z"].map(id => $(id).valueAsNumber);
  const rod = end ? state.model?.rods.find(item => item.name === name) : null;
  const point = rod ? (end === "A" ? rod.a : rod.b) : name;
  const owner = state.model?.owners[point];
  const location = name && owner ? `${point} · ${owner}` : name;
  $("load-hint").textContent = [
    `Magnitude ${display(Math.hypot(...vector))} ${end ? "N·m" : "N"}`, location,
  ].filter(Boolean).join(" · ");
}

function renderLoads() {
  const applied = $("load-rows");
  if (state.loads.length) {
    applied.replaceChildren(...state.loads.map((load, index) => {
      const name = nameButton(load.target);
      const unit = load.end ? "N·m" : "N";
      const tr = row([
        td(name), td(load.end ? `M${load.end} (${unit})` : "F (N)"),
        ...load.xyz.map(value => td(display(value), "num")),
        td(display(Math.hypot(...load.xyz)), "num"),
      ]);
      tr.dataset.interactive = "true";
      tr.dataset.selected = String(index === state.loadIndex);
      tr.addEventListener("click", () => chooseLoad(index));
      return tr;
    }));
  } else {
    emptyRow(applied, 6, "No external loads configured.");
  }
  const total = [0, 0, 0];
  for (const load of state.loads) {
    if (load.end) continue;
    for (let axis = 0; axis < 3; axis++) total[axis] += load.xyz[axis];
  }
  $("load-total").textContent = `Net force ${xyzText(total)} N · Magnitude ${display(Math.hypot(...total))} N`;
  syncButtons();
}

function chooseLoad(index) {
  const load = state.loads[index];
  state.loadIndex = index;
  $("load-type").value = load.end;
  updateLoadTargets();
  $("load-target").value = load.target;
  for (const [axis, value] of load.xyz.entries()) $(`load-${"xyz"[axis]}`).value = value;
  renderLoadHint();
  for (const [rowIndex, tr] of [...$("load-rows").rows].entries()) {
    tr.dataset.selected = String(index === rowIndex);
  }
  syncButtons();
}

function formLoad() {
  const target = $("load-target").value;
  const end = $("load-type").value;
  const xyz = [..."xyz"].map(axis => numeric($(`load-${axis}`), axis.toUpperCase()));
  const candidate = [{ target, xyz, end }];
  validateLoads(state.model, candidate);
  return candidate[0];
}

function setLoads(loads, message, active = -1) {
  state.loads = loads;
  state.loadIndex = active;
  clearStaticResults();
  renderLoads();
  status(message);
}

function clearStaticResults() {
  state.equilibrium = null;
  state.results = {};
  state.stressLimit = 0;
  renderResults();
  renderViewport();
  syncButtons();
}

function solveCurrent(show = true) {
  try {
    const equilibrium = solve(state.model, state.loads);
    const results = memberResults(equilibrium, state.sections);
    state.equilibrium = equilibrium;
    state.results = results;
    state.stressLimit = maximumStress(results);
    renderResults();
    renderViewport();
    syncButtons();
    if (show) switchTab("results");
    const freedom = equilibrium.rank < equilibrium.equations ? " This load balances with a remaining free direction." : "";
    status(`Solved ${Object.keys(results).length} axial members. Equilibrium rank ${equilibrium.rank}/${equilibrium.equations}.${freedom}`);
  } catch (error) {
    clearStaticResults();
    status(error.message, true);
  }
}

function maximumStress(results) {
  return Object.values(results).reduce((peak, result) => Math.max(peak, Math.abs(result.stress_mpa)), 0);
}

function renderResults() {
  const eq = state.equilibrium;
  $("support-force").textContent = eq ? xyzText(eq.supportForce) : "—";
  $("support-moment").textContent = eq ? xyzText(eq.supportMoment) : "—";
  $("support-moment-label").textContent = eq ? `Support moment about ${eq.origin ?? "origin"} (N·m)` : "Support moment (N·m)";
  const items = Object.entries(state.results).sort((a, b) => Math.abs(b[1].stress_mpa) - Math.abs(a[1].stress_mpa));
  const body = $("result-rows");
  if (!items.length) {
    emptyRow(body, 8, "Solve the current load case to view forces, stress and deformation.");
    return;
  }
  body.replaceChildren(...items.map(([name, values]) => {
    const loadState = values.force_n > 1e-8 ? "Tension" : values.force_n < -1e-8 ? "Compression" : "Zero";
    const cells = [values.force_n, values.stress_mpa, values.strain * 1e6, values.change_mm, values.length_mm, values.area_mm2];
    const tr = row([td(nameButton(name)), td(loadState, `state-${loadState.toLowerCase()}`), ...cells.map(value => td(display(value), "num"))]);
    tr.dataset.interactive = "true";
    tr.dataset.member = name;
    tr.dataset.selected = String(state.selected?.name === name && state.selected.kind === "rod");
    tr.addEventListener("click", () => selectGeometry("rod", name));
    return tr;
  }));
}

function readSection() {
  const section = {
    outer_mm: numeric($("outer-mm"), "Outer diameter"),
    inner_mm: numeric($("inner-mm"), "Inner diameter"),
    young_gpa: numeric($("young-gpa"), "Young’s modulus"),
  };
  const youngMpa = section.young_gpa * 1000;
  if (!Number.isFinite(youngMpa) || youngMpa <= 0) {
    throw new Error("Young’s modulus must be finite and greater than zero.");
  }
  sectionProperties(section);
  return section;
}

const propertyDefinitions = [
  ["Area A", "area_mm2", "mm²"], ["Area moment I", "second_moment_mm4", "mm⁴"],
  ["Section modulus S", "modulus_mm3", "mm³"], ["Radius of gyration r", "gyration_mm", "mm"],
  ["Polar moment J", "polar_moment_mm4", "mm⁴"], ["Polar modulus Zₚ", "polar_modulus_mm3", "mm³"],
  ["Polar gyration rₚ", "polar_gyration_mm", "mm"],
];

function inspectSection(name, updateSelection = true) {
  const section = state.sections[name];
  if (!section) return;
  $("section-member").value = name;
  $("outer-mm").value = section.outer_mm;
  $("inner-mm").value = section.inner_mm;
  $("young-gpa").value = section.young_gpa;
  if (updateSelection) {
    state.selected = { kind: "rod", name };
    for (const tr of $("section-rows").rows) {
      tr.dataset.selected = String(tr.dataset.member === name);
      const checkbox = tr.querySelector("input");
      if (checkbox) checkbox.checked = state.checked.has(tr.dataset.member);
    }
    refreshHighlights();
  }
  renderSectionPreview();
}

function renderSectionPreview() {
  const list = $("section-properties");
  try {
    const section = readSection();
    const properties = sectionProperties(section);
    drawSection($("section-preview"), section);
    list.replaceChildren(...propertyDefinitions.map(([label, key, unit]) => metric(label, `${display(properties[key])} ${unit}`)));
  } catch (error) {
    drawSection($("section-preview"), null);
    list.replaceChildren(metric("Section properties", error.message));
  }
}

function renderSections() {
  const body = $("section-rows");
  const entries = Object.entries(state.sections);
  if (!entries.length) {
    emptyRow(body, 5, "No axial members in this geometry.");
    return;
  }
  body.replaceChildren(...entries.map(([name, section]) => {
    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.checked = state.checked.has(name);
    checkbox.setAttribute("aria-label", `Select ${name} for section assignment`);
    checkbox.addEventListener("change", () => checkbox.checked ? state.checked.add(name) : state.checked.delete(name));
    const dimensions = [section.outer_mm, section.inner_mm, section.young_gpa];
    const tr = row([td(checkbox), td(nameButton(name)), ...dimensions.map(value => td(display(value), "num"))]);
    tr.dataset.member = name;
    tr.dataset.selected = String(state.selected?.name === name && state.selected.kind === "rod");
    tr.dataset.interactive = "true";
    tr.addEventListener("click", event => {
      if (event.target !== checkbox) inspectSection(name);
    });
    return tr;
  }));
}

function applySections(all) {
  const section = readSection();
  const fallback = $("section-member").value;
  const names = all ? Object.keys(state.sections) : state.checked.size ? [...state.checked] : [fallback].filter(Boolean);
  if (!names.length) throw new Error("Select at least one member.");
  const updated = { ...state.sections };
  for (const name of names) updated[name] = section;
  const results = state.equilibrium ? memberResults(state.equilibrium, updated) : {};
  const peak = state.history ? historyStressMaximum(state.history, updated) : 0;
  state.sections = updated;
  state.results = results;
  state.stressLimit = maximumStress(results);
  state.historyPeak = peak;
  renderSections();
  renderResults();
  renderViewport();
  status(`Updated section properties for ${names.length} member(s).`);
}

function selectGeometry(kind, name) {
  state.selected = kind && name ? { kind, name } : null;
  if (kind === "rod" && Object.hasOwn(state.sections, name)) {
    inspectSection(name, false);
    if (state.history) {
      $("history-member").value = name;
      $("track-governing").checked = false;
      renderHistory();
    }
  }
  refreshHighlights();
}

function refreshHighlights() {
  for (const tr of [...$("result-rows").rows, ...$("section-rows").rows]) {
    tr.dataset.selected = String(state.selected?.kind === "rod" && tr.dataset.member === state.selected.name);
  }
  renderViewport();
}

function historyStressMaximum(history, sections) {
  let peak = 0;
  for (const [name, forces] of Object.entries(history.forceN)) {
    const area = sectionProperties(sections[name]).area_mm2;
    for (const force of forces) peak = Math.max(peak, Math.abs(force) / area);
  }
  if (!Number.isFinite(peak)) throw new Error("History stress exceeds the numerical range.");
  return peak;
}

function renderViewport() {
  let { loads, results, selected, stressLimit } = state;
  if (state.tab === "history" && state.history) {
    const current = state.history.equilibria[state.frame];
    results = memberResults(current, state.sections);
    loads = state.history.loads[state.frame];
    stressLimit = state.historyPeak;
    if ($("track-governing").checked) selected = { kind: "rod", name: state.history.largestLoads[0][state.frame] };
  }
  views.setData({
    results, loads, selection: selected, labels: $("view-labels").checked,
    forceComponents: $("view-loads").checked, maxStress: stressLimit,
  });
  $("stress-limit").textContent = `${stressLimit > 0 ? display(stressLimit) : "—"} MPa`;
  let description = "Select a member or hardpoint to inspect it.";
  if (selected?.kind === "point" && state.model.points[selected.name]) {
    const xyz = state.model.points[selected.name].map(value => value * 1000);
    const body = state.model.owners[selected.name];
    description = [selected.name, body, `${xyzText(xyz)} mm`].filter(Boolean).join(" · ");
  } else if (selected?.kind === "rod") {
    const result = results[selected.name];
    const rod = state.model?.rods.find(item => item.name === selected.name);
    if (rod) description = [rod.name, `${rod.a} / ${rod.b}`,
      result && `${display(result.force_n)} N · ${display(result.stress_mpa)} MPa`].filter(Boolean).join(" · ");
  }
  $("geometry-selection").textContent = description;
}

function installHistory(history) {
  const peak = historyStressMaximum(history, state.sections);
  stopPlayback();
  state.history = history;
  state.historyPeak = peak;
  state.historyStats.clear();
  state.frame = 0;
  options($("history-member"), Object.keys(history.forceN));
  if (state.selected?.kind === "rod" && Object.hasOwn(history.forceN, state.selected.name)) {
    $("history-member").value = state.selected.name;
  }
  switchTab("history");
  syncButtons();
  status(`Solved ${history.timeS.length} history frames.`);
}

function renderHistory() {
  const { history, frame } = state;
  $("history-results").hidden = !history;
  $("history-source").textContent = history?.source ?? "Uses the mass and acceleration settings from the CG acceleration preset.";
  if (!history) {
    plot.setData(null, 0, "", false);
    return;
  }
  const member = $("history-member").value;
  const governing = $("track-governing").checked;
  const slider = $("history-scrubber");
  slider.max = history.timeS.length - 1;
  slider.value = frame;
  $("history-time").value = `${display(history.timeS[frame])} s`;
  $("governing-member").textContent = `Governing at this frame · ${history.largestLoads[0][frame]} · ${display(history.largestLoads[1][frame])} N`;
  if (member) {
    if (!state.historyStats.has(member)) {
      const stats = historyStatistics(history, member);
      state.historyStats.set(member, [
        metric("Peak axial force", `${display(stats.peak_n)} N at ${display(stats.peak_time_s)} s`),
        metric("Mean |force|", `${display(stats.mean_absolute_n)} N`),
        metric("Signed mean", `${display(stats.mean_n)} N`),
      ]);
    }
    const container = $("history-statistics");
    const metrics = state.historyStats.get(member);
    if (container.firstElementChild !== metrics[0]) container.replaceChildren(...metrics);
  }
  plot.setData(history, frame, member, governing);
}

function seek(index, manual = false) {
  if (!state.history) return;
  if (manual) stopPlayback();
  state.frame = Math.max(0, Math.min(index, state.history.timeS.length - 1));
  renderHistory();
  renderViewport();
}

function stopPlayback() {
  if (state.playback !== null) cancelAnimationFrame(state.playback);
  state.playback = null;
  $("play-history").textContent = "Play";
}

function togglePlayback() {
  if (state.playback !== null) {
    stopPlayback();
    return;
  }
  const times = state.history?.timeS;
  if (!times) return;
  if (state.frame === times.length - 1) seek(0);
  const start = performance.now();
  const firstTime = times[state.frame];
  $("play-history").textContent = "Pause";
  function tick(now) {
    const time = firstTime + (now - start) / 1000;
    let index = state.frame;
    while (index + 1 < times.length && times[index + 1] <= time) index++;
    if (index !== state.frame) seek(index);
    if (index === times.length - 1) stopPlayback();
    else state.playback = requestAnimationFrame(tick);
  }
  state.playback = requestAnimationFrame(tick);
}

function openPointEditor() {
  const body = $("point-rows");
  body.replaceChildren(...Object.entries(state.model.points).map(([name, xyz]) => {
    const cells = xyz.map((coordinate, axis) => {
      const input = document.createElement("input");
      input.type = "number";
      input.step = "any";
      input.value = coordinate * 1000;
      input.setAttribute("aria-label", `${name} ${"XYZ"[axis]} in millimeters`);
      return td(input);
    });
    const tr = row([td(name), ...cells]);
    tr.dataset.point = name;
    return tr;
  }));
  $("editor-error").textContent = "";
  $("geometry-editor").showModal();
  if (state.selected?.kind === "point") {
    const selected = [...body.rows].find(tr => tr.dataset.point === state.selected.name);
    selected?.querySelector("input").focus();
  }
}

function readEditor() {
  return Object.fromEntries([...$("point-rows").rows].map(tr => {
    const name = tr.dataset.point;
    const xyz = [...tr.querySelectorAll("input")].map((input, axis) => .001 * numeric(input, `${name} ${"XYZ"[axis]}`));
    return [name, xyz];
  }));
}

function editAction(exportOnly) {
  try {
    const points = readEditor();
    if (exportOnly) {
      download("hardpoints-edited.csv", formatHardpointCsv(points, state.model.memberMounts));
      return;
    }
    const model = buildModel(points, state.topology, state.model.memberMounts);
    install(model, state.source, { topology: state.topology, sections: state.sections, loads: state.loads, resetView: false });
    $("geometry-editor").close();
    status("Hardpoints updated. Reapply any geometry-dependent preset before solving.");
  } catch (error) {
    $("editor-error").textContent = error.message;
  }
}

function download(name, content, type = "text/csv;charset=utf-8") {
  const url = URL.createObjectURL(new Blob([content], { type }));
  const link = document.createElement("a");
  link.href = url;
  link.download = name;
  document.body.append(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function exportData(kind) {
  if (!state.model) return;
  const { model, topology, loads, sections, equilibrium, results, history } = state;
  switch (kind) {
    case "geometry":
      download("hardpoints.csv", formatHardpointCsv(model.points, model.memberMounts));
      break;
    case "project":
      download("suspension-statics.json", formatProject({
        model, topology, loads, sections, caseName: $("preset").value,
        settings: readSettings(), riseTime: readRise(),
      }), "application/json;charset=utf-8");
      break;
    case "loads":
      download("external-loads.csv", formatLoadsCsv(loads));
      break;
    case "results":
      if (equilibrium) download("member-results.csv", formatResultsCsv(equilibrium, results));
      break;
    case "history":
      if (history) download("load-history.csv", formatHistoryCsv(history, model));
      break;
  }
}

function importData(kind, text, name) {
  switch (kind) {
    case "geometry": {
      const { points, memberMounts } = parseHardpointCsv(text);
      install(buildModel(points, null, memberMounts), name);
      switchTab("loads");
      break;
    }
    case "project": {
      const saved = parseProject(text);
      install(saved.model, name, saved);
      $("preset").value = saved.caseName;
      $("mass").value = saved.settings.massKg;
      for (const [axis, value] of saved.settings.accelerationMps2.entries()) {
        $(`a${"xyz"[axis]}`).value = value;
      }
      $("gravity").checked = saved.settings.includeGravity;
      $("enable-downforce").checked = saved.settings.downforceN != null;
      $("downforce").value = saved.settings.downforceN ?? 0;
      $("rise-time").value = saved.riseTime;
      renderPreset();
      switchTab("loads");
      status(`Opened ${name}. The saved external loads are ready to solve.`);
      break;
    }
    case "topology": {
      const topology = JSON.parse(text.replace(/^\uFEFF/, ""));
      if (!topology || typeof topology !== "object" || Array.isArray(topology) || topology.format === "SuspensionStaticsLab") {
        throw new Error("Choose a custom topology JSON with fixed points, bodies and rods.");
      }
      install(buildModel(state.model.points, topology), name, { topology });
      switchTab("loads");
      break;
    }
    case "loads": {
      const loads = parseLoadsCsv(text, state.model);
      setLoads(loads, `Imported ${loads.length} external loads.`);
      switchTab("loads");
      break;
    }
    case "lap":
      installHistory(lapHistory(state.model, text, {
        massKg: $("mass").value,
        includeGravity: $("gravity").checked,
        fileName: name,
      }));
      break;
  }
}

const tabs = [...document.querySelectorAll('[role="tab"]')];
for (const tab of tabs) {
  tab.addEventListener("click", () => switchTab(tab.id.slice(4)));
  tab.addEventListener("keydown", event => {
    const index = tabs.indexOf(tab);
    const destination = {
      ArrowRight: (index + 1) % tabs.length, ArrowLeft: (index + tabs.length - 1) % tabs.length,
      Home: 0, End: tabs.length - 1,
    }[event.key];
    if (destination === undefined) return;
    event.preventDefault();
    switchTab(tabs[destination].id.slice(4), true);
  });
}

for (const button of document.querySelectorAll("[data-view]")) {
  button.addEventListener("click", () => {
    for (const item of document.querySelectorAll("[data-view]")) item.setAttribute("aria-pressed", String(item === button));
    views.setView(button.dataset.view);
  });
}
for (const id of ["view-labels", "view-loads"]) $(id).addEventListener("change", renderViewport);
for (const id of ["preset", "enable-downforce", "ax", "ay", "az"]) $(id).addEventListener("input", renderPreset);
$("apply-preset").addEventListener("click", () => perform(() => {
  const name = $("preset").value;
  const loads = exampleLoads(state.model, name, name === "acceleration" ? readSettings() : {});
  setLoads(loads, "Preset applied. Solve to update the axial member results.");
}));
$("solve").addEventListener("click", () => solveCurrent());
$("load-type").addEventListener("change", updateLoadTargets);
$("load-target").addEventListener("change", updateLoadTargets);
for (const id of ["load-x", "load-y", "load-z"]) $(id).addEventListener("input", renderLoadHint);
$("add-load").addEventListener("click", () => perform(() => setLoads([...state.loads, formLoad()], "Load added. Solve to update the results.", state.loads.length)));
$("update-load").addEventListener("click", () => perform(() => {
  const loads = state.loads.slice();
  loads[state.loadIndex] = formLoad();
  setLoads(loads, "Load updated. Solve to update the results.", state.loadIndex);
}));
$("delete-load").addEventListener("click", () => setLoads(state.loads.filter((_, index) => index !== state.loadIndex), "Load deleted."));
$("clear-loads").addEventListener("click", () => setLoads([], "External loads cleared."));
$("section-member").addEventListener("change", () => inspectSection($("section-member").value));
for (const id of ["outer-mm", "inner-mm", "young-gpa"]) $(id).addEventListener("input", renderSectionPreview);
$("apply-sections").addEventListener("click", () => perform(() => applySections(false)));
$("apply-all-sections").addEventListener("click", () => perform(() => applySections(true)));
$("generate-history").addEventListener("click", () => perform(() => installHistory(accelerationHistory(state.model, readRise(), readSettings()))));
$("history-member").addEventListener("change", () => {
  state.selected = { kind: "rod", name: $("history-member").value };
  $("track-governing").checked = false;
  refreshHighlights();
  renderHistory();
});
$("track-governing").addEventListener("change", () => { renderHistory(); renderViewport(); });
$("history-scrubber").addEventListener("input", () => seek(Number($("history-scrubber").value), true));
$("play-history").addEventListener("click", togglePlayback);
$("edit-points").addEventListener("click", openPointEditor);
$("close-editor").addEventListener("click", () => $("geometry-editor").close());
$("apply-points").addEventListener("click", () => editAction(false));
$("export-edited").addEventListener("click", () => editAction(true));

for (const button of document.querySelectorAll("[data-example]")) {
  button.addEventListener("click", () => {
    button.closest("details").open = false;
    loadExample(button.dataset.example).catch(error => status(error.message, true));
  });
}
for (const button of document.querySelectorAll("[data-export]")) {
  button.addEventListener("click", () => {
    const menu = button.closest("details");
    if (menu) menu.open = false;
    perform(() => exportData(button.dataset.export));
  });
}
const fileInput = $("file-input");
for (const button of document.querySelectorAll("[data-file]")) {
  button.addEventListener("click", () => {
    const menu = button.closest("details");
    if (menu) menu.open = false;
    const kind = button.dataset.file;
    fileInput.dataset.kind = kind;
    fileInput.accept = ["project", "topology"].includes(kind) ? ".json,application/json" : ".csv,text/csv";
    fileInput.value = "";
    fileInput.click();
  });
}
fileInput.addEventListener("change", async () => {
  const file = fileInput.files[0];
  if (!file) return;
  const kind = fileInput.dataset.kind;
  const revision = ++state.revision;
  try {
    const content = await file.text();
    if (revision === state.revision) importData(kind, content, file.name);
  } catch (error) {
    if (revision === state.revision) status(`${file.name}: ${error.message}`, true);
  }
});

document.addEventListener("click", event => {
  for (const menu of document.querySelectorAll(".file-menu[open]")) {
    if (!menu.contains(event.target)) menu.open = false;
  }
});
document.addEventListener("keydown", event => {
  if (event.key !== "Escape") return;
  for (const menu of document.querySelectorAll(".file-menu[open]")) {
    menu.open = false;
    menu.querySelector("summary").focus();
  }
});

renderPreset();
renderLoads();
renderResults();
renderSections();
syncButtons();
loadExample("default_hardpoints.csv").catch(error => status(error.message, true));
