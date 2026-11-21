/** Portable project files and tabular exports for Suspension Statics Lab. */

import {
  ACCEL, BUMP, CORNERS, DEFAULT_RISE_S, DEFAULT_SECTION, STATIC,
  buildModel, formatCsv, parseCsv, sectionProperties,
} from './engine.js';

const FORMAT = 'SuspensionStaticsLab';
const VERSION = 1;
const CASE_LABELS = new Map([
  [STATIC, '靜態負載'], [ACCEL, '重心加速'], [BUMP, '單輪受衝擊'],
]);
const CASE_IDS = new Map([
  ...Array.from(CASE_LABELS, ([id, label]) => [label, id]),
  ...Array.from(CASE_LABELS.keys(), (id) => [id, id]),
  ['空力下壓', STATIC],
]);
const LOAD_COLUMNS = ['target', 'end', 'x', 'y', 'z'];
const AXES = ['x', 'y', 'z'];
const isRecord = (value) => value !== null && typeof value === 'object' && !Array.isArray(value);

function finiteNumber(value, label) {
  if (typeof value === 'string' && /^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[+-]?\d+)?$/i.test(value.trim())) {
    value = Number(value);
  }
  if (!Number.isFinite(value)) throw new Error(`${label} must be a finite number.`);
  return value;
}

function xyzVector(values, label) {
  if ((!Array.isArray(values) && !ArrayBuffer.isView(values)) || values.length !== 3) {
    throw new Error(`${label} needs three XYZ components.`);
  }
  return Array.from(values, (value) => finiteNumber(value, label));
}

export function validateLoads(model, loads) {
  if (!Array.isArray(loads)) throw new Error('Loads must be a list.');
  const rods = new Map(model.rods.map((rod) => [rod.name, rod]));
  const loadPoints = new Set([
    ...Object.keys(model.owners),
    ...model.rods.flatMap(({ a, b }) => [a, b]),
  ]);
  return loads.map((load, index) => {
    const label = `Load ${index + 1}`;
    if (!isRecord(load) || typeof load.target !== 'string' || !['', 'A', 'B'].includes(load.end)) {
      throw new Error(`${label} needs a valid target and an end of A, B, or blank.`);
    }
    const { target, end, xyz } = load;
    if (end) {
      const point = rods.get(target)?.[end.toLowerCase()];
      if (point === undefined || !Object.hasOwn(model.owners, point)) {
        throw new Error(`${label} couple requires a rod end on a rigid body.`);
      }
    } else if (!loadPoints.has(target)) {
      throw new Error(`${label} uses an unknown load point: ${target}.`);
    }
    return { target, xyz: xyzVector(xyz, label), end };
  });
}

function validateSections(model, sections) {
  if (!isRecord(sections) || Object.keys(sections).length !== model.rods.length
      || model.rods.some(({ name }) => !Object.hasOwn(sections, name))) {
    throw new Error('Project section names must match the rod list.');
  }
  return Object.fromEntries(model.rods.map(({ name }) => {
    const values = sections[name];
    if (!isRecord(values)) throw new Error(`${name} needs section settings.`);
    const section = Object.fromEntries(Object.keys(DEFAULT_SECTION).map((field) => [
      field, finiteNumber(values[field], `${name} ${field}`),
    ]));
    try {
      sectionProperties(section);
      const youngMpa = 1000 * section.young_gpa;
      if (!Number.isFinite(youngMpa) || youngMpa <= 0) throw new Error('Young’s modulus must be finite and positive.');
    } catch (error) {
      throw new Error(`${name}: ${error.message}`);
    }
    return [name, section];
  }));
}

function validateSettings(settings, riseTime) {
  const { massKg, accelerationMps2, includeGravity, downforceN } = settings;
  if (typeof includeGravity !== 'boolean') throw new Error('Project gravity setting must be true or false.');
  const downforce = downforceN == null ? null : finiteNumber(downforceN, 'Downforce');
  if (downforce < 0) throw new Error('Downforce must be zero or greater.');
  const duration = finiteNumber(riseTime, 'Rise time');
  if (duration <= 0) throw new Error('Rise time must be greater than zero.');
  return {
    settings: {
      massKg: finiteNumber(massKg, 'CG mass'),
      accelerationMps2: xyzVector(accelerationMps2, 'CG acceleration'),
      includeGravity,
      downforceN: downforce,
    },
    riseTime: duration,
  };
}

export function parseProject(text) {
  let data;
  try {
    data = JSON.parse(text.replace(/^\uFEFF/, ''));
  } catch {
    throw new Error('The project file is not valid JSON.');
  }
  if (!isRecord(data) || data.format !== FORMAT) throw new Error('Choose a Suspension Statics Lab project JSON.');
  if (data.schema_version !== VERSION) throw new Error('This project version is not supported.');
  const { hardpoints_m: points, topology = null, member_mounts: mounts, case_settings: raw } = data;
  if (!isRecord(points) || !Object.keys(points).length || Object.keys(points).some((name) => !name.trim())) {
    throw new Error('The project needs named hardpoint coordinates.');
  }
  const model = buildModel(points, topology, mounts);
  const sections = validateSections(model, data.sections);
  const loads = validateLoads(model, data.loads);
  const caseName = CASE_IDS.get(raw?.case);
  if (!isRecord(raw) || !caseName) throw new Error('The project has an invalid load case.');
  const configuration = validateSettings({
    massKg: raw.mass_kg,
    accelerationMps2: raw.acceleration_mps2,
    includeGravity: raw.include_gravity,
    downforceN: raw.downforce_n,
  }, Object.hasOwn(raw, 'rise_time_s') ? raw.rise_time_s : DEFAULT_RISE_S);
  return { model, topology, sections, loads, caseName, ...configuration };
}

export function formatProject({ model, topology = null, loads, sections, caseName, settings, riseTime }) {
  const label = CASE_LABELS.get(caseName);
  if (!label) throw new Error('Choose a valid load case.');
  const { settings: validated, riseTime: duration } = validateSettings(settings, riseTime);
  const data = {
    format: FORMAT,
    schema_version: VERSION,
    hardpoints_m: model.points,
    topology,
    loads: validateLoads(model, loads),
    sections: validateSections(model, sections),
    case_settings: {
      case: label,
      mass_kg: validated.massKg,
      acceleration_mps2: validated.accelerationMps2,
      include_gravity: validated.includeGravity,
      rise_time_s: duration,
      downforce_n: validated.downforceN,
    },
  };
  if (Object.keys(model.memberMounts).length) data.member_mounts = model.memberMounts;
  return `${JSON.stringify(data, null, 2)}\n`;
}

export function parseLoadsCsv(text, model) {
  const [header = [], ...rows] = parseCsv(text);
  const names = header.map((name) => name.trim().toLowerCase());
  const columns = new Map(names.map((name, index) => [name, index]));
  if (names.length !== LOAD_COLUMNS.length || !LOAD_COLUMNS.every((name) => columns.has(name))) {
    throw new Error('Load CSV needs the columns target, end, x, y and z.');
  }
  return validateLoads(model, rows.map((row, index) => {
    if (row.length !== names.length) throw new Error(`Load CSV row ${index + 2} has an unexpected number of fields.`);
    return {
      target: row[columns.get('target')],
      end: row[columns.get('end')].trim().toUpperCase(),
      xyz: AXES.map((axis) => row[columns.get(axis)]),
    };
  }));
}

export function formatLoadsCsv(loads) {
  return formatCsv([LOAD_COLUMNS, ...loads.map(({ target, end, xyz }) => [target, end, ...xyz])]);
}

export function formatResultsCsv(equilibrium, results) {
  const header = [
    'member', 'axial_force_N', 'stress_MPa', 'strain_microstrain', 'length_change_mm', 'original_length_mm', 'area_mm2',
    ...AXES.map((axis) => `support_force_${axis}_N`),
    ...AXES.map((axis) => `support_moment_${axis}_Nm`),
    'support_moment_reference',
  ];
  const support = [...equilibrium.supportForce, ...equilibrium.supportMoment, equilibrium.origin ?? 'coordinate origin'];
  const rows = Object.entries(results).map(([name, result]) => [
    name, result.force_n, result.stress_mpa, 1e6 * result.strain, result.change_mm, result.length_mm, result.area_mm2,
    ...support,
  ]);
  return formatCsv([header, ...rows]);
}

function directContactForces(history, model) {
  const wheels = new Map(model.wheels.map(({ corner, contact }) => [corner, contact]));
  if (!CORNERS.every((corner) => wheels.has(corner))) return null;
  const contacts = CORNERS.map((corner) => wheels.get(corner));
  const frames = [];
  for (const loads of history.loads) {
    const byContact = new Map(loads.map((load) => [load.target, load]));
    if (loads.length !== contacts.length
        || !contacts.every((contact) => byContact.has(contact) && !byContact.get(contact).end)) return null;
    frames.push(contacts.flatMap((contact) => Array.from(byContact.get(contact).xyz)));
  }
  return frames;
}

export function formatHistoryCsv(history, model) {
  const { timeS, accelerationMps2, forceN, largestLoads } = history;
  const originalForces = directContactForces(history, model);
  const members = Object.entries(forceN);
  const memberColumns = members.map(([member]) => `member_${member}_N`);
  const uniqueNames = new Set(memberColumns.map((column) => column.toLowerCase()));
  const header = [
    'time_s', ...AXES.map((axis) => `a${axis}_m_s2`),
    ...(originalForces ? CORNERS.flatMap((corner) => AXES.map((axis) => `f${axis}_${corner}_N`)) : []),
    ...(uniqueNames.size === members.length ? memberColumns
      : members.map(([member], index) => `member_${index + 1}_${member}_N`)),
    'governing_member', 'governing_force_N',
  ];
  const rows = Array.from(timeS, (time, index) => [
    time,
    ...Array.from(accelerationMps2[index], (value) => Number.isFinite(value) ? value : ''),
    ...(originalForces?.[index] ?? []),
    ...members.map(([, series]) => series[index]),
    largestLoads[0][index], largestLoads[1][index],
  ]);
  return formatCsv([header, ...rows]);
}
