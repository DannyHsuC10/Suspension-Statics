/** Fixed-geometry suspension equilibrium and load histories. */

export const CORNERS = ['FL', 'FR', 'RL', 'RR'];
export const CHASSIS = 'CHASSIS';
export const STATIC = 'static';
export const ACCEL = 'acceleration';
export const BUMP = 'bump';
export const GRIP = 'grip-circle';
export const CASES = [ACCEL, GRIP, BUMP];
export const MASS_KG = 321;
export const GRAVITY = 9.81;
export const DEFAULT_TIRE_MU = 1.0;
export const DEFAULT_ACCELERATION = [0, 0, 0];
export const DEFAULT_RISE_S = 1;
export const DEFAULT_SECTION = { outer_mm: 19, inner_mm: 15, young_gpa: 205 };

const AXES = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
const PROJECT_COLUMN = 'jacobian_project';
const WHEEL_LIFT = 'The example distribution gives negative normal load at one or more tires. Adjust the CG or acceleration, or enter wheel loads directly.';
class WheelLiftError extends Error {}
const ALIASES = {
  UFI: 'UCA_F', URI: 'UCA_R', LFI: 'LCA_F', LRI: 'LCA_R', WHEEL_CENTER: 'WC',
  ROD_IN: 'RACK', ROD_OUT: 'TOE', TIE_IN: 'RACK', TIE_OUT: 'TOE',
  TIE_ROD_IN: 'RACK', TIE_ROD_OUT: 'TOE', PUSH_OUT_LCA: 'PUSH_OUT',
  PUSHROD_IN: 'PUSH_IN', PUSHROD_OUT: 'PUSH_OUT',
};
const ARM_POINTS = ['UCA_F', 'UCA_R', 'LCA_F', 'LCA_R', 'UBJ', 'LBJ'];
const CORNER_POINTS = new Set([
  ...ARM_POINTS, 'RACK', 'TOE', 'WC', 'CONTACT', 'CONSTRUCTION_CONTACT', 'FORWARD',
  'PUSH_IN', 'PUSH_OUT', 'PUSH_OUT_UPRIGHT', 'PUSH_OUT_UCA',
  'ROCKER_A', 'ROCKER_B', 'DAMPER_IN', 'DAMPER_BODY',
]);

const own = (object, key) => Object.hasOwn(object, key);
const isRecord = (value) => value !== null && typeof value === 'object' && !Array.isArray(value);
const isName = (value) => typeof value === 'string' && value.trim() !== '';
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const scale = (a, value) => a.map((component) => component * value);
const subtract = (a, b) => a.map((component, index) => component - b[index]);
const norm = (a) => Math.hypot(...a);

function number(value, label) {
  if (typeof value === 'string' && /^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[+-]?\d+)?$/i.test(value.trim())) {
    value = Number(value);
  }
  if (!Number.isFinite(value)) throw new Error(`${label} must be a finite number.`);
  return value;
}

function vector(values, label) {
  if ((!Array.isArray(values) && !ArrayBuffer.isView(values)) || values.length !== 3) {
    throw new Error(`${label} needs three finite XYZ components.`);
  }
  return Array.from(values, (value) => number(value, label));
}

function unitVector(a, b, label) {
  const difference = subtract(b, a);
  const length = norm(difference);
  if (!Number.isFinite(length) || length <= 1e-9) {
    throw new Error(`${label} has coincident endpoints or coordinates outside the numerical range.`);
  }
  return [scale(difference, 1 / length), length];
}

function csvRows(input, limit = Infinity) {
  const text = input.replace(/^\uFEFF/, '');
  const rows = [];
  let fields = [];
  let cell = '';
  let line = 1;
  let quoted = false;
  let closed = false;
  let hasContent = false;
  const endCell = () => {
    fields.push(cell);
    cell = '';
    closed = false;
  };
  const endRow = () => {
    endCell();
    if (hasContent) rows.push({ fields, line });
    fields = [];
    hasContent = false;
  };
  for (let index = 0; index < text.length; index += 1) {
    const character = text[index];
    const next = text[index + 1];
    if (quoted) {
      if (character === '"' && next === '"') {
        cell += '"';
        index += 1;
      } else if (character === '"') {
        quoted = false;
        closed = true;
      } else {
        cell += character;
        if (character === '\n' || (character === '\r' && next !== '\n')) line += 1;
      }
    } else if (character === ',') {
      hasContent = true;
      endCell();
    } else if (character === '\r' || character === '\n') {
      endRow();
      if (rows.length === limit) return rows;
      if (character === '\r' && next === '\n') index += 1;
      line += 1;
    } else if (closed) {
      throw new Error(`Unexpected text after a quoted CSV value on line ${line}.`);
    } else {
      hasContent = true;
      if (character === '"' && cell === '') quoted = true;
      else cell += character;
    }
  }
  if (quoted) throw new Error(`Unclosed quoted CSV value on line ${line}.`);
  if (hasContent) endRow();
  return rows;
}

export function parseCsv(text) {
  return csvRows(text).map(({ fields }) => fields);
}

export function readCsvHeader(text) {
  return (csvRows(text, 1)[0]?.fields ?? []).map((name) => name.trim().toLowerCase());
}

function csvTable(input) {
  const rows = csvRows(input);
  const header = (rows.shift()?.fields ?? []).map((field) => field.trim().toLowerCase());
  const names = new Set(header);
  if (names.size !== header.length) throw new Error('CSV column names must be unique.');
  return { names, rows: rows.map(({ fields: values, line: rowLine }) => {
    if (values.length !== header.length) throw new Error(`CSV line ${rowLine} has an unexpected number of fields.`);
    return { line: rowLine, values: Object.fromEntries(header.map((field, index) => [field, values[index]])) };
  }) };
}

export function parseHardpointCsv(text) {
  const { names, rows } = csvTable(text);
  const units = ['m', 'mm'].filter((unit) => ['x', 'y', 'z'].every((axis) => names.has(`${axis}_${unit}`)));
  const nameColumn = names.has('name') ? 'name' : 'point';
  if (units.length !== 1 || !names.has(nameColumn)) {
    throw new Error('Hardpoint CSV needs a name or point column and a complete XYZ set in meters or millimeters.');
  }
  const [unit] = units;
  const meterScale = unit === 'mm' ? 0.001 : 1;
  const points = new Map();
  let payload = '';
  for (const { line, values } of rows) {
    let name = values[nameColumn].trim();
    if (!name) throw new Error(`CSV line ${line} has no hardpoint name.`);
    if (nameColumn === 'point' && values.corner?.trim()) name = `${values.corner.trim()}_${name}`;
    if (points.has(name)) throw new Error(`Duplicate hardpoint ${name}.`);
    points.set(name, ['x', 'y', 'z'].map((axis) => number(values[`${axis}_${unit}`], `${name} ${axis.toUpperCase()}`) * meterScale));
    const settings = values[PROJECT_COLUMN]?.trim();
    if (settings) {
      if (payload) throw new Error('A hardpoint CSV can contain only one Jacobian project settings cell.');
      payload = settings;
    }
  }
  if (points.size === 0) throw new Error('The CSV contains no hardpoints.');
  const memberMounts = new Map();
  if (payload) {
    let settings;
    try {
      settings = JSON.parse(payload);
    } catch {
      throw new Error('The embedded Jacobian project settings could not be parsed.');
    }
    const members = isRecord(settings) && !own(settings, 'deformable_members') ? [] : settings?.deformable_members;
    if (!Array.isArray(members)) throw new Error('Jacobian deformable_members must be a list.');
    const pointNames = new Set([...points.keys()].map((name) => name.toLowerCase()));
    const memberNames = new Set();
    for (const member of members) {
      if (!isRecord(member) || !isName(member.name)) throw new Error('A deformable member needs a rod name.');
      const name = member.name.trim();
      if (memberNames.has(name.toLowerCase())) throw new Error(`Duplicate rod ${name}.`);
      memberNames.add(name.toLowerCase());
      const mounts = [];
      for (const label of ['A', 'B']) {
        const point = `${name}_${label}`;
        if (pointNames.has(point.toLowerCase())) throw new Error(`Duplicate hardpoint ${point}.`);
        const end = member[label.toLowerCase()];
        // The embedded Jacobian endpoints are meters regardless of the CSV header.
        points.set(point, vector(end?.xyz, point));
        pointNames.add(point.toLowerCase());
        mounts.push(end.mount);
      }
      memberMounts.set(name, mounts);
    }
  }
  return { points: Object.fromEntries(points), memberMounts: Object.fromEntries(memberMounts) };
}

function csvCell(value) {
  const text = String(value);
  return /[",\r\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
}

export function formatCsv(rows) {
  if (!rows.length) return '';
  const lines = rows.map((row) => {
    const cells = Array.from(row, csvCell);
    return cells.length === 1 && cells[0] === '' ? '""' : cells.join(',');
  });
  return `${lines.join('\r\n')}\r\n`;
}

export function formatHardpointCsv(points, memberMounts = {}) {
  const header = ['name', 'x_m', 'y_m', 'z_m'];
  const endpoints = new Set();
  const members = Object.entries(memberMounts).map(([name, mounts]) => {
    const a = `${name}_A`;
    const b = `${name}_B`;
    endpoints.add(a);
    endpoints.add(b);
    return { name, a: { xyz: vector(points[a], a), mount: mounts[0] }, b: { xyz: vector(points[b], b), mount: mounts[1] } };
  });
  if (members.length) header.push(PROJECT_COLUMN);
  const rows = [header];
  let payload = JSON.stringify({ version: 1, deformable_members: members });
  for (const [name, coordinates] of Object.entries(points)) {
    if (endpoints.has(name)) continue;
    const row = [name, ...vector(coordinates, name)];
    if (members.length) row.push(payload);
    rows.push(row);
    payload = '';
  }
  return formatCsv(rows);
}

function wheelPoints(points) {
  const corners = Object.fromEntries(CORNERS.map((corner) => [corner, Object.create(null)]));
  const names = Object.keys(points).map((name) => {
    const upper = name.toUpperCase();
    const underscore = upper.indexOf('_');
    const prefix = underscore === -1 ? upper : upper.slice(0, underscore);
    const suffix = underscore === -1 ? '' : upper.slice(underscore + 1);
    return { name, upper, prefix, suffix };
  });
  const recognized = [...ARM_POINTS, 'WC'];
  const hasPrefixes = names.some(({ prefix, suffix }) => CORNERS.includes(prefix) && recognized.includes(ALIASES[suffix] ?? suffix));
  for (const { name, upper, prefix, suffix } of names) {
    const unprefixed = !CORNERS.includes(prefix) && !hasPrefixes;
    const corner = unprefixed ? 'FL' : prefix;
    const rawKey = unprefixed ? upper : suffix;
    const key = ALIASES[rawKey] ?? rawKey;
    if (!CORNERS.includes(corner) || !CORNER_POINTS.has(key)) continue;
    if (own(corners[corner], key)) throw new Error(`${corner} has more than one ${key} hardpoint.`);
    corners[corner][key] = name;
  }
  const wheels = [];
  for (const corner of CORNERS) {
    const parts = corners[corner];
    if (!parts.WC) continue;
    const center = parts.WC;
    let contact = parts.CONTACT || parts.CONSTRUCTION_CONTACT;
    if (!contact) {
      const [x, y, z] = points[center];
      contact = `${corner}_CONTACT`;
      points[contact] = [x, y, z >= 0.1 && z <= 0.45 ? 0 : z - 0.196];
    }
    wheels.push({ corner, center, contact, heading: parts.FORWARD ?? null });
  }
  return { corners, wheels };
}

function connectEdges(edges, names) {
  for (let start = 0; start < names.length; start += 1) {
    for (let end = start + 1; end < names.length; end += 1) {
      edges.push([names[start], names[end]]);
    }
  }
}

function customModel(points, topology, wheels, cg) {
  if (!isRecord(topology)) throw new Error('Topology JSON must be an object.');
  const { fixed = [], bodies = {}, rods: members = [] } = topology;
  if (!Array.isArray(fixed) || !isRecord(bodies) || !Array.isArray(members)) {
    throw new Error('Topology needs a fixed array, bodies object and rods array.');
  }
  if (own(bodies, CHASSIS) || Object.keys(bodies).some((name) => !isName(name))) {
    throw new Error('Rigid bodies need names, and CHASSIS is reserved.');
  }
  const owners = Object.create(null);
  const rigidEdges = [];
  for (const [body, names] of [[CHASSIS, fixed], ...Object.entries(bodies)]) {
    if (!Array.isArray(names) || (body !== CHASSIS && !names.length)) throw new Error(`${body} needs a list of hardpoint names.`);
    for (const name of names) {
      if (typeof name !== 'string' || !own(points, name) || own(owners, name)) {
        throw new Error(`${body} uses an unknown or already owned hardpoint ${name}.`);
      }
      owners[name] = body;
    }
    if (body !== CHASSIS) connectEdges(rigidEdges, names);
  }
  const labels = new Set();
  const rods = members.map((member) => {
    if (!isRecord(member) || ![member.name, member.a, member.b].every(isName)) {
      throw new Error('Each rod needs name, a and b fields.');
    }
    const { name, a, b } = member;
    if (labels.has(name) || a === b || !own(points, a) || !own(points, b)) {
      throw new Error(`Rod ${name} uses a duplicate name or invalid endpoints.`);
    }
    labels.add(name);
    return { name, a, b };
  });
  const cop = Object.keys(points).find((name) => name.toUpperCase() === 'COP') ?? null;
  return { points, owners: { ...owners }, bodies: Object.keys(bodies), rods, rigidEdges, wheels, cg, cop,
    joints: [], notes: [], memberMounts: {} };
}

function equivalentPushrodEnds(points) {
  const { corners } = wheelPoints({ ...points });
  const ends = [];
  for (const [corner, local] of Object.entries(corners)) {
    if (!local.PUSH_IN || local.PUSH_OUT_UPRIGHT || (local.PUSH_OUT_UCA && local.PUSH_OUT)) continue;
    const original = local.PUSH_OUT_UCA ?? local.PUSH_OUT;
    const joint = local.PUSH_OUT_UCA ? local.UBJ : local.LBJ;
    if (original && joint) ends.push({ original, joint, replacement: `${corner}_PUSH_OUT_UPRIGHT` });
  }
  return ends;
}

export function hasEquivalentPushrodEnds(points) {
  return equivalentPushrodEnds(points).length > 0;
}

export function relocateArmPushrods(points) {
  const renamed = new Map();
  for (const { original, joint, replacement } of equivalentPushrodEnds(points)) {
    renamed.set(original, replacement);
    points[replacement] = [...points[joint]];
    delete points[original];
  }
  return renamed;
}

export function buildModel(hardpoints, topology = null, memberMounts = {}) {
  if (!isRecord(hardpoints)) throw new Error('Hardpoints must be an object of XYZ coordinates.');
  const points = Object.fromEntries(Object.entries(hardpoints).map(([name, coordinates]) => [name, vector(coordinates, name)]));
  const { corners, wheels } = wheelPoints(points);
  let cg = Object.keys(points).find((name) => name.toUpperCase() === 'CG') ?? null;
  if (!cg && wheels.length) {
    cg = 'CG';
    const mean = [0, 1].map((index) => wheels.reduce((sum, wheel) => sum + points[wheel.center][index], 0) / wheels.length);
    points[cg] = [...mean, Math.min(...wheels.map((wheel) => points[wheel.contact][2])) + 0.31];
  }
  if (topology !== null) return customModel(points, topology, wheels, cg);
  memberMounts ??= {};
  if (!isRecord(memberMounts)) throw new Error('Rod mount settings need names with A and B mounts.');
  const addedRods = [];
  const mountedPoints = Object.fromEntries([CHASSIS, ...CORNERS].map((mount) => [mount, []]));
  const memberNames = new Set();
  for (const [name, mounts] of Object.entries(memberMounts)) {
    if (!isName(name) || memberNames.has(name.toLowerCase())) throw new Error('Imported rods must have unique names.');
    memberNames.add(name.toLowerCase());
    if (!Array.isArray(mounts) || mounts.length !== 2) throw new Error(`${name} needs A and B mounts.`);
    const a = `${name}_A`;
    const b = `${name}_B`;
    [a, b].forEach((point, index) => {
      const mount = mounts[index];
      if (!own(points, point)) throw new Error(`Cannot find rod endpoint ${point}.`);
      if (typeof mount !== 'string' || !own(mountedPoints, mount)) {
        throw new Error(`${point} must mount to CHASSIS or an FL, FR, RL or RR rocker.`);
      }
      mountedPoints[mount].push(point);
    });
    addedRods.push({ name, a, b });
  }

  const owners = Object.create(null);
  if (cg) owners[cg] = CHASSIS;
  for (const point of mountedPoints[CHASSIS]) owners[point] = CHASSIS;
  const cop = Object.keys(points).find((name) => name.toUpperCase() === 'COP') ?? null;
  if (cop) owners[cop] = CHASSIS;
  const rods = [];
  const bodies = [];
  const joints = [];
  const edges = [];
  const notes = [];
  if (addedRods.length) notes.push(`Imported ${addedRods.length} axial rod${addedRods.length === 1 ? '' : 's'} from the CSV.`);
  const contacts = Object.fromEntries(wheels.map((wheel) => [wheel.corner, wheel.contact]));
  for (const [corner, local] of Object.entries(corners)) {
    if (!ARM_POINTS.some((key) => own(local, key)) && !contacts[corner]) continue;
    const assign = (body, keys) => {
      for (const key of keys) if (local[key]) owners[local[key]] = body;
    };
    const addRod = (name, a, b) => {
      if (local[a] && local[b]) rods.push({ name: `${corner}_${name}`, a: local[a], b: local[b] });
    };
    const drawEdges = (keys) => connectEdges(edges, keys.map((key) => local[key]).filter(Boolean));
    const upright = `${corner}_UPRIGHT`;
    assign(upright, ['UBJ', 'LBJ', 'TOE', 'WC', 'FORWARD']);
    if (contacts[corner]) owners[contacts[corner]] = upright;
    if (Object.values(owners).includes(upright)) bodies.push(upright);
    assign(CHASSIS, ['UCA_F', 'UCA_R', 'LCA_F', 'LCA_R', 'RACK', 'ROCKER_A', 'ROCKER_B', 'DAMPER_BODY']);
    drawEdges(['UBJ', 'LBJ', 'TOE', 'WC']);
    if (local.WC) edges.push([local.WC, contacts[corner]]);

    const outboard = ['PUSH_OUT', 'PUSH_OUT_UCA', 'PUSH_OUT_UPRIGHT'].filter((key) => local[key]);
    if (outboard.length > 1) throw new Error(`${corner} has more than one pushrod outboard hardpoint.`);
    const push = outboard[0];
    const pushConnected = local.PUSH_IN && push;
    for (const [arm, ball, pushKey] of [['UCA', 'UBJ', 'PUSH_OUT_UCA'], ['LCA', 'LBJ', 'PUSH_OUT']]) {
      const a = `${arm}_F`;
      const b = `${arm}_R`;
      if (pushConnected && push === pushKey && local[a] && local[b] && local[ball]) {
        const body = `${corner}_${arm}`;
        bodies.push(body);
        owners[local[push]] = body;
        joints.push({ body, base: CHASSIS, point: local[a], axisEnd: local[b] });
        joints.push({ body, base: upright, point: local[ball], axisEnd: null });
        drawEdges([a, b, ball, push]);
        notes.push(`${body} uses a rigid arm and hinge at the pushrod mount.`);
      } else {
        for (const mount of [a, b]) addRod(`${mount}-${ball}`, mount, ball);
      }
    }
    addRod('TIE_ROD', 'RACK', 'TOE');
    if (pushConnected && push === 'PUSH_OUT_UPRIGHT') {
      owners[local[push]] = upright;
      drawEdges(['LBJ', 'WC', push]);
    }
    if (pushConnected && own(owners, local[push])) {
      addRod('PUSH_ROD', push, 'PUSH_IN');
      const legacyDamper = local.DAMPER_IN && local.DAMPER_BODY;
      const memberEnds = mountedPoints[corner];
      if (local.ROCKER_A && local.ROCKER_B && (legacyDamper || memberEnds.length)) {
        const body = `${corner}_ROCKER`;
        bodies.push(body);
        const rockerPoints = [local.PUSH_IN, ...memberEnds];
        if (legacyDamper) {
          rockerPoints.unshift(local.DAMPER_IN);
          addRod('DAMPER', 'DAMPER_IN', 'DAMPER_BODY');
        }
        for (const point of rockerPoints) owners[point] = body;
        joints.push({ body, base: CHASSIS, point: local.ROCKER_A, axisEnd: local.ROCKER_B });
        connectEdges(edges, [local.ROCKER_A, local.ROCKER_B, ...rockerPoints]);
      } else {
        owners[local.PUSH_IN] = CHASSIS;
        drawEdges(['ROCKER_A', 'ROCKER_B', 'PUSH_IN']);
        notes.push(`${corner} pushrod inboard end is fixed to the chassis. A rocker axis and damper can define a hinged rocker.`);
      }
    } else {
      notes.push(`${corner} has an incomplete pushrod load path.`);
    }
  }

  for (const corner of CORNERS) {
    if (mountedPoints[corner].length && !bodies.includes(`${corner}_ROCKER`)) {
      throw new Error(`${corner} imported rods require ROCKER_A, ROCKER_B and a complete pushrod connection.`);
    }
  }
  if (rods.some((rod) => memberNames.has(rod.name.toLowerCase()))) {
    throw new Error('An imported axial rod has the same name as a suspension rod.');
  }
  rods.push(...addedRods);
  if (!rods.length) notes.push('Only hardpoints are defined. Use standard suspension point names or a custom topology.');
  const rigidEdges = [...new Map(edges.map((edge) => [JSON.stringify(edge), edge])).values()];
  return { points, owners: { ...owners }, bodies, rods, joints, rigidEdges, wheels, cg, cop, notes, memberMounts };
}

function dot(a, b) {
  let total = 0;
  for (let index = 0; index < a.length; index += 1) total += a[index] * b[index];
  return total;
}

function transpose(matrix, columns) {
  return Array.from({ length: columns }, (_, column) => Float64Array.from(matrix, (row) => row[column]));
}

function rotateColumns(a, b, cosine, sine) {
  for (let index = 0; index < a.length; index += 1) {
    const original = a[index];
    a[index] = cosine * original - sine * b[index];
    b[index] = sine * original + cosine * b[index];
  }
}

// One-sided Jacobi SVD rotates columns directly, avoiding squared conditioning
// from normal equations. The transpose path handles minimum-norm wide systems.
function leastSquares(matrix, columnCount) {
  let largest = 0;
  for (const row of matrix) {
    for (const value of row) {
      if (!Number.isFinite(value)) throw new Error('Coordinates or loads exceed the numerical range.');
      largest = Math.max(largest, Math.abs(value));
    }
  }
  if (!largest) return { rank: 0, compute: () => new Float64Array(columnCount) };
  const transposed = matrix.length < columnCount;
  const originalColumns = transposed ? matrix : transpose(matrix, columnCount);
  const columns = originalColumns.map((column) => Float64Array.from(column, (value) => value / largest));
  const right = columns.map((_, index) => Float64Array.from(columns, (unused, row) => Number(row === index)));
  const count = columns.length;
  for (let sweep = 0; sweep < Math.max(30, 2 * count); sweep += 1) {
    let moved = false;
    for (let left = 0; left < count - 1; left += 1) {
      for (let next = left + 1; next < count; next += 1) {
        const a = columns[left];
        const b = columns[next];
        const aa = dot(a, a);
        const bb = dot(b, b);
        const ab = dot(a, b);
        if (Math.abs(ab) <= 8 * Number.EPSILON * Math.sqrt(aa) * Math.sqrt(bb)) continue;
        const offset = (bb - aa) / (2 * ab);
        const tangent = (offset < 0 ? -1 : 1) / (Math.abs(offset) + Math.hypot(1, offset));
        const cosine = 1 / Math.hypot(1, tangent);
        const sine = tangent * cosine;
        rotateColumns(a, b, cosine, sine);
        rotateColumns(right[left], right[next], cosine, sine);
        moved = true;
      }
    }
    if (!moved) break;
  }
  const singular = columns.map((column) => Math.sqrt(dot(column, column)));
  const threshold = 1e-10 * Math.max(...singular);
  const components = [];
  for (let index = 0; index < count; index += 1) {
    if (singular[index] <= threshold) continue;
    const unitColumn = columns[index].map((component) => component / singular[index]);
    components.push({
      left: transposed ? right[index] : unitColumn,
      right: transposed ? unitColumn : right[index],
      singular: singular[index],
    });
  }
  return { rank: components.length, compute(rhs) {
    const result = new Float64Array(columnCount);
    for (const component of components) {
      const factor = (dot(component.left, rhs) / largest) / component.singular;
      for (let index = 0; index < columnCount; index += 1) result[index] += component.right[index] * factor;
    }
    return result;
  } };
}

export class FrameError extends Error {
  constructor(index, message) {
    super(message);
    this.name = 'FrameError';
    this.index = index;
  }
}

export function solve(model, loads = []) {
  return solveFrames(model, [loads])[0];
}

function solveFrames(model, loadFrames, skipUnbalanced = false) {
  if (!model.rods.length) throw new Error('There are no axial rods to solve.');
  const points = model.points;
  const rods = new Map(model.rods.map((rod) => [rod.name, rod]));
  const owners = new Map(Object.entries(model.owners));
  const pins = new Map();
  for (const rod of model.rods) {
    for (const point of [rod.a, rod.b]) {
      if (owners.has(point)) continue;
      const pin = Symbol(point);
      pins.set(point, pin);
      owners.set(point, pin);
    }
  }
  const pinIds = [...pins.entries()].sort(([a], [b]) => (a < b ? -1 : Number(a > b))).map(([, id]) => id);
  const incident = new Map([...model.bodies, ...pinIds].map((body) => [body, []]));
  for (const [point, owner] of owners) if (incident.has(owner)) incident.get(owner).push(points[point]);
  for (const joint of model.joints) {
    for (const body of [joint.body, joint.base]) {
      if (!incident.has(body)) continue;
      incident.get(body).push(points[joint.point]);
      if (joint.axisEnd) incident.get(body).push(points[joint.axisEnd]);
    }
  }

  const rows = new Map();
  let nrows = 0;
  for (const [body, positions] of incident) {
    const width = typeof body === 'symbol' ? 3 : 6;
    const center = AXES.map((_, axis) => positions.reduce((sum, position) => sum + position[axis], 0) / positions.length);
    const size = Math.max(0.001, ...positions.map((position) => norm(subtract(position, center))));
    rows.set(body, { start: nrows, width, center, size });
    nrows += width;
  }

  const apply = (values, body, position, force, moment = [0, 0, 0]) => {
    if (body === CHASSIS) return;
    const { start, width, center, size } = rows.get(body);
    for (let index = 0; index < 3; index += 1) values[start + index] += force[index];
    if (width === 6) {
      const torque = cross(subtract(position, center), force);
      for (let index = 0; index < 3; index += 1) values[start + 3 + index] += (torque[index] + moment[index]) / size;
    }
  };

  const rodLengths = [];
  const columns = [];
  for (const { name, a, b } of model.rods) {
    const [direction, length] = unitVector(points[a], points[b], name);
    const column = new Float64Array(nrows);
    apply(column, owners.get(a), points[a], direction);
    apply(column, owners.get(b), points[b], scale(direction, -1));
    columns.push(column);
    rodLengths.push([name, length * 1000]);
  }

  for (const joint of model.joints) {
    const position = points[joint.point];
    for (const direction of AXES) {
      const column = new Float64Array(nrows);
      apply(column, joint.body, position, direction);
      apply(column, joint.base, position, scale(direction, -1));
      columns.push(column);
    }
    if (joint.axisEnd) {
      const [axis] = unitVector(position, points[joint.axisEnd], `${joint.body} hinge`);
      const nearest = AXES.reduce((best, _, index) => (Math.abs(axis[index]) < Math.abs(axis[best]) ? index : best), 0);
      const perpendicular = cross(axis, AXES[nearest]);
      const normal = scale(perpendicular, 1 / norm(perpendicular));
      for (const direction of [normal, cross(axis, normal)]) {
        const column = new Float64Array(nrows);
        const moment = scale(direction, rows.get(joint.body).size);
        apply(column, joint.body, position, [0, 0, 0], moment);
        apply(column, joint.base, position, [0, 0, 0], scale(moment, -1));
        columns.push(column);
      }
    }
  }

  const origin = model.cg ? points[model.cg] : [0, 0, 0];
  const influences = new Map();
  const influence = (target, end) => {
    const key = JSON.stringify([target, end]);
    if (influences.has(key)) return influences.get(key);
    let point = target;
    if (end) {
      const rod = rods.get(target);
      if (!rod || !['A', 'B'].includes(end)) throw new Error(`Cannot find the selected end of rod ${target}.`);
      point = end === 'A' ? rod.a : rod.b;
      if (pins.has(point)) throw new Error(`${point} is a force-only pin. Choose a rod end attached to a rigid body.`);
    } else if (!owners.has(target)) {
      throw new Error(`${target} is not connected to the force model. Choose a rod end or rigid-body hardpoint.`);
    }
    const position = points[point];
    const terms = AXES.map((direction) => {
      const term = new Float64Array(nrows + 6);
      if (end) {
        apply(term, owners.get(point), position, [0, 0, 0], direction);
        term.set(direction, nrows + 3);
      } else {
        apply(term, owners.get(point), position, direction);
        term.set(direction, nrows);
        term.set(cross(subtract(position, origin), direction), nrows + 3);
      }
      return term;
    });
    influences.set(key, terms);
    return terms;
  };

  const applied = loadFrames.map((loads, index) => {
    const terms = new Float64Array(nrows + 6);
    for (const load of loads) {
      try {
        const xyz = vector(load.xyz, 'Force or moment');
        const coefficients = influence(load.target, load.end ?? '');
        for (let axis = 0; axis < 3; axis += 1) {
          for (let row = 0; row < terms.length; row += 1) terms[row] += xyz[axis] * coefficients[axis][row];
        }
      } catch (error) {
        throw new FrameError(index, error.message);
      }
    }
    if (!terms.every(Number.isFinite)) throw new Error('Coordinates or loads exceed the numerical range.');
    return terms;
  });
  const matrix = transpose(columns, nrows);
  const decomposition = leastSquares(matrix, columns.length);
  const { rank } = decomposition;
  if (rank < columns.length) {
    throw new Error(`Internal forces have no unique solution (matrix rank ${rank}/${columns.length}). Check duplicate rods, collinear hardpoints or redundant constraints.`);
  }
  const lengthsMm = Object.fromEntries(rodLengths);
  return applied.map((terms, index) => {
    const targets = terms.subarray(0, nrows);
    const unknowns = decomposition.compute(targets.map((value) => -value));
    if (!unknowns.every(Number.isFinite)) throw new Error('Internal forces exceed the numerical range.');
    const residual = Float64Array.from(matrix, (row, i) => dot(row, unknowns) + targets[i]);
    if (!residual.every(Number.isFinite)) throw new Error('Equilibrium residual exceeds the numerical range.');
    const magnitude = targets.reduce((largest, value) => Math.max(largest, Math.abs(value)), 1);
    if (residual.some((value) => Math.abs(value) > 1e-7 * magnitude)) {
      if (skipUnbalanced) return null;
      let worst;
      let largest = -1;
      for (const [body, { start, width }] of rows) {
        const size = norm(residual.subarray(start, start + width));
        if (size > largest) {
          largest = size;
          worst = typeof body === 'symbol' ? body.description : body;
        }
      }
      throw new FrameError(index, `${worst} cannot balance this load (matrix rank ${rank}/${nrows}). Check pushrods, tie rods, supports or load direction.`);
    }
    return {
      forces: Object.fromEntries(model.rods.map(({ name }, rod) => [name, unknowns[rod]])),
      lengthsMm,
      supportForce: Array.from(terms.subarray(nrows, nrows + 3), (value) => -value),
      supportMoment: Array.from(terms.subarray(nrows + 3), (value) => -value),
      origin: model.cg,
      rank,
      equations: nrows,
    };
  });
}

function sectionArea(section) {
  const outer = number(section.outer_mm, 'Outer diameter');
  const inner = number(section.inner_mm, 'Inner diameter');
  if (outer <= inner || inner < 0) throw new Error('Section diameters must satisfy outer > inner ≥ 0.');
  const area = Math.PI / 4 * (outer - inner) * (outer + inner);
  if (!Number.isFinite(area) || area <= 0) throw new Error('Section area exceeds the numerical range.');
  return area;
}

export function sectionProperties(section = DEFAULT_SECTION) {
  const area = sectionArea(section);
  const gyration = Math.hypot(section.outer_mm, section.inner_mm) / 4;
  const secondMoment = area * gyration * gyration;
  const modulus = secondMoment / (section.outer_mm / 2);
  const properties = {
    area_mm2: area,
    second_moment_mm4: secondMoment,
    modulus_mm3: modulus,
    gyration_mm: gyration,
    polar_moment_mm4: 2 * secondMoment,
    polar_modulus_mm3: 2 * modulus,
    polar_gyration_mm: Math.SQRT2 * gyration,
  };
  if (Object.values(properties).some((value) => !Number.isFinite(value) || value <= 0)) {
    throw new Error('Section properties exceed the numerical range.');
  }
  return properties;
}

export function memberResults(equilibrium, sections) {
  return Object.fromEntries(Object.entries(equilibrium.forces).map(([name, force]) => {
    if (!own(sections, name)) throw new Error(`No section is defined for ${name}.`);
    const section = sections[name];
    const area = sectionArea(section);
    const youngMpa = number(section.young_gpa, 'Young’s modulus') * 1000;
    if (!Number.isFinite(youngMpa) || youngMpa <= 0) throw new Error('Young’s modulus must be finite and positive.');
    const length = equilibrium.lengthsMm[name];
    const stress = force / area;
    const strain = stress / youngMpa;
    const change = strain * length;
    if (![stress, strain, change].every(Number.isFinite)) {
      throw new Error(`${name} stress, strain or change in length exceeds the numerical range.`);
    }
    return [name, { force_n: force, length_mm: length, area_mm2: area, stress_mpa: stress, strain, change_mm: change }];
  }));
}

function accelerationSetting(settings) {
  return settings.accelerationMps2 ?? settings.acceleration_mps2 ?? DEFAULT_ACCELERATION;
}

function downforceSetting(settings) {
  const enabled = settings.downforceEnabled ?? settings.downforce_enabled ?? true;
  const value = enabled ? settings.downforceN ?? settings.downforce_n ?? 0 : 0;
  const downforce = number(value, 'Downforce');
  if (downforce < 0) throw new Error('Downforce must be zero or greater.');
  return downforce;
}

export function tireMuCoefficient(value = DEFAULT_TIRE_MU) {
  const coefficient = number(value, 'Tire grip coefficient μ');
  if (coefficient < 0) throw new Error('Tire grip coefficient μ must be zero or greater.');
  return coefficient;
}

function presetParameters(settings, accelerating) {
  const mass = number(settings.massKg ?? settings.mass_kg ?? MASS_KG, 'Vehicle mass');
  if (mass <= 0) throw new Error('Vehicle mass must be positive.');
  const gravity = !accelerating || (settings.includeGravity ?? settings.include_gravity ?? true) ? GRAVITY : 0;
  const acceleration = accelerating ? vector(accelerationSetting(settings), 'Acceleration') : [0, 0, 0];
  const downforce = accelerating ? downforceSetting(settings) : 0;
  const tireMu = accelerating ? tireMuCoefficient(settings.tireMu ?? settings.tire_mu) : DEFAULT_TIRE_MU;
  const apparentWeight = mass * gravity + mass * acceleration[2] + downforce;
  if (!Number.isFinite(apparentWeight)) throw new Error('Mass, acceleration or downforce exceeds the numerical range.');
  if (apparentWeight < 0) throw new WheelLiftError(WHEEL_LIFT);
  return { mass, acceleration, tireMu, apparentWeight };
}

export function gripAccelerationLimit(settings = {}) {
  const { mass, tireMu, apparentWeight } = presetParameters(settings, true);
  if (tireMu === 0 || apparentWeight === 0) return 0;
  const limit = tireMu * (apparentWeight / mass);
  if (!Number.isFinite(limit)) throw new Error('The friction-circle acceleration limit exceeds the numerical range.');
  return limit;
}

export function exampleLoads(model, caseName, settings = {}) {
  if (caseName !== STATIC && !CASES.includes(caseName)) throw new Error('Choose a valid example load case.');
  if (caseName === BUMP) {
    if (!model.wheels.length) throw new Error('The single-wheel bump needs at least one wheel center (WC).');
    return [{ target: model.wheels[0].contact, xyz: [0, 0, 1800], end: '' }];
  }
  if (model.wheels.length !== 4 || !model.cg) throw new Error('This case needs FL, FR, RL and RR wheel centers and a center of gravity.');
  const contacts = model.wheels.map((wheel) => wheel.contact);
  if ([model.cg, ...contacts].some((point) => !own(model.owners, point))) {
    throw new Error('This case needs a rigid body or chassis assigned to the CG and all tire contacts.');
  }
  const relative = contacts.map((point) => subtract(model.points[point], model.points[model.cg]));
  const geometryBalance = [[1, 1, 1, 1], relative.map((point) => point[1]), relative.map((point) => -point[0])];
  const combined = caseName === GRIP;
  const { mass, acceleration, tireMu, apparentWeight } = presetParameters(settings, caseName === ACCEL || combined);
  const inertial = scale(acceleration, -mass);
  if (!inertial.every(Number.isFinite)) throw new Error('Mass or acceleration exceeds the numerical range.');

  const longitudinalDemand = -inertial[0];
  const lateralDemand = -inertial[1];
  const longitudinalRatio = combined && apparentWeight > 0 ? longitudinalDemand / apparentWeight : 0;
  const lateralRatio = apparentWeight > 0 ? lateralDemand / apparentWeight : 0;
  const requiredForce = combined ? Math.hypot(longitudinalDemand, lateralDemand) : Math.abs(lateralDemand);
  // Compare ratios so μ × normal load cannot overflow the grip test.
  const requiredRatio = Math.hypot(longitudinalRatio, lateralRatio);
  const tolerance = 1e-12 * Math.max(requiredRatio, tireMu);
  if ((apparentWeight === 0 && requiredForce > 0) || !Number.isFinite(requiredRatio)
      || requiredRatio - tireMu > tolerance) {
    const kind = combined ? 'combined' : 'lateral';
    const need = Number(requiredForce.toPrecision(6));
    const available = Number((tireMu * apparentWeight).toPrecision(6));
    throw new Error(`Insufficient ${kind} grip: ${need} N required, but μ × normal load provides ${available} N.`);
  }

  const frontX = (relative[0][0] + relative[1][0]) / 2;
  const rearX = (relative[2][0] + relative[3][0]) / 2;
  if (Math.abs(frontX - rearX) < 1e-6) throw new Error('Front and rear axles need different X positions.');
  const share = -rearX / (frontX - rearX);
  const nominal = [share / 2, share / 2, (1 - share) / 2, (1 - share) / 2];
  const geometryDistribution = leastSquares(geometryBalance, 4);
  if (geometryDistribution.rank < 3) throw new Error('The tire contacts do not span a usable area in the XY plane.');
  const correction = geometryDistribution.compute(subtract([1, 0, 0], geometryBalance.map((row) => dot(row, nominal))));
  const shares = nominal.map((value, index) => value + correction[index]);
  const longitudinal = scale(shares, longitudinalDemand);
  const coupled = longitudinalRatio !== 0 || lateralRatio !== 0;
  // Components proportional to Fz contribute to the roll and pitch lever arms.
  const verticalBalance = coupled ? [
    geometryBalance[0],
    relative.map(([, y, z]) => y - lateralRatio * z),
    relative.map(([x, , z]) => longitudinalRatio * z - x),
  ] : geometryBalance;
  const verticalDistribution = coupled ? leastSquares(verticalBalance, 4) : geometryDistribution;
  if (verticalDistribution.rank < 3) {
    throw new Error('The effective tire contact positions cannot resolve roll and pitch balance at this acceleration.');
  }
  const pitchMoment = combined ? 0 : dot(relative.map((point) => point[2]), longitudinal);
  const initial = scale(shares, apparentWeight);
  const target = [apparentWeight, 0, -pitchMoment];
  const increment = verticalDistribution.compute(subtract(target, verticalBalance.map((row) => dot(row, initial))));
  const reactions = initial.map((value, index) => {
    const normal = value + increment[index];
    const fx = combined ? longitudinalRatio * normal : longitudinal[index];
    return [fx, lateralRatio * normal, normal];
  });
  if (reactions.some((reaction) => !reaction.every(Number.isFinite))) throw new Error('Tire loads exceed the numerical range.');
  if (reactions.some((reaction) => reaction[2] < -1e-6)) {
    throw new WheelLiftError(WHEEL_LIFT);
  }
  return [
    { target: model.cg, xyz: [inertial[0], inertial[1], -apparentWeight], end: '' },
    ...contacts.map((target, index) => ({ target, xyz: reactions[index], end: '' })),
  ];
}

function gripSample(model, settings, acceleration) {
  try {
    return { acceleration, loads: exampleLoads(model, GRIP, { ...settings, accelerationMps2: acceleration }) };
  } catch (error) {
    if (!(error instanceof WheelLiftError)) throw error;
    return null;
  }
}

function refineGripBoundary(sample, low, high, lowSample, highSample = null) {
  const inside = Boolean(lowSample);
  let best = lowSample ?? highSample;
  for (let step = 0; step < 24; step += 1) {
    const middle = (low + high) / 2;
    const candidate = sample(middle);
    if (candidate) best = candidate;
    if (Boolean(candidate) === inside) low = middle;
    else high = middle;
  }
  return best;
}

export function fitGripAcceleration(model, settings) {
  const requested = vector(accelerationSetting(settings), 'Acceleration');
  const [ax, ay, az] = requested;
  const sample = factor => gripSample(model, settings, [ax * factor, ay * factor, az]);
  const full = sample(1);
  if (full) return { ...full, reduced: false };
  const center = sample(0);
  if (!center) throw new WheelLiftError('Wheel lift persists at zero horizontal acceleration. Check CG, vertical acceleration and downforce.');
  for (let step = 23; step > 0; step -= 1) {
    const candidate = sample(step / 24);
    if (candidate) return { ...refineGripBoundary(sample, step / 24, (step + 1) / 24, candidate), reduced: true };
  }
  return { ...refineGripBoundary(sample, 0, 1 / 24, center), reduced: true };
}

export function scanGripRegion(model, settings) {
  const az = vector(accelerationSetting(settings), 'Acceleration')[2];
  const limit = gripAccelerationLimit({ ...settings, accelerationMps2: [0, 0, az] });
  const center = gripSample(model, settings, [0, 0, az]);
  const samples = center ? [center] : [];
  let attempted = 1;
  const directions = 120;
  const rings = 24;
  if (limit > 0) {
    for (let direction = 0; direction < directions; direction += 1) {
      const angle = 2 * Math.PI * direction / directions;
      const ax = limit * Math.cos(angle);
      const ay = limit * Math.sin(angle);
      const sample = radius => gripSample(model, settings, [ax * radius, ay * radius, az]);
      let previous = center;
      for (let ring = 1; ring <= rings; ring += 1) {
        const current = sample(ring / rings);
        attempted += 1;
        if (current) samples.push(current);
        if (Boolean(previous) !== Boolean(current)) {
          samples.push(refineGripBoundary(sample, (ring - 1) / rings, ring / rings, previous, current));
          attempted += 1;
        }
        previous = current;
      }
    }
  }
  if (!samples.length) throw new Error('No sampled acceleration satisfies the wheel-contact constraint.');
  const equilibria = solveFrames(model, samples.map(({ loads }) => loads), true);
  const peaks = new Map();
  let feasible = 0;
  for (const [index, equilibrium] of equilibria.entries()) {
    if (!equilibrium) continue;
    feasible += 1;
    for (const [member, force] of Object.entries(equilibrium.forces)) {
      if (!peaks.has(member) || Math.abs(force) > Math.abs(peaks.get(member).force)) {
        peaks.set(member, { member, force, acceleration: samples[index].acceleration });
      }
    }
  }
  if (!feasible) throw new Error('No sampled acceleration could balance the suspension. Check the geometry and constraints.');
  return { peaks: [...peaks.values()].sort((a, b) => Math.abs(b.force) - Math.abs(a.force)), feasible, attempted };
}

function timeError(time, error) {
  return new Error(`t = ${Number(time.toPrecision(6))} s: ${error.message}`);
}

function accelerationFrames(model, times, accelerations, settings) {
  const caseName = settings.caseName === GRIP ? GRIP : ACCEL;
  return accelerations.map((accelerationMps2, index) => {
    try {
      return exampleLoads(model, caseName, { ...settings, accelerationMps2 });
    } catch (error) {
      throw timeError(times[index], error);
    }
  });
}

function solveHistory(model, timeS, loads, accelerationMps2, source) {
  let equilibria;
  try {
    equilibria = solveFrames(model, loads);
  } catch (error) {
    if (error instanceof FrameError) throw timeError(timeS[error.index], error);
    throw error;
  }
  const names = model.rods.map((rod) => rod.name);
  const forceN = Object.fromEntries(names.map((name) => [name, equilibria.map((frame) => frame.forces[name])]));
  const largestNames = [];
  const largestForces = [];
  for (const frame of equilibria) {
    let selected = names[0];
    for (const name of names) {
      if (Math.abs(frame.forces[name]) > Math.abs(frame.forces[selected])) selected = name;
    }
    largestNames.push(selected);
    largestForces.push(frame.forces[selected]);
  }
  return { timeS, loads, equilibria, forceN, accelerationMps2, source, largestLoads: [largestNames, largestForces] };
}

export function accelerationHistory(model, duration, settings = {}) {
  duration = number(duration, 'Rise time');
  if (duration <= 0) throw new Error('Rise time must be greater than 0 seconds.');
  const target = vector(accelerationSetting(settings), 'Acceleration');
  const ratios = Array.from({ length: 101 }, (_, index) => index / 100);
  const timeS = scale(ratios, duration);
  const accelerationMps2 = ratios.map((ratio) => scale(target, ratio));
  const loads = accelerationFrames(model, timeS, accelerationMps2, settings);
  const source = settings.caseName === GRIP ? 'Friction-circle acceleration rise' : 'Acceleration rise';
  return solveHistory(model, timeS, loads, accelerationMps2, source);
}

function readLapCsv(text) {
  const { names, rows } = csvTable(text);
  if (!names.has('time_s')) throw new Error('Lap CSV needs a time_s column.');
  const wheelColumns = CORNERS.flatMap((corner) => ['x', 'y', 'z'].map((axis) => `f${axis}_${corner.toLowerCase()}_n`));
  const direct = wheelColumns.every((name) => names.has(name));
  const accelerationColumns = ['x', 'y', 'z'].map((axis, index) => {
    const options = [[`a${axis}_m_s2`, 1], [`a${axis}_mps2`, 1], [`a${axis}_g`, GRAVITY], [['long_g', 'lat_g', ''][index], GRAVITY]];
    const found = options.find(([name]) => name && names.has(name));
    if (!found && !direct && axis !== 'z') {
      throw new Error('Lap CSV needs Fx/Fy/Fz for all four tires, or longitudinal and lateral acceleration.');
    }
    return found ?? [null, 1];
  });
  const timeS = [];
  const wheelForces = [];
  const accelerationMps2 = [];
  for (const { line, values } of rows) {
    const label = `CSV line ${line}`;
    const time = number(values.time_s, `${label} time_s`);
    if (timeS.length && time <= timeS[timeS.length - 1]) throw new Error(`${label} time_s must be greater than the previous time.`);
    timeS.push(time);
    if (direct) wheelForces.push(wheelColumns.map((name) => number(values[name], `${label} ${name}`)));
    accelerationMps2.push(accelerationColumns.map(([name, factor]) => {
      if (!name || (direct && !values[name].trim())) return direct ? NaN : 0;
      const value = number(values[name], `${label} ${name}`) * factor;
      if (!Number.isFinite(value)) throw new Error(`${label} ${name} exceeds the numerical range after conversion.`);
      return value;
    }));
  }
  if (timeS.length < 2) throw new Error('Lap CSV needs at least two valid time samples.');
  if (!Number.isFinite(timeS[timeS.length - 1] - timeS[0])) throw new Error('Lap CSV time range exceeds the numerical range.');
  return { timeS, wheelForces: direct ? wheelForces : null, accelerationMps2 };
}

export function lapHistory(model, csvText, settings = {}) {
  const { timeS, wheelForces, accelerationMps2 } = readLapCsv(csvText);
  let loads;
  let source;
  if (wheelForces) {
    const contacts = new Map(model.wheels.map((wheel) => [wheel.corner, wheel.contact]));
    if (CORNERS.some((corner) => !contacts.has(corner))) throw new Error('Direct wheel loads need all four wheel positions.');
    loads = wheelForces.map((frame) => CORNERS.map((corner, index) => ({
      target: contacts.get(corner), xyz: frame.slice(3 * index, 3 * index + 3), end: '',
    })));
    source = 'CSV wheel contact loads';
  } else {
    loads = accelerationFrames(model, timeS, accelerationMps2, settings);
    source = settings.caseName === GRIP ? 'CSV friction-circle wheel loads' : 'CSV acceleration-derived wheel loads';
  }
  if (settings.fileName) source = `${settings.fileName} · ${source}`;
  return solveHistory(model, timeS, loads, accelerationMps2, source);
}

export function historyStatistics(history, rod) {
  if (!own(history.forceN, rod)) throw new Error(`The history has no loaded rod named ${rod}.`);
  const times = history.timeS;
  const force = history.forceN[rod];
  const duration = times[times.length - 1] - times[0];
  let peak = 0;
  let mean = 0;
  let meanAbsolute = 0;
  for (let index = 1; index < times.length; index += 1) {
    const left = force[index - 1];
    const right = force[index];
    const weight = (times[index] - times[index - 1]) / duration;
    if (Math.abs(right) > Math.abs(force[peak])) peak = index;
    mean += weight * (0.5 * left + 0.5 * right);
    const a = Math.abs(left);
    const b = Math.abs(right);
    let absolute = 0.5 * a + 0.5 * b;
    if (a > 0 && b > 0 && (left < 0) !== (right < 0)) {
      // Integrate the two triangles separately across the zero crossing.
      const largest = Math.max(a, b);
      const fraction = (a / largest) / (a / largest + b / largest);
      absolute = 0.5 * a * fraction + 0.5 * b * (1 - fraction);
    }
    meanAbsolute += weight * absolute;
  }
  return { peak_n: force[peak], peak_time_s: times[peak], mean_n: mean, mean_absolute_n: meanAbsolute };
}
