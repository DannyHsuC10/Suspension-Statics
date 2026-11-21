const BACKGROUND = "#111b2b";
const TEXT = "#e5edf5";
const MUTED = "#a9bbcd";
const GRID = "#26384b";
const ROD = "#79b9ef";
const SELECTED = "#eced9b";
const MOVING_POINT = "#78dccd";
const AXIS_COLORS = ["#fb9693", "#88dfb9", "#8dbdff"];
const AXES = ["X", "Y", "Z"];
const FONT = "system-ui, sans-serif";
const MONO = "ui-monospace, Consolas, monospace";
const VIEWS = {
  iso: [.64, .43],
  top: [0, Math.PI / 2],
  front: [Math.PI / 2, 0],
  side: [0, 0],
};

const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

function formatNumber(value, precision = 4) {
  return Math.abs(value) < 1e-12 ? "0" : String(Number(value.toPrecision(precision)));
}

function fitCanvas(canvas, context) {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (!width || !height) return null;
  const ratio = Math.max(1, window.devicePixelRatio || 1);
  const pixelsWide = Math.round(width * ratio);
  const pixelsHigh = Math.round(height * ratio);
  if (canvas.width !== pixelsWide) canvas.width = pixelsWide;
  if (canvas.height !== pixelsHigh) canvas.height = pixelsHigh;
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  return { width, height, ratio };
}

function fillBackground(context, width, height) {
  context.fillStyle = BACKGROUND;
  context.fillRect(0, 0, width, height);
}

function line(context, points, color, width = 1, dash = []) {
  context.beginPath();
  for (const [index, [x, y]] of points.entries()) {
    if (index === 0) context.moveTo(x, y);
    else context.lineTo(x, y);
  }
  context.strokeStyle = color;
  context.lineWidth = width;
  context.lineCap = "round";
  context.lineJoin = "round";
  context.setLineDash(dash);
  context.stroke();
  if (dash.length) context.setLineDash([]);
}

function circle(context, x, y, radius, fill, stroke = null, width = 1) {
  context.beginPath();
  context.arc(x, y, radius, 0, 2 * Math.PI);
  context.fillStyle = fill;
  context.fill();
  if (stroke) {
    context.strokeStyle = stroke;
    context.lineWidth = width;
    context.stroke();
  }
}

function label(context, content, x, y, {
  color = TEXT, size = 13, align = "left", baseline = "middle", weight = 400, family = FONT,
} = {}) {
  context.fillStyle = color;
  context.font = `${weight} ${size}px ${family}`;
  context.textAlign = align;
  context.textBaseline = baseline;
  context.fillText(content, x, y);
}

function arrow(context, start, end, color, width = 1.5, head = 6) {
  line(context, [start, end], color, width);
  const angle = Math.atan2(end[1] - start[1], end[0] - start[0]);
  const base = [end[0] - head * Math.cos(angle), end[1] - head * Math.sin(angle)];
  const side = [head * .43 * Math.sin(angle), -head * .43 * Math.cos(angle)];
  context.beginPath();
  context.moveTo(...end);
  context.lineTo(base[0] + side[0], base[1] + side[1]);
  context.lineTo(base[0] - side[0], base[1] - side[1]);
  context.closePath();
  context.fillStyle = color;
  context.fill();
}

function stressColor(fraction) {
  const amounts = [121, 185, 239].map((from, i) => Math.round(from + fraction * ([255, 72, 77][i] - from)));
  return `rgb(${amounts.join(", ")})`;
}

function tyreOutlines(points, wheel) {
  const center = points[wheel.center];
  const contact = points[wheel.contact];
  const radius = center[2] - contact[2] > .03 ? center[2] - contact[2] : .196;
  const forward = wheel.heading ? points[wheel.heading] : null;
  const delta = forward ? [forward[0] - center[0], forward[1] - center[1]] : [1, 0];
  const length = Math.hypot(...delta);
  const heading = length > 1e-9 ? delta.map(value => value / length) : [1, 0];
  const lateral = [-heading[1], heading[0]];
  const rings = [-1, 1].map(side => Array.from({ length: 49 }, (_, index) => {
    const angle = index * Math.PI / 24;
    const run = radius * Math.cos(angle);
    const tread = side * .09525;
    return [
      center[0] + heading[0] * run + lateral[0] * tread,
      center[1] + heading[1] * run + lateral[1] * tread,
      center[2] + radius * Math.sin(angle),
    ];
  }));
  return { center, contact, radius, rings };
}

function sceneBounds(points, tyres) {
  const low = [Infinity, Infinity, Infinity];
  const high = [-Infinity, -Infinity, -Infinity];
  const limits = [...Object.values(points)];
  for (const { center, radius } of tyres) {
    limits.push(center.map(value => value - radius), center.map(value => value + radius));
  }
  for (const point of limits) {
    for (let i = 0; i < 3; i++) {
      low[i] = Math.min(low[i], point[i]);
      high[i] = Math.max(high[i], point[i]);
    }
  }
  const center = low.map((value, i) => .5 * value + .5 * high[i]);
  const span = high.map((value, i) => value - low[i]);
  const ground = tyres.length ? Math.min(...tyres.map(tyre => tyre.contact[2])) : low[2];
  return { center, span, ground, dimension: Math.max(.02, ...span) };
}

function segmentDistance(x, y, a, b) {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const squaredLength = dx * dx + dy * dy;
  const at = squaredLength ? clamp(((x - a[0]) * dx + (y - a[1]) * dy) / squaredLength, 0, 1) : 0;
  return Math.hypot(x - (a[0] + dx * at), y - (a[1] + dy * at));
}

function relativePoint(canvas, event) {
  const bounds = canvas.getBoundingClientRect();
  return [event.clientX - bounds.left, event.clientY - bounds.top];
}

export class Viewport {
  constructor(canvas, onPick) {
    this.canvas = canvas;
    this.context = canvas.getContext("2d", { alpha: false });
    this.onPick = onPick;
    this.model = null;
    this.data = { results: {}, loads: [], selection: null, labels: false, forceComponents: false, maxStress: null };
    this.view = "iso";
    [this.azimuth, this.elevation] = VIEWS.iso;
    this.zoom = 1;
    this.pan = [0, 0];
    this.pointer = null;
    this.pendingDraw = false;
    this.picks = [];
    this.observer = new ResizeObserver(() => this.requestDraw());
    this.observer.observe(canvas);
    canvas.addEventListener("contextmenu", event => event.preventDefault());
    canvas.addEventListener("pointerdown", event => this.startDrag(event));
    canvas.addEventListener("pointermove", event => this.moveDrag(event));
    canvas.addEventListener("pointerup", event => this.endDrag(event));
    canvas.addEventListener("pointercancel", event => this.endDrag(event, true));
    canvas.addEventListener("lostpointercapture", event => this.endDrag(event, true));
    canvas.addEventListener("wheel", event => {
      event.preventDefault();
      const [x, y] = relativePoint(canvas, event);
      const delta = event.deltaY * (event.deltaMode === 1 ? 16 : event.deltaMode === 2 ? canvas.clientHeight : 1);
      this.zoomAt(x, y, Math.exp(clamp(-delta / 420, -.4, .4)));
    }, { passive: false });
    canvas.addEventListener("keydown", event => this.onKey(event));
  }

  setModel(model, reset = true) {
    this.model = model;
    this.tyres = (model?.wheels ?? []).map(wheel => tyreOutlines(model.points, wheel));
    this.bounds = model && Object.keys(model.points).length ? sceneBounds(model.points, this.tyres) : null;
    if (reset) this.setView(this.view);
    else this.requestDraw();
  }

  setView(view) {
    if (!Object.hasOwn(VIEWS, view)) return;
    this.view = view;
    [this.azimuth, this.elevation] = VIEWS[view];
    this.zoom = 1;
    this.pan = [0, 0];
    this.requestDraw();
  }

  setData(data) {
    this.data = { ...this.data, ...data };
    this.requestDraw();
  }

  requestDraw() {
    if (this.pendingDraw) return;
    this.pendingDraw = true;
    requestAnimationFrame(() => {
      this.pendingDraw = false;
      this.draw();
    });
  }

  zoomAt(x, y, factor) {
    const zoom = clamp(this.zoom * factor, .25, 8);
    const delta = 1 - zoom / this.zoom;
    this.pan[0] += (x - this.canvas.clientWidth / 2 - this.pan[0]) * delta;
    this.pan[1] += (y - this.canvas.clientHeight / 2 - this.pan[1]) * delta;
    this.zoom = zoom;
    this.requestDraw();
  }

  onKey(event) {
    const { key } = event;
    if (key === "Home") {
      this.setView(this.view);
    } else if (["+", "=", "-", "_"].includes(key)) {
      const scale = key === "+" || key === "=" ? 1.15 : 1 / 1.15;
      this.zoomAt(this.canvas.clientWidth / 2, this.canvas.clientHeight / 2, scale);
    } else if (["ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown"].includes(key)) {
      const x = Number(key === "ArrowRight") - Number(key === "ArrowLeft");
      const y = Number(key === "ArrowDown") - Number(key === "ArrowUp");
      if (event.shiftKey) {
        this.pan[0] += 12 * x;
        this.pan[1] += 12 * y;
      } else {
        this.azimuth += .09 * x;
        this.elevation = clamp(this.elevation + .09 * y, -Math.PI / 2, Math.PI / 2);
      }
      this.requestDraw();
    } else {
      return;
    }
    event.preventDefault();
  }

  startDrag(event) {
    if (this.pointer || ![0, 1, 2].includes(event.button)) return;
    event.preventDefault();
    this.canvas.focus({ preventScroll: true });
    this.pointer = {
      id: event.pointerId,
      x: event.clientX,
      y: event.clientY,
      pan: event.button !== 0 || event.shiftKey,
      moved: false,
    };
    this.canvas.setPointerCapture(event.pointerId);
  }

  moveDrag(event) {
    const drag = this.pointer;
    if (!drag || drag.id !== event.pointerId) return;
    const dx = event.clientX - drag.x;
    const dy = event.clientY - drag.y;
    if (!drag.moved && Math.abs(dx) + Math.abs(dy) < 4) return;
    if (drag.pan) {
      this.pan[0] += dx;
      this.pan[1] += dy;
    } else {
      this.azimuth += dx * .009;
      this.elevation = clamp(this.elevation + dy * .009, -Math.PI / 2, Math.PI / 2);
    }
    Object.assign(drag, { x: event.clientX, y: event.clientY, moved: true });
    this.requestDraw();
  }

  endDrag(event, cancel = false) {
    const drag = this.pointer;
    if (!drag || drag.id !== event.pointerId) return;
    this.pointer = null;
    if (cancel || drag.moved || drag.pan) return;
    const [x, y] = relativePoint(this.canvas, event);
    let bestPoint = null;
    let bestRod = null;
    for (const pick of this.picks) {
      const distance = pick.kind === "point" ? Math.hypot(x - pick.a[0], y - pick.a[1]) : segmentDistance(x, y, pick.a, pick.b);
      if (pick.kind === "point" && distance <= 7 && (!bestPoint || distance <= bestPoint.distance)) {
        bestPoint = { ...pick, distance };
      } else if (pick.kind === "rod" && distance <= 7 && (!bestRod || distance <= bestRod.distance)) {
        bestRod = { ...pick, distance };
      }
    }
    const selected = bestPoint ?? bestRod;
    if (selected) this.onPick?.(selected.kind, selected.name);
  }

  draw() {
    const context = this.context;
    const size = fitCanvas(this.canvas, context);
    if (!size) return;
    const { width, height } = size;
    fillBackground(context, width, height);
    this.picks = [];
    if (!this.bounds || width < 100 || height < 100) return;

    const { center, span, dimension, ground } = this.bounds;
    const sinA = Math.sin(this.azimuth);
    const cosA = Math.cos(this.azimuth);
    const sinE = Math.sin(this.elevation);
    const cosE = Math.cos(this.elevation);
    const horizontal = [cosA, -sinA, 0];
    const vertical = [sinA * sinE, cosA * sinE, -cosE];
    const depth = [sinA * cosE, cosA * cosE, sinE];
    const extent = [horizontal, vertical].map(axis => Math.max(.07 * dimension, dot(axis.map(Math.abs), span)));
    const sidePadding = this.data.forceComponents ? 106 : 72;
    const scale = this.zoom * Math.min(Math.max(35, width - sidePadding) / extent[0], Math.max(35, height - 105) / extent[1]);
    const offset = [width / 2 + this.pan[0], height / 2 + this.pan[1]];
    const project = point => {
      const shift = point.map((value, i) => value - center[i]);
      return [
        offset[0] + scale * dot(horizontal, shift),
        offset[1] + scale * dot(vertical, shift),
        dot(depth, shift),
      ];
    };

    const half = span.slice(0, 2).map(value => Math.max(value / 2, dimension * .23) * 1.06);
    const [x0, y0] = center.slice(0, 2).map((value, i) => value - half[i]);
    const [x1, y1] = center.slice(0, 2).map((value, i) => value + half[i]);
    for (let i = 0; i <= 10; i++) {
      const x = x0 + (x1 - x0) * i / 10;
      const y = y0 + (y1 - y0) * i / 10;
      line(context, [project([x, y0, ground]), project([x, y1, ground])], GRID);
      line(context, [project([x0, y, ground]), project([x1, y, ground])], GRID);
    }

    for (const tyre of this.tyres) {
      const [left, right] = tyre.rings.map(ring => ring.map(project));
      for (let i = 0; i < left.length - 1; i += 6) {
        const polygon = [...left.slice(i, i + 7), ...right.slice(i, i + 7).reverse()];
        context.beginPath();
        context.moveTo(polygon[0][0], polygon[0][1]);
        for (const [x, y] of polygon.slice(1)) context.lineTo(x, y);
        context.closePath();
        context.fillStyle = "#8aacc00d";
        context.fill();
        line(context, [left[i], right[i]], "#426076");
      }
      line(context, left, "#668298", 1.4);
      line(context, right, "#668298", 1.4);
    }

    const { model } = this;
    const { results = {}, selection, labels, maxStress } = this.data;
    const pixels = Object.fromEntries(Object.entries(model.points).map(([name, point]) => [name, project(point)]));
    for (const [a, b] of model.rigidEdges) line(context, [pixels[a], pixels[b]], "#718a9d", 1, [4, 4]);

    const stresses = Object.values(results);
    const maximum = maxStress ?? stresses.reduce((peak, result) => Math.max(peak, Math.abs(result.stress_mpa)), 0);
    const rods = model.rods.map(({ name, a, b }) => ({ kind: "rod", name, a: pixels[a], b: pixels[b] }))
      .sort((first, second) => (first.a[2] + first.b[2]) - (second.a[2] + second.b[2]));
    for (const rod of rods) {
      const result = Object.hasOwn(results, rod.name) ? results[rod.name] : null;
      const color = result && maximum > 0 ? stressColor(clamp(Math.abs(result.stress_mpa) / maximum, 0, 1)) : ROD;
      if (selection?.kind === "rod" && selection.name === rod.name) line(context, [rod.a, rod.b], SELECTED, 7);
      line(context, [rod.a, rod.b], color, 3);
      this.picks.push(rod);
    }

    const axisVectors = AXES.map((_, i) => [horizontal[i], vertical[i]]);
    if (this.data.forceComponents) this.drawForceComponents(pixels, axisVectors);
    for (const [name, pixel] of Object.entries(pixels).sort((a, b) => a[1][2] - b[1][2])) {
      const [x, y] = pixel;
      const selected = selection?.kind === "point" && selection.name === name;
      const body = Object.hasOwn(model.owners, name) ? model.owners[name] : null;
      const chassis = body === "CHASSIS";
      const color = selected ? SELECTED : chassis ? MUTED : body ? MOVING_POINT : "#8194a7";
      const radius = selected ? 5 : 3;
      if (chassis) {
        context.fillStyle = BACKGROUND;
        context.fillRect(x - radius - 1, y - radius - 1, radius * 2 + 2, radius * 2 + 2);
        context.fillStyle = color;
        context.fillRect(x - radius, y - radius, radius * 2, radius * 2);
      } else {
        circle(context, x, y, radius, color, BACKGROUND);
      }
      if (labels || selected) label(context, name, x + 6, y - 7, { color, size: 11, family: MONO });
      this.picks.push({ kind: "point", name, a: pixel });
    }
    this.drawAxes(axisVectors, height);
    if (stresses.length) this.drawScale(maximum, width, height);
  }

  drawAxes(axisVectors, height) {
    const origin = [37, height - 38];
    axisVectors.forEach(([x, y], i) => {
      if (Math.hypot(x, y) < .1) return;
      const color = AXIS_COLORS[i];
      const tip = [origin[0] + 21 * x, origin[1] + 21 * y];
      arrow(this.context, origin, tip, color);
      label(this.context, AXES[i], tip[0] + 9 * x, tip[1] + 9 * y,
        { color, size: 11, align: "center", weight: 700, family: MONO });
    });
  }

  drawScale(maximum, width, height) {
    const context = this.context;
    const right = width - 19;
    const left = right - Math.min(116, width * .38);
    const y = height - 37;
    const gradient = context.createLinearGradient(left, y, right, y);
    gradient.addColorStop(0, stressColor(0));
    gradient.addColorStop(1, stressColor(1));
    label(context, "|σ|  MPa", left, y - 10, { size: 11, family: MONO });
    context.fillStyle = gradient;
    context.fillRect(left, y, right - left, 5);
    label(context, "0", left, y + 15, { color: MUTED, size: 11, family: MONO });
    label(context, formatNumber(maximum), right, y + 15, { color: MUTED, size: 11, align: "right", family: MONO });
  }

  drawForceComponents(pixels, axisVectors) {
    const forces = new Map();
    for (const load of this.data.loads ?? []) {
      if (load.end || !Object.hasOwn(pixels, load.target)) continue;
      const sum = forces.get(load.target) ?? [0, 0, 0];
      load.xyz.forEach((value, i) => { sum[i] += value; });
      forces.set(load.target, sum);
    }
    for (const [name, xyz] of forces) {
      const origin = pixels[name].slice(0, 2);
      xyz.forEach((value, i) => {
        const [x, y] = axisVectors[i].map(component => component * Math.sign(value));
        const length = Math.hypot(x, y);
        if (Math.abs(value) <= 1e-11 || length < .2) return;
        const tip = [origin[0] + 34 * x, origin[1] + 34 * y];
        const color = AXIS_COLORS[i];
        arrow(this.context, origin, tip, color, 2);
        label(this.context, `${value > 0 ? "+" : "−"}${AXES[i]}`, tip[0] + 9 * x / length, tip[1] + 9 * y / length,
          { color, size: 11, align: "center", weight: 700, family: MONO });
      });
    }
  }
}

const sections = new WeakMap();

export function drawSection(canvas, section) {
  let preview = sections.get(canvas);
  if (!preview) {
    preview = { context: canvas.getContext("2d", { alpha: false }), section };
    preview.observer = new ResizeObserver(() => paintSection(canvas, preview));
    preview.observer.observe(canvas);
    sections.set(canvas, preview);
  }
  preview.section = section;
  paintSection(canvas, preview);
}

function paintSection(canvas, { context, section }) {
  const size = fitCanvas(canvas, context);
  if (!size) return;
  const { width, height } = size;
  fillBackground(context, width, height);
  const outer = section?.outer_mm;
  const inner = section?.inner_mm;
  if (!Number.isFinite(outer) || !Number.isFinite(inner) || inner < 0 || outer <= inner) {
    label(context, "Enter valid diameters", width / 2, height / 2, { color: MUTED, align: "center" });
    return;
  }
  const x = width / 2;
  const y = height * .41;
  const outerRadius = Math.min(width * .30, height * .27);
  const innerRadius = outerRadius * inner / outer;
  const outerColor = "#8dc9fc";
  const innerColor = "#86dfce";
  circle(context, x, y, outerRadius, "#386178", outerColor, 2);
  if (innerRadius > 0) circle(context, x, y, innerRadius, BACKGROUND, innerColor);
  const reach = outerRadius + 9;
  line(context, [[x - reach, y], [x + reach, y]], MUTED, .8, [5, 4]);
  line(context, [[x, y - reach], [x, y + reach]], MUTED, .8, [5, 4]);

  const dimY = y + outerRadius + 10;
  for (const edge of [x - outerRadius, x + outerRadius]) {
    line(context, [[edge, y + 4], [edge, dimY + 3]], MUTED, .8);
  }
  arrow(context, [x, dimY], [x - outerRadius, dimY], outerColor);
  arrow(context, [x, dimY], [x + outerRadius, dimY], outerColor);
  label(context, "Outer diameter Dₒ", x, dimY + 8, { size: 12, align: "center", baseline: "top" });

  if (innerRadius > 17) {
    arrow(context, [x, y], [x - innerRadius, y], innerColor);
    arrow(context, [x, y], [x + innerRadius, y], innerColor);
    label(context, "Dᵢ", x, y - 8, { color: innerColor, size: 13, align: "center" });
  } else if (innerRadius > 0) {
    const edge = innerRadius / Math.SQRT2;
    arrow(context, [width - 22, y - outerRadius - 2], [x + edge, y - edge], innerColor);
    label(context, "Dᵢ", width - 19, y - outerRadius - 6, { color: innerColor, size: 13 });
  } else {
    label(context, "Solid", x, y - outerRadius * .53, { size: 12, align: "center" });
  }
}

function nearestTime(times, target) {
  let low = 0;
  let high = times.length - 1;
  while (low < high) {
    const middle = Math.floor((low + high) / 2);
    if (times[middle] < target) low = middle + 1;
    else high = middle;
  }
  if (low > 0 && target - times[low - 1] < times[low] - target) return low - 1;
  return low;
}

export class HistoryPlot {
  constructor(canvas, onFrame) {
    this.canvas = canvas;
    this.onFrame = onFrame;
    this.context = canvas.getContext("2d", { alpha: false });
    this.layer = document.createElement("canvas");
    this.layerContext = this.layer.getContext("2d", { alpha: false });
    this.history = null;
    this.frame = 0;
    this.member = null;
    this.governing = false;
    this.cache = null;
    this.pointer = null;
    this.observer = new ResizeObserver(() => this.draw());
    this.observer.observe(canvas);
    canvas.addEventListener("pointerdown", event => {
      if (!this.plot || this.pointer !== null || event.button !== 0) return;
      event.preventDefault();
      canvas.focus({ preventScroll: true });
      this.pointer = event.pointerId;
      canvas.setPointerCapture(event.pointerId);
      this.seek(event);
    });
    canvas.addEventListener("pointermove", event => {
      if (this.pointer === event.pointerId) this.seek(event);
    });
    for (const name of ["pointerup", "pointercancel", "lostpointercapture"]) {
      canvas.addEventListener(name, event => {
        if (event.pointerId === this.pointer) this.pointer = null;
      });
    }
    canvas.addEventListener("keydown", event => this.onKey(event));
  }

  setData(history, frame, member, governing) {
    if (history !== this.history || member !== this.member || governing !== this.governing) this.cache = null;
    this.history = history;
    this.frame = history ? clamp(frame, 0, history.timeS.length - 1) : 0;
    this.member = member;
    this.governing = governing;
    this.draw();
  }

  seek(event) {
    if (!this.history || !this.plot) return;
    const [x] = relativePoint(this.canvas, event);
    const ratio = clamp((x - this.plot.left) / (this.plot.right - this.plot.left), 0, 1);
    const times = this.history.timeS;
    this.onFrame?.(nearestTime(times, times[0] + ratio * (times.at(-1) - times[0])));
  }

  onKey(event) {
    if (!this.history) return;
    const last = this.history.timeS.length - 1;
    const { key } = event;
    let frame;
    if (key === "ArrowLeft") frame = Math.max(0, this.frame - 1);
    else if (key === "ArrowRight") frame = Math.min(last, this.frame + 1);
    else if (key === "Home") frame = 0;
    else if (key === "End") frame = last;
    else return;
    event.preventDefault();
    this.onFrame?.(frame);
  }

  draw() {
    const size = fitCanvas(this.canvas, this.context);
    if (!size) return;
    const { width, height, ratio } = size;
    if (!this.cache || this.cache.width !== width || this.cache.height !== height || this.cache.ratio !== ratio) {
      this.layer.width = this.canvas.width;
      this.layer.height = this.canvas.height;
      this.layerContext.setTransform(ratio, 0, 0, ratio, 0, 0);
      this.drawBase(width, height);
      this.cache = size;
    }
    this.context.drawImage(this.layer, 0, 0, width, height);
    if (!this.plot) return;
    const x = this.plot.xAt(this.history.timeS[this.frame]);
    for (const { top, bottom, traces, yAt } of this.plot.panels) {
      line(this.context, [[x, top], [x, bottom]], SELECTED, 1.3);
      for (const { values, color } of traces) {
        const value = values[this.frame];
        if (Number.isFinite(value)) circle(this.context, x, yAt(value), 3.5, color, BACKGROUND, 1);
      }
    }
  }

  drawBase(width, height) {
    const context = this.layerContext;
    this.plot = null;
    fillBackground(context, width, height);
    if (!this.history) {
      if (width > 220) label(context, "Generate a history or import a lap CSV", width / 2, height / 2,
        { color: MUTED, align: "center" });
      return;
    }
    const left = width >= 375 ? 65 : 55;
    const right = width - 24;
    const top = 31;
    const bottom = height - 42;
    const gap = 38;
    const middle = (top + bottom - gap) / 2;
    if (right - left < 60 || middle - top < 24) return;

    const times = this.history.timeS;
    const timeStart = times[0];
    const duration = times.at(-1) - timeStart;
    const xAt = time => left + (time - timeStart) / duration * (right - left);
    const accelerations = AXES.map((_, i) => ({
      values: this.history.accelerationMps2.map(row => row[i]),
      color: AXIS_COLORS[i],
    }));
    const force = this.governing ? this.history.largestLoads[1].map(Math.abs) : this.history.forceN[this.member] ?? [];
    this.plot = { left, right, xAt, panels: [] };

    const { hasData } = this.drawPanel(top, middle, "CG acceleration (m/s²)", accelerations);
    if (width >= 320) {
      AXES.forEach((axis, i) => label(context, `a${axis.toLowerCase()}`, right - 22 * (2 - i), top - 12, {
        color: AXIS_COLORS[i], align: "right", size: 11, family: MONO,
      }));
    }
    if (!hasData) label(context, "No acceleration channels", (left + right) / 2, (top + middle) / 2,
      { color: MUTED, size: 12, align: "center" });
    this.drawPanel(middle + gap, bottom, this.governing ? "Governing |axial force| (N)" : "Member axial force (N)",
      [{ values: force, color: ROD }]);
    for (let i = 0; i <= 4; i++) {
      const x = left + (right - left) * i / 4;
      label(context, formatNumber(timeStart + duration * i / 4), x, bottom + 13,
        { size: 11, family: MONO, align: "center", color: MUTED });
    }
    label(context, "Time (s)", (left + right) / 2, height - 10, { size: 12, align: "center" });
  }

  drawPanel(top, bottom, title, traces) {
    const context = this.layerContext;
    const { left, right, xAt } = this.plot;
    let low = 0;
    let high = 0;
    let hasData = false;
    for (const { values } of traces) {
      for (const value of values) {
        if (!Number.isFinite(value)) continue;
        hasData = true;
        low = Math.min(low, value);
        high = Math.max(high, value);
      }
    }
    const padding = high > low ? Math.max((high - low) * .08, 1e-9) : 1;
    low -= padding;
    high += padding;
    const yAt = value => bottom - (value - low) / (high - low) * (bottom - top);

    label(context, title, left, top - 12, { size: 13, weight: 600 });
    for (let i = 0; i <= 4; i++) {
      const x = left + (right - left) * i / 4;
      line(context, [[x, top], [x, bottom]], GRID);
    }
    const intervals = bottom - top > 90 ? 3 : 2;
    for (let i = 0; i <= intervals; i++) {
      const value = low + (high - low) * i / intervals;
      const y = yAt(value);
      line(context, [[left, y], [right, y]], GRID);
      label(context, formatNumber(value, 3), left - 7, y,
        { color: MUTED, size: 11, family: MONO, align: "right" });
    }
    line(context, [[left, yAt(0)], [right, yAt(0)]], "#526a82", 1, [4, 4]);
    const times = this.history.timeS;
    for (const { values, color } of traces) {
      context.beginPath();
      let run = 0;
      for (let i = 0; i <= values.length; i++) {
        if (!Number.isFinite(values[i])) {
          if (run === 1) {
            const x = xAt(times[i - 1]);
            const y = yAt(values[i - 1]);
            context.moveTo(x - 1, y);
            context.lineTo(x + 1, y);
          }
          run = 0;
          continue;
        }
        const x = xAt(times[i]);
        const y = yAt(values[i]);
        if (run === 0) context.moveTo(x, y);
        else context.lineTo(x, y);
        run++;
      }
      context.strokeStyle = color;
      context.lineWidth = 1.8;
      context.lineJoin = "round";
      context.lineCap = "round";
      context.stroke();
    }
    this.plot.panels.push({ top, bottom, traces, yAt });
    return { hasData };
  }
}
