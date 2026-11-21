// Contact between the circular envelopes of rods on the original hardpoints.
const EPSILON = 1e-9;
const MM_PER_M = 1000;

const difference = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];

function compareNames(a, b) {
  if (a < b) return -1;
  return a > b ? 1 : 0;
}

function referenceSegment(rod, points, sections) {
  const outerMm = sections[rod.name]?.outer_mm;
  const a = points[rod.a];
  const b = points[rod.b];
  if (!a || !b || !Number.isFinite(outerMm) || outerMm <= 0) return null;
  const axis = difference(b, a);
  const length = Math.hypot(...axis);
  if (!Number.isFinite(length) || length <= EPSILON) return null;
  return {
    name: rod.name,
    a,
    b,
    length,
    axis: axis.map(value => value / length),
    radius: outerMm / (2 * MM_PER_M),
  };
}

function coincides(a, b) {
  return Math.hypot(...difference(a, b)) <= EPSILON;
}

function trim(segment, start, offset) {
  const removed = start ? segment.a : segment.b;
  const direction = start ? offset : -offset;
  const endpoint = removed.map((value, axis) => value + segment.axis[axis] * direction);
  return {
    ...segment,
    a: start ? endpoint : segment.a,
    b: start ? segment.b : endpoint,
    length: segment.length - offset,
  };
}

function endpointDistance(point, segment) {
  const offset = difference(point, segment.a);
  const along = Math.min(segment.length, Math.max(0, dot(offset, segment.axis)));
  return Math.hypot(...offset.map((value, i) => value - along * segment.axis[i]));
}

function segmentDistance(first, second) {
  // One of the endpoints is closest unless the two closest points are interior.
  let distance = Math.min(
    endpointDistance(first.a, second),
    endpointDistance(first.b, second),
    endpointDistance(second.a, first),
    endpointDistance(second.b, first),
  );
  const normal = cross(first.axis, second.axis);
  const normalLength = Math.hypot(...normal);
  if (normalLength > 0) {
    const unitNormal = normal.map(value => value / normalLength);
    const offset = difference(second.a, first.a);
    const firstAlong = dot(cross(offset, second.axis), unitNormal) / normalLength;
    const secondAlong = dot(cross(offset, first.axis), unitNormal) / normalLength;
    if (firstAlong >= 0 && firstAlong <= first.length && secondAlong >= 0 && secondAlong <= second.length) {
      distance = Math.min(distance, Math.abs(dot(offset, unitNormal)));
    }
  }
  return distance;
}

export function findInterferences(model, sections) {
  const rods = model.rods.map(rod => referenceSegment(rod, model.points, sections)).filter(Boolean);
  const interferences = [];
  for (let i = 0; i < rods.length; i++) {
    for (let j = i + 1; j < rods.length; j++) {
      let first = rods[i];
      let second = rods[j];
      const required = first.radius + second.radius;
      const aa = coincides(first.a, second.a);
      const ab = coincides(first.a, second.b);
      const ba = coincides(first.b, second.a);
      const bb = coincides(first.b, second.b);
      const duplicate = (aa && bb) || (ab && ba);
      if (!duplicate && (aa || ab || ba || bb)) {
        // Ignore intended contact immediately around a shared joint.
        const offset = 2 * required;
        if (first.length <= offset || second.length <= offset) continue;
        first = trim(first, aa || ab, offset);
        second = trim(second, aa || ba, offset);
      }
      const distance = segmentDistance(first, second);
      if (distance < required - EPSILON) {
        const [rodA, rodB] = [first.name, second.name].sort(compareNames);
        interferences.push({
          rodA,
          rodB,
          distanceMm: distance * MM_PER_M,
          requiredMm: required * MM_PER_M,
          overlapMm: (required - distance) * MM_PER_M,
        });
      }
    }
  }
  return interferences.sort((a, b) => b.overlapMm - a.overlapMm
    || compareNames(a.rodA, b.rodA) || compareNames(a.rodB, b.rodB));
}
