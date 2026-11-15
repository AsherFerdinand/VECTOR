import { aStar } from "./astar";
import type {
  LaneEdge,
  PathPoint,
  Point,
  RoadGraph,
  RoadPosition,
  Route,
} from "./types";

const distance = (a: Point, b: Point) => Math.hypot(a.x - b.x, a.y - b.y);
function pointAt(edge: LaneEdge, offset: number): Point {
  let traversed = 0;
  for (let i = 1; i < edge.points.length; i++) {
    const a = edge.points[i - 1],
      b = edge.points[i],
      length = distance(a, b);
    if (traversed + length >= offset - 1e-8) {
      const t = Math.max(0, Math.min(1, (offset - traversed) / length));
      return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
    }
    traversed += length;
  }
  return { ...edge.points[edge.points.length - 1] };
}
function validPosition(
  graph: RoadGraph,
  position: RoadPosition,
): LaneEdge | null {
  if (
    !position ||
    ![position.x, position.y, position.offset, position.heading].every(
      Number.isFinite,
    )
  )
    return null;
  const edge = graph.edges.find(
    (e) => e.id === position.edgeId && e.kind === "lane",
  );
  if (!edge || position.offset < 0 || position.offset > edge.length + 1e-7)
    return null;
  if (
    distance(position, pointAt(edge, position.offset)) > 1e-5 ||
    Math.abs(
      Math.atan2(
        Math.sin(position.heading - edge.heading),
        Math.cos(position.heading - edge.heading),
      ),
    ) > 1e-5
  )
    return null;
  return edge;
}
/** Checks position metadata against its directed lane and sampled geometry. */
export function isRoadPosition(
  graph: RoadGraph,
  position: RoadPosition,
): boolean {
  return validPosition(graph, position) !== null;
}
function slice(edge: LaneEdge, from: number, to: number): Point[] {
  const output = [pointAt(edge, from)];
  let s = 0;
  for (let i = 1; i < edge.points.length; i++) {
    s += distance(edge.points[i - 1], edge.points[i]);
    if (s > from + 1e-8 && s < to - 1e-8) output.push({ ...edge.points[i] });
  }
  output.push(pointAt(edge, to));
  return output;
}
/** Partial start/destination lanes join a directed A* route; no teleporting or reverse edges. */
export function planRoute(
  graph: RoadGraph,
  start: RoadPosition,
  destination: RoadPosition,
): Route | null {
  const first = validPosition(graph, start),
    last = validPosition(graph, destination);
  if (!first || !last) return null;
  let edges: LaneEdge[], raw: Point[];
  if (first.id === last.id && destination.offset >= start.offset) {
    edges = [first];
    raw = slice(first, start.offset, destination.offset);
  } else {
    const middle = aStar(graph, first.to, last.from);
    if (!middle) return null;
    edges = [first, ...middle, last];
    raw = [
      ...slice(first, start.offset, first.length),
      ...middle.flatMap((e) => e.points),
      ...slice(last, 0, destination.offset),
    ];
  }
  const points: PathPoint[] = [];
  for (const p of raw) {
    const previous = points[points.length - 1];
    if (previous && distance(previous, p) < 1e-7) continue;
    points.push({
      ...p,
      s: previous ? previous.s + distance(previous, p) : 0,
      curvature: 0,
    });
  }
  for (let i = 1; i < points.length - 1; i++) {
    const a = points[i - 1],
      b = points[i],
      c = points[i + 1];
    const ab = distance(a, b),
      bc = distance(b, c),
      ac = distance(a, c);
    // Signed circumcircle curvature; zero on straight sections.
    points[i].curvature =
      ab * bc * ac > 1e-10
        ? (2 * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))) /
          (ab * bc * ac)
        : 0;
  }
  if (points.length > 1) {
    points[0].curvature = points[1].curvature;
    points[points.length - 1].curvature = points[points.length - 2].curvature;
  }
  if ((points[points.length - 1]?.s ?? 0) < 1) return null;
  return {
    points,
    length: points[points.length - 1]?.s ?? 0,
    edgeIds: edges.map((e) => e.id),
    start: { ...start },
    destination: { ...destination },
  };
}
