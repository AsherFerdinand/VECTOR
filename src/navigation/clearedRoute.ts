import type { RoadGraph, RoadPosition, Route, LaneEdge } from "./types";
import type { WorldObject } from "../world/types";
import type { ClearanceResult } from "../simulation/collisionTypes";
import { planRoute } from "./route";
import { validateRouteClearance } from "../simulation/collision";

function edgeRoute(edge: LaneEdge): Route {
  let s = 0;
  const points = edge.points.map((p, i) => {
    if (i)
      s += Math.hypot(p.x - edge.points[i - 1].x, p.y - edge.points[i - 1].y);
    return { ...p, s, curvature: 0 };
  });
  const last = points.at(-1)!,
    prior = points.at(-2)!;
  return {
    points,
    length: s,
    edgeIds: [edge.id],
    start: { ...points[0], edgeId: edge.id, offset: 0, heading: edge.heading },
    destination: {
      ...last,
      edgeId: edge.id,
      offset: edge.length,
      heading: Math.atan2(last.y - prior.y, last.x - prior.x),
    },
  };
}
/** Static map preflight only. Dynamic actors never enter this planner. */
export function planClearedRoute(
  graph: RoadGraph,
  start: RoadPosition,
  destination: RoadPosition,
  staticMapObjects: WorldObject[],
  margin: number,
): {
  route: Route | null;
  clearance: ClearanceResult | null;
  rerouted: boolean;
} {
  const route = planRoute(graph, start, destination);
  if (!route) return { route: null, clearance: null, rerouted: false };
  const clearance = validateRouteClearance(route, staticMapObjects, margin);
  if (clearance.valid) return { route, clearance, rerouted: false };
  if (!clearance.reason.includes("intersects"))
    return { route: null, clearance, rerouted: false };
  const edges = graph.edges.filter(
    (edge) =>
      validateRouteClearance(edgeRoute(edge), staticMapObjects, margin, false)
        .valid,
  );
  const alternative = planRoute({ ...graph, edges }, start, destination);
  if (!alternative) return { route: null, clearance, rerouted: false };
  const checked = validateRouteClearance(alternative, staticMapObjects, margin);
  return {
    route: checked.valid ? alternative : null,
    clearance: checked,
    rerouted: checked.valid,
  };
}
