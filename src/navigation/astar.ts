import type { LaneEdge, RoadGraph } from "./types";

/** Distance-cost A*: Euclidean distance is admissible for sampled road curves. */
export function aStar(
  graph: RoadGraph,
  startNodeId: string,
  goalNodeId: string,
): LaneEdge[] | null {
  if (!graph.nodes[startNodeId] || !graph.nodes[goalNodeId]) return null;
  const goal = graph.nodes[goalNodeId];
  const heuristic = (id: string) =>
    Math.hypot(graph.nodes[id].x - goal.x, graph.nodes[id].y - goal.y);
  const open = new Set([startNodeId]);
  const costs = new Map<string, number>([[startNodeId, 0]]);
  const parent = new Map<string, LaneEdge>();
  while (open.size) {
    let current = "";
    let score = Infinity;
    for (const id of open) {
      const candidate = costs.get(id)! + heuristic(id);
      if (candidate < score) {
        score = candidate;
        current = id;
      }
    }
    if (current === goalNodeId) {
      const result: LaneEdge[] = [];
      while (current !== startNodeId) {
        const edge = parent.get(current)!;
        result.unshift(edge);
        current = edge.from;
      }
      return result;
    }
    open.delete(current);
    for (const edge of graph.edges) {
      if (edge.from !== current) continue;
      const candidate = costs.get(current)! + edge.length;
      if (candidate < (costs.get(edge.to) ?? Infinity) - 1e-10) {
        costs.set(edge.to, candidate);
        parent.set(edge.to, edge);
        open.add(edge.to);
      }
    }
  }
  return null;
}
