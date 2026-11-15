/** Ground-plane coordinates in metres, heading counterclockwise from +x. */
export interface Point {
  x: number;
  y: number;
}
export interface PathPoint extends Point {
  s: number;
  curvature: number;
}
export interface LaneNode extends Point {
  id: string;
}
export interface LaneEdge {
  id: string;
  from: string;
  to: string;
  points: Point[];
  length: number;
  kind: "lane" | "turn";
  roadId?: string;
  heading: number;
}
export interface RoadGraph {
  nodes: Record<string, LaneNode>;
  edges: LaneEdge[];
}
export interface RoadPosition extends Point {
  edgeId: string;
  offset: number;
  heading: number;
}
export interface Route {
  points: PathPoint[];
  length: number;
  edgeIds: string[];
  start: RoadPosition;
  destination: RoadPosition;
}
