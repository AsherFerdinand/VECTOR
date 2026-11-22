import type { Route } from "../navigation/types";
import type { VehicleState } from "../simulation/types";
import { vehicleFootprint } from "../simulation/collision";

/** Explicit test-road map, separate from the town's single lane per direction.
 * Two 3.5m lanes in each direction. No connectivity to the town is implied. */
export const CORRIDOR = Object.freeze({
  id: "validation-multilane",
  minX: -60,
  maxX: 180,
  minY: -92,
  maxY: -78,
  dividerY: -85,
  laneWidth: 3.5,
  lanes: [
    { id: "east-outer", y: -90.25, direction: 1 },
    { id: "east-inner", y: -86.75, direction: 1 },
    { id: "west-inner", y: -83.25, direction: -1 },
    { id: "west-outer", y: -79.75, direction: -1 },
  ],
});
export type CorridorLane = "east-outer" | "east-inner";
export function corridorContains(
  state: VehicleState,
  margin = 0,
  eastbound = true,
): boolean {
  return vehicleFootprint(state, margin).every(
    (p) =>
      p.x >= CORRIDOR.minX &&
      p.x <= CORRIDOR.maxX &&
      p.y >= CORRIDOR.minY &&
      p.y <= (eastbound ? CORRIDOR.dividerY : CORRIDOR.maxY),
  );
}
export function corridorLane(y: number): CorridorLane {
  return Math.abs(y + 90.25) <= Math.abs(y + 86.75)
    ? "east-outer"
    : "east-inner";
}
export function laneY(lane: CorridorLane): number {
  return lane === "east-outer" ? -90.25 : -86.75;
}
export function corridorRoute(
  start: VehicleState,
  destinationX = 150,
  lane: CorridorLane = corridorLane(start.y),
): Route {
  const y = laneY(lane),
    length = destinationX - start.x;
  if (length <= 0 || destinationX > CORRIDOR.maxX - 4)
    throw new RangeError(
      "Corridor destination must be forward and leave body clearance",
    );
  const edgeId = `corridor-${lane}`,
    count = Math.ceil(length / 0.5) + 1;
  return {
    points: Array.from({ length: count }, (_, i) => ({
      x: start.x + (length * i) / (count - 1),
      y,
      s: (length * i) / (count - 1),
      curvature: 0,
    })),
    length,
    edgeIds: [edgeId],
    start: {
      x: start.x,
      y,
      heading: 0,
      edgeId,
      offset: start.x - CORRIDOR.minX,
    },
    destination: {
      x: destinationX,
      y,
      heading: 0,
      edgeId,
      offset: destinationX - CORRIDOR.minX,
    },
  };
}
