import { town } from "./town";
import type { Point } from "../navigation/types";
import type { WorldObject } from "./types";

export const TRAFFIC_LIGHT_POLE_WIDTH = 0.13;
export const TRAFFIC_LIGHT_POLE_HEIGHT = 3.8;
export const TREE_TRUNK_WIDTH = 0.4;
export const TREE_TRUNK_HEIGHT = 2.4;

/** Ground-level solid footprints share positions with the visual scene. */
export const trafficLightPlacements = town.intersections.flatMap(
  ([x, y], junction) => [
    { id: `traffic-light-${junction}-a`, x: x - 5.6, y: y - 5.6, angle: 0 },
    {
      id: `traffic-light-${junction}-b`,
      x: x + 5.6,
      y: y + 5.6,
      angle: Math.PI,
    },
  ],
);
export const treePlacements = [-14, -7, 0, 7, 14].flatMap((x) =>
  [-13, 13].map((y) => ({ id: `tree-${x}-${y}`, x, y })),
);

function rectangle(
  x: number,
  y: number,
  width: number,
  depth: number,
): Point[] {
  return [
    { x: x - width / 2, y: y - depth / 2 },
    { x: x + width / 2, y: y - depth / 2 },
    { x: x + width / 2, y: y + depth / 2 },
    { x: x - width / 2, y: y + depth / 2 },
  ];
}
/** Roof overhangs, light lenses, foliage, markings and sidewalks are not ground solids. */
export function worldObjects(): WorldObject[] {
  const object = (
    id: string,
    objectClass: WorldObject["objectClass"],
    polygon: Point[],
    height: number,
  ): WorldObject => ({
    id,
    objectClass,
    polygon,
    height,
    velocity: { x: 0, y: 0 },
    collidable: true,
  });
  return [
    ...town.buildings.map((b, index) =>
      object(
        `building-${index}`,
        "building",
        rectangle(b.x, b.y, b.width, b.depth),
        b.height,
      ),
    ),
    ...trafficLightPlacements.map((p) =>
      object(
        p.id,
        "traffic-light",
        rectangle(p.x, p.y, TRAFFIC_LIGHT_POLE_WIDTH, TRAFFIC_LIGHT_POLE_WIDTH),
        TRAFFIC_LIGHT_POLE_HEIGHT,
      ),
    ),
    ...treePlacements.map((p) =>
      object(
        p.id,
        "static-obstacle",
        rectangle(p.x, p.y, TREE_TRUNK_WIDTH, TREE_TRUNK_WIDTH),
        TREE_TRUNK_HEIGHT,
      ),
    ),
  ];
}
