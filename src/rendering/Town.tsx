import { town, type Building } from "../world/town";
import {
  trafficLightPlacements,
  treePlacements,
  TRAFFIC_LIGHT_POLE_WIDTH,
  TRAFFIC_LIGHT_POLE_HEIGHT,
  TREE_TRUNK_WIDTH,
  TREE_TRUNK_HEIGHT,
} from "../world/objects";

function Box({
  position,
  size,
  color,
}: {
  position: [number, number, number];
  size: [number, number, number];
  color: string;
}) {
  return (
    <mesh position={position}>
      <boxGeometry args={size} />
      <meshLambertMaterial color={color} />
    </mesh>
  );
}

function BuildingMesh({ building: b }: { building: Building }) {
  return (
    <group position={[b.x, 0, -b.y]}>
      <Box
        position={[0, b.height / 2, 0]}
        size={[b.width, b.height, b.depth]}
        color={b.color}
      />
      <Box
        position={[0, b.height + 0.15, 0]}
        size={[b.width + 0.3, 0.3, b.depth + 0.3]}
        color="#53616a"
      />
      <Box
        position={[0, b.height + 1.1, 0]}
        size={[b.width * 0.3, 1.7, b.depth * 0.25]}
        color="#68767a"
      />
      {[2.4, 5.5, 8.6, 11.7]
        .filter((h) => h < b.height - 1)
        .map((h) => (
          <group key={h}>
            <Box
              position={[0, h, b.depth / 2 + 0.015]}
              size={[b.width * 0.72, 1.1, 0.03]}
              color="#48616c"
            />
            <Box
              position={[b.width / 2 + 0.015, h, 0]}
              size={[0.03, 1.1, b.depth * 0.72]}
              color="#48616c"
            />
          </group>
        ))}
    </group>
  );
}

function TrafficLight({
  x,
  y,
  angle,
}: {
  x: number;
  y: number;
  angle: number;
}) {
  return (
    <group position={[x, 0, -y]} rotation={[0, angle, 0]}>
      <Box
        position={[0, TRAFFIC_LIGHT_POLE_HEIGHT / 2, 0]}
        size={[
          TRAFFIC_LIGHT_POLE_WIDTH,
          TRAFFIC_LIGHT_POLE_HEIGHT,
          TRAFFIC_LIGHT_POLE_WIDTH,
        ]}
        color="#38494d"
      />
      <Box position={[0, 3.5, 0]} size={[0.45, 1.05, 0.35]} color="#17232a" />
      {["#de6258", "#b99a52", "#4b786b"].map((color, i) => (
        <mesh
          key={color}
          position={[0, 3.8 - i * 0.3, 0.19]}
          rotation={[Math.PI / 2, 0, 0]}
        >
          <cylinderGeometry args={[0.1, 0.1, 0.035, 8]} />
          <meshBasicMaterial color={color} />
        </mesh>
      ))}
    </group>
  );
}

export function Town() {
  const half = town.roadWidth / 2;
  return (
    <group>
      <Box position={[0, -0.22, 0]} size={[500, 0.4, 500]} color="#5a7467" />
      <Box position={[0, 0.015, 0]} size={[41, 0.08, 41]} color="#829381" />
      {town.roads.map((road) => {
        const horizontal = road.from[1] === road.to[1];
        const cx = (road.from[0] + road.to[0]) / 2,
          cy = (road.from[1] + road.to[1]) / 2;
        const length = Math.hypot(
          road.to[0] - road.from[0],
          road.to[1] - road.from[1],
        );
        return (
          <group
            key={road.id}
            position={[cx, 0, -cy]}
            rotation={[0, horizontal ? 0 : Math.PI / 2, 0]}
          >
            <Box
              position={[0, 0.025, 0]}
              size={[length, 0.1, town.roadWidth + town.sidewalkWidth * 2]}
              color="#a5afa9"
            />
            <Box
              position={[0, 0.09, 0]}
              size={[length, 0.045, town.roadWidth]}
              color="#35434a"
            />
            {[-half + 0.3, half - 0.3].map((z) => (
              <Box
                key={z}
                position={[0, 0.12, z]}
                size={[length, 0.015, 0.1]}
                color="#c3cfca"
              />
            ))}
            {Array.from({ length: 27 }, (_, i) => i * 4 - 52)
              .filter((x) => Math.abs(Math.abs(x) - 27) > 6)
              .map((x) => (
                <Box
                  key={x}
                  position={[x, 0.125, 0]}
                  size={[2, 0.015, 0.13]}
                  color="#dfd4aa"
                />
              ))}
          </group>
        );
      })}
      {town.intersections.map(([x, y]) => (
        <group key={`${x},${y}`} position={[x, 0, -y]}>
          <Box position={[0, 0.11, 0]} size={[9, 0.04, 9]} color="#35434a" />
          {[-1, 1].flatMap((side) =>
            Array.from({ length: 6 }, (_, i) => {
              const offset = -3.1 + i * 1.25;
              return (
                <group key={`${side}-${i}`}>
                  <Box
                    position={[offset, 0.14, side * 6]}
                    size={[0.65, 0.015, 2]}
                    color="#d6ded6"
                  />
                  <Box
                    position={[side * 6, 0.14, offset]}
                    size={[2, 0.015, 0.65]}
                    color="#d6ded6"
                  />
                </group>
              );
            }),
          )}
        </group>
      ))}
      {trafficLightPlacements.map((p) => (
        <TrafficLight key={p.id} x={p.x} y={p.y} angle={p.angle} />
      ))}
      {town.buildings.map((building, i) => (
        <BuildingMesh key={i} building={building} />
      ))}
      {treePlacements.map(({ id, x, y }) => (
        <group key={id} position={[x, 0, -y]}>
          <Box
            position={[0, TREE_TRUNK_HEIGHT / 2, 0]}
            size={[TREE_TRUNK_WIDTH, TREE_TRUNK_HEIGHT, TREE_TRUNK_WIDTH]}
            color="#766e58"
          />
          <mesh position={[0, 3.1, 0]}>
            <icosahedronGeometry args={[2.1, 0]} />
            <meshLambertMaterial color="#426a59" />
          </mesh>
        </group>
      ))}
      <Box position={[0, 0.1, 0]} size={[23, 0.1, 5]} color="#a5afa0" />
      <Box position={[0, 0.11, 0]} size={[5, 0.1, 23]} color="#a5afa0" />
    </group>
  );
}
