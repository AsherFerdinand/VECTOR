import { useRef } from "react";
import { useFrame } from "@react-three/fiber";
import type { MeshBasicMaterial } from "three";
import type { Simulation } from "../simulation/clock";
import type { SignalColor } from "../world/trafficTypes";
import { SIGNAL_MAP } from "../world/traffic";
import { town, type Building } from "../world/town";
import {
  trafficLightPlacements,
  treePlacements,
  TRAFFIC_LIGHT_POLE_WIDTH,
  TRAFFIC_LIGHT_POLE_HEIGHT,
  TREE_TRUNK_WIDTH,
  TREE_TRUNK_HEIGHT,
} from "../world/objects";

const SIGNAL_COLORS = ["red", "yellow", "green"];
const ACTIVE_LENSES = ["#ff625a", "#ffcf69", "#86e8b0"];
const DIMMED_LENSES = ["#442f31", "#453d2e", "#263b34"];
function applySignalColor(
  materials: MeshBasicMaterial[],
  color: SignalColor | undefined,
) {
  materials.forEach((material, index) =>
    material.color.set(
      color === SIGNAL_COLORS[index]
        ? ACTIVE_LENSES[index]
        : DIMMED_LENSES[index],
    ),
  );
}

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
  id,
  materials,
}: {
  x: number;
  y: number;
  angle: number;
  id: string;
  materials: Map<string, MeshBasicMaterial[]>;
}) {
  return (
    <group
      position={[x, 0, -y]}
      rotation={[0, angle, 0]}
      userData={{
        signalId: id,
        applySignalColor: (color: SignalColor | undefined) =>
          applySignalColor(materials.get(id) ?? [], color),
      }}
    >
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
          <meshBasicMaterial
            color={color}
            ref={(material) => {
              if (!material) return;
              const lenses = materials.get(id) ?? [];
              lenses[i] = material;
              materials.set(id, lenses);
            }}
          />
        </mesh>
      ))}
    </group>
  );
}

export function Town({ simulation }: { simulation?: Simulation }) {
  const lenses = useRef(new Map<string, MeshBasicMaterial[]>());
  useFrame(() => {
    const signals = simulation?.traffic.signals;
    for (const [id, materials] of lenses.current) {
      const signal = signals?.find((item) => item.id === id);
      applySignalColor(materials, signal?.color);
    }
  });
  const half = town.roadWidth / 2;
  return (
    <group userData={{ trafficSignals: true }}>
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
        </group>
      ))}
      {SIGNAL_MAP.map((signal) => {
        const [a, b] = signal.stopLine;
        const dx = b.x - a.x,
          dy = b.y - a.y;
        const crossing = signal.crossing;
        return (
          <group key={signal.id}>
            <mesh
              position={[(a.x + b.x) / 2, 0.15, -(a.y + b.y) / 2]}
              rotation={[0, Math.atan2(dy, dx), 0]}
            >
              <boxGeometry args={[Math.hypot(dx, dy), 0.015, 0.22]} />
              <meshBasicMaterial color="#e0e5dc" />
            </mesh>
            {Array.from({ length: 10 }, (_, i) => {
              const t = (i + 0.5) / 10;
              const x = crossing[0].x + (crossing[1].x - crossing[0].x) * t;
              const y = crossing[0].y + (crossing[1].y - crossing[0].y) * t;
              return (
                <mesh
                  key={i}
                  position={[x, 0.15, -y]}
                  rotation={[0, signal.heading, 0]}
                >
                  <boxGeometry args={[1.8, 0.015, 0.55]} />
                  <meshBasicMaterial color="#d6ded6" />
                </mesh>
              );
            })}
          </group>
        );
      })}
      {trafficLightPlacements.map((p) => (
        <TrafficLight
          key={p.id}
          id={p.id}
          x={p.x}
          y={p.y}
          angle={
            (SIGNAL_MAP.find((signal) => signal.id === p.id)?.heading ?? 0) -
            Math.PI / 2
          }
          materials={lenses.current}
        />
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
