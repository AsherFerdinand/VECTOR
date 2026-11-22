import { useRef } from "react";
import { useFrame } from "@react-three/fiber";
import type { Group } from "three";
import type { Simulation } from "../simulation/clock";
import { VEHICLE } from "../simulation/vehicle";
import type { TrafficActor } from "../world/trafficTypes";
import type { WorldObject } from "../world/types";

/** Actor motion comes solely from TrafficWorld; these meshes never integrate physics. */
function Actor({
  actor,
  simulation,
}: {
  actor: TrafficActor;
  simulation: Simulation;
}) {
  const group = useRef<Group>(null);
  useFrame(() => {
    const current = simulation.traffic.actors.find(
      (item) => item.id === actor.id,
    );
    if (!group.current) return;
    group.current.visible = current !== undefined;
    if (!current) return;
    group.current.position.set(current.state.x, 0, -current.state.y);
    group.current.rotation.y = current.state.heading;
  });
  if (actor.kind === "pedestrian")
    return (
      <group ref={group} userData={{ trafficActorId: actor.id }}>
        <mesh position={[0, 0.92, 0]}>
          <cylinderGeometry args={[0.19, 0.22, 0.85, 8]} />
          <meshLambertMaterial color="#e0a179" />
        </mesh>
        <mesh position={[0, 1.58, 0]}>
          <sphereGeometry args={[0.2, 8, 6]} />
          <meshLambertMaterial color="#e4c7af" />
        </mesh>
        {[-1, 1].map((side) => (
          <mesh key={side} position={[0, 0.28, side * 0.12]}>
            <boxGeometry args={[0.16, 0.56, 0.15]} />
            <meshLambertMaterial color="#354d64" />
          </mesh>
        ))}
      </group>
    );
  const centre = VEHICLE.wheelbase / 2;
  const color = actor.id.includes("lead") ? "#8caec5" : "#e7b0a1";
  return (
    <group ref={group} userData={{ trafficActorId: actor.id }}>
      <mesh position={[centre, 0.65, 0]}>
        <boxGeometry args={[actor.length, 0.65, actor.width]} />
        <meshLambertMaterial color={color} />
      </mesh>
      <mesh position={[centre - 0.15, 1.21, 0]}>
        <boxGeometry args={[actor.length * 0.52, 0.62, actor.width * 0.85]} />
        <meshLambertMaterial color="#354c59" />
      </mesh>
      <mesh position={[centre - 0.15, 1.55, 0]}>
        <boxGeometry args={[actor.length * 0.55, 0.1, actor.width * 0.89]} />
        <meshLambertMaterial color={color} />
      </mesh>
      {[0, VEHICLE.wheelbase].flatMap((x) =>
        [-1, 1].map((side) => (
          <mesh
            key={`${x}-${side}`}
            position={[x, 0.34, side * (actor.width / 2 - 0.1)]}
            rotation={[Math.PI / 2, 0, 0]}
          >
            <cylinderGeometry args={[0.33, 0.33, 0.2, 10]} />
            <meshLambertMaterial color="#1d2932" />
          </mesh>
        )),
      )}
      {[-1, 1].flatMap((side) =>
        [false, true].map((front) => (
          <mesh
            key={`${side}-${front}`}
            position={[
              centre + ((front ? 1 : -1) * actor.length) / 2,
              0.76,
              side * actor.width * 0.34,
            ]}
          >
            <boxGeometry args={[0.025, 0.16, 0.31]} />
            <meshBasicMaterial color={front ? "#f0edd4" : "#df716b"} />
          </mesh>
        )),
      )}
    </group>
  );
}

function StaticObstacle({ object }: { object: WorldObject }) {
  const center = object.polygon.reduce(
    (sum, point) => ({
      x: sum.x + point.x / object.polygon.length,
      y: sum.y + point.y / object.polygon.length,
    }),
    { x: 0, y: 0 },
  );
  const a = object.polygon[0],
    b = object.polygon[1],
    d = object.polygon[object.polygon.length - 1];
  const length = Math.hypot(b.x - a.x, b.y - a.y),
    width = Math.hypot(d.x - a.x, d.y - a.y);
  return (
    <group
      position={[center.x, 0, -center.y]}
      rotation={[0, Math.atan2(b.y - a.y, b.x - a.x), 0]}
    >
      <mesh position={[0, object.height / 2, 0]}>
        <boxGeometry args={[length, object.height, width]} />
        <meshLambertMaterial color="#c68861" />
      </mesh>
      <mesh position={[0, object.height * 0.65, width / 2 + 0.01]}>
        <boxGeometry args={[length * 0.8, 0.16, 0.025]} />
        <meshBasicMaterial color="#f0d0a2" />
      </mesh>
    </group>
  );
}

export function Traffic({ simulation }: { simulation: Simulation }) {
  // The application refreshes the actor roster at its bounded UI rate; actor
  // transforms are read directly every render frame, independently of React.
  const traffic = simulation.traffic;
  return (
    <group userData={{ trafficScene: true }}>
      {traffic.actors.map((actor) => (
        <Actor key={actor.id} actor={actor} simulation={simulation} />
      ))}
      {traffic.staticObstacles.map((object) => (
        <StaticObstacle key={object.id} object={object} />
      ))}
    </group>
  );
}
