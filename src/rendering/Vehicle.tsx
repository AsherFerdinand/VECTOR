import { useRef } from "react";
import { useFrame } from "@react-three/fiber";
import type { Group } from "three";
import type { Simulation } from "../simulation/clock";

function Wheel({
  front,
  side,
  simulation,
}: {
  front: boolean;
  side: number;
  simulation: Simulation;
}) {
  const steering = useRef<Group>(null),
    rolling = useRef<Group>(null);
  useFrame(() => {
    const s = simulation.snapshot;
    if (steering.current)
      steering.current.rotation.y = front ? s.control.steering : 0;
    if (rolling.current) rolling.current.rotation.z = -s.distance / 0.34;
  });
  return (
    <group position={[front ? 2.7 : 0, 0.36, side * 0.86]} ref={steering}>
      <group ref={rolling}>
        <mesh rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.34, 0.34, 0.25, 12]} />
          <meshLambertMaterial color="#18232b" />
        </mesh>
        <mesh position={[0, 0, side * 0.14]} rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.19, 0.19, 0.025, 8]} />
          <meshLambertMaterial color="#b4c9ce" />
        </mesh>
        <mesh position={[0, 0, side * 0.16]}>
          <boxGeometry args={[0.3, 0.055, 0.03]} />
          <meshLambertMaterial color="#657d87" />
        </mesh>
      </group>
    </group>
  );
}

/** Mesh origin is rear axle, forward +x; simulation y maps to Three.js -z. */
export function Vehicle({ simulation }: { simulation: Simulation }) {
  const vehicle = useRef<Group>(null);
  useFrame(() => {
    const s = simulation.snapshot.vehicle;
    if (vehicle.current) {
      vehicle.current.position.set(s.x, 0, -s.y);
      vehicle.current.rotation.y = s.heading;
    }
  });
  return (
    <group ref={vehicle}>
      <mesh position={[1.35, 0.66, 0]}>
        <boxGeometry args={[4.5, 0.55, 1.8]} />
        <meshLambertMaterial color="#e0e9e5" />
      </mesh>
      <mesh position={[1.35, 0.95, 0]}>
        <boxGeometry args={[3.9, 0.25, 1.75]} />
        <meshLambertMaterial color="#f0f4ee" />
      </mesh>
      <mesh position={[1.1, 1.34, 0]}>
        <boxGeometry args={[2.2, 0.65, 1.5]} />
        <meshLambertMaterial color="#284553" />
      </mesh>
      <mesh position={[1.1, 1.69, 0]}>
        <boxGeometry args={[2.3, 0.09, 1.58]} />
        <meshLambertMaterial color="#e4ede7" />
      </mesh>
      {[-1, 1].map((side) => (
        <group key={side}>
          <Wheel front={false} side={side} simulation={simulation} />
          <Wheel front side={side} simulation={simulation} />
          <mesh position={[3.61, 0.79, side * 0.62]}>
            <boxGeometry args={[0.025, 0.17, 0.36]} />
            <meshBasicMaterial color="#e9f6d5" />
          </mesh>
          <mesh position={[-0.91, 0.79, side * 0.62]}>
            <boxGeometry args={[0.025, 0.17, 0.38]} />
            <meshBasicMaterial color="#e66e64" />
          </mesh>
          <mesh position={[1.8, 1.2, side * 0.99]}>
            <boxGeometry args={[0.35, 0.16, 0.25]} />
            <meshLambertMaterial color="#e0e9e5" />
          </mesh>
        </group>
      ))}
    </group>
  );
}
