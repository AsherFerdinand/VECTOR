/** SI units; rear-axle reference, heading counterclockwise from world +x. */
export interface VehicleState {
  x: number;
  y: number;
  heading: number;
  velocity: number;
}
export interface VehicleControl {
  steering: number;
  acceleration: number;
}
export interface DriverInput {
  forward: boolean;
  backward: boolean;
  left: boolean;
  right: boolean;
  brake: boolean;
}
export interface SimulationSnapshot {
  vehicle: VehicleState;
  control: VehicleControl;
  time: number;
  running: boolean;
  distance: number;
  droppedTime: number;
}
export type CameraMode = "overview" | "follow" | "cockpit";
