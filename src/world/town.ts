import data from "./town.json";

export interface Road {
  id: string;
  from: number[];
  to: number[];
}
export interface Building {
  x: number;
  y: number;
  width: number;
  depth: number;
  height: number;
  color: string;
}
/** The single map source used for road meshes and potential future navigation data. */
export const town = data;
