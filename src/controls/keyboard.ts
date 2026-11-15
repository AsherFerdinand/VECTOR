import type { CameraMode, DriverInput } from "../simulation/types";
const emptyInput = (): DriverInput => ({
  forward: false,
  backward: false,
  left: false,
  right: false,
  brake: false,
});

/** Input is sampled by the fixed-step controller; browser events never move meshes. */
export function keyboardControls(
  reset: () => void,
  camera: (mode: CameraMode) => void,
) {
  let input = emptyInput();
  const actions: Record<string, keyof DriverInput> = {
    KeyW: "forward",
    ArrowUp: "forward",
    KeyS: "backward",
    ArrowDown: "backward",
    KeyA: "left",
    ArrowLeft: "left",
    KeyD: "right",
    ArrowRight: "right",
    Space: "brake",
  };
  const pressed = new Set<string>();
  const clear = () => {
    pressed.clear();
    input = emptyInput();
  };
  const handle = (event: KeyboardEvent) => {
    if (
      event.target instanceof HTMLElement &&
      event.target.closest('input, textarea, select, [contenteditable="true"]')
    )
      return;
    if (event.code === "Space" && event.target instanceof HTMLButtonElement)
      return;
    if (actions[event.code]) {
      event.preventDefault();
      if (event.type === "keydown") pressed.add(event.code);
      else pressed.delete(event.code);
      input = emptyInput();
      for (const code of pressed) input[actions[code]] = true;
    }
    if (event.type === "keydown" && !event.repeat) {
      if (event.code === "KeyR") {
        clear();
        reset();
      }
      const modes: Record<string, CameraMode> = {
        Digit1: "overview",
        Digit2: "follow",
        Digit3: "cockpit",
      };
      if (modes[event.code]) camera(modes[event.code]);
    }
  };
  const visibility = () => {
    if (document.hidden) clear();
  };
  window.addEventListener("keydown", handle);
  window.addEventListener("keyup", handle);
  window.addEventListener("blur", clear);
  document.addEventListener("visibilitychange", visibility);
  return {
    read: () => input,
    clear,
    dispose: () => {
      window.removeEventListener("keydown", handle);
      window.removeEventListener("keyup", handle);
      window.removeEventListener("blur", clear);
      document.removeEventListener("visibilitychange", visibility);
    },
  };
}
