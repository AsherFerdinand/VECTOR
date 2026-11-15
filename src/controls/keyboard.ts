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
  manualTakeover: () => void = () => {},
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
    // Always release keys, including when focus moved into the map or a form.
    if (event.type === "keyup" && actions[event.code]) {
      pressed.delete(event.code);
      input = emptyInput();
      for (const code of pressed) input[actions[code]] = true;
      return;
    }
    if (
      event.target instanceof Element &&
      event.target.closest('input, textarea, select, [contenteditable="true"]')
    )
      return;
    if (event.code === "Space" && event.target instanceof HTMLButtonElement)
      return;
    if (actions[event.code]) {
      event.preventDefault();
      manualTakeover();
      pressed.add(event.code);
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
  const focus = (event: FocusEvent) => {
    if (
      event.target instanceof Element &&
      event.target.closest('input, textarea, select, [role="application"]')
    )
      clear();
  };
  document.addEventListener("focusin", focus);
  window.addEventListener("keydown", handle);
  window.addEventListener("keyup", handle);
  window.addEventListener("blur", clear);
  document.addEventListener("visibilitychange", visibility);
  return {
    read: () => input,
    clear,
    dispose: () => {
      document.removeEventListener("focusin", focus);
      window.removeEventListener("keydown", handle);
      window.removeEventListener("keyup", handle);
      window.removeEventListener("blur", clear);
      document.removeEventListener("visibilitychange", visibility);
    },
  };
}
