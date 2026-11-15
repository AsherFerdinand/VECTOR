import { useRef, useState, type KeyboardEvent, type MouseEvent } from "react";
import { town } from "../world/town";
import { projectToRoad } from "../navigation/graph";
import type { Point, RoadGraph, RoadPosition } from "../navigation/types";
import type { NavigationSnapshot } from "../autonomy/types";
import type { VehicleState } from "../simulation/types";

interface MinimapProps {
  graph: RoadGraph;
  navigation: NavigationSnapshot;
  vehicle: VehicleState;
  canSelectStart: boolean;
  onStart: (position: RoadPosition) => boolean;
  onDestination: (position: RoadPosition) => boolean;
  onActivate: () => void;
  onCancel: () => void;
  onClear: () => void;
}

/** An offline map in metres: world +y is SVG -y. No independent road layout. */
export function Minimap({
  graph,
  navigation,
  vehicle,
  canSelectStart,
  onStart,
  onDestination,
  onActivate,
  onCancel,
  onClear,
}: MinimapProps) {
  const [open, setOpen] = useState(true);
  const [selection, setSelection] = useState<"start" | "destination">(
    "destination",
  );
  const [feedback, setFeedback] = useState(
    "Select a destination on a road to preview its route.",
  );
  const [selectedDestination, setSelectedDestination] = useState<Point | null>(
    null,
  );
  const [cursor, setCursor] = useState<Point>({ x: 0, y: -29.25 });
  const svg = useRef<SVGSVGElement>(null);
  const route = navigation.route;
  const start = route?.start ?? vehicle;
  const destination = route?.destination ?? selectedDestination;
  const active = navigation.mode === "AUTONOMOUS";

  const choose = (point: Point) => {
    if (selection === "start" && !canSelectStart) {
      setFeedback(
        "Pause and stop the vehicle before selecting its starting pose.",
      );
      return;
    }
    const position = projectToRoad(graph, point, town.roadWidth / 2);
    if (!position) {
      setFeedback(
        "Select a lane away from intersections. Buildings, grass, and intersection turns cannot be selected.",
      );
      return;
    }
    setCursor(position);
    if (selection === "start") {
      if (!onStart(position)) {
        setFeedback(
          "Start could not be selected. See the navigation status below.",
        );
        return;
      }
      setSelection("destination");
      setFeedback(
        "Start aligned to the selected lane. Now select a destination.",
      );
    } else {
      if (!onDestination(position)) {
        setFeedback(
          "Destination was not accepted. See the navigation status below.",
        );
        return;
      }
      setSelectedDestination(position);
      setFeedback(
        "Destination selected. Review the route before starting autonomous driving.",
      );
    }
  };

  const mapClick = (event: MouseEvent<SVGSVGElement>) => {
    const matrix = svg.current?.getScreenCTM();
    if (!matrix) return;
    const point = new DOMPoint(event.clientX, event.clientY).matrixTransform(
      matrix.inverse(),
    );
    choose({ x: point.x, y: -point.y });
  };
  const mapKey = (event: KeyboardEvent<SVGSVGElement>) => {
    const movement: Record<string, [number, number]> = {
      ArrowLeft: [-1, 0],
      ArrowRight: [1, 0],
      ArrowUp: [0, 1],
      ArrowDown: [0, -1],
    };
    if (movement[event.key]) {
      event.preventDefault();
      event.stopPropagation();
      const [dx, dy] = movement[event.key];
      setCursor((p) => ({
        x: Math.max(-55, Math.min(55, p.x + dx)),
        y: Math.max(-55, Math.min(55, p.y + dy)),
      }));
    } else if (event.key === "Enter" || event.key === " ") {
      event.preventDefault();
      event.stopPropagation();
      choose(cursor);
    }
  };

  return (
    <section className="nav-panel" aria-label="GPS navigation">
      <button
        className="nav-header"
        aria-expanded={open}
        aria-controls="navigation-content"
        onClick={() => setOpen(!open)}
      >
        <span>
          <span className="eyebrow">MAP-BASED NAVIGATION</span>
          <strong>Plan your drive</strong>
        </span>
        <span aria-hidden="true">{open ? "−" : "+"}</span>
      </button>
      {open && (
        <div className="nav-content" id="navigation-content">
          <p className="nav-assumption">
            Perfect localisation · static map · no obstacle response
          </p>
          <div className="nav-selection" aria-label="Map selection mode">
            <button
              aria-pressed={selection === "start"}
              disabled={!canSelectStart}
              onClick={() => {
                setSelection("start");
                setFeedback(
                  "Select a lane for the starting position and heading.",
                );
              }}
            >
              1. Select start
            </button>
            <button
              aria-pressed={selection === "destination"}
              onClick={() => {
                setSelection("destination");
                setFeedback("Select a destination on a lane.");
              }}
            >
              2. Destination
            </button>
          </div>
          <svg
            ref={svg}
            className="nav-map"
            viewBox="-62 -62 124 124"
            role="application"
            tabIndex={0}
            aria-label={`Navigation map. Selecting ${selection}. Click a lane, or use arrow keys to move the crosshair and Enter to select.`}
            onClick={mapClick}
            onKeyDown={mapKey}
          >
            <title>VECTOR town navigation map</title>
            <rect
              x="-62"
              y="-62"
              width="124"
              height="124"
              rx="3"
              fill="#25372f"
            />
            {town.buildings.map((b, i) => (
              <rect
                key={i}
                x={b.x - b.width / 2}
                y={-b.y - b.depth / 2}
                width={b.width}
                height={b.depth}
                fill="#65736f"
                rx="1"
              />
            ))}
            {town.roads.map((road) => (
              <g key={road.id}>
                <line
                  x1={road.from[0]}
                  y1={-road.from[1]}
                  x2={road.to[0]}
                  y2={-road.to[1]}
                  stroke="#89938f"
                  strokeWidth={town.roadWidth + town.sidewalkWidth * 2}
                />
                <line
                  x1={road.from[0]}
                  y1={-road.from[1]}
                  x2={road.to[0]}
                  y2={-road.to[1]}
                  stroke="#34454a"
                  strokeWidth={town.roadWidth}
                />
                <line
                  x1={road.from[0]}
                  y1={-road.from[1]}
                  x2={road.to[0]}
                  y2={-road.to[1]}
                  stroke="#9baba4"
                  strokeWidth=".3"
                  strokeDasharray="2 2"
                />
              </g>
            ))}
            {route && (
              <polyline
                points={route.points.map((p) => `${p.x},${-p.y}`).join(" ")}
                fill="none"
                stroke="#58dfcc"
                strokeWidth="1.4"
                strokeLinejoin="round"
                strokeLinecap="round"
              />
            )}
            <circle
              cx={start.x}
              cy={-start.y}
              r="2.2"
              fill="#16251e"
              stroke="#b0efbe"
              strokeWidth=".8"
            >
              <title>Start</title>
            </circle>
            {destination && (
              <g transform={`translate(${destination.x},${-destination.y})`}>
                <circle
                  r="2.4"
                  fill="#efb18e"
                  stroke="#1b2830"
                  strokeWidth=".7"
                />
                <circle r=".8" fill="#29343a" />
                <title>Destination</title>
              </g>
            )}
            <g
              className="nav-crosshair"
              transform={`translate(${cursor.x},${-cursor.y})`}
              fill="none"
              stroke="#d0d9a9"
              strokeWidth=".4"
              pointerEvents="none"
            >
              <circle r="2.8" />
              <path d="M-4 0H4M0-4V4" />
            </g>
            <g
              transform={`translate(${vehicle.x},${-vehicle.y}) rotate(${(-vehicle.heading * 180) / Math.PI})`}
            >
              <path
                d="M3.2 0 -2.2 -1.6 -1.3 0 -2.2 1.6Z"
                fill="#f7faf4"
                stroke="#112927"
                strokeWidth=".5"
              />
              <title>Vehicle</title>
            </g>
          </svg>
          <div className="nav-legend">
            <span>
              <i style={{ background: "#b0efbe" }} /> Start
            </span>
            <span>
              <i style={{ background: "#efb18e" }} /> Destination
            </span>
            <span>
              <i style={{ background: "#58dfcc" }} /> Route
            </span>
          </div>
          {!active && (
            <p className="nav-feedback" role="status">
              {feedback}
            </p>
          )}
          {navigation.message && (
            <p className="nav-feedback" role="status">
              {navigation.message}
            </p>
          )}
          <p className="nav-hint">
            Keyboard: focus map, move crosshair with arrows, press Enter to
            select.
          </p>
          {!canSelectStart && (
            <p className="nav-hint">
              Start selection requires a stationary, paused vehicle.
            </p>
          )}
          <div className="nav-summary">
            <span>
              {route
                ? `${route.length.toFixed(1)} m planned`
                : "No route preview"}
            </span>
            <span>
              {navigation.remainingDistance === null
                ? "— remaining"
                : `${navigation.remainingDistance.toFixed(1)} m remaining`}
            </span>
          </div>
          <div className="nav-actions">
            <button
              className="primary"
              disabled={
                !route ||
                active ||
                navigation.state === "ROUTE_UNAVAILABLE" ||
                navigation.state === "DESTINATION_REACHED"
              }
              onClick={onActivate}
            >
              Start autonomous drive
            </button>
            <button disabled={!active} onClick={onCancel}>
              Manual takeover
            </button>
            <button
              onClick={() => {
                setSelectedDestination(null);
                setFeedback("Navigation cleared. Select a new destination.");
                onClear();
              }}
            >
              Clear route
            </button>
          </div>
        </div>
      )}
    </section>
  );
}
