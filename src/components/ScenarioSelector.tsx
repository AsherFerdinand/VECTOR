import type { ScenarioId } from "../world/trafficTypes";
import { SCENARIOS } from "../world/traffic";

interface ScenarioSelectorProps {
  scenario: ScenarioId | null;
  onSelect: (id: ScenarioId) => void;
  timeGap: number;
  onTimeGap: (gap: number) => void;
}

/** Scenario selection is an explicit deterministic reset, never a hidden event. */
export function ScenarioSelector({
  scenario,
  onSelect,
  timeGap,
  onTimeGap,
}: ScenarioSelectorProps) {
  const selected = SCENARIOS.find((item) => item.id === scenario);
  return (
    <section className="scenario-selector" aria-label="Driving scenarios">
      <div className="scenario-heading">
        <span className="eyebrow">REPRODUCIBLE EXPERIMENTS</span>
        <h2>Driving scenario</h2>
      </div>
      <div className="scenario-fields">
        <label>
          Scenario
          <select
            value={scenario ?? ""}
            onChange={(event) => onSelect(event.target.value as ScenarioId)}
          >
            <option value="" disabled>
              Custom navigation
            </option>
            {SCENARIOS.map((item) => (
              <option key={item.id} value={item.id}>
                {item.label}
              </option>
            ))}
          </select>
        </label>
        <label>
          Following time gap <span className="scenario-unit">s</span>
          <input
            type="number"
            min="1"
            max="3"
            step="0.25"
            value={timeGap}
            onChange={(event) => {
              const value = event.target.valueAsNumber;
              if (Number.isFinite(value))
                onTimeGap(Math.max(1, Math.min(3, value)));
            }}
          />
        </label>
      </div>
      <p className="scenario-description">
        {selected?.description ??
          "Select a repeatable traffic experiment, or continue using the navigation map."}
      </p>
      <p className="scenario-reset-note">
        Selecting a scenario resets the simulation and leaves it paused. Start
        autonomous drive to run it.
      </p>
    </section>
  );
}
