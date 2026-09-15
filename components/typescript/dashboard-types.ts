/**
 * uav-safe-sim - TypeScript dashboard types (REFERENCE component)
 *
 * Language rationale: TypeScript is assigned to the dashboard because
 * typed telemetry views catch schema drift at compile time. The types
 * below mirror src/uav/provenance.py and src/uav/health.py - the Python
 * modules are the authoritative schema, and this file is a *candidate*
 * mirror whose drift is an experimental observable.
 *
 * Status: REFERENCE - not type-checked here (no tsc in CI yet).
 * Compatibility with the Python provenance schema is NOT assumed; it is
 * established only by schema conformance tests once a build step exists.
 */

export type HealthState =
  | 'HEALTHY'
  | 'DEGRADED'
  | 'STALE'
  | 'FAILED'
  | 'UNKNOWN';

/** Mirrors uav.provenance.DecisionRecord.as_dict() - all times in seconds,
 *  all positions in meters (ENU world frame), never mixed units. */
export interface DecisionRecord {
  t: number;
  component: 'safety' | 'fusion' | 'contracts' | 'health' | 'perception';
  decision: string;
  inputs: Record<string, unknown>;
  uncertainty: number | null;
  model_version: string | null;
  consumer: string | null;
  rationale: string | null;
}

export interface HealthTransition {
  t: number;
  sensor: 'gps' | 'imu' | 'depth' | 'camera' | 'wifi_csi';
  from: HealthState;
  to: HealthState;
  reason: string | null;
}

export interface VetoEvent {
  t: number;
  reason: string;
}

/** Provenance is mandatory: this filter is the dashboard's own enforcement
 *  of the observability contract - a decision without provenance is a bug
 *  and is reported as such, not silently rendered. */
export function assertProvenance(rec: DecisionRecord): string | null {
  if (rec.model_version === null || rec.model_version === undefined) {
    return `decision at t=${rec.t} from '${rec.component}' has no model_version`;
  }
  if (!Number.isFinite(rec.t)) {
    return 'decision has non-finite timestamp';
  }
  return null;
}

export interface RunSummary {
  missionState: string;
  collisions: number;
  vetoes: VetoEvent[];
  healthTransitions: HealthTransition[];
  decisions: DecisionRecord[];
  provenanceErrors: string[];
}

export function summarize(run: RunSummary): string {
  const errors = run.decisions
    .map(assertProvenance)
    .filter((e): e is string => e !== null);
  return [
    `final state:      ${run.missionState}`,
    `collisions:       ${run.collisions}`,
    `safety vetoes:   ${run.vetoes.length}`,
    `health changes:   ${run.healthTransitions.length}`,
    `decisions logged: ${run.decisions.length}`,
    `provenance bugs:  ${errors.length}`,
  ].join('\n');
}
