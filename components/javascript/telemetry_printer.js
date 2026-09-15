#!/usr/bin/env node
// uav-safe-sim - JavaScript simulation glue / telemetry consumer (verified)
//
// Language rationale: JavaScript is assigned to lightweight glue because
// the provenance stream is JSON and Node needs no build step for tooling.
// It is explicitly NOT assigned to anything flight-critical.
//
// Reads provenance DecisionRecords (JSON Lines) on stdin and verifies the
// observability contract: every decision must carry t, component, decision,
// model_version. Records that fail validation are counted and reported -
// a decision without provenance is a bug.
//
// Status: VERIFIED - syntax-checked and executed with node in CI:
//   node --check telemetry_printer.js
//   cat sample.jsonl | node telemetry_printer.js

'use strict';

const readline = require('readline');

const REQUIRED = ['t', 'component', 'decision', 'model_version'];
let total = 0;
let invalid = 0;
const byComponent = {};
let firstInvalid = null;

function validate(rec) {
  if (rec === null || typeof rec !== 'object') return 'not_an_object';
  for (const key of REQUIRED) {
    if (rec[key] === undefined || rec[key] === null) return `missing_${key}`;
  }
  if (typeof rec.t !== 'number' || !Number.isFinite(rec.t)) return 'bad_t';
  if (typeof rec.component !== 'string' || typeof rec.decision !== 'string') {
    return 'bad_component_or_decision';
  }
  if (typeof rec.model_version !== 'string' || !rec.model_version) {
    return 'missing_model_version';
  }
  return null;
}

const rl = readline.createInterface({ input: process.stdin });
rl.on('line', (line) => {
  line = line.trim();
  if (!line || line.startsWith('#')) return;
  total += 1;
  let rec;
  try {
    rec = JSON.parse(line);
  } catch (err) {
    invalid += 1;
    if (!firstInvalid) firstInvalid = `unparseable_json: ${err.message}`;
    return;
  }
  const problem = validate(rec);
  if (problem) {
    invalid += 1;
    if (!firstInvalid) firstInvalid = `${problem} (t=${rec.t})`;
    return;
  }
  byComponent[rec.component] = (byComponent[rec.component] || 0) + 1;
});
rl.on('close', () => {
  const lines = ['provenance telemetry summary',
                 `  total records:   ${total}`,
                 `  invalid records: ${invalid}`];
  for (const [comp, n] of Object.entries(byComponent).sort()) {
    lines.push(`  ${comp.padEnd(12)} ${n}`);
  }
  if (invalid > 0) lines.push(`  FIRST FAILURE: ${firstInvalid}`);
  console.log(lines.join('\n'));
  process.exitCode = invalid > 0 ? 1 : 0;
});
