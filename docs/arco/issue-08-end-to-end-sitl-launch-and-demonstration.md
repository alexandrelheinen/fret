# Issue 08: End-to-End SITL Launch and Demonstration

## Goal

Provide a single reproducible launch workflow that runs full ARCO-to-FRET obstacle-aware manipulation in Gazebo.

## Problem Statement

Without a canonical launch path, reproducibility and review confidence are weak.

## Functional Requirements

1. Implement one launch entrypoint for full pipeline:
- world + robot,
- perception pipeline,
- occupancy adapter,
- ARCO planner adapter,
- trajectory execution.

2. Support configurable scenario and planner profiles.
3. Produce logs/artifacts for post-run analysis.

## Non-Functional Requirements

- Startup sequence must be deterministic and documented.
- Failure points must emit actionable diagnostics.

## V-Cycle Tasks

### Descending Branch

- Specify launch graph, node ordering, and readiness checks.
- Define demo scenario protocol and expected outcomes.
- Define smoke-test criteria for CI/local validation.

### Ascending Branch

- Launch smoke tests for full pipeline startup.
- System test verifying one complete obstacle-avoiding motion.
- Demo artifact package (video/log/plot) attached in issue.

## Deliverables

- unified launch file(s) and profile configs.
- runbook in docs.
- demo artifact package.

## Acceptance Criteria

- One documented command sequence runs full pipeline.
- Robot reaches target while avoiding configured obstacles.
- Artifacts provide traceability for planning and control decisions.

## Dependencies

- Issue 02 to Issue 07.
