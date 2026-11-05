# Data Provenance

Every number in this folder was either measured on the vehicle, taken from a datasheet, or computed from other values in the same table. Nothing comes from a simulator run.

| Status | Meaning |
|---|---|
| `measured` | Measured on the physical vehicle; the instrument and date are in the row. |
| `recorded` | Taken from the team's calibration records (the `robotek-learn` repository) where the measurement was logged; the date is the record date. |
| `derived` | Computed from other rows; the formula is in the `method` column. |
| `modelled` | Read from the CAD or URDF, not weighed. |
| `estimated` | Engineering estimate, not measured; the uncertainty column reflects that. |
| `declared` | Taken from a datasheet or a product label. |
| `pending` | Not measured yet. The row exists so the gap is visible, not to be filled with a guess. |

`measurements.csv` is the single source of truth for the documentation. If a document quotes a number that is not here, the document is wrong.

`power-budget.csv` is the power budget. The current column is marked `measured`, `datasheet` or `pending` per component.

`test-runs.csv` holds the raw run log. One row per run, one file per trace in `data/traces/` once the first instrumented session is recorded.
