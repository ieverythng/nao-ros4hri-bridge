# LocateAnything Local Benchmark — 2026-06-25

Output folder: `/home/juanbeck/Watson/reports/openclaw/locateanything-tests-20260625_195402`

## Timed CLI runs

| Case | Prompt | Exit | Wall time (s) | Detections | Labels | Quality note |
|---|---:|---:|---:|---:|---|---|
| `bus_single` | `bus` | 0 | 41.46 | 1 | 1 | Failure: single-word bus prompt boxed a small sign/edge artifact, not the bus. |
| `alpaca_sep` | `alpaca</c>sunglasses</c>guitar` | 0 | 21.055 | 18 | guitar, spaca, sunglasses | Good recall: alpacas/sunglasses/guitars found; duplicates; typo label `spaca`; boxes loose but useful. |
| `bus_sep` | `bus</c>person</c>backpack` | 0 | 13.399 | 5 | person | Good people detections; did not return bus/backpack; one small edge person/noise box. |
| `zidane_sep` | `person</c>tie</c>ball` | 0 | 13.993 | 8 | person | Finds main people but very noisy/duplicated oversized person boxes; misses tie/ball. |
| `dog_sep` | `dog</c>bicycle</c>car` | 0 | 11.517 | 3 | dog | Finds foreground dog; duplicate dog box; false-positive dog on vehicle/van; misses bicycle/car. |

## Key findings

- Cold CLI invocation includes model load/offload, so these are **end-to-end CLI wall times**, not persistent-server inference latency.
- Successful q8_0 CUDA runs: **11.5s–21.1s** for separator prompts; single-word `bus` took **41.46s** and failed semantically.
- The model has good open-vocabulary recall when prompt terms are separated with `</c>`, but outputs need deterministic post-processing: label normalization, duplicate removal/NMS, allowed-label filtering, and prompt templates.
- Launch hardening: prompts with spaces broke under PowerShell `Start-Process -ArgumentList`; use call operator/argv arrays or avoid shell splitting entirely.
- Production direction: persistent LocateAnything service endpoint is strongly preferred; CLI cold-start is too slow for interactive NAO use.

## Files

- `alpaca_band_multi.stderr.log` (26 bytes)
- `alpaca_band_multi.stdout.log` (920 bytes)
- `alpaca_sep.json` (1041 bytes)
- `alpaca_sep.png` (458462 bytes)
- `alpaca_sep.stderr.log` (1878 bytes)
- `alpaca_sep.stdout.log` (0 bytes)
- `bus_multi.stderr.log` (26 bytes)
- `bus_multi.stdout.log` (920 bytes)
- `bus_sep.json` (287 bytes)
- `bus_sep.png` (1895489 bytes)
- `bus_sep.stderr.log` (1870 bytes)
- `bus_sep.stdout.log` (0 bytes)
- `bus_single.json` (63 bytes)
- `bus_single.png` (1936445 bytes)
- `bus_single.stderr.log` (493 bytes)
- `bus_single.stdout.log` (0 bytes)
- `dog_bike_car.stderr.log` (26 bytes)
- `dog_bike_car.stdout.log` (920 bytes)
- `dog_sep.json` (171 bytes)
- `dog_sep.png` (893768 bytes)
- `dog_sep.stderr.log` (1870 bytes)
- `dog_sep.stdout.log` (0 bytes)
- `summary.json` (6556 bytes)
- `summary_v2.json` (5713 bytes)
- `zidane_people.stderr.log` (26 bytes)
- `zidane_people.stdout.log` (920 bytes)
- `zidane_sep.json` (460 bytes)
- `zidane_sep.png` (1479056 bytes)
- `zidane_sep.stderr.log` (1876 bytes)
- `zidane_sep.stdout.log` (0 bytes)
