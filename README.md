# AUV control stack (Pixhawk 2.4.8 + Raspberry Pi 4B / BlueOS)

Outer-loop control, navigation, failsafe and a software-in-the-loop (SITL) test bench for a single-thruster, four-fin autonomous underwater vehicle. The inner attitude loop and fin mixer live on the Pixhawk (ArduSub 4.7.2 + Lua); everything here runs on the Pi or on a laptop.

**Status: simulation only. Nothing has run on hardware.** All vehicle behaviour below is predicted by the SITL plant, not observed.

## Layout

| Path | What |
|---|---|
| `auv/auv/` | Control library: `nav_filter`, `guidance`, `controllers`, `station_keep`, `failsafe`, `breadcrumb`, `mission`, `water`, `core`, `mavlink_io`, `types` |
| `auv/sim/` | SITL: finned-AUV plant, sensor models, log replay, scenarios, tuning, the Studio GUI |
| `auv/extension/` | BlueOS extension (Docker, FastAPI web app, token-protected mission upload) |
| `auv/gui/` | Static results page |
| `auv/docs/` | `TRIM.md` (fresh/salt buoyancy trim study), `ALGO.png` (control-stack diagram) |
| `auv/tests/` | pytest suite (201 pass, 23 skipped) |
| `logs/` | Field logs from tethered ROV runs (23 CSVs) |
| `AUV_COMPLETE_PLAN_Pixhawk_2.4.8.docx` | Master plan, battery spec, failsafe ladder, phases |
| `ardusub.apj` | ArduSub 4.7.2 build for the Pixhawk1 target (Lua scripting compiled in; 2 MB boards only) |

## Control architecture

- **Pixhawk:** attitude and rate PIDs plus the Lua fin mixer, with a 250 ms dead-man that returns the fins to trim if the Pi stops.
- **Pi (20 Hz):** navigation filter, line-of-sight guidance with integral action, depth-to-pitch and heading-to-yaw laws, station keeping, failsafe ladder H0-H8, mission state machine, breadcrumb return.
- **Interface:** `SET_ATTITUDE_TARGET` plus thrust and a 1 Hz heartbeat. Lat/lon is only the mission file format.
- Diagram: `auv/docs/ALGO.png`.

## Quick start

```bash
cd auv
python3 -m venv .venv && .venv/bin/pip install numpy pytest matplotlib pillow pymavlink fastapi uvicorn httpx
.venv/bin/python -m pytest                 # unit tests
.venv/bin/python -m sim.run_sitl           # 13 scenarios; writes results.json, plots, ONE sim/out/sitl_<timestamp>.csv
.venv/bin/python -m sim.replay_nav         # replay all logs/ through the nav filter
.venv/bin/python -m sim.studio             # one window: animation, results, algorithm diagram, code browser
.venv/bin/python -m sim.viewer --list      # animate a single field log
```

Studio keys: `1` Animate, `2` Results, `3` Algorithm, `4` Code, space pause, arrows switch source/file, up/down speed or scroll. The in-window "Run SITL" button is not verified; run `sim.run_sitl` from a terminal instead.

## SITL pass criteria

A normal scenario passes only if it did not abort, reached the last waypoint (closest approach within the 8 m capture radius), surfaced, and met cross-track RMS < 2 m and depth RMS < 0.3 m. A failsafe scenario passes if it aborted and surfaced. Each result lists `why_failed`.

Last run: 10 of 13 pass. Failing: `compass_bias_3deg` (2.36 m cross-track), `two_waypoint_fresh_m2N` (2.22 m, known-marginal old trim), `wrong_ballast_salt_in_fresh` (salt trim in fresh water sinks to the depth limit and does not surface).

## Chi-square

`nav_filter.chi2_gate` applies a 2-DOF chi-square test (d2 <= 13.82, p = 0.999) to GPS fixes, and the accepted fix is blended as a measurement, not a teleport. It only runs when surfaced. The DVL is gated by validity and FOM < 0.5, and depth has no outlier gate.

## Fresh water and salt water

Trim for slightly positive buoyancy in fresh water (B = +0.5 to +1.0 N), then add about 0.55 kg of lead for salt. See `auv/docs/TRIM.md`. Set `BARO_SPEC_GRAV` and the DVL sound speed per deployment (`auv/auv/water.py`). A salt-trimmed vehicle in a lake is unrecoverable in the model.

## Station keeping

`station_keep.py`: hold into the current, loiter on a 10 m circle in zero current, or float at the surface. Holds within about 1.3 m at 0.1 m/s current and 0.07 m at 0.2 m/s in a kinematic test. With zero current the vehicle still ends about 11 m from the hold point.

## Known gaps and unverified assumptions

- ArduSub's handling of `SET_ATTITUDE_TARGET.thrust` (climb centred on 0.5 vs forward thrust), custom mode numbers (GUIDED 4, SURFACE 9), and the DVL/depth MAVLink message sources are assumptions. Bench-test with props off.
- BlueOS Docker labels, endpoint and base image are unverified; no Docker build has been run.
- Nothing feeds the water-track update on the Pi, so the current-abort rung (H7b) will not fire on hardware.
- The Lua fin mixer needs a 2 MB flash board with scripting enabled (gate G1 in the plan).
- Field logs are tethered ROV runs with 1 s timestamps and no truth; they validate sensor behaviour only. 18 of 23 are usable for replay.
- SITL plant parameters (drag, thrust, heave drag) are estimates and must be re-identified on the real vehicle.
