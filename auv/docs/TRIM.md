# Trim, ballast and water type

Generated numbers: `.venv/bin/python -m sim.trim_sweep`. Library: `auv/water.py`.

## Sign convention
- **Net buoyancy B (N): positive = floats.** Plant `net_weight_N = -B` (positive sinks).
- Pitch: positive = nose up. Positive B needs **nose-down** pitch to hold depth (hold pitch printed negative).
- Hull (buoyancy mass, not dynamic mass): 19.92 kg, 20 L displaced. Fresh, no ballast: B = +0.78 N.
- Salt (1025 kg/m3) adds ~+4.9 N, so ~0.55 kg lead (11340 kg/m3) restores B = +0.78 N.

## Method
Open-loop plant, cruise 0.514 m/s (thrust 0.055). Columns: pitch for zero vertical speed;
time from surface to 2.5 m / 10 m at the depth controller pitch limit (-20 deg); time from 10 m to 0.3 m
with the harness surface command (pitch +0.3 rad, thrust 0.15) and passively (thrust 0, pitch 0).
`n/a` = never gets there (dive) or never rises (passive, sinks). Salt rows use density-scaled drag (x1.025).


### Fresh, heave_drag_k=100.0
```
B_N  hold_pitch_deg  dive2.5_s  dive10_s  surf_active_s  surf_passive_s
  -3    20.8     9.7    32.3   136.5     n/a 
  -2    16.9    10.7    35.7    96.7     n/a 
  -1    11.9    12.4    41.3    70.0     n/a 
   0    -0.0    18.6    64.3    43.0     n/a 
 0.5    -8.4    30.2   106.7    34.3   141.1
   1   -11.9    43.7   149.2    31.4    99.8
 1.5   -14.6    65.7   215.8    29.5    81.5
   2   -16.9   106.1   344.9    28.1    70.6
   3   -20.8     n/a      n/a     26.0    57.6
   4   -24.2     n/a      n/a     24.4    49.9
   5   -27.6     n/a      n/a     23.2    44.7
```

### Salt, heave_drag_k=100.0
```
B_N  hold_pitch_deg  dive2.5_s  dive10_s  surf_active_s  surf_passive_s
  -3    20.8     9.7    32.6   137.9     n/a 
  -2    16.9    10.8    36.0    97.7     n/a 
  -1    11.9    12.5    41.7    70.7     n/a 
   0    -0.0    18.7    65.0    43.4     n/a 
 0.5    -8.4    30.4   107.8    34.6   142.8
   1   -11.9    44.3   151.1    31.7   101.0
 1.5   -14.6    66.1   218.1    29.8    82.5
   2   -16.9   106.8   348.5    28.3    71.4
   3   -20.8     n/a      n/a     26.2    58.3
   4   -24.2     n/a      n/a     24.6    50.5
   5   -27.6     n/a      n/a     23.4    45.2
```

### Heave-drag sensitivity (fresh): hold pitch deg / passive surf s
```
B_N  k=50           k=100          k=150          k=200        
  -3    30.9/   n/a     20.8/   n/a     16.9/   n/a     14.6/   n/a 
  -2    24.2/   n/a     16.9/   n/a     13.8/   n/a     11.9/   n/a 
  -1    16.9/   n/a     11.9/   n/a      9.7/   n/a      8.4/   n/a 
   0    -0.0/   n/a     -0.0/   n/a     -0.0/   n/a     -0.0/   n/a 
 0.5   -11.9/  103    -8.4/  141    -6.9/  171    -5.9/  197
   1   -16.9/   73   -11.9/  100    -9.7/  121    -8.4/  139
 1.5   -20.8/   59   -14.6/   82   -11.9/   99   -10.3/  114
   2   -24.2/   51   -16.9/   71   -13.8/   86   -11.9/   98
   3   -30.9/   42   -20.8/   58   -16.9/   70   -14.6/   80
   4   -37.0/   36   -24.2/   50   -19.6/   61   -16.9/   70
   5   -42.7/   32   -27.6/   45   -22.0/   54   -19.0/   62
```

(`k` = heave drag N/(m/s)^2. Sweep cells are hold pitch deg / passive surface time s.)

## Findings
1. Hold pitch is set by sqrt(|B|/k): w_ss = sqrt(B/k), pitch = asin(w_ss/u). At B=2 N, k=100: -17 deg of the 20 deg
   limit. At k=50 it exceeds the limit. The earlier -2 N fresh case (B=+2) is therefore marginal at best and
   fails for k <= ~60. Heave drag is a guess and swings the answer by 2x, so the design must tolerate
   k = 50..200.
2. Dive time grows fast with B: B=+1 -> 44 s / 149 s (2.5 m / 10 m) at full nose-down; B=+2 -> 106 s / 345 s;
   B>=+3 cannot dive at cruise speed at all (descent rate u*sin(20deg) = 0.176 m/s < w_ss).
3. Negative B dives easily but has no passive recovery; active surfacing from 10 m takes 70 s (B=-1) to 137 s
   (B=-3), and needs thrust + working fins. Loss of Pi/thrust = vehicle stays down. Not acceptable as a default.
4. Salt vs fresh differ by <2% in every column once B is matched. Water type only matters through B.

## Recommended trim
- **Fresh: B = +0.5 .. +1.0 N (target +0.78 N, no ballast).** Hold pitch -8..-12 deg at k=100, worst case
  -17 deg (k=50, B=1.0), 85% of limit. Dive to 2.5 m in 30-44 s, 10 m in 107-149 s. Passive surfacing from
  10 m: 100-141 s (k=100), up to ~200 s at k=200.
- **Salt: same B window via ~0.55 kg lead** (pre-dive checklist computes it from measured dry mass).
- Trim in a tank to +/-0.25 N (~25 g); weigh ballast, do not eyeball.
- **Max safe negative: B = -1 N** (hold +8..+17 deg nose up for k=200..50, active surface 70 s). Below that
  (B<=-2) k=50 pushes hold pitch past 20 deg. Anything negative has no passive surfacing; treat as degraded.
- **Max safe positive: B = +1.5 N** (k=50 hold pitch -20.8 deg = limit). Do not exceed +1.0 on a loaded dive.
- Wrong ballast: salt ballast left on in fresh gives B = -4.1 N: hold needs ~+24 deg nose up (limit 20).
  `wrong_ballast_salt_in_fresh`: vehicle sinks past the 12 m limit (12.4 m), H7 abort, climbs under thrust
  but only reaches ~0.5 m, never <0.3 m. Honest result: not recoverable to GPS depth without thrust. The checklist
  must make ballast/water mismatch impossible (weigh, tick water type).

## Consequences for control
- Positive buoyancy means: surfacing is passive; **descending and holding depth need continuous nose-down
  pitch with speed**. Zero speed or thrust loss -> float up (the fail-safe we want).
- Depth hold at depth requires a steady -8..-17 deg trim pitch. The current DepthController integral authority
  is ki*int_limit = 0.01*10 = 0.1 rad (5.7 deg); the rest comes from P, giving a steady offset (about 0.5 m
  rms in `two_waypoint_fresh`, 1.2 m shallow at B=+2). Request: raise int_limit to >= 30 (0.3 rad) or add a trim
  pitch feed-forward, and make the dive-timeout scale with the B window (at B=+1 10 m takes ~150 s).
- Dive timeout must be >= ~2x the sweep dive time for the highest trim in the window.
- `two_waypoint_fresh_m2N` (B=+2) is kept as the known-marginal case and still ends in "dive timeout".

## Water settings (pre-dive checklist, `water.predive_checklist`)
| setting | fresh | salt |
|---|---|---|
| BARO_SPEC_GRAV | 1.0 | 1.024 |
| DVL sound speed (m/s) | 1482 | 1521 |
| ballast (lead) | 0 kg | ~0.55 kg |
| DVL error if left at 1500 | +1.2% high | -1.4% low |
| depth error if spec grav wrong | fresh set in salt: reads 2.5% deep (0.25 m at 10 m) | salt set in fresh: 2.4% shallow |

Brackish: `water.custom_water(density)` interpolates sound speed and spec grav.
