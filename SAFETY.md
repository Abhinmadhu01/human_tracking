# Safety Guide & Pre-Flight Checklists

Operating an autonomous human tracking quadcopter involves dynamic proximity to human targets. Strict safety practices are mandatory.

## Primary Flight Rules

1. **Keep a Safe Standoff Distance**:
   * The drone controller regulates standoff distance via bounding box height. **Never configure `target_distance_m` lower than 2.0 meters** for outdoor testing.
   * Maintain a minimum safety perimeter of 5.0 meters during bench testing and initial flights.
2. **Propellers Off on Bench**:
   * **Always remove all propellers** when testing detection models, tuning PID parameters, or checking MAVROS communication on the bench.
3. **Manual Override is Paramount**:
   * Keep your hands on the RC transmitter sticks at all times.
   * Assign a high-visibility switch to immediately flip flight mode from **GUIDED** back to **Loiter** or **AltHold**.
4. **Geofence Protection**:
   * Configure a software ceiling (`max_altitude: 15.0m`) and floor (`min_altitude: 2.0m`).
   * Configure maximum allowable distance from home (`max_range_m: 30.0m`). If the drone exceeds this radius, the stack commands immediate failsafe.

## Pre-Flight Checklist

- [ ] Propellers inspected for stress fractures, securely tightened
- [ ] IMX500 camera mount firmly fastened at 45° tilt with zero mechanical play
- [ ] CSI ribbon cable strain-relieved and safely away from spinning propellers
- [ ] 5V / 5A BEC voltage tested with multimeter ($5.05	ext{ V} - 5.15	ext{ V}$)
- [ ] GPS 3D fix verified outdoors with $> 12$ satellites locked
- [ ] RC transmitter mode switch mapped: Position 1 = Loiter, Position 2 = AltHold, Position 3 = GUIDED
- [ ] Live video stream verified in QGroundControl (Port 5000)
- [ ] Target person clearly wearing distinguishable clothing with good contrast against background

---

<div align="center">

| [⬅️ Previous: **Project Overview (Home)**](README.md) | [📖 **Documentation Roadmap**](README.md#documentation-roadmap) | [➡️ Next: **02 - Hardware Setup**](docs/hardware/README.md) |
| :--- | :---: | ---: |

</div>
