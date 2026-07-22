# Runner — Architecture & Current-State Specification

**Version 0.4** · supersedes v0.3 · 2026-07-21
Mattias Kroeze · MSc Autonomous Systems, DTU
Autonomous 1/18-scale RC research platform

---

## 1 · What Runner is

Runner is a stock LaTrax Prerunner 1/18-scale RC car converted into a self-contained autonomous research platform. The research contribution is **infrastructure-free iterative racing**: characterizing how localization quality bounds lap-time convergence on a fully self-contained, commodity-sensor vehicle.

All compute and sensing is onboard — no external motion capture, no fixed anchors required for baseline operation. The novel self-improving controller (ILC / Learning-MPC) is DTU thesis work, deliberately deferred. This platform exists to make that work measurable.

**Guiding build principles**

- **Phase-gated scope.** A capability is added only once the failure mode that justifies it has been observed — not preemptively because the hardware is owned.
- **Prototype-first.** Iterate on a working platform over premature optimization; localization quality is the contribution, not mechanical polish.
- **Diagnostic before fix.** Confirm the actual failure from terminal output before changing anything. Never author against assumptions.
- **Extrinsic repeatability is load-bearing.** Rigid, repeatable sensor geometry matters more than millimetre precision. Angular repeatability is the real constraint.
- **One owner per resource.** Every TF edge, every serial port, every PWM channel has exactly one writer. Most of this project's hardest bugs were violations of this rule.

---

## 2 · Phase structure

Phase names describe intent, not a schedule. Phase 0 is bring-up — getting the car to *"I'm ready to try to drive myself."* Phase 1 is the car learning to drive itself. Phase 2 is racing.

| Phase | Goal | Status |
|---|---|---|
| Phase 0 | Bring-up: sensing, teleop, localization. Ends at working SLAM localization. | In progress |
| Phase 1 | Navigation & self-driving. Wheel odom + EKF fusion, Nav2 autonomy, UWB. | Not started |
| Phase 2 | Racing. Pure Pursuit → MPCC → LMPC. DTU thesis control work. | Not started |

**Phase 0 remaining, in order**

1. Design and build crash protection (halo). *In progress.*
2. EKF translation-freeze retest on the corrected stack. *Next — the only item before the gate.*
3. Working SLAM localization — the Phase 0 exit gate.

Wheel encoder (was item 2) is **done**: GPIO 22, calibrated 0.010282 m/edge, `runner_encoder` publishes signed `/wheel/odom` (unfused).

---

## 3 · Hardware

### 3.1 Compute & sensing

| Component | Detail | Interface |
|---|---|---|
| Raspberry Pi 5 8GB | Ubuntu 24.04, ROS 2 Jazzy | — |
| LD19 LiDAR | 10 Hz, 360°, scan window is occlusion datum | UART `/dev/ttyAMA0` @ 230400 |
| BNO085 IMU | On-chip fusion, 50 Hz, GPIO 26 reset | UART `/dev/ttyAMA2` @ 3 Mbaud |
| Hall encoder | US1881 bipolar latch, 8 magnets in spur gear; 0.010282 m/edge | GPIO 22, both edges |
| X1201 UPS HAT | 2×18650, compute power only | I2C 0x36; GPIO 6 AC-loss (active-low); GPIO 16 charge-ctrl |
| DualSense | Teleop controller | Bluetooth |

Power domains are deliberately separated: **the X1201 UPS powers compute only; the stock NiMH traction pack powers the motor/ESC.** A brownout on one domain cannot take down the other.

### 3.2 Mount

**Finalized.** Changing it now requires a good reason. A structural rail spans the aluminum bulkheads; a removable pegboard-pattern carrier plate uses M3 captured nuts (chosen over heat-set inserts to avoid loading printed plastic in the weak Z-axis). Rail-and-sled layout allows repositioning.

- **LiDAR + IMU are rigidly co-mounted on the same structural member** — keeps the `base_link→base_laser` and `base_link→imu_link` extrinsics stable. Decoupling onto separate members was explicitly rejected.
- **LiDAR is the highest point** and sits in the front third — required so the Pi stack behind it doesn't cut a blind wedge in the rear scan arc.
- **The scan window (low horizontal slot), not the sensor body top,** is the datum for all clearance and occlusion checks.
- **GPIO interconnect:** perfboard HAT with right-angle JST-GH connectors; adds no rail footprint.

### 3.3 Drivetrain — now a tracked area

*The v0.2 assumption that the chassis is stock and therefore untracked is retired.* The following diverge from stock:

- Aluminum front and rear bulkheads (these carry the differentials).
- Aluminum shocks.
- Replaced suspension arms — still plastic.
- 8 neodymium magnets pressed into the spur gear on the driveshaft for encoder pickup.
- ESC relocated from stock position.
- Stock motor, stock ESC, stock traction battery otherwise retained.
- Differentials: reassembled and confirmed driving correctly. **Resolved.** (An earlier direction inversion cost an evening for lack of a reference photo — see D-07/D-10.)

### 3.4 GPIO map & chip convention

| GPIO | Pin | Owner | Function |
|---|---|---|---|
| 2 / 3 | 3 / 5 | UPS + battery | I2C1 — MAX17040 fuel gauge (0x36) |
| 4 / 5 | 7 / 29 | IMU | UART2 → `/dev/ttyAMA2` (BNO085) |
| 6 | 31 | UPS | AC-loss detect — active-low (low = AC loss, high = adapter OK) |
| 12 | 32 | Motor | ESC PWM |
| 13 | 33 | Motor | Steering PWM |
| 14 / 15 | 8 / 10 | LiDAR | UART0 → `/dev/ttyAMA0` (LD19) |
| 16 | 36 | UPS | Charge control — UPS-driven (low = enabled, high = disabled) |
| 22 | 15 | Encoder | Hall latch, both edges |
| 23 | 16 | reserved | Quadrature 2nd channel / UWB IRQ spare |
| 26 | 37 | IMU | BNO085 reset (active-high pulse) |

**Chip convention.** The Pi 5 40-pin header is `gpiochip4` / label `pinctrl-rp1` — **not** `gpiochip0`, which is the SoC-internal `gpio-brcmstb` bank. Opening chip 0 for a header pin drives a dead internal line. Open GPIO **by label** (`pinctrl-rp1`) so code survives Pi 5 chip renumbering across kernels. This was a systemic bug: the IMU reset (silent no-op — the BNO085 self-resets, so it went unnoticed) and the hall calibration both opened chip 0. Fixed in `629d316` (IMU) and `91c192b` (calibration).

---

## 4 · Software architecture

### 4.1 Packages

| Package | Node | Role |
|---|---|---|
| `runner_bringup` | — | Launch files, config, calibration scripts. No nodes. |
| `runner_imu` | `bno085_node` | BNO085 → `/imu/data` @ 50 Hz; error count → `/imu/read_errors`. |
| `runner_motor` | `motor_node` | `/cmd_vel` → ESC + steering PWM, with curve, arming, watchdog; publishes `/motor/direction` (Int8). |
| `runner_encoder` | `encoder_node` | Hall edges → signed `/wheel/odom` (nav_msgs/Odometry, unfused). |
| `runner_teleop` | `teleop_node` | `/joy` → `/cmd_vel`, dead-man gated. |
| `runner_battery` | `battery_node` | Fuel gauge → `/battery` @ 1 Hz (systemd service). |
| `ldlidar_stl_ros2` | `ldlidar_..._node` | LD19 → `/scan` (third-party). |
| `rf2o_laser_odometry` | `rf2o_..._node` | Laser odometry → `/odom_rf2o` (third-party, forked). |

### 4.2 Launch tree

Three tiers. The invariant that keeps the serial ports sane: **no atomic launch file may be included by two composites that could run simultaneously.** The three top-level composites are mutually exclusive — running two double-instantiates the sensor drivers and corrupts both UARTs.

```
drive.launch.py = sensors + teleop
map.launch.py   = sensors + localization
full.launch.py  = sensors + teleop + localization

sensors      = tf_static + lidar + imu
localization = rf2o + ekf + slam
```

`ekf_minimal.launch.py` stands alone (RF2O vx only, no IMU) as a diagnostic fallback for isolating the EKF.

### 4.3 Transform tree

One publisher per edge. Enforced after discovering duplicate and mis-rooted publishers in v0.2.

```
map → odom             slam_toolbox   (dynamic)
odom → base_link       EKF            (dynamic, sole owner)
base_link → base_laser tf_static      (static)
base_link → imu_link   tf_static      (static)
```

RF2O can publish `odom→base_link` but is suppressed: `publish_tf: false` plus a `/tf → /tf_disabled` remap (RF2O ignores the flag alone at runtime). The EKF is the sole owner.

### 4.4 Static extrinsics

**`base_link` is the center of the rear axle, projected to the ground plane** — the reference point the bicycle model in Pure Pursuit / MPCC / Nav2 Ackermann all assume. REP-103 (+x fwd, +y left, +z up).

| Edge | x (m) | y (m) | z (m) | yaw (rad) |
|---|---|---|---|---|
| `base_link→base_laser` | 0.132 | 0.000 | 0.1135 | 0 |
| `base_link→imu_link` | 0.082 | 0.0025 | 0.1060 | 3.14159 |

- Laser z is measured to the **scan window**, not the sensor body.
- IMU yaw of π: board mounted +X toward the vehicle rear (X back, Y right, Z up — a clean 180° about Z).
- Values are current against the finalized mount. Re-measurement is low-priority — angular repeatability (guaranteed by the metal standoffs) is what matters, not sub-mm translation.

### 4.5 Sensor fusion (EKF)

`robot_localization`, 2D mode, 15 Hz. Two configs:

- **`ekf.yaml`** — fuses `/odom_rf2o` (vx, vyaw) and `/imu/data` (vyaw). The production config.
- **`ekf_minimal.yaml`** — `/odom_rf2o` vx only, no IMU. Diagnostic isolation config.

Only velocities are fused, no absolute pose or orientation — appropriate given RF2O provides odometry and there is no absolute reference in Phase 0.

### 4.6 Wheel encoder & direction

Single-channel US1881 hall latch on GPIO 22, 8 magnets in the spur gear → 8 edges/rev. Calibrated at **0.010282 m/edge** (0.082255 m/spur-rev; ~97.3 edges/m). Count is speed-independent across the tested 20–26 Hz range → no missed edges, no slip. Honest uncertainty ±0.5%, tape- and rolling-radius-dominated.

`encoder_node` (`runner_encoder`) counts edges in fixed 50 ms windows: `speed = edges × 0.010282 / 0.05`. A single-channel latch gives **unsigned** speed only; the sign is taken directly from `/motor/direction` each window — no rest-gating. Published as `nav_msgs/Odometry` on `/wheel/odom` (twist.linear.x only; pose and all other components untrusted via large covariance). **Not fused into the EKF** — that is Phase 1 (add as `odom1`, vx only).

**Sign source.** `motor_node` publishes `/motor/direction` (`std_msgs/Int8`, −1/0/+1) as a pure observer, emitted at the ESC pulse write. Its FSM (STOP/FWD/BRAKE/REV) reports **BRAKE = +1**, so a decelerating-but-still-forward car is signed correctly through the whole brake. Reverse is only reachable from a stop.

**Known limitation.** A pre-stop release-and-repress into reverse can briefly sign a still-coasting-forward car as reverse — a sub-creep-floor cosmetic error on an unfused topic. True low-speed signed direction requires a second hall channel (quadrature, GPIO 23) — the ratified Phase 1 upgrade. A rest-gate on the sign flip was tried and reverted: no single stationary-timeout satisfies both sign-correctness (wants long) and responsiveness (wants short). VESC/brushless is explicitly off the table.

---

## 5 · Teleop & motor control

### 5.1 Control mapping

- Axis 0 = steering (live regardless of dead-man). Axis 5 = throttle. Axis 2 = brake/reverse.
- Triggers rest at +1, full press at −1; remapped as `(1−axis)/2 → 0…1`.
- **Dead-man on the X button (`buttons[0]`)** gates BOTH throttle and brake/reverse — on a brushed ESC, L2 past neutral is reverse throttle, so an ungated brake is ungated reverse. Steering stays live.
- **All axis/button indices are ROS parameters, not hardcoded** — they shift across USB/Bluetooth transport and driver versions.

### 5.2 ESC curve (deadband + expo)

Measured motor onset (wheels-up): forward whines at ~1550 µs, reverse at ~1450 µs. The curve reclaims the dead PWM band below onset, then applies expo to the live band so there is fine control just above onset and coarse control near full trigger.

| Constant | Value | Meaning |
|---|---|---|
| `NEUTRAL_US` | 1500 | ESC neutral / coast |
| `FWD_ONSET_US` | 1550 | Forward motor onset |
| `REV_ONSET_US` | 1450 | Reverse motor onset |
| `THR_MAX_US` | 1750 | Forward cap (Phase 0 ceiling) |
| `BRK_MAX_US` | 1250 | Reverse cap (Phase 0 ceiling) |
| `CROSS_FRAC` | 0.05 | Input fraction to cross deadband |
| `EXPO` | 2.0 | Live-band shaping exponent |

**`THR_MAX_US = 1750` is a deliberate Phase 0 safety ceiling, not a hardware limit** (full range would be 2000). Bench/basement testing and RF2O's sensitivity to inter-scan motion both favor a lower ceiling. It remains a named constant precisely so it can be raised later — by data, once localization-confidence telemetry exists, not by feel.

### 5.3 Safety layers

- **Dead-man (X):** no longitudinal command unless held. Guards the controller-bump-on-the-table case that destroyed the original mount.
- **Motor watchdog:** no `/cmd_vel` for 200 ms → ESC to neutral. **The load-bearing layer** — it survives `teleop_node` or `joy_node` dying, because a dead-man in a dead process persists its last state. Independent of anything upstream.
- **ESC arm sequence holds neutral,** not an extreme. The prior sequence sent 1000 µs (full reverse) for 1 s to an already-armed ESC — that was the startup lurch, now fixed.

*Residual hazard, logged not solved:* if `motor_node` is SIGKILLed or segfaults, `stop()` never runs and the ESC holds its last pulse forever. No software on the Pi can cover this — it is the primary trigger for the Phase 1+ microcontroller/VESC discussion.

---

## 6 · Persistent services & tooling

- **`runner-foxglove.service`** — Foxglove bridge on `:8765`, systemd, restart-on-failure. Always available so a bare Pi can be inspected. Pure websocket, touches no hardware.
- **`runner-battery.service`** — battery monitor, systemd. Deliberately decoupled from sensor bringup; alone on its I2C bus (0x36), so no contention.
- **`runner-pwm-setup.service`** — oneshot, exports PWM channels and sets permissions at boot. Replaces the old manual sysfs `chown`/`chmod` that reset on reboot.

**Rule: observers (Foxglove, battery) auto-boot; exclusive-hardware drivers (LiDAR, IMU) never auto-boot** — auto-booting a driver that also gets launched by hand recreates the double-open UART corruption.

- VS Code tasks: one per launch file, plus a "stop all" that SIGINTs launch process groups while sparing the systemd services.

---

## 7 · Open items & known issues

- **Crash protection (halo):** not designed, not built. Measurement in progress. Must anchor to suspension towers via socketed legs (not shell posts — too compliant), with a sacrificial shear element so failure is binary and the mount extrinsics can't be silently corrupted.
- **Wheel encoder:** done — GPIO 22, calibrated, `runner_encoder` publishing unfused `/wheel/odom`. EKF fusion deferred to Phase 1 (`odom1`, vx). Sign carries a logged sub-creep-floor error; quadrature (GPIO 23) is the Phase 1 fix.
- **EKF translation freeze:** not validated as fixed. Strongly suspected to have been caused by the garbage v0.2 TF config (base_laser mis-rooted, 132 mm off). Retest on the corrected stack before treating as resolved. *This is the current next task.*
- **IMU reset now live:** the GPIO 26 reset was a silent no-op (wrong chip bank) until this session; it now actually pulses the BNO085 on launch. Confirm `/imu/data` streams clean before leaning on it in the EKF retest.
- **GPIO 6 AC-loss confirmed working** on chip 4 (active-low). The earlier "non-functional" verdict was the chip-0 bug, not a dead pin. Un-revert `24e5ad4` when the battery node is reworked to read source status and add low-voltage shutdown against the 3.20 V floor.
- **flake8 has no exclude config** — 861 pre-existing hits are mostly build/generated artifacts. Add a `setup.cfg`/`.flake8` exclude so lint is a usable signal. Low priority.
- **`rf2o_laser_odometry` is a forked gitlink with no `.gitmodules` mapping.** The fork carries the `/tf → /tf_disabled` remap. Vendored fork by accident; formalize it or it becomes a "works on my Pi" landmine.
- **`x120x` is an untracked nested repo at workspace root** (suptronics UPS tooling). Decide whether to vendor or ignore it explicitly.
- **UPS discharge rate unmodeled.** Do not extrapolate runtime from two points on the Li-ion voltage plateau — log a full session and find the knee.

---

## 8 · Decision log

Append-only. Each entry records a decision that diverges from the obvious default, with its reasoning, so the *why* survives. New decisions append; existing entries are never edited.

| ID | Decision | Reasoning |
|---|---|---|
| D-01 | `base_link` = rear axle, ground-projected | Bicycle-model reference for all downstream controllers. |
| D-02 | LiDAR + IMU co-mounted on one member | Stable extrinsics; binary intact/detached failure mode. |
| D-03 | LiDAR is highest point, front third | Avoids rear blind wedge in 360° scan. |
| D-04 | One publisher per TF edge; EKF owns `odom→base_link` | Eliminates startup-race and mis-root corruption. |
| D-05 | 3 mutually-exclusive composites; sensors never double-launched | Two drivers on one UART corrupt each other. |
| D-06 | Dead-man gates throttle AND brake | L2 past neutral is reverse throttle on a brushed ESC. |
| D-07 | Drivetrain promoted to a tracked area | Chassis is no longer stock; diff/bulkhead changes must be logged. |
| D-08 | `THR_MAX` capped at 1750 for Phase 0 | SLAM sensitivity + bench margin; raise later by telemetry. |
| D-09 | Motor watchdog is the primary safety layer | Survives upstream process death; dead-man cannot. |
| D-10 | Photograph subassemblies before closing them up | Diff inversion cost an evening for lack of a reference. |
| D-11 | Encoder on GPIO 22; GPIO 23 reserved (quadrature / UWB-IRQ); GPIO 16 is UPS-owned charge control | 22/23 freed from the retired walking-cane build; 16 found UPS-driven in the X1201 wiki and logged before it bit. |
| D-12 | Pi 5 header GPIO is `gpiochip4` / `pinctrl-rp1`; open by label | Chip 0 is the internal brcmstb bank — opening it drove dead lines (IMU reset no-op, calibration). By-label survives kernel renumbering. |
| D-13 | `motor_node` publishes `/motor/direction` (Int8, −1/0/+1) as a pure observer; BRAKE = +1 | Single-channel encoder needs an external sign; FSM emits actual commanded direction at the pulse write. BRAKE = +1 keeps braking sign-correct. |
| D-14 | Encoder signs directly from `/motor/direction`, no rest-gate | No single stationary-timeout serves both sign-correctness (wants long) and responsiveness (wants short); the gate made the common case worse. Residual error is cosmetic on an unfused topic. |
| D-15 | Encoder sign kept independent of RF2O | Deriving sign from RF2O couples the backup to the source it exists to cross-check; the EKF is the correct layer to arbitrate disagreement. |
| D-16 | Quadrature (GPIO 23) is the Phase 1 true-direction path; VESC stays off the table | The hall sensor was chosen specifically to avoid VESC/brushless cost. Quadrature adds a more-independent direction source, not less. |
| D-17 | GPIO 6 AC-loss detection confirmed functional; un-revert `24e5ad4` | The "non-functional" verdict was the chip-0 bug, not a dead pin — it toggles on chip 4. Polarity active-low. |

---

## Appendix · Workflow conventions

- **Spec discipline:** changes that diverge from this document require a new decision-log entry. Artifact lineage follows `runner_*` naming.
- **Codex workflow:** read-only investigative prompt first to establish ground truth, then a separate action prompt against confirmed reality. Never author against assumptions.
- **Git:** Codex commits, Matti pushes manually. Prompts end with a commit instruction and never a push.
- Codex on the Pi points at `~/runner_ws`. CAD in Onshape. Remote access via Tailscale; VS Code Remote-SSH.
