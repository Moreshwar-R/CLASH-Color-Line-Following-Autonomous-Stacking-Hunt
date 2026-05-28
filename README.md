# ⚙ CLASH — Color Line-following & Autonomous Stacking Hunt

An autonomous robot built for CLASH, a two-phase robotics competition held at BuildClub SSN, Sri Sivasubramaniya Nadar College of Engineering. The robot navigates a line-based arena, identifies coloured cubes, and autonomously picks and places them at designated zones — no human intervention after start.

---

### Phases

**Phase 1 — Stacking Challenge**
- Scanned and analysed all cubes on the arena
- Identified the cube with the maximum colour count
- Picked it up and placed it in the finish zone
- Completed in **35 seconds**

**Phase 2 — On-the-Spot Rescue Challenge**
- Undisclosed complex map revealed on event day
- Navigated paths with loops, T-junctions, intersections, and right-angle turns
- Picked only Red and Blue cubes, parked them in their respective zones
- Green cubes were detected and deliberately left untouched

| Colour | Identity | Action |
|--------|----------|--------|
| Red | Bomb Hazard | Pick and place |
| Blue | Radioactive Material | Pick and place |
| Green | Civilians | Must not touch |

---

### Architecture

```
┌─────────────────────────────────────────────┐
│                   ESP32                      │
│                                             │
│  IR Array ──► Line Follow Logic             │
│  TCS3200  ──► Colour Detection              │
│  HC-SR04  ──► Proximity Check               │
│                    │                        │
│         ┌──────────┴──────────┐             │
│      TB6612FNG            Servo Control     │
│      N20 Motors           SG90 + MG995      │
└─────────────────────────────────────────────┘
         │                        │
  2× N20 Wheels              Gripper + Lift Arm
  1× Castor Wheel
```

---

### Hardware

**Compute & Control**
- ESP32 — main microcontroller, handles all logic and Bluetooth
- TB6612FNG — dual H-bridge motor driver for drive wheels

**Sensing**
- 8-array IR sensor (TCRT5000) — black line detection across 6 active channels
- TCS3200 colour sensor — identifies Red, Blue, Green via pulse-width RGB reading
- HC-SR04 ultrasonic sensor — detects cube presence before triggering pick

**Actuation**
- N20 6V gear motors × 2 — left and right drive wheels
- N20 wheels × 2 — main drive
- Castor wheel × 1 — front passive support
- SG90 servo (Kit4Curious gripper) — opens and closes the robotic gripper
- MG995 Tower Pro servo — raises and lowers the gripper arm

**Power**
- 7.4V 4000mAh 2S Li-ion battery
- LM2596 buck converter — stepped-down logic rail for ESP32 and sensors
- XL4015 5A buck converter — separate higher-current rail for servo and motors

---

### Software

Written in **Arduino C++**.

| File | Description |
|------|-------------|
| `code/stage1.ino` | Line following, colour detection, pick sequence for Phase 1 |
| `code/stage2.ino` | Full autonomous state machine for Phase 2 — detect, pick, navigate, place |

**Logic overview:**
- 6-sensor weighted line following with gap-bridging recovery
- TCS3200 colour identification using calibrated RGB pulse-width ranges
- Ultrasonic proximity guard before every pick attempt
- Phase-based state machine controlling the full pick → carry → place flow
- Smooth servo actuation via incremental stepping loops
- Bluetooth start/stop trigger via `BluetoothSerial`

---

### Robot

![Robot Build](robot/robot-build.jpg)

---

### Runs

- Phase 1 — [Watch on YouTube](https://youtu.be/sqrNhwVpCkE)
- Phase 2 — [Watch on YouTube](https://youtu.be/hyZNEee651o)


---

### Topics

`esp32` `line-follower` `autonomous-robot` `tcs3200` `tb6612fng` `arduino` `robotics` `colour-detection` `servo` `buildclub-ssn` `clash`

---

### Contributors

Moreshwar R, Monish B. S, Vikram N. K, Nawin S. P

---
