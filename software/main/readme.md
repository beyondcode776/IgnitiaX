# Agribot Arm — Direct RC Control (README)

**Last updated:** 2025-08-25  
**Author:** Generated for Shaurya’s Agribot project — direct RC → joint control for 1× Dynamixel XL330 (shoulder) + 2× DS3225MG (elbow pitch, elbow yaw) using FlySky FS-i6S/FS-iA10B receiver and Raspberry Pi 4.

---

> Purpose: this README fully documents the `direct_rc_control.py` program and everything required to run, tune, debug, and extend it. It’s written to be machine- and human-actionable so another AI or teammate who knows nothing about your hardware can pick it up and work from it. Read it top → bottom. Follow checklists before powering the robot.

---

# Table of contents

1. Overview (what this repo/script does)
2. High-level architecture (blocks & dataflow)
3. Required hardware & wiring summary (what to connect where)
4. Software prerequisites & installation
5. i-BUS protocol & receiver details (what the code expects)
6. PCA9685 (ServoKit) and DS3225MG servo details
7. Dynamixel XL330 specifics (Protocol 2.0 mapping used)
8. The `direct_rc_control.py` program — detailed walk-through
   - config block (every parameter explained)
   - IBus class (how frames are parsed)
   - motor setup functions
   - helper math & FK safety
   - main control loop (step-by-step)
9. Example run & sample logs
10. Calibration procedure (step-by-step, safe)
11. Tuning parameters (recommendations)
12. Safety checklist & fail-safes
13. Troubleshooting (symptoms, causes, fixes)
14. Testing without hardware (simulators & test harness)
15. Extensions you can add later
16. Notes, limitations, and final checklist
17. Appendix: useful commands & short reference

---

# 1. Overview

`direct_rc_control.py` implements **direct joint control of a 3-DOF arm**:

- **Input:** FlySky transmitter connected to an FS-iA10B receiver. The receiver outputs i-BUS frames to Raspberry Pi UART (`/dev/serial0`) through a level shifter.
- **Processing:** The Pi reads i-BUS channel values, normalizes them to `-1..+1`, applies deadband, scales to joint angular speeds (deg/s), integrates speeds at a fixed loop rate to update joint *angles*, clamps those angles to configured limits, optionally enforces a simple horizon safety (prevent `z < 0`), and sends position commands to motors.
- **Output:**  
  - **Shoulder** (Dynamixel XL330) — position mode via Dynamixel SDK (Protocol 2.0).  
  - **Elbow pitch & elbow yaw** (DS3225MG servos) — via PCA9685 (Adafruit ServoKit).

**Why direct control?** For manual teleoperation (joystick control) this is simpler and lower-latency than running IK. The stick maps to velocity; operator “drives” joints directly.

---

# 2. High-level architecture

Blocks:

- **FlySky transmitter** → radio → **FS-iA10B receiver**.
- Receiver i-BUS signal → (level shifter) → Pi UART RX (`/dev/serial0`).
- Pi runs `direct_rc_control.py`:
  - i-BUS parser → normalized channels
  - channel → joint mapping + reversing + deadband
  - normalized * speed → integrate → new angles
  - apply limits + horizon check
  - send commands:
    - PCA9685 (I²C) → DS3225MG servos (elbow bend, elbow yaw)
    - Dynamixel via `/dev/ttyUSB0` (U2D2/USB) → shoulder
- Power rails:
  - UBEC / power supply → DS3225 (5–6 V)
  - U2D2 / Dynamixel power (follow your Dynamixel power plan)
  - Pi powered from its own supply
  - **All grounds tied together**.

Dataflow timing:

- Loop frequency: `LOOP_HZ` (default 50 Hz). i-BUS frames are expected to come continuously; code integrates joystick speed every loop.

---

# 3. Required hardware & wiring summary

**Core components**
- Raspberry Pi 4 (Raspbian / Raspberry Pi OS).
- FlySky FS-i6S transmitter + FS-iA10B receiver.
- Dynamixel XL330-M288-T (shoulder).
- DS3225MG (x2) high-torque 270° servos for elbow pitch, elbow yaw.
- PCA9685 16-channel PWM driver (Adafruit or clone) — I²C.
- USB2Dynamixel / U2D2 or other USB-TTL adapter for Dynamixel (shows up as `/dev/ttyUSB0`).
- UBEC / regulated 5–6 V supply for DS3225MG servos capable of peak stall currents.
- Level shifter for i-BUS signal (5 V → 3.3 V).
- Wires, connectors, fuses as required.

**Basic wiring mapping (default in code)**
- Receiver i-BUS signal → level shifter → Pi `GPIO15` (RXD) → serial device `/dev/serial0` (ensure serial console disabled).
- Receiver VCC → 5 V supply; receiver GND → common ground.
- PCA9685: `SDA` → Pi `GPIO2` (pin 3), `SCL` → `GPIO3` (pin 5); PCA9685 V+ to UBEC 6 V (as in your diagram) and GND common.
- DS3225 servos connected to PCA9685 channels:
  - `BEND_CH` = 0 (default)
  - `YAW_CH` = 1 (default)  
  - **Note:** code expects yaw logical `ψ` mapped to servo command `ψ + 135°` (servo center = 135°).
- Dynamixel XL330 connected to USB2D2 → Pi USB as `/dev/ttyUSB0`. Power Dynamixel per your power plan (U2D2 can power it or use separate but common ground).

**Important power & safety**
- Never power DS3225 from Pi 5 V. Use UBEC / battery regulator sized for servo stall currents.
- Common ground across Pi, PCA9685, UBEC, Dynamixel power.
- Always start with low servo speeds and no load.

---

# 4. Software prerequisites & installation

**System**
- Raspberry Pi OS (64 or 32 bit), up-to-date. Pi 4 recommended.

**Enable interfaces**
- `raspi-config`:
  - Interfacing Options → I2C → enable.
  - Interfacing Options → Serial → disable serial console, enable serial hardware port (so `/dev/serial0` is available).
- Reboot after changes.

**Install packages**
```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-smbus i2c-tools
pip3 install adafruit-circuitpython-servokit pyserial dynamixel-sdk numpy
```

`adafruit-circuitpython-servokit` provides `ServoKit`; `pyserial` for serial iBUS; `dynamixel-sdk` for Dynamixel comms.

(Optional) `RPi.GPIO` can be installed if you plan to use fallback PWM instead of PCA9685:
```bash
pip3 install RPi.GPIO
```

---

# 5. i-BUS protocol details (what the program expects)

The program reads **FlySky i-BUS frames** from the serial RX. Typical i-BUS frame (32 bytes):

- `0x20` start byte, then `0x40`, then pairs of bytes for channel values (little-endian u16), then 2-byte checksum. The script reads 32 bytes and verifies checksum:

**Checksum calculation used**:
```
csum_calc = (0xFFFF - sum(data[0:30])) & 0xFFFF
```
The code reads one byte until `0x20`, then reads the rest 31 bytes, verifies header and checksum, then parses channels as `uint16`. Channel values typically range ~1000..2000 (microseconds-like scale). The code maps:

```
norm = (value - 1500.0) / 500.0
norm_clamped = clamp(norm, -1.0, +1.0)
```

Hence:
- 1500 → center → 0.0
- 1000 → full negative → -1.0
- 2000 → full positive → +1.0

**If your receiver is not using i-BUS** (or you use PWM per-channel signals), you must change how inputs are read. Our code uses i-BUS because FS-iA10B supports it and it’s robust.

---

# 6. PCA9685 (ServoKit) & DS3225MG details

- PCA9685 is an I²C PWM driver capable of 16 channels. We use the Adafruit `ServoKit` abstraction.
- DS3225MG is a 270° digital servo (check specs). We configure `actuation_range = 270` and pulse width range `500–2500 µs` in the script — common for wider-range servos. If your servos behave oddly, tune pulse widths (min_us / max_us).

**Mapping in code**
- `kit.servo[BEND_CH].actuation_range = 270`
- `kit.servo[BEND_CH].set_pulse_width_range(500, 2500)`

**Note about yaw mapping**  
- The code maps logical yaw `ψ in [-90, +90]` to servo angle `ψ + 135`. This centers logical yaw `0` at servo angle `135°` (middle of 0..270). That mapping gives symmetrical ±90 capability. You can change offsets if needed.

---

# 7. Dynamixel XL330 specifics (Protocol 2.0) used in code

We assume **Protocol 2.0** (XL family). Addresses used:

- `ADDR_OPERATING_MODE = 11` — set to `3` for Position mode
- `ADDR_TORQUE_ENABLE = 64`
- `ADDR_GOAL_POSITION = 116` (4 bytes, ticks 0..4095 ≈ 0..360°)
- `ADDR_PRESENT_POSITION = 132` (4 bytes)

**Position mapping**
- The script maps *logical* degrees `deg` to ticks using:
```py
mdeg = (deg + OFFSETS["shoulder"] + 180.0) % 360.0
ticks = int((mdeg / 360.0) * 4095.0)
```
This maps logical `-180..+180` to `0..4095`. `OFFSETS["shoulder"]` is a calibration offset.

**Initialization**
- The code disables torque, sets operating mode to position (`3`), then enables torque.

**IMPORTANT**: If your real Dynamixel model differs, adjust protocol version, addresses, and ticks per revolution.

---

# 8. `direct_rc_control.py` — detailed walk-through (section by section)

Below is a descriptive run-through of the full program, explaining every configurable constant and each function. You can open the script to view the small comments inline as well.

---

## 8.1 Configuration parameters (top of file)

These are where you set the behavior without touching runtime logic.

- `L1`, `L2` — link lengths in cm (used by horizon safety only). Set to `L1=70.0`, `L2=45.0`.
- `CH_MAP` — maps joint names to i-BUS channel indices (0-based). Default:
  ```py
  CH_MAP = {"elbow_yaw": 0, "elbow_bend": 1, "shoulder": 2}
  ```
- `REVERSE` — flip axis sign if the physical wiring/horn produces reversed direction.
- `SPEED` — maximum angular velocity (deg/s) at full stick deflection (`±1`).
- `LIMITS` — (min_deg, max_deg) per joint. The code clamps angles to these.
- `OFFSETS` — calibration offsets applied before sending to the motor. Example: `OFFSETS["elbow_yaw"]` adjusts the 135° center mapping to the real mechanical center.
- `DEADBAND` — normalized stick deadband; reduces jitter near center.
- `LOOP_HZ` — main control loop frequency (default 50 Hz).
- `ENABLE_HORIZON_SAFETY` — boolean to enable basic horizon (z >= 0) safety.
- Hardware: serial ports, PCA9685 I²C address, Dynamixel port/baud, PCA channels for servos.

---

## 8.2 IBus class (parser)

**What it does**
- Opens the serial port (default `/dev/serial0`, baud 115200).
- `read_channels()` — reads and returns a list of parsed channel values (u16), or `None` if no full frame is available.
- Frame parsing steps:
  1. Read bytes until `0x20` start byte.
  2. Read rest 31 bytes. Validate header `0x40`.
  3. Compute checksum and compare.
  4. Extract 14 channels (as 16-bit little-endian values).
- `norm(v)` — normalize raw channel to `[-1, +1]` via `(v - 1500)/500`.

**Notes**
- `read_channels()` blocks up to the serial timeout set in `serial.Serial(...)`. The main loop accounts for `None` gracefully.

---

## 8.3 Motor setup functions

**PCA9685 (ServoKit) initialization**
- `ServoKit(channels=16, address=PCA9685_ADDR)`, set `kit.frequency = 50`.
- Configure each servo channel’s actuation range and pulse width.
- `pca_write_deg(channel, angle_deg)` clamps to `0 .. 270` and writes `kit.servo[channel].angle = angle_deg`.

**Dynamixel initialization**
- Open serial to `DXL_PORT`.
- Use `PacketHandler(PROTOCOL=2.0)` and `PortHandler`.
- `dxl_init_position_mode()`: disable torque, set operating mode to `3` (position), re-enable torque.
- `deg_to_dxl_ticks(deg)` and `dxl_write_deg(deg)` convert logical deg → ticks and write `Goal Position` at address `116`.

---

## 8.4 Helper math & FK safety

- `clamp(v, lo, hi)` — generic clamp.
- `apply_limits(joint, angle)` — apply per-joint limits.
- `fk_z(shoulder_deg, elbow_deg, yaw_deg)` — computes `z = L1*sin(shoulder) + L2*sin(shoulder + elbow)` (yaw doesn't affect Z). Used if `ENABLE_HORIZON_SAFETY=True`.
- `horizon_safe(ns, ne, ny)` returns `True` if computed z ≥ 0.

---

## 8.5 Main control loop (the core) — step-by-step

1. **Initialization**
   - Create `IBus` object to read i-BUS frames.
   - Configure PCA9685 and Dynamixel.
   - Set an initial safe pose: `shoulder`, `elbow`, `yaw` (defaults in code).
   - Send initial position commands to motors.

2. **Loop timing**
   - `dt = 1.0 / LOOP_HZ`. The loop uses sleep to maintain cadence. `t_last` tracks time.

3. **Read i-BUS frame**
   - `ch = ibus.read_channels()` reads one frame (or `None`).
   - If `ch` is not `None`, `failsafe_timer` updated.

4. **Normalize & map channels**
   - For each mapped joint:
     - `val = IBus.norm(ch[CH_MAP[joint]])`
     - `val = -val` if `REVERSE[joint]` is `True`.
     - Apply deadband: if `abs(val) < DEADBAND`, set to `0`.
   - Compute speeds:
     - `nspeed = val_shoulder * SPEED["shoulder"]` (deg/s)
     - `espeed = val_elbow * ...` etc.

5. **Integrate speeds → new angles**
   - `ns = shoulder + nspeed * dt`
   - `ne = elbow + espeed * dt`
   - `ny = yaw + yspeed * dt`

6. **Enforce limits**
   - `ns = apply_limits("shoulder", ns)` etc.

7. **Horizon Safety (optional)**
   - If enabled and new pose violates `z>=0`, try heuristics:
     - First: freeze elbow change and try shoulder+yaw.
     - Then: freeze shoulder change.
     - Then: freeze yaw change.
     - Finally: revert to previous pose if none succeed (safe fallback).

8. **Send commands to motors**
   - `dxl_write_deg(ns)` → Dynamixel shoulder position command.
   - `pca_write_deg(BEND_CH, ne + OFFSETS["elbow_bend"])`
   - `pca_write_deg(YAW_CH, ny + 135.0 + OFFSETS["elbow_yaw"])` (yaw shifted to servo center).

9. **Update state**
   - `shoulder, elbow, yaw = ns, ne, ny`

10. **Failsafe**
    - If `time.time() - failsafe_timer > FAILSAFE_SEC` (default 0.5 s) — no fresh i-BUS frames — the loop holds position (you can optionally implement return-to-home or torque-off).

11. **Exit**
    - On KeyboardInterrupt or exception: turn off Dynamixel torque and close the port; cleanup PCA/PWM if needed.

---

# 9. Example run & sample logs

**Start the script**
```bash
python3 direct_rc_control.py
```

**Expected console output**
```
iBUS listening on /dev/serial0
PCA9685 ready at 0x40, Dynamixel on /dev/ttyUSB0 id=1
target pose sent: shoulder=10.00, elbow=60.00, yaw=0.00
(loop messages suppressed by default)
```

If you add debug prints (recommended during calibration), you might see lines like:
```
CH[2] raw=1640 norm=0.28 -> shoulder speed=22.4 deg/s -> delta=0.448 deg
New angles -> shoulder=10.448, elbow=60.000, yaw=0.000
DXL write: 10.45deg -> ticks=... ; PCA write bend=60.00 yaw=135.00
```

**Note:** The default script is quiet for realtime responsiveness; enable verbose logging by adding prints where helpful (e.g., channel values, normalized, speeds, new angles).

---

# 10. Calibration procedure (step-by-step, safe)

Follow this exact order. Do not skip.

### 0. Preparations (bench)
- Remove any load from the arm (gripper removed or open).
- Place the arm in a visual safe zone (no obstacles).
- Have an accessible power kill switch.

### 1. Wiring & boot
- Wire everything (common ground). Power only the Pi initially; leave servos unpowered until later steps if you prefer.
- Ensure PCA9685 V+ connected to UBEC (5–6V), not the Pi 5V.
- Ensure receiver VCC and PCA9685 power are connected to proper supplies.

### 2. Software readiness
- Enable I2C and serial hardware. Reboot Pi.
- Install dependencies.
- Confirm `/dev/serial0` and `/dev/ttyUSB0` exist. Use `ls /dev/tty*`.

### 3. Dry run with no load
- Keep `SPEED` values small (e.g., `shoulder: 30`, `elbow_bend: 40`, `elbow_yaw: 40`).
- Run the script.
- Move transmitter sticks slightly and watch motor movement.

### 4. Centering & offsets
- Physically move arms to desired **logical zero** pose (e.g., arm straight forward).
- With code idle, read actual motor present positions:
  - For Dynamixel: use `packetHandler.read4ByteTxRx` at `ADDR_PRESENT_POSITION` to get ticks → convert to degrees. Or visually set servo horn to desired neutral and record where servos rest.
- Set `OFFSETS["shoulder"]`, `OFFSETS["elbow_bend"]`, `OFFSETS["elbow_yaw"]` to correct differences so logical `0` produces mechanical center.

### 5. Limits
- With small speeds, move sticks to extremes slowly.
- Observe mechanical stops and note where to clamp. Update `LIMITS` to safe min/max.

### 6. Horizon safety (optional)
- Enable `ENABLE_HORIZON_SAFETY = True`.
- Try moves that would push end effector below table; verify guard acts as intended.

### 7. Full-speed test
- Gradually increase `SPEED` values to desired responsiveness.
- If servos twitch or drop voltage, check UBEC and wiring.

### 8. Final verification
- Test sequences of realistic motions you'll use in operation with no obstacles.

---

# 11. Tuning parameters (recommended ranges & what changing them does)

- `LOOP_HZ`: 20–100. Default 50 Hz. Lower reduces CPU usage, increases control lag. 50 is a good balance.
- `DEADBAND`: 0.02–0.12. Higher reduces jitter but reduces sensitivity.
- `SPEED`:
  - Shoulder: 30–120 deg/s (start low).
  - Elbow bend: 40–150 deg/s.
  - Elbow yaw: 60–200 deg/s.
- `STEP_DELAY` — not in direct control code (used in earlier IK variant) — ignore.
- `FAILSAFE_SEC`: 0.3–1.0. Shorter reacts faster; longer tolerates occasional missing frames.
- `OFFSETS`: calibrate physically; small (±10s deg).
- `PCA pulse widths`: `500–2500 µs` widely used for 270° servos. If motion hits hard stops early, widen range cautiously; if jitter, narrow.

---

# 12. Safety checklist & fail-safes

**Before powering servos**
- UBEC rated for servo stall current (start with 5 A for single DS3225; for multiple servos ensure combined capability).
- Fuses in battery feed recommended.
- All grounds tied: Pi, PCA9685 ground, UBEC ground, Dynamixel ground.
- Level shifter on i-BUS in place (Pi RX must remain 3.3 V tolerant).
- Serial console disabled (so `/dev/serial0` free).

**Run-time safety**
- Keep an emergency kill switch accessible.
- Start with conservative `SPEED`.
- Verify `LIMITS` and `ENABLE_HORIZON_SAFETY`.
- Use `FAILSAFE_SEC` smaller than RC loss-to-fail duration or expand to auto-tuck.

**Power cut behavior**
- On script exit, code writes `TORQUE_ENABLE = 0` to Dynamixel and closes port. DS3225 servos remain powered by PCA9685 unless you explicit kill servo power. Use a physical power switch for total power cut.

---

# 13. Troubleshooting (symptoms, likely causes, fixes)

**No i-BUS frames / no movement**
- Check receiver bound to transmitter.
- Check level shifter, RX wiring to Pi.
- Confirm serial hardware enabled; ensure `/dev/serial0` exists.
- Test with `sudo cat /dev/serial0` (raw) or small Python serial reader to inspect bytes. Ensure receiver sends i-BUS.

**Servo jitter**
- Insufficient power supply; check UBEC voltage under load.
- No common ground between UBEC and Pi.
- If using RPi.GPIO PWM fallback: jitter is normal — prefer PCA9685.

**Dynamixel not responding**
- Wrong port or baud rate. Use `ls /dev/ttyUSB*`.
- Wrong protocol or control table — ensure Protocol 2.0 & addresses match motor.
- Torque disabled at motor config — check with Dynamixel utility or SDK read.

**Angles not aligned with sticks**
- Swap `CH_MAP` indices or set `REVERSE` True/False.
- Re-calibrate `OFFSETS`.

**Horizon safety blocking valid motion**
- Horizon check is conservative; you can tune `L1`, `L2` or disable `ENABLE_HORIZON_SAFETY`.

**High CPU usage**
- Lower `LOOP_HZ` to 30 or 20.
- Ensure ML inference and RC control don’t run in same priority; use separate processes.

---

# 14. Testing without hardware (simulators & unit tests)

If you want to test the control logic without hardware:

1. **Simulate i-BUS frames**: create a small Python script that opens `/dev/ttySx` or a pseudo-serial (pty) and writes valid 32-byte i-BUS frames periodically. Use the same checksum formula. Example skeleton:

```python
# pseudo_ibus_sender.py (very simple)
import serial, time, struct

ser = serial.Serial('/dev/pts/5', 115200)  # write to the slave end of a pty
def make_frame(values):
    # values: list of 14 ints (1000..2000)
    data = bytearray(32)
    data[0] = 0x20; data[1] = 0x40
    idx = 2
    for v in values:
        data[idx] = v & 0xFF
        data[idx+1] = (v >> 8) & 0xFF
        idx += 2
    csum = (0xFFFF - sum(data[0:30])) & 0xFFFF
    data[30] = csum & 0xFF; data[31] = (csum>>8)&0xFF
    return bytes(data)

vals_center = [1500]*14
while True:
    ser.write(make_frame(vals_center))
    time.sleep(0.02)
```

2. **Modify code to read from that pseudo-serial** (set `IBUS_SERIAL` to the pty corresponding end).

3. **Replace physical motor writes with logging functions** that print or write to a file instead of controlling hardware. Validate integration logic and limits.

---

# 15. Extensions (recommended future work)

- **Gripper channel** mapping to a DS3225 or other motor. Add button thresholds for open/close.
- **Mode switch channel**: add a channel to switch between `direct` and `IK` modes (the codebase already includes an IK script earlier).
- **Velocity mode on Dynamixel** (instead of position writes) for smoother continuous rotation if needed (requires different control + safety).
- **Telemetry & web UI**: run a lightweight web server showing current joint angles, channel values, and logs.
- **Robot Operating System (ROS)** node wrapping for easier integration.
- **Closed-loop checks**: read present position from DXL and verify successful motion; detect stalls.
- **E-stop GPIO**: map a hardware input to disable motors immediately.
- **Multithread ML**: run the ML model in a separate process to avoid interfering with control loop; use message queue (ZeroMQ) or sockets to pass target coordinates.

---

# 16. Notes, limitations, and final checklist

**Notes**
- This README and script assume a particular wiring & motor set. Double-check your actual pins and channels.
- The horizon safety is basic and heuristic; do not rely on it for collision avoidance beyond preventing z < 0.
- I cannot test on your hardware; please follow safety steps and test incrementally.

**Final pre-run checklist**
1. Confirm wiring & common ground.
2. Confirm UBEC can supply current.
3. Confirm PCA9685 address and I²C enable.
4. Disable Pi serial console; enable serial hardware.
5. Install dependencies.
6. Start script with small `SPEED` settings.
7. Keep emergency power kill ready.

---

# 17. Appendix — useful commands and reference

**Enable I²C & serial console settings**
```bash
sudo raspi-config
# Interfaces -> I2C -> Enable
# Interfaces -> Serial -> Disable shell, enable serial
sudo reboot
```

**Install deps**
```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-smbus i2c-tools
pip3 install adafruit-circuitpython-servokit pyserial dynamixel-sdk numpy
```

**List serial devices**
```bash
ls -l /dev/tty*
# Expect /dev/serial0 (alias for hardware UART) and /dev/ttyUSB0 for USB adapter
```

**Check I²C devices**
```bash
i2cdetect -y 1
# Should show 0x40 for PCA9685 by default
```

**Quick Dynamixel test (python)**
Use basic SDK commands to ping motor and read present position (consult Dynamixel SDK docs).

---

# Closing

This README is intentionally exhaustive. If you pass this and the `direct_rc_control.py` file to another engineer or to an autonomous agent, they should be able to:

- Read the wiring & hardware assumptions,
- Install dependencies,
- Run the program safely,
- Calibrate offsets and limits,
- Diagnose basic issues,
- Extend the program for additional features.

