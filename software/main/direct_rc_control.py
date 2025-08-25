#!/usr/bin/env python3
"""
Direct RC → Arm Control (no IK)
- Reads FlySky iBUS from FS-iA10B via Pi UART (/dev/serial0)
- Maps channels to joint velocities (deg/s): shoulder (Dynamixel XL330), elbow bend (DS3225 on PCA9685), elbow yaw (DS3225 on PCA9685)
- Integrates velocities to angles with limits and sends commands.

WIRING (per your diagram):
- Receiver iBUS signal -> level shifter HV -> LV -> Raspberry Pi GPIO15 (RXD). 5V to receiver VCC, GND common.
- PCA9685 (0x40): SDA->GPIO2 (pin 3), SCL->GPIO3 (pin 5), V+ = 6V from UBEC, GND common.
- DS3225 servos to PCA9685 channels (default: bend=0, yaw=1).
- Dynamixel XL330 via USB-TTL to /dev/ttyUSB0 (Protocol 2.0).

Enable I2C and disable serial console; leave serial port enabled. Install deps:
  sudo apt-get install -y python3-smbus i2c-tools
  pip3 install adafruit-circuitpython-servokit pyserial dynamixel-sdk

Run:
  python3 direct_rc_control.py
Press Ctrl+C to stop.
"""
import time, math, struct, sys
from typing import Optional, List

# ---------- CONFIG ----------
# Geometry (used only if you enable horizon safety)
L1 = 70.0  # cm (shoulder->elbow)
L2 = 45.0  # cm (elbow->wrist)

# Channel mapping (0-based iBUS channel indices)
CH_MAP = {
    "elbow_yaw":   0,   # usually roll
    "elbow_bend":  1,   # usually pitch
    "shoulder":    2,   # throttle
    # Add 'gripper': 4, etc. if needed
}
# Reverse directions if your sticks feel backwards
REVERSE = {
    "elbow_yaw":  False,
    "elbow_bend": False,
    "shoulder":   False,
}

# Speed scales (deg/sec at full stick)
SPEED = {
    "shoulder":   80.0,
    "elbow_bend": 120.0,
    "elbow_yaw":  160.0,
}

# Joint limits (deg) — tune for your build
LIMITS = {
    "shoulder":   (-80.0,  90.0),
    "elbow_bend": (  0.0, 180.0),
    "elbow_yaw":  (-90.0,  90.0),  # logical yaw, servo will be centered at 135
}

# Servo offsets (deg) after mechanical calibration
OFFSETS = {
    "shoulder":   0.0,    # adds to logical shoulder angle
    "elbow_bend": 0.0,    # adds to PCA9685 elbow bend
    "elbow_yaw":  0.0,    # adds to (yaw + 135)
}

# Safety / smoothing
DEADBAND = 0.06           # stick deadband in normalized units (~60 µs)
LOOP_HZ  = 50.0
ENABLE_HORIZON_SAFETY = False  # set True to prevent z<0 based on FK

# Hardware configuration
IBUS_SERIAL = "/dev/serial0"    # GPIO15 RXD (after enabling UART)
IBUS_BAUD   = 115200
PCA9685_ADDR = 0x40
BEND_CH = 0
YAW_CH  = 1

DXL_ID   = 1
DXL_PORT = "/dev/ttyUSB0"
DXL_BAUD = 57600
# ---------------------------

# ---------- iBUS Reader ----------
import serial
class IBus:
    """Minimal FlySky iBUS receiver parser (32-byte frames)."""
    def __init__(self, port:str, baud:int=115200, timeout:float=0.02):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)

    def read_channels(self) -> Optional[List[int]]:
        # Packet: 0x20 0x40 [CH1 lo hi] ... [CH14 lo hi] [CHK lo hi]
        while True:
            b = self.ser.read(1)
            if not b:
                return None
            if b[0] != 0x20:
                continue
            rest = self.ser.read(31)
            if len(rest) != 31:
                return None
            if rest[0] != 0x40:
                continue
            data = b + rest  # 32 bytes
            # checksum
            csum_rx = data[30] | (data[31] << 8)
            csum_calc = (0xFFFF - sum(data[0:30])) & 0xFFFF
            if csum_rx != csum_calc:
                continue
            # parse channels (14 available, we may use first 6)
            ch = []
            for i in range(2, 30, 2):
                val = data[i] | (data[i+1] << 8)
                ch.append(val)   # 1000..2000 µs typical
            return ch

    @staticmethod
    def norm(v: int) -> float:
        """Map iBUS value (~1000..2000) to -1..+1 with clamp."""
        x = (v - 1500.0) / 500.0
        return max(-1.0, min(1.0, x))

# ---------- Motors ----------
from adafruit_servokit import ServoKit
from dynamixel_sdk import PortHandler, PacketHandler

# PCA9685 for DS3225
kit = ServoKit(channels=16, address=PCA9685_ADDR)
kit.frequency = 50
# Configure channels
kit.servo[BEND_CH].actuation_range = 270
kit.servo[BEND_CH].set_pulse_width_range(500, 2500)
kit.servo[YAW_CH].actuation_range = 270
kit.servo[YAW_CH].set_pulse_width_range(500, 2500)

def pca_write_deg(channel:int, angle_deg:float):
    # clamp to [0,270]
    angle_deg = max(0.0, min(270.0, angle_deg))
    kit.servo[channel].angle = angle_deg

# Dynamixel XL330 in Position Mode
DXL_PROTOCOL = 2.0
ADDR_OPERATING_MODE  = 11   # 3 = position
ADDR_TORQUE_ENABLE   = 64
ADDR_GOAL_POSITION   = 116  # 0..4095
ADDR_PRESENT_POSITION= 132

portHandler = PortHandler(DXL_PORT)
packetHandler = PacketHandler(DXL_PROTOCOL)
if not portHandler.openPort():
    print("ERROR: cannot open", DXL_PORT); sys.exit(1)
if not portHandler.setBaudRate(DXL_BAUD):
    print("ERROR: cannot set baud", DXL_BAUD); sys.exit(1)

def dxl_init_position_mode():
    # Disable torque to change mode
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 3)  # position
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

def deg_to_dxl_ticks(deg: float) -> int:
    # Logical shoulder degrees (-80..90) + offset -> 0..360 -> ticks (0..4095)
    mdeg = (deg + OFFSETS["shoulder"] + 180.0) % 360.0
    return int((mdeg / 360.0) * 4095.0)

def dxl_write_deg(deg: float):
    ticks = deg_to_dxl_ticks(deg)
    packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, ticks)

dxl_init_position_mode()

# ---------- Helpers ----------
def clamp(v, lo, hi): return max(lo, min(hi, v))

def apply_limits(joint:str, angle:float) -> float:
    lo, hi = LIMITS[joint]
    return clamp(angle, lo, hi)

def fk_z(shoulder_deg: float, elbow_deg: float, yaw_deg: float) -> float:
    """Return Z height in cm for safety check (yaw doesn't change Z)."""
    sp = math.radians(shoulder_deg)
    ep = math.radians(elbow_deg)
    z1 = L1 * math.sin(sp)
    z2 = L2 * math.sin(sp + ep)
    return z1 + z2

def horizon_safe(ns: float, ne: float, ny: float) -> bool:
    return fk_z(ns, ne, ny) >= 0.0

# ---------- Main Control Loop ----------
def main():
    ibus = IBus(IBUS_SERIAL, IBUS_BAUD)
    print("iBUS listening on", IBUS_SERIAL)
    print("PCA9685 ready at 0x%02X, Dynamixel on %s id=%d" % (PCA9685_ADDR, DXL_PORT, DXL_ID))

    # Start pose
    shoulder = apply_limits("shoulder", 10.0)
    elbow    = apply_limits("elbow_bend", 60.0)
    yaw      = apply_limits("elbow_yaw", 0.0)

    # Send initial positions
    dxl_write_deg(shoulder)
    pca_write_deg(BEND_CH, elbow + OFFSETS["elbow_bend"])
    pca_write_deg(YAW_CH,  yaw + 135.0 + OFFSETS["elbow_yaw"])

    dt = 1.0 / LOOP_HZ
    t_last = time.time()
    failsafe_timer = time.time()
    FAILSAFE_SEC = 0.5

    try:
        while True:
            now = time.time()
            # keep fixed loop timing
            if now - t_last < dt:
                time.sleep(dt - (now - t_last))
            t_last = time.time()

            ch = ibus.read_channels()
            if ch is not None:
                failsafe_timer = time.time()

                # normalize channels
                def get_norm(name):
                    idx = CH_MAP[name]
                    val = IBus.norm(ch[idx])
                    if REVERSE[name]:
                        val = -val
                    # deadband
                    if abs(val) < DEADBAND:
                        val = 0.0
                    return val

                nspeed = get_norm("shoulder")   * SPEED["shoulder"]
                espeed = get_norm("elbow_bend") * SPEED["elbow_bend"]
                yspeed = get_norm("elbow_yaw")  * SPEED["elbow_yaw"]

                # integrate speeds
                ns = shoulder + nspeed * dt
                ne = elbow    + espeed * dt
                ny = yaw      + yspeed * dt

                # joint limits
                ns = apply_limits("shoulder", ns)
                ne = apply_limits("elbow_bend", ne)
                ny = apply_limits("elbow_yaw", ny)

                # optional horizon safety
                if ENABLE_HORIZON_SAFETY and not horizon_safe(ns, ne, ny):
                    # simple heuristic: freeze elbow change first
                    if horizon_safe(ns, elbow, ny):
                        ne = elbow
                    elif horizon_safe(shoulder, ne, ny):
                        ns = shoulder
                    elif horizon_safe(ns, ne, yaw):
                        ny = yaw
                    else:
                        ns, ne, ny = shoulder, elbow, yaw

                # send
                dxl_write_deg(ns)
                pca_write_deg(BEND_CH, ne + OFFSETS["elbow_bend"])
                pca_write_deg(YAW_CH,  ny + 135.0 + OFFSETS["elbow_yaw"])

                # commit
                shoulder, elbow, yaw = ns, ne, ny

            # Fail-safe: if no fresh packet, hold position
            if time.time() - failsafe_timer > FAILSAFE_SEC:
                # (optional) you could slowly return to a safe pose
                pass

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Torque off
        packetHandler.write1ByteTxRx(portHandler, DXL_ID, 64, 0)
        portHandler.closePort()

if __name__ == "__main__":
    main()
