#!/usr/bin/env python3
"""
direct_rc_control_fixed.py
Critical fixes applied:
 - non-blocking, buffered i-BUS parser (doesn't block main loop)
 - robust PCA9685 initialization with dummy fallback
 - Dynamixel port open/baud check and ping at startup (flags dxl_ok)
 - hardware calls wrapped in try/except to avoid crashes
 - safe channel access (index validation)
 - graceful shutdown: disable DXL torque and set servos to safe angles if possible
 - basic logging added

This file is intended to run **on the Raspberry Pi 4** with hardware attached.
Dependencies (on Pi):
  pip3 install adafruit-circuitpython-servokit pyserial dynamixel-sdk

Run:
  python3 direct_rc_control_fixed.py
"""
import time, math, sys
import logging
from typing import Optional, List

# ---------- CONFIG ----------
L1 = 70.0
L2 = 45.0

CH_MAP = {"elbow_yaw":0, "elbow_bend":1, "shoulder":2}
REVERSE = {"elbow_yaw":False, "elbow_bend":False, "shoulder":False}

SPEED = {"shoulder":80.0, "elbow_bend":120.0, "elbow_yaw":160.0}

LIMITS = {"shoulder":(-80.0,90.0), "elbow_bend":(0.0,180.0), "elbow_yaw":(-90.0,90.0)}
OFFSETS = {"shoulder":0.0, "elbow_bend":0.0, "elbow_yaw":0.0}

DEADBAND = 0.06
LOOP_HZ = 50.0
ENABLE_HORIZON_SAFETY = False

IBUS_SERIAL = "/dev/serial0"
IBUS_BAUD = 115200
PCA9685_ADDR = 0x40
BEND_CH = 0
YAW_CH  = 1

DXL_ID   = 1
DXL_PORT = "/dev/ttyUSB0"
DXL_BAUD = 57600

SAFE_BEND_ANGLE = 90.0
SAFE_YAW_ANGLE = 135.0  # servo-centered value used by code
SAFE_SHOULDER = 10.0

FAILSAFE_SEC = 0.5

# -------- logging ---------
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

# ---------- iBUS Reader (buffered, non-blocking) ----------
import serial
class IBus:
    """Buffered, non-blocking FlySky i-BUS reader (32-byte frames)."""
    def __init__(self, port:str, baud:int=115200, timeout:float=0.01):
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        except Exception as e:
            logging.exception("Failed to open serial port for iBUS (%s): %s", port, e)
            raise
        self.buf = bytearray()

    def read_channels(self) -> Optional[List[int]]:
        """Return list of channel u16 values, or None if no full frame available."""
        try:
            n = self.ser.in_waiting
            if n:
                self.buf += self.ser.read(n)
        except Exception as e:
            logging.exception("iBUS serial read error: %s", e)
            return None

        # parse as many full frames as possible
        while len(self.buf) >= 32:
            # find sync bytes at start
            if self.buf[0] != 0x20 or self.buf[1] != 0x40:
                # try to resync by discarding first byte
                self.buf.pop(0)
                continue
            frame = bytes(self.buf[:32])
            csum_rx = frame[30] | (frame[31] << 8)
            csum_calc = (0xFFFF - sum(frame[0:30])) & 0xFFFF
            if csum_rx != csum_calc:
                # corrupted frame - drop first byte and continue
                logging.debug("iBUS checksum mismatch; dropping byte and resyncing")
                self.buf.pop(0)
                continue
            # valid frame - parse channels
            ch = []
            for i in range(2, 30, 2):
                ch.append(frame[i] | (frame[i+1] << 8))
            # remove consumed frame from buffer
            del self.buf[:32]
            return ch
        return None

    @staticmethod
    def norm(v: int) -> float:
        x = (v - 1500.0) / 500.0
        return max(-1.0, min(1.0, x))

# ---------- Motors (init wrapped) ----------
pca_ok = False
kit = None
try:
    from adafruit_servokit import ServoKit
    kit = ServoKit(channels=16, address=PCA9685_ADDR)
    kit.frequency = 50
    # configure channels (may raise if hardware not present)
    kit.servo[BEND_CH].actuation_range = 270
    kit.servo[BEND_CH].set_pulse_width_range(500, 2500)
    kit.servo[YAW_CH].actuation_range = 270
    kit.servo[YAW_CH].set_pulse_width_range(500, 2500)
    pca_ok = True
    logging.info("PCA9685 (ServoKit) initialized at 0x%02X", PCA9685_ADDR)
except Exception as e:
    logging.exception("PCA9685 initialization failed - continuing in limited mode: %s", e)
    pca_ok = False
    class _DummyServo:
        def __init__(self): self.angle = None
    class _DummyKit:
        def __init__(self):
            self.servo = [_DummyServo() for _ in range(16)]
    kit = _DummyKit()

def pca_write_deg(channel:int, angle_deg:float):
    angle_deg = max(0.0, min(270.0, angle_deg))
    try:
        kit.servo[channel].angle = angle_deg
    except Exception as e:
        logging.exception("Failed to write PCA servo channel %d: %s", channel, e)

# Dynamixel setup
dxl_ok = False
try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
    portHandler = PortHandler(DXL_PORT)
    packetHandler = PacketHandler(2.0)
    if not portHandler.openPort():
        logging.error("Cannot open Dynamixel port %s", DXL_PORT)
    elif not portHandler.setBaudRate(DXL_BAUD):
        logging.error("Cannot set Dynamixel baud %d", DXL_BAUD)
    else:
        # ping to check motor presence
        try:
            # PacketHandler.ping returns (result, error) for protocol 2.0
            dxl_comm_result, dxl_error = packetHandler.ping(portHandler, DXL_ID)
            if dxl_comm_result != COMM_SUCCESS:
                logging.error("Dynamixel ping failed: %s", packetHandler.getTxRxResult(dxl_comm_result))
            else:
                logging.info("Dynamixel ID %d present", DXL_ID)
                dxl_ok = True
        except Exception as e:
            logging.exception("Dynamixel ping exception: %s", e)
except Exception as e:
    logging.exception("Dynamixel SDK init failed: %s", e)
    dxl_ok = False

def dxl_init_position_mode():
    if not dxl_ok:
        logging.warning("Skipping DXL init: dxl_ok is False")
        return
    try:
        packetHandler.write1ByteTxRx(portHandler, DXL_ID, 64, 0)  # torque off to change mode
        packetHandler.write1ByteTxRx(portHandler, DXL_ID, 11, 3)  # operating mode = 3 (position)
        packetHandler.write1ByteTxRx(portHandler, DXL_ID, 64, 1)  # torque on
        logging.info("DXL set to position mode and torque enabled")
    except Exception as e:
        logging.exception("DXL position mode init failed: %s", e)

def deg_to_dxl_ticks(deg: float) -> int:
    mdeg = (deg + OFFSETS["shoulder"] + 180.0) % 360.0
    return int((mdeg / 360.0) * 4095.0)

def dxl_write_deg(deg: float):
    if not dxl_ok:
        # ignore writes but log when verbose
        logging.debug("dxl_write_deg ignored (dxl_ok False): %.2f", deg)
        return
    try:
        ticks = deg_to_dxl_ticks(deg)
        packetHandler.write4ByteTxRx(portHandler, DXL_ID, 116, ticks)
    except Exception as e:
        logging.exception("Dynamixel write failed: %s", e)

if dxl_ok:
    dxl_init_position_mode()

# ---------- Helpers ----------
def clamp(v, lo, hi): return max(lo, min(hi, v))
def apply_limits(joint:str, angle:float) -> float:
    lo, hi = LIMITS[joint]; return clamp(angle, lo, hi)
def fk_z(shoulder_deg: float, elbow_deg: float, yaw_deg: float) -> float:
    sp = math.radians(shoulder_deg); ep = math.radians(elbow_deg)
    z1 = L1 * math.sin(sp); z2 = L2 * math.sin(sp + ep); return z1 + z2
def horizon_safe(ns: float, ne: float, ny: float) -> bool:
    return fk_z(ns, ne, ny) >= 0.0

# ---------- Main Control Loop ----------
def main():
    try:
        ibus = IBus(IBUS_SERIAL, IBUS_BAUD)
    except Exception:
        logging.error("IBus could not be initialized - exiting")
        return

    logging.info("iBUS listening on %s", IBUS_SERIAL)
    logging.info("PCA9685 at 0x%02X (ok=%s), Dynamixel on %s id=%d (ok=%s)",
                 PCA9685_ADDR, pca_ok, DXL_PORT, DXL_ID, dxl_ok)

    # Start pose
    shoulder = apply_limits("shoulder", SAFE_SHOULDER)
    elbow    = apply_limits("elbow_bend", SAFE_BEND_ANGLE)
    yaw      = apply_limits("elbow_yaw", 0.0)

    # Send initial positions (wrapped in try/except)
    try:
        dxl_write_deg(shoulder)
    except Exception:
        logging.exception("Initial DXL write failed")
    try:
        pca_write_deg(BEND_CH, elbow + OFFSETS["elbow_bend"])
        pca_write_deg(YAW_CH, SAFE_YAW_ANGLE + OFFSETS["elbow_yaw"])
    except Exception:
        logging.exception("Initial PCA writes failed")

    dt = 1.0 / LOOP_HZ
    t_last = time.time()
    failsafe_timer = time.time()
    failsafe_flag = False

    try:
        while True:
            now = time.time()
            if now - t_last < dt:
                time.sleep(dt - (now - t_last))
            t_last = time.time()

            ch = ibus.read_channels()
            if ch is not None:
                failsafe_timer = time.time()
                if failsafe_flag:
                    logging.info("iBUS resumed")
                    failsafe_flag = False

                def get_norm(name):
                    idx = CH_MAP.get(name, None)
                    if idx is None:
                        logging.error("CH_MAP missing entry for %s", name); return 0.0
                    if idx < 0 or idx >= len(ch):
                        # channel missing: treat as center (no input)
                        logging.debug("Channel %d missing in frame; using center", idx)
                        raw = 1500
                    else:
                        raw = ch[idx]
                    val = IBus.norm(raw)
                    if REVERSE.get(name, False):
                        val = -val
                    if abs(val) < DEADBAND:
                        val = 0.0
                    return val

                nspeed = get_norm("shoulder")   * SPEED["shoulder"]
                espeed = get_norm("elbow_bend") * SPEED["elbow_bend"]
                yspeed = get_norm("elbow_yaw")  * SPEED["elbow_yaw"]

                ns = shoulder + nspeed * dt
                ne = elbow    + espeed * dt
                ny = yaw      + yspeed * dt

                ns = apply_limits("shoulder", ns)
                ne = apply_limits("elbow_bend", ne)
                ny = apply_limits("elbow_yaw", ny)

                if ENABLE_HORIZON_SAFETY and not horizon_safe(ns, ne, ny):
                    if horizon_safe(ns, elbow, ny):
                        ne = elbow
                    elif horizon_safe(shoulder, ne, ny):
                        ns = shoulder
                    elif horizon_safe(ns, ne, yaw):
                        ny = yaw
                    else:
                        ns, ne, ny = shoulder, elbow, yaw

                # send commands (wrapped)
                try:
                    dxl_write_deg(ns)
                except Exception:
                    logging.exception("DXL send error")
                try:
                    pca_write_deg(BEND_CH, ne + OFFSETS["elbow_bend"])
                    pca_write_deg(YAW_CH, ny + 135.0 + OFFSETS["elbow_yaw"])
                except Exception:
                    logging.exception("PCA send error")

                shoulder, elbow, yaw = ns, ne, ny

            # Failsafe handling
            if time.time() - failsafe_timer > FAILSAFE_SEC:
                if not failsafe_flag:
                    logging.warning("Failsafe: no iBUS packet for %.3f s", FAILSAFE_SEC)
                    failsafe_flag = True
                # keep holding last position; optionally you could move to safe pose after longer timeout

    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received - shutting down")
    except Exception as e:
        logging.exception("Unhandled exception in main loop: %s", e)
    finally:
        logging.info("Shutting down: disabling torque (if available) and setting safe PCA positions")
        try:
            if dxl_ok:
                # disable torque
                packetHandler.write1ByteTxRx(portHandler, DXL_ID, 64, 0)
                logging.info("Dynamixel torque disabled")
        except Exception:
            logging.exception("Failed to disable Dynamixel torque")

        try:
            pca_write_deg(BEND_CH, SAFE_BEND_ANGLE)
            pca_write_deg(YAW_CH, SAFE_YAW_ANGLE)
            # small pause to let servos move before power-off
            time.sleep(0.2)
        except Exception:
            logging.exception("Failed during PCA shutdown")

        try:
            if dxl_ok:
                portHandler.closePort()
        except Exception:
            logging.exception("Failed to close DXL port")

if __name__ == "__main__":
    main()
