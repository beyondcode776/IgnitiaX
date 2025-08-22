#Read Flysky Receiver Channel (PWM Input)
import pigpio
import time

pi = pigpio.pi()
RX_PIN = 18  # GPIO pin connected to FS-iA10B channel output

def get_pulsewidth(gpio):
    return pi.get_servo_pulsewidth(gpio)

pi.set_mode(RX_PIN, pigpio.INPUT)

try:
    while True:
        pulse = get_pulsewidth(RX_PIN)  # should be ~1000–2000 µs
        print("Receiver:", pulse)
        time.sleep(0.05)
except KeyboardInterrupt:
    pi.stop()
# To run this code, ensure you have pigpio library installed and the pigpio daemon running.
# Control DS3225 Servo (PWM Output)
import pigpio
import time

pi = pigpio.pi()
SERVO_PIN = 17  # GPIO connected to servo signal

try:
    while True:
        for angle in range(0, 270, 10):
            pulse = int((angle / 270.0) * 1000 + 1000)  # map 0–270° → 1000–2000 µs
            pi.set_servo_pulsewidth(SERVO_PIN, pulse)
            time.sleep(0.1)

except KeyboardInterrupt:
    pi.set_servo_pulsewidth(SERVO_PIN, 0)  # stop
    pi.stop()
# Control Dynamixel XL330 (Using dynamixel_sdk)
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import time

# Control table addresses for XL330
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
LEN_GOAL_POSITION       = 4
LEN_PRESENT_POSITION    = 4

PROTOCOL_VERSION        = 2.0
DXL_ID                  = 1
BAUDRATE                = 57600
DEVICENAME              = '/dev/ttyUSB0'   # U2D2 adapter

TORQUE_ENABLE           = 1
TORQUE_DISABLE          = 0

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# Enable torque
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

# Move Dynamixel between positions
try:
    while True:
        packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, 0)       # 0°
        time.sleep(2)
        packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, 2048)    # mid
        time.sleep(2)
        packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, 4095)    # max
        time.sleep(2)

except KeyboardInterrupt:
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()
# To run this code, ensure you have the dynamixel_sdk library installed and the U2D2 adapter connected.