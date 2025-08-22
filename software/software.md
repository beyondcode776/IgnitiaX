# Wiring
## (A) Flysky FS-iA10B → Raspberry Pi

Receiver outputs PWM (50Hz, 1000–2000µs pulses).

Connect channel output pin → Raspberry Pi GPIO (must be PWM-capable OR read with pigpio library).

Receiver VCC (5V) → Pi 5V pin, GND → Pi GND.
---
 ## (B) DS3225 Servos → Raspberry Pi

Important: RPi GPIO cannot supply servo power. Use a separate 5–6V power supply (≥5A).

Servo Signal → Pi GPIO PWM pin (through software PWM).

Servo VCC → external supply.

Servo GND → Common with Raspberry Pi ground.
---
## (C) Dynamixel XL330 → Raspberry Pi

Uses half-duplex TTL UART (Protocol 2.0).

You’ll need:

U2D2 USB to Dynamixel adapter (recommended), OR

Build a half-duplex circuit using a single Pi UART (TX/RX merged).

Power: 5–6V external supply (shared with DS3225 supply).
Calibration Strategy
# Calibration Strategy
## Read raw receiver values using pigpio → get 1000–2000 µs range for each stick.

## Map receiver values → motor commands:

DS3225: map(1000–2000, 0–270°) → convert to PWM output.

Dynamixel: map(1000–2000, 0–4095) → convert to Dynamixel position.

## Test each motor individually first.

## Finally, create a main script that:

Reads 4 channels from Flysky.

Maps them to 3 DS3225 + 1 Dynamixel.