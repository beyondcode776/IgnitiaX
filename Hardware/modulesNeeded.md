# Note: suggested through chatgpt.

---

# 📦 Required Modules (per motor type)

### 1. **For Johnson 12V DC motors (x4, for rover movement)**

* You cannot drive them directly from Pi.
* You need **motor driver modules** that can handle **12 V, several amps**.

✅ **Recommended module:**

* **BTS7960 (43A H-bridge)** → 1 module can control 1 DC motor (both direction + speed via PWM).
* Since you have **4 motors**, you can either:

  * Use **4x BTS7960** (1 per motor, if you want full independent control), OR
  * Use **2x BTS7960** (1 for left side motors, 1 for right side motors — easier if wheels are paired per side).

---

### 2. **For DS3225MG 25kg Servos (x4, for base/joints/gripper)**

* These are hobby-grade PWM servos.
* The Pi **cannot generate stable PWM for many servos** on its own.

✅ **Required module:**

* **PCA9685 16-channel Servo Driver (I²C controlled)**

  * Handles all 4 servos easily.
  * Provides smooth PWM signals.
  * Needs a **separate 5–6 V power supply** capable of 10–15 A for all servos.

---

### 3. **For Dynamixel XL330-M288-T (x1, shoulder)**

* Dynamixel uses a **special half-duplex TTL serial protocol**, not standard hobby PWM.
* Needs both **communication interface** and **stable 5–6 V supply**.

✅ **Required module:**

* **Robotis U2D2 (USB to TTL Dynamixel adapter)** → safest and plug-and-play with Raspberry Pi.

  * Alternative: a **USB–TTL half-duplex transceiver module**, but U2D2 is much easier.

---

### 4. **For Flysky FS-iA10B receiver → Raspberry Pi**

* The receiver outputs **iBUS (serial)** or PWM per channel.
* Using **iBUS** is better (one wire, cleaner).

✅ **Required module:**

* No extra hardware needed — just wire the **iBUS pin (signal)** from FS-iA10B to **Pi’s UART RX (GPIO15)**.
* Make sure Pi and receiver **share GND**.
* If you prefer PWM channel reading instead of iBUS, then you’d need **a multi-channel PWM-to-USB adapter** or dedicate Pi GPIOs, but iBUS is far superior.

---

### 5. **For Power Distribution**

Since you’re mixing **12 V DC motors**, **5–6 V servos**, **5 V Dynamixel**, and **5 V Pi**, you need regulated supplies.

✅ **Required modules:**

* **Buck converters (DC–DC step-down modules)**:

  * One **5–6 V, 10–15 A buck** → for servo rail (PCA9685 + DS3225MGs)
  * One **5 V, 3 A buck** → for Raspberry Pi 4
  * One **5–6 V, 2 A buck** (can be same as servo supply if sized right) → for Dynamixel XL330

Optional but strongly recommended:

* **Power distribution board (PDB)** or fuse box
* **Large electrolytic capacitors (1000–2200 µF)** across servo rail to absorb spikes

---

# ✅ Final Shopping / Module List

* **4× Johnson motors:**

  * 2× or 4× **BTS7960 H-Bridge motor driver modules**

* **4× DS3225 servos:**

  * 1× **PCA9685 16-channel servo driver module**

* **1× XL330 Dynamixel servo:**

  * 1× **Robotis U2D2 USB-to-TTL adapter**

* **Flysky FS-iA10B receiver → Pi:**

  * Direct UART connection (no extra module)

* **Power:**

  * 2–3× **DC–DC Buck converter modules** (rated appropriately)
  * Optional: **Fuse box / PDB**
  * Large **capacitors** (servo rail)

---
