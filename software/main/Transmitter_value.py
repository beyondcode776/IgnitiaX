import pigpio
import time
CHANNEL_PINS = {
    "CH1": 17,
    "CH2": 27,
    "CH3": 22,
    "CH4": 23,
    "CH5": 24,
    "CH6": 25
}
pi = pigpio.pi()
if not pi.connected:
    exit(0)
channel_values = {ch: 1500 for ch in CHANNEL_PINS}
def callback(channel, level, tick, ch_name):
    global channel_values
    if level == 1:  # Rising edge
        channel_values[ch_name + "_tick"] = tick
    elif level == 0:  # Falling edge
        width = pigpio.tickDiff(channel_values[ch_name + "_tick"], tick)
        channel_values[ch_name] = width
for ch_name, pin in CHANNEL_PINS.items():
    pi.set_mode(pin, pigpio.INPUT)
    pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
    pi.callback(pin, pigpio.EITHER_EDGE, lambda g, l, t, ch=ch_name: callback(g, l, t, ch))
print("Reading all 6 channels... Press CTRL+C to stop.")
try:
    while True:
        text_output = []
        for ch_name in CHANNEL_PINS:
            val = channel_values[ch_name]
            
            # Convert values into text
            if val < 1200:
                status = "LOW"
            elif val > 1800:
                status = "HIGH"
            else:
                status = "MID"
            
            text_output.append(f"{ch_name}: {val} µs ({status})")
        
        print(" | ".join(text_output))
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Exiting...")
    pi.stop()
