import re
import matplotlib.pyplot as plt

accel = []
gyro = []
alt = []
height = []

fs = 100               # delay_ms=10 -> ~100 Hz
window_seconds = 5
window_size = fs * window_seconds

plt.ion()
fig, axs = plt.subplots(4, 1, figsize=(10, 10))

# Line 1 in your sprintf:
# "Accel: %.2f | g_mag: %.2f | Alt: %.2f m | dAlt/dt: %.2f m/s"
p_line1 = re.compile(
    r'Accel:\s*([\d\.\-]+)\s*\|\s*g_mag:\s*([\d\.\-]+)\s*\|\s*Alt:\s*([\d\.\-]+)'
)

# Line 2 in your sprintf contains HeightDrop:
p_height = re.compile(r'HeightDrop:\s*([\d\.\-]+)')

while True:
    accel.clear()
    gyro.clear()
    alt.clear()
    height.clear()

    with open("teraterm.log", "r") as f:
        for line in f:
            m1 = p_line1.search(line)
            if m1:
                accel.append(float(m1.group(1)))
                gyro.append(float(m1.group(2)))
                alt.append(float(m1.group(3)))
                continue

            m2 = p_height.search(line)
            if m2:
                height.append(float(m2.group(1)))

    # HeightDrop is printed on a different line, so align by the shortest list
    n = min(len(accel), len(gyro), len(alt), len(height))
    if n == 0:
        plt.pause(0.5)
        continue

    k = min(window_size, n)

    accel_w = accel[-k:]
    gyro_w  = gyro[-k:]
    alt_w   = alt[-k:]
    h_w     = height[-k:]

    t = [i / fs for i in range(k)]

    for ax in axs:
        ax.clear()

    # Acceleration
    axs[0].plot(t, accel_w, color="blue")
    axs[0].set_title("Acceleration (Last 5s)")
    axs[0].set_ylabel("m/s²")
    axs[0].set_xlim(0, window_seconds)
    axs[0].set_ylim(0, 15)
    axs[0].grid(True)

    # Gyro magnitude
    axs[1].plot(t, gyro_w, color="red")
    axs[1].set_title("Gyro Magnitude (Last 5s)")
    axs[1].set_ylabel("dps")
    axs[1].set_xlim(0, window_seconds)
    axs[1].set_ylim(0, 300)
    axs[1].grid(True)

    # HeightDrop
    axs[2].plot(t, h_w, color="green")
    axs[2].set_title("Height Drop (Last 5s)")
    axs[2].set_ylabel("m")
    axs[2].set_xlim(0, window_seconds)
    axs[2].set_ylim(0, 2)
    axs[2].grid(True)

    # Altitude
    axs[3].plot(t, alt_w, color="purple")
    axs[3].set_title("Altitude (Last 5s)")
    axs[3].set_ylabel("m")
    axs[3].set_xlabel("Time (s)")
    axs[3].set_xlim(0, window_seconds)
    axs[3].grid(True)

    plt.pause(0.5)