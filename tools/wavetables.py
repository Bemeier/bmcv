import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parameters
TABLE_SIZE = 256
FRAMES = 100
MORPH_SPEED = 1 / FRAMES  # How much to increment t per frame

# Define two waveforms to morph between
x = np.linspace(0, 1, TABLE_SIZE, endpoint=False)
wave_a = 2 * x - 1                    # Saw
wave_b = np.sin(2 * np.pi * x) # sin
wave_c = 2 * np.abs(2 * x - 1) - 1        # Tri
wave_d = 1 - 2 * x  # saw rev
wave_e = np.sign(np.sin(2 * np.pi * x + np.pi)) # sqe

waves = [wave_a, wave_b, wave_c, wave_d, wave_e ]

# Setup the plot
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_xlim(0, TABLE_SIZE)
ax.set_ylim(-1.2, 1.2)
ax.set_title("Waveform Morphing (Sine → Square)")

# Animation update function
def update(frame):
    t_all = frame * MORPH_SPEED
    wave_index = int(np.floor(t_all * len(waves)))
    num_pairs = len(waves)
    pair_index = int(np.floor(t_all * num_pairs))
    t_current = (t_all * num_pairs) % 1
    
    wave_index_1 = pair_index % num_pairs
    wave_index_2 = (wave_index_1 + 1) % num_pairs
    interp_wave = (1 - t_current) * waves[wave_index_1] + t_current * waves[wave_index_2]
    
    line.set_data(np.arange(TABLE_SIZE), interp_wave)
    return line,

# Create the animation
ani = FuncAnimation(fig, update, frames=FRAMES, interval=50, blit=True, repeat=True)
plt.show()
