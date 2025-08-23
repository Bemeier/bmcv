import numpy as np
import soundfile as sf

N = 256
M = 8
tables = []

for i in range(M):
    # load slice (single cycle WAV)
    data, sr = sf.read(f"./data/{i:02d}.wav")
    # resample / trim to exactly N samples
    data = np.interp(np.linspace(0, len(data), N, endpoint=False),
                     np.arange(len(data)), data)
    # normalize to -32767..32767
    data_i16 = np.int16(np.clip(data / np.max(np.abs(data)) * 32767, -32767, 32767))
    tables.append(data_i16)

tables = np.stack(tables)

# Write to header file
with open("wavetables.h", "w") as f:
    f.write("#pragma once\n\n")
    f.write(f"#define N {N}\n#define M {M}\n\n")
    f.write("const int16_t shape_table[M][N] = {\n")
    for row in tables:
        f.write("  { " + ", ".join(map(str, row)) + " },\n")
    f.write("};\n")