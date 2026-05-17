import numpy as np
import soundfile as sf

N = 256                  # samples per wavetable
M = 13                   # source wavetable count
INTERP_STEPS = 8         # number of in-between frames

REMOVE_DC = True
ENFORCE_WRAP = True

def resample_periodic(data, n_samples):
    src_x = np.arange(len(data) + 1)
    src_y = np.concatenate([data, data[:1]])
    dst_x = np.linspace(0, len(data), n_samples, endpoint=False)
    return np.interp(dst_x, src_x, src_y)

def remove_dc(x):
    return x - np.mean(x)

def normalize(x):
    lo = np.min(x)
    hi = np.max(x)
    rng = hi - lo
    if rng < 1e-9:
        return np.zeros_like(x)
    y = 2.0 * ((x - lo) / rng) - 1.0
    return y

def enforce_continuity(x):
    avg = 0.5 * (x[0] + x[-1])
    x[0] = avg
    x[-1] = avg
    return x

source_tables = []

for i in range(M):
    data, sr = sf.read(f"./data/{i:02d}.wav")
    if data.ndim > 1:
        data = np.mean(data, axis=1)
    data = resample_periodic(data, N)
    if REMOVE_DC:
        data = remove_dc(data)
    data = normalize(data)
    if ENFORCE_WRAP:
        data = enforce_continuity(data)
    source_tables.append(data)

expanded_tables = []

for i in range(M):
    a = source_tables[i]
    b = source_tables[(i + 1) % M]
    for s in range(INTERP_STEPS + 1):
        t = s / (INTERP_STEPS + 1)
        wave = (1.0 - t) * a + t * b
        if REMOVE_DC:
            wave = remove_dc(wave)
        wave = normalize(wave)
        if ENFORCE_WRAP:
            wave = enforce_continuity(wave)
        expanded_tables.append(wave)

tables_i16 = []

for wave in expanded_tables:
    wave_i16 = np.int16(
        np.clip(
            wave * 32767,
            -32767,
            32767
        )
    )
    tables_i16.append(wave_i16)

tables = np.stack(tables_i16)
M_OUT = len(tables_i16)

with open("../BMCVFirmware/Core/Inc/wavetables.h", "w") as f:
    f.write("#ifndef INC_WAVETABLES_H_\n")
    f.write("#define INC_WAVETABLES_H_\n")
    f.write("#pragma once\n")
    f.write("#include <math.h>\n\n")
    f.write(f"#define N {N}\n")
    f.write(f"#define M {M_OUT}\n\n")
    f.write("const int16_t shape_table[M][N] = {\n")
    for row in tables:
        f.write("  { ")
        f.write(", ".join(map(str, row)))
        f.write(" },\n")
    f.write("};\n\n")
    
    f.write("#endif /* INC_WAVETABLES_H_ */")

print(f"Generated {M_OUT} wavetable frames.")