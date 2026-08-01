// Minimal mono 16-bit PCM WAV writer. No dependencies beyond stdio/stdint.
#ifndef BMCV_WAV_WRITER_H_
#define BMCV_WAV_WRITER_H_

#include <stdint.h>
#include <stdio.h>

static inline void wav_write_u32le(FILE* f, uint32_t v)
{
  uint8_t b[4] = {(uint8_t) v, (uint8_t) (v >> 8), (uint8_t) (v >> 16), (uint8_t) (v >> 24)};
  fwrite(b, 1, 4, f);
}

static inline void wav_write_u16le(FILE* f, uint16_t v)
{
  uint8_t b[2] = {(uint8_t) v, (uint8_t) (v >> 8)};
  fwrite(b, 1, 2, f);
}

// samples: mono float samples in roughly [-1, 1]; out-of-range values are clamped.
static inline int wav_write_mono_f32(const char* path, const float* samples, uint32_t count, uint32_t sample_rate)
{
  FILE* f = fopen(path, "wb");
  if (!f)
    return 0;

  const uint16_t bits_per_sample = 16;
  const uint16_t channels        = 1;
  const uint32_t byte_rate       = sample_rate * channels * (bits_per_sample / 8);
  const uint16_t block_align     = channels * (bits_per_sample / 8);
  const uint32_t data_bytes      = count * block_align;

  fwrite("RIFF", 1, 4, f);
  wav_write_u32le(f, 36 + data_bytes);
  fwrite("WAVE", 1, 4, f);

  fwrite("fmt ", 1, 4, f);
  wav_write_u32le(f, 16);
  wav_write_u16le(f, 1); // PCM
  wav_write_u16le(f, channels);
  wav_write_u32le(f, sample_rate);
  wav_write_u32le(f, byte_rate);
  wav_write_u16le(f, block_align);
  wav_write_u16le(f, bits_per_sample);

  fwrite("data", 1, 4, f);
  wav_write_u32le(f, data_bytes);

  for (uint32_t i = 0; i < count; i++)
  {
    float s = samples[i];
    if (s > 1.0f)
      s = 1.0f;
    if (s < -1.0f)
      s = -1.0f;
    wav_write_u16le(f, (int16_t) (s * 32767.0f));
  }

  fclose(f);
  return 1;
}

#endif /* BMCV_WAV_WRITER_H_ */
