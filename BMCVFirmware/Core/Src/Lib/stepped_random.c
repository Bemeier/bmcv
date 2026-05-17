#include "stepped_random.h"
#include "helpers.h"

float stepped_random(float phase, float shape, float mod)
{
    float m = 0.5f * (mod + 1.0f);
    float steps = 1.0f + m * 63.0f;

    float morph = 0.5f * (shape + 1.0f);
    float seedf = morph * 1024.0f;

    int seed0 = (int)seedf;
    int seed1 = seed0 + 1;

    float seed_t = smoothstep(fractf(seedf));

    float x = phase * steps;
    int i0 = (int)floorf(x);
    int i1 = i0 + 1;

    float t = smoothstep(fractf(x));
    //float t = fractf(x) * 0.2f;
    //float t = smoothstep(fractf(x)) * 0.35f;


    float a0 = hash11(i0 ^ (seed0 * 0x9E3779B9));
    float a1 = hash11(i1 ^ (seed0 * 0x9E3779B9));

    float b0 = hash11(i0 ^ (seed1 * 0x9E3779B9));
    float b1 = hash11(i1 ^ (seed1 * 0x9E3779B9));

    float va = lerp(a0, a1, t);
    float vb = lerp(b0, b1, t);

    return lerp(va, vb, seed_t);
}