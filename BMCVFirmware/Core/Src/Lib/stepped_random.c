#include "stepped_random.h"

#include <stdint.h>
#include "helpers.h"

float stepped_random(float phase, float shape)
{
    const int STEPS = 32;

    // map shape to continuous morph coordinate
    float morph = (shape + 1.0f) * 0.5f;

    // convert morph into floating seed-space
    float seedf = morph * 1024.0f;

    int seed0 = (int)seedf;
    int seed1 = seed0 + 1;

    float seed_t = smoothstep(fractf(seedf));

    // phase -> step domain
    float x = phase * (float)STEPS;

    int i0 = (int)x;
    int i1 = (i0 + 1) % STEPS;

    //float t = smoothstep(fractf(x));
    //float t = fractf(x) * 0.2f;
    float t = smoothstep(fractf(x)) * 0.35f;

    // two neighboring random worlds
    float a0 = hash11(i0 ^ (seed0 * 0x9E3779B9));
    float a1 = hash11(i1 ^ (seed0 * 0x9E3779B9));

    float b0 = hash11(i0 ^ (seed1 * 0x9E3779B9));
    float b1 = hash11(i1 ^ (seed1 * 0x9E3779B9));

    // interpolate inside each world
    float va = lerp(a0, a1, t);
    float vb = lerp(b0, b1, t);

    // morph between worlds
    return lerp(va, vb, seed_t);
}