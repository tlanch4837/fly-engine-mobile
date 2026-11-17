// The ocean's compute passes at 64 x 64 (ocean.h OceanSettings::tier 2, the flight tier: the sea seen from the air): water.hlsl with
// a smaller FFT. See water_m.hlsl.
#define OCEAN_N 64
#include "water.hlsl"
