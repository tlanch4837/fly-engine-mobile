// The ocean's compute passes for the mobile tier (ocean.h OceanSettings::tier 1: phones, standalone headsets): water.hlsl with
// 128 x 128 cascades. The FFT's size is fixed when a compute shader compiles, so it gets its own file; the water pass itself reads
// the grid size from the shore buffer's header and is the same shader for both tiers.
#define OCEAN_N 128
#include "water.hlsl"
