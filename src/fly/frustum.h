// View-frustum culling of world-space boxes, from a view-projection matrix (Gribb and Hartmann, "Fast Extraction of Viewing Frustum
// Planes from the World-View-Projection Matrix", 2001). Side planes and "in front of the eye" only: the far plane culls nothing here.
// Platform-free game code.
#pragma once
#include "../vecmath.h"

namespace fly {

struct Frustum {
    float p[5][4] = {};

    explicit Frustum(const tbd::Mat4& vp) {
        const auto row = [&](int r, int c) { return vp.At(r, c); };
        for (int c = 0; c < 4; ++c) {
            p[0][c] = row(3, c) + row(0, c); // left
            p[1][c] = row(3, c) - row(0, c); // right
            p[2][c] = row(3, c) + row(1, c); // bottom
            p[3][c] = row(3, c) - row(1, c); // top
            p[4][c] = row(3, c);             // w > 0: in front of the eye
        }
    }

    // Whether any of the box [lo, hi] can be on screen: for each plane, the box's corner furthest along its normal.
    bool BoxVisible(tbd::Vec3 lo, tbd::Vec3 hi) const {
        for (const auto& q : p) {
            const float x = q[0] >= 0.0f ? hi.x : lo.x, y = q[1] >= 0.0f ? hi.y : lo.y, z = q[2] >= 0.0f ? hi.z : lo.z;
            if (q[0] * x + q[1] * y + q[2] * z + q[3] < 0.0f) return false;
        }
        return true;
    }
};

} // namespace fly
