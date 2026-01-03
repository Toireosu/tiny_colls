#pragma once

namespace tiny_colls {
template <typename T>
struct AABB {
    T top;
    T bottom;
    T left;
    T right;
};

using AABB_f = AABB<float>;
using AABB_d = AABB<double>;
}