#pragma once

namespace tiny_colls {
template <typename T>
struct point {
    T x;
    T y;
};

using point_f = point<float>;
using point_d = point<double>;
}