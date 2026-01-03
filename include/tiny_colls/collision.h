#pragma once

namespace tiny_colls {
template<typename T>
struct collision {
    T axis_x = T(0);
    T axis_y = T(0);
    T overlap = T(0); 
};

using collision_f = collision<float>;
using collision_d = collision<double>;
}