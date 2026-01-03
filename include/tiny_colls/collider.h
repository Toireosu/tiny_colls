#pragma once

#include <vector>
#include <numbers>
#include <cmath>
#include <cassert>
#include <array>
#include <memory>
#include "tiny_colls/collision.h"
#include "tiny_colls/point.h"
#include "tiny_colls/aabb.h"

namespace tiny_colls {
template<typename T>
class collider {
    static_assert(std::is_floating_point<T>::value, "collider<T>: T must be floating point");
public:
    collider(const collider& c);
    collider() noexcept = default;
    collider& operator=(const collider& c);
    collider(collider&&) noexcept = default;
    collider& operator=(collider&&) noexcept = default;

    collider& set_position(T x, T y);
    collider& set_rotation(T rotation);

    std::vector<point<T>> get_shape() const;
    AABB<T> get_bounding_box() const;
    // RAW FORMAT
    // i = 0:   position x
    // i = 1:   position y
    // i = 2:   rotation
    // i = 3..n: vertices (i: x, i + 1: y)
    std::vector<T> get_raw() const;

    bool is_point_in(T x, T y);
    bool is_colliding_with(const collider& other, collision<T>& out);

    static void set_ellipse_vertex_count(int count);

    static collider rect(T width, T height);
    static collider poly(T width, T height, int N);
    static collider ellipse(T a, T b);
    static collider circle(T radius);
    static collider capsule(T width, T height);
    static collider line(T length);
    static collider rounded_rect(T width, T height, T roundness);
    static collider from_points(const std::vector<point<T>>& points);
    // RAW FORMAT
    // i = 0:   position x
    // i = 1:   position y
    // i = 2:   rotation
    // i = 3..n: vertices (i: x, i + 1: y)
    static collider raw(const std::vector<T>& data);
private:    
    static int ellipse_vertex_count;
    
    struct Impl;
    collider(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl;
};

using collider_f = collider<float>;
using collider_d = collider<double>;

}

#include "tiny_colls/details/collider_impl.h"