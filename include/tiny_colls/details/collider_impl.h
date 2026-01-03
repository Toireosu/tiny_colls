#pragma once

#include <vector>
#include <numbers>
#include <cmath>
#include <cassert>
#include <algorithm>
#include <array>
#include "tiny_colls/details/vec.h"
#include "tiny_colls/details/proj.h"
#include "tiny_colls/collision.h"

namespace tiny_colls {
using details::vec;
using details::proj;

template<typename T>
struct collider<T>::Impl {
    Impl(std::vector<vec<T>> vertices)    
        : vertices(vertices), position(0, 0), rotation(0.0f) { 
        transform();
    }
    
    static std::vector<vec<T>> calculate_axes(std::vector<vec<T>> vertices) {
        std::vector<vec<T>> axes;
        
        for (size_t i = 0; i < vertices.size(); ++i) {
            auto a = vertices[i];
            auto b = vertices[(i + 1) % vertices.size()];

            vec<T> edge = b - a;
            if (edge.dot(edge) < 1e-7)
                continue;

            axes.push_back(edge.perp().normalize());
        } 

        return axes;
    }

    void transform() {
        t_vertices.clear();
        t_axes.clear();

        for (const auto& v : vertices) {
            t_vertices.push_back(v.rotate(this->rotation) + this->position); 
        }

        t_axes = calculate_axes(t_vertices);

        proj<T> x_aabb = project(vec<T>(1, 0));
        proj<T> y_aabb = project(vec<T>(0, 1));
    
        t_aabb.top = y_aabb.max;
        t_aabb.bottom = y_aabb.min;
        t_aabb.left = x_aabb.min;
        t_aabb.right = x_aabb.max;
    }

    const std::vector<vec<T>>& get_axes() const { return t_axes; }
    proj<T> project(const vec<T>& axis) const {
        // TODO 
        T min = std::numeric_limits<T>::max();
        T max = std::numeric_limits<T>::lowest();

        for (const auto& v : this->t_vertices) {
            T dot = axis.dot(v);

            if (dot < min)
                min = dot;
            if (dot > max)
                max = dot;
        }

        return proj<T>(min, max);
    }

    void ensure_transformed() {
        if (!dirty) return;
        transform();
        dirty = false;
    }

    std::vector<vec<T>> vertices;
    vec<T> position;
    T rotation;
    
    std::vector<vec<T>> t_vertices;
    std::vector<vec<T>> t_axes;
    AABB<T> t_aabb;

    bool dirty = false;
};


template<typename T>
collider<T>::collider(const collider& c) : impl(c.impl ? std::make_unique<Impl>(*c.impl) : nullptr) { }

template<typename T>
collider<T>& collider<T>::operator=(const collider& c) {
    if (this == &c) {
        return *this;
    }
    
    impl = c.impl ? std::make_unique<Impl>(*c.impl) : nullptr;
    return *this;
}

template<typename T>
collider<T>& collider<T>::set_position(T x, T y) {
    if (!this->impl) {
        throw std::logic_error("Trying to set position on non-initialized collider.");
    }
    this->impl->position.x = x;
    this->impl->position.y = y;
    this->impl->dirty = true;
    return *this;
}

template <typename T>
collider<T>& collider<T>::set_rotation(T rotation) {
    if (!this->impl) {
        throw std::logic_error("Trying to set rotation on non-initialized collider.");
    }
    this->impl->rotation = rotation;
    this->impl->dirty = true;
    return *this;
};

template <typename T>
std::vector<point<T>> collider<T>::get_shape() const { 
    if (!this->impl) {
        throw std::logic_error("Cannot get shape from non-initialized collider.");
    }
    impl->ensure_transformed();

    std::vector<point<T>> shape;
    for (auto vert : impl->t_vertices) {
        shape.push_back({ vert.x, vert.y });
    }
    
    return shape; 
} 

template <typename T>
AABB<T> collider<T>::get_bounding_box() const {
    if (!this->impl) {
        throw std::logic_error("Cannot get bounding box from non-initialized collider.");
    }
    impl->ensure_transformed();

    return impl->t_aabb;
};

// 0: position x
// 1: position y
// 2: rotation
// 3-n: vertices
template <typename T>
std::vector<T> collider<T>::get_raw() const {
    if (!this->impl) {
        throw std::logic_error("Cannot get raw data from non-initialized collider.");
    }
    std::vector<T> raw;
    raw.reserve(3 + impl->vertices.size() * T(2));
    
    raw.push_back(impl->position.x);
    raw.push_back(impl->position.y);
    
    raw.push_back(impl->rotation);

    for (auto& v : impl->vertices) {
        raw.push_back(v.x);
        raw.push_back(v.y);
    }

    return raw;
}

template <typename T>
bool collider<T>::is_point_in(T x, T y) {
    if (!this->impl) {
        throw std::logic_error("Cannot check is point in on non-initialized collider.");
    }
    impl->ensure_transformed();

    const std::vector<vec<T>>& axes = impl->get_axes();
    vec<T> point = vec<T>(x, y);

    for (auto& axis : axes) {
        proj<T> this_proj = impl->project(axis);
        T point_d = axis.dot(point);

        if (this_proj.max < point_d || point_d < this_proj.min) {
            return false;
        }
    }

    return true;
}

template <typename T>
bool collider<T>::is_colliding_with(const collider<T>& other, collision<T>& out) {
    if (!this->impl || !other.impl) {
        throw std::logic_error("Cannot check collision on non-initialized collider.");
    }
    
    if (this == &other) return false;

    this->impl->ensure_transformed();
    other.impl->ensure_transformed();


    std::vector<vec<T>> axes = this->impl->get_axes();
    std::vector<vec<T>> other_axes = other.impl->get_axes();

    axes.insert(axes.end(), other_axes.begin(), other_axes.end());

    if (axes.empty()) return false; // Nothing to check?

    T smallest_overlap = std::numeric_limits<T>::max();
    vec<T> overlap_axis(0, 0);
    for (auto& axis : axes) {
        proj<T> this_proj = this->impl->project(axis);
        proj<T> other_proj = other.impl->project(axis);

        if (this_proj.max < other_proj.min || other_proj.max < this_proj.min) {
            return false;
        } else {
            T overlap0 = this_proj.max - other_proj.min;
            T overlap1 = other_proj.max - this_proj.min;

            T overlap = (overlap0 < overlap1) ? overlap0 : -overlap1;
            if (std::abs(overlap) < std::abs(smallest_overlap)) {
                smallest_overlap = overlap;
                overlap_axis = axis;
            }
        }
    }

    vec<T> delta = this->impl->position - other.impl->position;

    if (delta.dot(overlap_axis) < 0) {
        overlap_axis = -overlap_axis;
    }

    out = collision<T> { overlap_axis.x, overlap_axis.y, smallest_overlap };
    return true;
}

template <typename T>
void collider<T>::set_ellipse_vertex_count(int count) { 
    if (count < 8) {
        throw std::invalid_argument("Ellipse vertex count must be at least 8");
    } 
    ellipse_vertex_count = std::max(count, 8); 
}

template <typename T>
collider<T> collider<T>::rect(T width, T height) {
    T half_width = width / T(2); 
    T half_height = height / T(2);

    return collider(
        std::make_unique<Impl>(
            std::vector<vec<T>> { 
                vec<T>(-half_width, -half_height), vec<T>(half_width, -half_height),
                    vec<T>(half_width, half_height), vec<T>(-half_width, half_height),
            }
        )
    );
}

template <typename T>
collider<T> collider<T>::poly(T width, T height, int N) {
    if (N <= 2) {
        throw std::invalid_argument("Vertex count must be 3 or higher!");
    }

    T step = T(2) * std::numbers::pi_v<T> / T(N);

    std::vector<vec<T>> vertices;
    vertices.reserve(N);

    T half_width = width / T(2);
    T half_height = height / T(2);

    for (int i = 0; i < N; i++) {
        vertices.push_back(vec<T>(half_width * std::cos(step * i), half_height * std::sin(step * i)));
    }

    return collider<T>(std::make_unique<Impl>(vertices));
}

template <typename T>
collider<T> collider<T>::ellipse(T a, T b) {
    return poly(a * T(2), b * T(2), ellipse_vertex_count);
}

template <typename T>
collider<T> collider<T>::circle(T radius) {
    return ellipse(radius, radius);
}

template <typename T>
collider<T> collider<T>::capsule(T width, T height) {
    T radius = width / T(2);
    T step = T(2) * std::numbers::pi_v<T> / T(ellipse_vertex_count);
    vec<T> cursor = vec<T>(0, radius - height / T(2));

    std::vector<vec<T>> vertices;

    int i = 0;
    for (; i < ellipse_vertex_count / 2; i++) {
        vec<T> o = vec<T>(radius * std::cos(step * i), radius * std::sin(step * i));
        vertices.push_back(cursor + o);
    }

    cursor = vec<T>(0, height / T(2) - radius);

    for (; i < ellipse_vertex_count; i++) {
        vec<T> o = vec<T>(radius * std::cos(step * i), radius * std::sin(step * i));
        vertices.push_back(cursor + o);
    }

    return collider<T>(std::make_unique<Impl>(vertices));
} 

template <typename T>
collider<T> collider<T>::line(T length) {
    return collider<T>::rect(0, length);
}

template <typename T>
collider<T> collider<T>::rounded_rect(T width, T height, T roundness) {
    if (roundness < T(0) || roundness > T(1)) {
        throw std::invalid_argument("Roundness should be between [0, 1].");
    }
    
    T half_width = width / T(2);
    T half_height = height / T(2);

    T radius_x = roundness * half_width;
    T radius_y = roundness * half_height;

    int corner_n = ellipse_vertex_count / 4;
    assert(corner_n > 1 && "Too few vertices to create rounded rectangles");

    T step = std::numbers::pi_v<T> / (T(corner_n) * T(2));

    const std::array<vec<T>, 4> corners = {
        vec<T>(half_width - radius_x, half_height - radius_y),
        vec<T>(-half_width + radius_x, half_height - radius_y),
        vec<T>(-half_width + radius_x, -half_height + radius_y),
        vec<T>(half_width - radius_x, -half_height + radius_y),
    };

    const T corner_rot[4] = {
        T(0),
        std::numbers::pi_v<T> / T(2),
        std::numbers::pi_v<T>,
        T(3) * std::numbers::pi_v<T> / T(2),
    };

    std::vector<vec<T>> vertices;
    int i = 0;
    vec<T> cursor = vec<T>(0, 0);
    
    const int SEGS = 4;
    for (int i = 0; i < SEGS; i++) {
        cursor = corners[i];
        for (int j = 0; j < corner_n; j++) {
            vec<T> o = vec<T>(radius_x * std::cos(corner_rot[i] + step * j), radius_y * std::sin(corner_rot[i] + step * j));
            vertices.push_back(cursor + o);
        }
    }

    return collider<T>(std::make_unique<Impl>(vertices));
}

template <typename T>
collider<T> collider<T>::from_points(const std::vector<point<T>>& points) {
    // Monotone chain algo

    std::vector<vec<T>> vertices;
    for (const auto& p : points) {
        vertices.push_back(vec<T>(p.x, p.y));
    }

    if (vertices.size() <= 3) return collider<T>(std::make_unique<Impl>(vertices));

    std::sort(vertices.begin(), vertices.end());

    auto cross = [](const vec<T> &o, const vec<T> &a, const vec<T> &b) {
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (a.x - o.x); 
    };
    
    std::vector<vec<T>> hull(2 * vertices.size());

    int k = 0; 

    // Build lower hull
    for (int i = 0; i < vertices.size(); i++) {
        while (k >= 2 && cross(hull[k - 2], hull[k-1], vertices[i]) <= 0) k--;
        hull[k++] = vertices[i];
    }

    
    // Build lower hull
    for (int i = vertices.size() - 1, t = k + 1; i > 0; i--) {
        while (k >= t && cross(hull[k - 2], hull[k-1], vertices[i - 1]) <= 0) k--;
        hull[k++] = vertices[i - 1];
    }
    
    hull.resize(k - 1);

    return collider<T>(std::make_unique<Impl>(hull));
}

template <typename T>
collider<T> collider<T>::raw(const std::vector<T>& data) {
    if (data.size() < 6) {
        throw std::invalid_argument("Raw collider data needs to be of size 6 or more.");
    }

    int i = 0; 
    T x = data[i++];
    T y = data[i++];
    T rotation = data[i++];

    int v_len = (data.size() - 3) / 2;
    if (v_len % 2) {
        throw std::invalid_argument("Uneven vertices vector in raw collider data.");
    }

    std::vector<vec<T>> vertices;
    vertices.reserve(v_len);

    for (; i < data.size(); i += 2) {
        if (!std::isfinite(data[i])) throw std::invalid_argument("Vertex containing non numeric value.");
        if (!std::isfinite(data[i + 1])) throw std::invalid_argument("Vertex containing non numeric value.");

        vertices.push_back(vec<T>(data[i], data[i + 1]));
    }

    return collider<T>(std::make_unique<Impl>(vertices)).set_position(x, y).set_rotation(rotation);
}

template <typename T>
collider<T>::collider(std::unique_ptr<Impl> impl) : impl(std::move(impl)) { } 

template <typename T>
int collider<T>::ellipse_vertex_count = 16;

}