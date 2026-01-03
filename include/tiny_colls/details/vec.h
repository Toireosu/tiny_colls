#pragma once

#include <concepts>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace tiny_colls::details {
template <typename T>
class vec {
    static_assert(std::is_floating_point<T>::value, "vec<T>: T must be numeric");
public:
    explicit vec() {}
    explicit vec(T x, T y) : x(x), y(y) {}
    vec(const vec& other) : x(other.x), y(other.y) {}

    T x = 0;
    T y = 0;

    vec<T> rotate(T theta) const {
        return vec<T>(
            std::cos(theta) * x - std::sin(theta) * y,
            std::sin(theta) * x + std::cos(theta) * y
        );
    }

    vec<T> rotate_degrees(T theta) const {
        return rotate(theta * std::numbers::pi / 180);
    } 

    vec<T> operator+(const vec<T> &other) const {
        return vec<T>(
            this->x + other.x,
            this->y + other.y
        );
    } 

    vec<T> operator*(T factor) const {
        return vec<T>(
            this->x * factor,
            this->y * factor
        );
    } 

    vec<T> operator-(const vec<T> &other) const {
        return vec<T>(
            this->x - other.x,
            this->y - other.y
        );
    } 

    vec<T> operator-() {
        return vec<T>(
            -this->x,
            -this->y
        );
    }

    T dot(const vec<T> &other) const {
        return this->x * other.x + this->y * other.y;   
    }

    bool operator<(const vec<T> &v) const {
        return this->x < v.x || (this->x == v.x && this->y < v.y);
    }

    vec<T> normalize() const {
        T magnitude = std::sqrt(this->dot(*this));
        if (magnitude == 0.0f)
            throw std::logic_error("Cannot normalize {0, 0} vector.");
        
        return vec<T>(
            this->x / magnitude,
            this->y / magnitude
        );
    }

    vec<T> perp() const {
        return vec<T>(-this->y, this->x);
    }
};
}