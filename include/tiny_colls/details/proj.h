#pragma once

#include <type_traits>

namespace tiny_colls::details {
template <typename T>
class proj {
    static_assert(std::is_arithmetic<T>::value, "proj<T>: T must be numeric");
public:
    explicit proj(T min, T max) : min(min), max(max) {}
    T min;
    T max;
};
}