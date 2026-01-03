#include <cassert>
#include <numeric>
#include "tiny_colls.h"

using namespace tiny_colls;

#define EPSILON 1e-6

bool is_shape_same(const collider_f& a, const collider_f& b) {
    auto a_shape = a.get_shape();
    auto b_shape = b.get_shape();

    if (a_shape.size() != b_shape.size()) return false;

    for (int i = 0; i < a_shape.size(); i++) {
        if (std::abs(a_shape[i].x - b_shape[i].x) > EPSILON) return false;
        if (std::abs(a_shape[i].y - b_shape[i].y) > EPSILON) return false;
    }

    return true;
}

#define assert_throws(exp, str) \
    { \
        bool threw = false; \
        try { exp; } \
        catch (const std::exception& e) { threw = true; } \
        assert(threw && str);\
    }

#define assert_not_throws(exp, str) \
    { \
        bool threw = false; \
        try { exp; } \
        catch (const std::exception& e) { threw = true; } \
        assert(!threw && str); \
    }

collider_f& nudge(collider_f c) {
    return c.set_position(0.1f, 0.1f);
} 

void test_empty_collider() {
    collider_f coll;
    assert_throws(coll.get_shape(), "Unintialized collider should not return shape.");
}

void test_raw_save_and_load() {
    auto coll0 = collider_f::rect(5.0f, 2.0f).set_position(5.0f, 3.0f).set_rotation(2.0f);
    auto data = coll0.get_raw();
    auto coll1 = collider_f::raw(data);

    assert(is_shape_same(coll0, coll1) && "Testing raw save and load");
}

void test_rect_garbage() {
    // Same test as line

    collider_f rect0;
    collider_f rect1;

    assert_not_throws(rect0 = collider_f::rect(0.0f, 0.0f), "Creating zero size should not throw.");
    assert_not_throws(rect1 = collider_f::rect(0.0f, 0.0f), "Creating zero size should not throw.");

    collision_f c;
    assert(!rect0.is_colliding_with(rect1, c) && "No axes should be created.");

    collider_f rect2 = nudge(collider_f::rect(20.0f, 10.0f));
    collider_f rect3;

    assert_not_throws(rect3 = collider_f::rect(-20, -10.0f), "Negative numbers should not throw.");

    assert(rect2.is_colliding_with(rect3, c) && "'Inverted' colliders should work as usual.");
}

void test_poly_garbage() {
    // This tests both ellipse and circle as well

    collider_f poly0;
    collider_f poly1;

    assert_not_throws(poly0 = collider_f::poly(0.0f, 0.0f, 10), "Creating zero size should not throw.");
    assert_not_throws(poly1 = collider_f::poly(0.0f, 0.0f, 10), "Creating zero size should not throw.");

    collision_f c;
    assert(!poly0.is_colliding_with(poly1, c) && "No axes should be created.");

    collider_f poly2 = nudge(collider_f::poly(20.0f, 10.0f, 10));
    collider_f poly3;

    assert_not_throws(poly3 = collider_f::poly(-20, -10.0f, 10), "Negative numbers should not throw.");

    assert(poly2.is_colliding_with(poly3, c) && "'Inverted' colliders should work as usual.");
}

void test_poly_garbage_n() {
    assert_throws(collider_f poly0 = collider_f::poly(10.0f, 10.0f, 0), "Creating zero vertex poly should throw.");
    assert_throws(collider_f poly0 = collider_f::poly(10.0f, 10.0f, -10), "Creating negative vertex poly should throw.");
}

void test_capsule_garbage() {
    collider_f capsule0;
    collider_f capsule1;

    assert_not_throws(capsule0 = collider_f::capsule(0.0f, 0.0f), "Creating zero size should not throw.");
    assert_not_throws(capsule1 = collider_f::capsule(0.0f, 0.0f), "Creating zero size should not throw.");

    collision_f c;
    assert(!capsule0.is_colliding_with(capsule1, c) && "No axes should be created.");

    collider_f capsule2 = nudge(collider_f::capsule(20.0f, 10.0f));
    collider_f capsule3;

    assert_not_throws(capsule3 = collider_f::capsule(-20, -10.0f), "Negative numbers should not throw.");

    assert(capsule2.is_colliding_with(capsule3, c) && "'Inverted' colliders should work as usual.");
}

void test_rounded_rect_garbage() {
    collider_f rr0;
    collider_f rr1;

    assert_not_throws(rr0 = collider_f::rounded_rect(0.0f, 0.0f, 0.2f), "Creating zero size should not throw.");
    assert_not_throws(rr1 = collider_f::rounded_rect(0.0f, 0.0f, 0.2f), "Creating zero size should not throw.");

    collision_f c;
    assert(!rr0.is_colliding_with(rr1, c) && "No axes should be created.");

    collider_f rr2 = nudge(collider_f::rounded_rect(20.0f, 10.0f, 0.2f));
    collider_f rr3;

    assert_not_throws(rr3 = collider_f::rounded_rect(-20, -10.0f, 0.2f), "Negative numbers should not throw.");

    assert(rr2.is_colliding_with(rr3, c) && "'Inverted' colliders should work as usual.");
}

void test_rounded_rect_garbage_roundness() {
    assert_throws(collider_f poly0 = collider_f::rounded_rect(10.0f, 10.0f, -1.0f), "Creating rounded rect with negative roundness should throw.");
    assert_throws(collider_f poly0 = collider_f::rounded_rect(10.0f, 10.0f, 2.0f), "Creating rounded rect with roundness > 1.0f should throw.");
}

void test_from_points_garbage() {
    collider_f points0;
    collider_f points1;

    assert_not_throws(points0 = collider_f::from_points(std::vector<point_f>()), "Creating zero size should not throw.");
    assert_not_throws(points1 = collider_f::from_points(std::vector<point_f>()), "Creating zero size should not throw.");

    collision_f c;
    assert(!points0.is_colliding_with(points1, c) && "No axes should be created.");
}

void test_raw_garbage() {
    assert_throws(collider_f::raw(std::vector<float>()), "Creating collider from empty vector should throw.");
    assert_throws(collider_f::raw({ 25, 32, 54 }), "Creating collider from too small vector should throw.");
    assert_throws(collider_f::raw({ 25, 32, 54, 25, 32, 54, 25, 32, 12 }), "Creating collider from uneven vertices should throw.");
    assert_throws(collider_f::raw({ 25, 32, 54, 25, 32, 54, std::numeric_limits<float>::infinity(), 32, 12, 23 }), "Creating collider from non finite vertices should throw.");
}

void test_ellipse_vertex_count_low() {
    assert_throws(collider_f::set_ellipse_vertex_count(-10), "Setting vertex count too low should throw.");
}

int main() {
    test_empty_collider();
    test_raw_save_and_load();
    test_rect_garbage();
    test_poly_garbage();
    test_poly_garbage_n();
    test_capsule_garbage();
    test_rounded_rect_garbage();
    test_rounded_rect_garbage_roundness();
    test_from_points_garbage();
    test_raw_garbage();
    test_ellipse_vertex_count_low();
}