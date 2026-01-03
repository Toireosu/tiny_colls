#include <tiny_colls.h>
#include <iostream>

using namespace tiny_colls;

int main() {
    auto a = collider_f::rect(20, 30).set_rotation(3.14f / 2);
    auto b = collider_f::ellipse(10, 15).set_position(-10, -15);

    collision_f coll;
    if (a.is_colliding_with(b, coll)) {
        float mt_x = coll.axis_x * coll.overlap;
        float mt_y = coll.axis_y * coll.overlap;
        std::cout << "Minimum translation vector: " << mt_x << ", " << mt_y << std::endl; 
    } else {
        std::cout << "No collision detected!" << std::endl;
    }

    return EXIT_SUCCESS;
}