<div align="center">
    <h2 align="center">tiny_colls</h2>
    <h3>Tiny SAT collision detection in C++.</h3>
</div>
 

### About the project

This project is a one class solution for detecting collisions between convex polygons using the Separating Axis Theorem. It is built to be easy to use and to be able to be used in any type of project.   

The main reason for creating this project was to train my game development mathematics and sharpen my C++ skills. It is free to use by anyone in any project.  

### Usage
```cpp
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
```
Output: 
```text
Minimum translation vector: 0.00795621, 9.99204
```
<p align="right">(<a href="#about-the-project">back to top</a>)</p>

### API
The API revolves around a single *main* class **collider** with three **POD** classes, **point**, **AABB** and, **collider**, that are given as the result from methods on the *main* class.

#### Main Class (collider)
```cpp
// Static factory methods
static collider rect(T width, T height);
static collider poly(T width, T height, int N);
static collider ellipse(T a, T b);
static collider circle(T radius);
static collider capsule(T width, T height);
static collider line(T length);
static collider rounded_rect(T width, T height, T roundness);
static collider from_points(const std::vector<point<T>>& points);
static collider raw(const std::vector<T>& data);

// Setters
collider& set_position(T x, T y);
collider& set_rotation(T rotation);

// Getters
std::vector<point<T>> get_shape() const;
AABB<T> get_bounding_box() const;
std::vector<T> get_raw() const;

// Collision check 
bool is_point_in(T x, T y);
bool is_colliding_with(const collider& other, collision<T>& out);

// Global setter for specifing the number of vertices of an ellipse (default 16) 
static void set_ellipse_vertex_count(int count);

// Convenient alises
using collider_f = collider<float>;
using collider_d = collider<double>;
```
#### Notes
To be able to to save a set state of a collider, perhaps for level construction or such, two methods are given:

```cpp
std::vector<T> get_raw() const;
static collider raw(const std::vector<T>& data);
```

These take/return a vector of T using a proprietary format specified as follows:  

```text
i = 0:      position x
i = 1:      position y
i = 2:      rotation
i = 3..n:   vertices (i: x, i + 1: y)
```

#### POD
```cpp
struct point {
    T x;
    T y;
};

struct AABB {
    T top;
    T bottom;
    T left;
    T right;
};

struct collision {
    T axis_x;
    T axis_y;
    T overlap; 
};

using point_f = point<float>;
using point_d = point<double>;

using AABB_f = AABB<float>;
using AABB_d = AABB<double>;

using collision_f = collision<float>;
using collision_d = collision<double>;
```

<p align="right">(<a href="#about-the-project">back to top</a>)</p>

### License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#about-the-project">back to top</a>)</p>

### Resources
* [Style guideline](https://google.github.io/styleguide/cppguide.html) 
* [SAT](https://dyn4j.org/2010/01/sat/)
* [Monotone chain](https://en.wikibooks.org/wiki/Algorithm_ImplementationGeometry/Convex_hull/Monotone_chain)
* [README inspiration](https://github.com/othneildrew/Best-README-Template?tab=readme-ov-file)

<p align="right">(<a href="#about-the-project">back to top</a>)</p>