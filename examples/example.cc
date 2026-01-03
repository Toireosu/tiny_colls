// TINY_COLLS EXAMPLE USAGE
// Move "player" collider with mouse, rotate left/right with left/right mouse button, 
// show/hide axis-aligned bounding boxes by pressing space.

#include "raylib.h"
#include "raymath.h"
#include "tiny_colls.h"
#include <iostream>
#include <memory>

#include "tiny_colls/details/vec.h"

using namespace tiny_colls;

const float FRICTION = 10.0f; 
const float TRACK_SPEED = 1200.0f;
const float DAMPING = 0.97f;

struct demo_vars {
    bool show_aabb = false;
};

void draw_shape(const std::vector<tiny_colls::point_f>& shape, Color color) {
    auto last = shape.begin();
    for (auto it = last + 1; it != shape.end(); ++it, ++last) {
        DrawLine(
            last->x,
            last->y,
            it->x,
            it->y,
            color
        );
    }


    auto first = shape.begin();
    DrawLine(
        last->x,
        last->y,
        first->x,
        first->y,
        color
    );
}

struct entity {
    float rotation;
    Vector2 position; 
    Vector2 velocity;
    bool is_hovered = false;
    collider_f coll;
    entity(float rotation, Vector2 position, collider_f coll) : rotation(rotation), position(position), velocity({0, 0}), coll(std::move(coll)) { 
        update(0);
    } 

    
    virtual void update(float delta) {
        velocity = Vector2Subtract(velocity, Vector2Scale(velocity, FRICTION * delta));
        position = Vector2Add(position, Vector2Scale(velocity, delta));
        transform();
    }

    virtual void draw(const demo_vars& d_vars) {
        draw_shape(coll.get_shape(), is_hovered ? GREEN : RED);

        if (d_vars.show_aabb) {
            AABB aabb = coll.get_bounding_box();
            draw_shape({
                    {aabb.left, aabb.bottom},
                    {aabb.right, aabb.bottom},
                    {aabb.right, aabb.top},
                    {aabb.left, aabb.top},
                },
                BLUE
            );
        }
    }

    protected:
    void transform() {
        coll.set_position(position.x, position.y).set_rotation(rotation);
    }
};

struct entity_spec : public entity {

    std::vector<point_f> points;
    entity_spec(float rotation, Vector2 position, std::vector<point_f> points) : points(points), entity(rotation, position, collider_f::from_points(points)) { 
        update(0);
    } 

    void draw(const demo_vars& d_vars) override {
        std::vector<point_f> t_points;
        t_points.reserve(points.size());

        for (const auto& point : points) {
            t_points.push_back({ point.x + position.x, point.y + position.y });
        }
        
        draw_shape(t_points, WHITE);
        entity::draw(d_vars);
    }
};

struct player : public entity {
    Vector2 target;
    player(float rotation, Vector2 position, collider_f coll) : target({0, 0}), entity(rotation, position, std::move(coll)) { } 

    void update(float delta) override {
        velocity = Vector2Add(
            velocity,
            Vector2Scale(
                Vector2Subtract(target, position),
                TRACK_SPEED * delta
            )
        );

        velocity = Vector2Scale(velocity, DAMPING);

        position = Vector2Add(
            position,
            Vector2Scale(
                velocity,
                delta
            )
        );

        transform();
    }
};

std::vector<float> raw_collider_data = {
    1077, 225, 0.791049, 25, 12.5, 24.1481, 18.9705, 21.6506, 25, 17.6777, 30.1777, 12.5, 34.1506, 6.47048, 36.6481, -1.09278e-06, 37.5, -6.47048, 36.6481, -12.5, 34.1506, -17.6777, 30.1777, -21.6506, 25, -24.1481, 18.9705, -25, -12.5, -24.1481, -18.9705, -21.6506, -25, -17.6777, -30.1777, -12.5, -34.1506, -6.47047, -36.6481, 2.98122e-07, -37.5, 6.47048, -36.6481, 12.5, -34.1506, 17.6777, -30.1777, 21.6506, -25, 24.1481, -18.9705,
};

int main() {
    InitWindow(1280, 720, "tiny_colls x raylib Example");

    std::vector<std::unique_ptr<entity>> entities;

    collider_f::set_ellipse_vertex_count(24);

    player& p = *(player*)entities.emplace_back(
        std::make_unique<player>(
            0.0f,
            Vector2(0, 0),
            collider_f::capsule(50, 25)
        )
    ).get();

    entities.emplace_back(
        std::make_unique<entity>(
            0.6f,
            Vector2(450, 250),
            collider_f::rect(70, 70)
        )
    );

    entities.emplace_back(
        std::make_unique<entity>(
            2.1f,
            Vector2(600, 600),
            collider_f::ellipse(40, 25)
        )
    );

    entities.emplace_back(
        std::make_unique<entity>(
            2.1f,
            Vector2(800, 550),
            collider_f::circle(50)
        )
    );

    entities.emplace_back(
        std::make_unique<entity>(
            0.0f,
            Vector2(500, 100),
            collider_f::poly(60, 60, 6)
        )
    );

    entities.emplace_back(
        std::make_unique<entity>(
            0.3f,
            Vector2(1000, 500),
            collider_f::line(500)
        )
    );

    entities.emplace_back(
        std::make_unique<entity>(
            0.0f,
            Vector2(100, 550),
            collider_f::rounded_rect(120, 80, 0.4f)
        )
    );

    std::vector<point_f> con_p = {
        { -24, -21 },
        { 0, -9 },
        { 23, -26 },
        { 9, 0 },
        { 25, 14 },
        { -12, 36 },
        { -19, 23 },
        { -8, 0 },
    };

    entities.emplace_back(
        std::make_unique<entity_spec>(
            0.0f,
            Vector2(300, 550),
            con_p
        )
    );

    entities.emplace_back(
        std::make_unique<entity>(
            raw_collider_data[2],
            Vector2(raw_collider_data[0], raw_collider_data[1]),
            collider_f::raw(raw_collider_data)
        )
    );

    demo_vars d_vars;

    while(!WindowShouldClose()) {
        BeginDrawing();

        ClearBackground(BLACK);

        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            p.rotation += GetFrameTime();
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            p.rotation -= GetFrameTime();
        }

        if (IsKeyPressed(KEY_SPACE)) {
            d_vars.show_aabb = !d_vars.show_aabb;
        }

        p.target = GetMousePosition();

        float delta = GetFrameTime();

        for (auto& entity : entities) {             
            entity->update(delta);
        }

        auto collisions_once = [&entities](std::unique_ptr<entity>& entity) {
            for (auto& other : entities) {
                tiny_colls::collision_f collision;
                if (entity->coll.is_colliding_with(other->coll, collision)) {
                    Vector2 normal = { collision.axis_x, collision.axis_y };
                    float vn = Vector2DotProduct(entity->velocity, normal);
                    entity->velocity = Vector2Subtract(entity->velocity, Vector2Scale(normal, vn));

                    const float SLOP = 0.01f;
                    const float POS_BIAS = 0.2f;

                    if (collision.overlap > SLOP) {
                        entity->position = Vector2Add(entity->position, Vector2Scale(normal, (collision.overlap - SLOP) * POS_BIAS));
                    }

                    return true;
                }
            }

            return false;
        };

        for (auto& entity : entities) {
            entity->is_hovered = entity->coll.is_point_in(p.target.x, p.target.y);

            collisions_once(entity);
        }

        for (auto& entity : entities) {
            entity->draw(d_vars);
        }

        
        
        DrawFPS(0, 0);
        EndDrawing();
    }
}