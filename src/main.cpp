#include "raylib.h"
#include <cmath>
#include <cstdint>
#include <cwchar>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <string>
#include <math.h>
#include <algorithm>
#include <vector>
#include <cassert>

#define TARGET_FPS 30
#define FIXED_UPDATE_SECS 0.2
#define WIN_WIDTH 1280
#define WIN_HEIGHT 720
#define OBJECTS_MAX_SIZE 10000
#define RECTS_INITIAL_SIZE 10
#define CIRCLE_INITIAL_SIZE 50
#define CIRCLE_FORCE 500.0
#define CIRCLE_RADIUS 10

#define HEXCOLOR(hex, color)         \
      color.r = (hex >> 24) & 0xFF;  \
      color.g = (hex >> 16) & 0xFF;  \
      color.b = (hex >> 8) & 0xFF;   \
      color.a = hex & 0xFF;

Vector2 vector2_sub(Vector2 v1, Vector2 v2) {
    return Vector2{
        v1.x - v2.x,
        v1.y - v2.y
    };
}

Vector2 vector2_add(Vector2 v1, Vector2 v2) {
    return Vector2{
        v1.x + v2.x,
        v1.y + v2.y
    };
}

float vector2_length(Vector2 v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

Vector2 vector2_normalized(Vector2 v) {
    float length = vector2_length(v);
    if (length == 0)
        return Vector2{
            0, 0
        };

    return Vector2 {
        v.x / length,
        v.y / length
    };
}

float vector2_dot_product(Vector2 v1, Vector2 v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

Vector2 vector2_scale(Vector2 v, float scale) {
    return Vector2{v.x * scale, v.y * scale};
}

struct Circle {
    float x;
    float y;
    float radius;
};

void circle_resolve_collision_to_rect(const Circle& self, Vector2& velocity, const Rectangle& rect) {
    Vector2 closestPoint{
        std::clamp(self.x, rect.x, rect.x + rect.width),
        std::clamp(self.y, rect.y, rect.y + rect.height)
    };

    Vector2 difference = vector2_sub({ self.x, self.y }, closestPoint);
    Vector2 collisionNormal = vector2_normalized(difference);
    float velocityAlongNormal = vector2_dot_product(velocity, collisionNormal);

    velocity = vector2_sub(
        velocity, vector2_scale(collisionNormal, 2 * velocityAlongNormal)
    );
}

void circle_resolve_collision_to_circle(
    const Circle& self,
    Vector2& velocity1,
    const Circle& other,
    Vector2& velocity2
) {
    Vector2 self_pos{
        self.x,
        self.y
    };
    Vector2 other_pos{
        self.x,
        self.y
    };
    Vector2 distance = vector2_sub(other_pos, self_pos);
    float distanceLength = std::sqrt(distance.x * distance.x + distance.y * distance.y);

    // Check if collision has occurred
    float totalRadius = self.radius + other.radius;
    if (distanceLength > totalRadius) {
        return; // No collision
    }

    // Calculate collision normal
    Vector2 collisionNormal = vector2_normalized(distance);

    // Calculate the relative velocity along the collision normal
    Vector2 relativeVelocity = vector2_sub(velocity2, velocity1);
    float velocityAlongNormal = vector2_dot_product(relativeVelocity, collisionNormal);

    if (velocityAlongNormal > 0) {
        return;
    }

    float mass1 = 5;
    float mass2 = 5;

    float e = 1.0f; // Coefficient of restitution (elasticity), 1.0 for perfectly elastic collision
    float j = -(1 + e) * velocityAlongNormal;
    j /= (1 / mass1 + 1 / mass2);

    // Apply impulse to velocities
    Vector2 impulse = vector2_scale(collisionNormal, j);
    velocity1 = vector2_sub(velocity1, vector2_scale(impulse, 1 / mass1));
    velocity2 = vector2_add(velocity2, vector2_scale(impulse, 1 / mass2));

    // Move circles to correct for penetration
    // float overlap = totalRadius - distanceLength;
    // Vector2 correction = vector2_scale(collisionNormal, overlap / 2.0f);
    // self_pos = vector2_sub(self_pos, correction);
    // other_pos = vector2_add(other_pos, correction);
}

bool is_rect_contains(const Rectangle& rect1, const Rectangle& rect2) {
    return !((rect1.x + rect1.width <= rect2.x) ||
        (rect1.x >= rect2.x + rect2.width) ||
        (rect1.y + rect1.height <= rect2.y) ||
        (rect1.y >= rect2.y + rect2.height));
}

bool is_rect_contains(const Rectangle& rect, const Circle& circle) {
    float closest_x = std::max(rect.x, std::min(circle.x, rect.x + rect.width));
    float closest_y = std::max(rect.y, std::min(circle.y, rect.y + rect.height));

    float distance_x = circle.x - closest_x;
    float distance_y = circle.y - closest_y;

    return (distance_x * distance_x + distance_y * distance_y) <= (circle.radius * circle.radius);
}

// collider
struct Entity {
    enum Type {
        None,
        Circle,
        Rect
    } type;
    void* data;
    void* source;
};

std::string rectangle_to_string(const Rectangle& rect) {
    return "{ x=" + std::to_string((int)rect.x) +
        ", y=" + std::to_string((int)rect.y) +
        ", width=" + std::to_string((int)rect.width) +
        ", height=" + std::to_string((int)rect.height) + " }";
}

std::string circle_to_string(const Circle& circle) {
    return "{ x=" + std::to_string((int)circle.x) +
    ", y=" + std::to_string((int)circle.y) + ", radius=" +
    std::to_string((int)circle.radius) + " }";
}

std::string entity_to_string(const Entity& entity) {
    switch (entity.type) {
        case Entity::Type::Circle: {
            Circle* circle = (Circle*) entity.data;
            return std::to_string((unsigned long long)entity.data) +
            ' ' + circle_to_string(*circle);
        }
        case Entity::Type::Rect: {
            Rectangle* rect = (Rectangle*) entity.data;
            return std::to_string((unsigned long long)entity.data) +
            ' '  + rectangle_to_string(*rect);
        }
        default: break;
    }
    return "{NONE}";
}

#define QUADTREE_ENTITY_CAPACITY 4
#define QUADTREE_MAX_DEPTH 6
// TODO rewrite to loops instead of recursion
class QuadTree {
public:
    struct QuadTreeList {
        QuadTree* curr;
        QuadTree* next;
    };

    int depth;
    std::vector<Entity> entities;
    Rectangle boundary;
    // NORTHWEST = 0
    // NORTHEAST = 1
    // SOUTHEAST = 2
    // SOUTHWEST = 3
    QuadTree* child[4]{0};

    QuadTree() : depth(0), boundary{0,0,0,0} {
        entities.reserve(QUADTREE_ENTITY_CAPACITY*QUADTREE_ENTITY_CAPACITY);
    }
    QuadTree(Rectangle _boundary, int depth) :
        depth(depth), boundary(_boundary) {
        entities.reserve(QUADTREE_ENTITY_CAPACITY*QUADTREE_ENTITY_CAPACITY);
    }

    ~QuadTree() {
        if (child[0] != nullptr) {
            delete child[0];
            delete child[1];
            delete child[2];
            delete child[3];
        }
        else {
            entities.clear();
        }
    }

    void clear() {
        if (child[0] != nullptr) {
            delete child[0];
            delete child[1];
            delete child[2];
            delete child[3];
            child[0] = nullptr;
            child[1] = nullptr;
            child[2] = nullptr;
            child[3] = nullptr;
        }
        entities.clear();
    }

    bool contains(const Entity& entity) const {
        bool inside = false;
        switch(entity.type)
        {
            case Entity::Type::Circle: {
                Circle* circle = (Circle*) entity.data;
                inside = is_rect_contains(boundary, *circle);
                break;
            }
            case Entity::Type::Rect: {
                Rectangle* rect = (Rectangle*) entity.data;
                inside = is_rect_contains(boundary, *rect);
                break;
            }
            default: break;
        }
        return inside;
    }

    void range(const Rectangle& range, std::vector<Entity>& arr) const {
        std::set<void*> s{};
        _range(range, arr, s);
    }

    void insert(const Entity& entity) {
        if (!contains(entity)) {
            return;
        }

        if (( entities.size() < QUADTREE_ENTITY_CAPACITY && child[0] == nullptr )
            || depth >= QUADTREE_MAX_DEPTH) {
            entities.push_back(entity);
        }
        else {
            if (child[0] == nullptr) {
                Rectangle rect{
                    boundary.x, boundary.y,
                    boundary.width * .5f,
                    boundary.height * .5f
                };
                child[0] = new QuadTree(rect, depth+1);
                rect.x = rect.x + rect.width;
                child[1] = new QuadTree(rect, depth+1);
                rect.y = rect.y + rect.height;
                child[2] = new QuadTree(rect, depth+1);
                rect.x = boundary.x;
                child[3] = new QuadTree(rect, depth+1);

                for (size_t i=0;i < entities.size();i++) {
                    child[0]->insert(entities[i]);
                    child[1]->insert(entities[i]);
                    child[2]->insert(entities[i]);
                    child[3]->insert(entities[i]);
                    entities[i].type = Entity::Type::None;
                    entities[i].data = nullptr;
                }

                entities.clear();
            }

            child[0]->insert(entity);
            child[1]->insert(entity);
            child[2]->insert(entity);
            child[3]->insert(entity);
        }
    }

    void visualize() const {
        DrawRectangleLines(
            (int)boundary.x,
            (int)boundary.y,
            (int)boundary.width,
            (int)boundary.height,
            RED
        );
        std::string t = "{ x=" + std::to_string((int)boundary.x) + ", y=" + std::to_string((int)boundary.y)
            + ", width=" + std::to_string((int)boundary.width) + ", height=" + std::to_string((int)boundary.height) + " }";

        if (child[0] != nullptr) {
            child[0]->visualize();
            child[1]->visualize();
            child[2]->visualize();
            child[3]->visualize();
        }
        // else {
        //     DrawText(t.c_str(), boundary.x + 5, boundary.y + 5, 16, BLACK);
        // }
    }

    void visualize_text() const {
        std::cout << std::string(depth, '\t') << this << " entities_size=" << entities.size() << ' '
            << "depth=" << depth << ' '
            << rectangle_to_string(boundary) << '\n';
        for (size_t i=0;i< entities.size();i++) {
            std::cout << std::string( depth, '\t') << ' '
            << entity_to_string(entities[i]) << '\n';
        }

        if (child[0] != nullptr) {
            child[0]->visualize_text();
            child[1]->visualize_text();
            child[2]->visualize_text();
            child[3]->visualize_text();
        }
    }

    int count_entities_unique() const {
        std::set<void*> st = set_entities();
        return st.size();
    }
private:
    void _range(
        const Rectangle& range,
        std::vector<Entity>& arr,
        std::set<void*>& s
    ) const {
        if (!is_rect_contains(boundary, range)) {
            return;
        }

        if (child[0] != nullptr) {
            child[0]->_range(range, arr, s);
            child[1]->_range(range, arr, s);
            child[2]->_range(range, arr, s);
            child[3]->_range(range, arr, s);
        }
        else {
            for (size_t i=0;i<entities.size();i++) {
                if (s.find(entities[i].data) != s.end()) {
                    continue;
                }
                s.insert(entities[i].data);
                switch(entities[i].type) {
                    case Entity::Type::Circle:
                        {
                            Circle* circle = (Circle*) entities[i].data;
                            if (is_rect_contains(range, *circle)) {
                                arr.push_back(entities[i]);
                            }
                        }
                        break;
                    case Entity::Type::Rect:
                        {
                            Rectangle* rect = (Rectangle*) entities[i].data;
                            if (is_rect_contains(range, *rect)) {
                                arr.push_back(entities[i]);
                            }
                        }
                    default:
                        break;
                }
            }
        }
    }

    std::set<void*> set_entities() const {
        std::set<void*> st;
        if (child[0] != nullptr) {
            std::set<void*> s = child[0]->set_entities();
            st.insert(s.begin(), s.end());
            s = child[1]->set_entities();
            st.insert(s.begin(), s.end());
            s = child[2]->set_entities();
            st.insert(s.begin(), s.end());
            s = child[3]->set_entities();
            st.insert(s.begin(), s.end());
        }
        else {
            for (size_t i=0;i<entities.size();i++) {
                st.insert(entities[i].data);
            }
        }
        return st;
    }
};

struct CircleObj {
    Circle c;
    Vector2 velocity;

    void resolve_collision(const Rectangle& rect) {
        circle_resolve_collision_to_rect(c, velocity, rect);
    }

    void resolve_collision(CircleObj& other) {
        circle_resolve_collision_to_circle(
            this->c,
            this->velocity,
            other.c,
            other.velocity
        );
    }

    void response_collision(double delta) {
        this->c.x += this->velocity.x * delta;
        this->c.y += this->velocity.y * delta;
    }
};

struct {
    double delta;
    double fixed_update;

    Rectangle rects[OBJECTS_MAX_SIZE];

    CircleObj circles[OBJECTS_MAX_SIZE];
    bool viz_circles_pos;
    bool viz_circles_collider;

    QuadTree quad_tree;
    bool viz_quad_tree;
    bool viz_quad_tree_range;
    bool quad_tree_show_time;

    std::mt19937 random_gen;
} DATA;

int main(void)
{
    InitWindow(WIN_WIDTH, WIN_HEIGHT, "Quad Tree - Demo");
    SetTargetFPS(TARGET_FPS);

    DATA.delta = 0.0;
    DATA.fixed_update = 0.0;

    DATA.quad_tree.boundary.x = 0;
    DATA.quad_tree.boundary.y = 0;
    DATA.quad_tree.boundary.width = WIN_WIDTH;
    DATA.quad_tree.boundary.height = WIN_HEIGHT;

    DATA.viz_quad_tree = false;
    DATA.viz_circles_pos = false;
    DATA.viz_quad_tree_range = false;
    DATA.viz_circles_collider = false;
    DATA.quad_tree_show_time = false;

    std::random_device rd;
    DATA.random_gen.seed(rd());
    std::uniform_int_distribution<> g_circ_rand_force(-CIRCLE_FORCE, CIRCLE_FORCE);

    int circle_objects_size = 0;
    int rects_objects_size = 0;
    {
        std::uniform_int_distribution<> rand_posx(25, WIN_WIDTH - 50);
        std::uniform_int_distribution<> rand_posy(25, WIN_HEIGHT - 50);
        std::uniform_int_distribution<> rand_width(15, 30);
        std::uniform_int_distribution<> rand_height(15, 30);

        for (size_t i=0;i<RECTS_INITIAL_SIZE;i++) {
            DATA.rects[i].x = rand_posx(DATA.random_gen);
            DATA.rects[i].y = rand_posy(DATA.random_gen);
            DATA.rects[i].width = rand_width(DATA.random_gen);
            DATA.rects[i].height = rand_height(DATA.random_gen);
            rects_objects_size = i;

            std::vector<Entity> et;
            do {
                DATA.quad_tree.range(DATA.rects[i], et);
                if (et.size() > 1) {
                    DATA.rects[i].x = rand_posx(DATA.random_gen);
                    DATA.rects[i].y = rand_posy(DATA.random_gen);
                }
            }
            while (et.size() > 1);

            DATA.quad_tree.insert(Entity{
                Entity::Type::Rect,
                (void*) &DATA.rects[i],
                (void*) &DATA.rects[i]
            });
        }
        DATA.quad_tree.clear();

        if (RECTS_INITIAL_SIZE != 0)
            rects_objects_size++;

        for (size_t i=0;i<CIRCLE_INITIAL_SIZE;i++) {
            DATA.circles[i].c.x = rand_posx(DATA.random_gen);
            DATA.circles[i].c.y = rand_posy(DATA.random_gen);
            DATA.circles[i].c.radius = CIRCLE_RADIUS;
            float mass = 5;
            DATA.circles[i].velocity = Vector2{
                (float)g_circ_rand_force(DATA.random_gen) / mass,
                (float)g_circ_rand_force(DATA.random_gen) / mass,
            };
            circle_objects_size = i;
        }
        if (CIRCLE_INITIAL_SIZE != 0)
            circle_objects_size++;
    }

    // DATA.quad_tree.visualize_text();
    // main loop
    while (!WindowShouldClose())
    {
        double frame_start_time = GetTime();
        bool is_physics = DATA.fixed_update > FIXED_UPDATE_SECS;

        BeginDrawing();
        ClearBackground(RAYWHITE);

        std::vector<std::string> infos;
        {
            double time_start = GetTime();

            for (int i=0;i<rects_objects_size;i++) {
                Rectangle rect = DATA.rects[i];

                DrawRectangleRec(rect, LIGHTGRAY);
                DrawRectangleLinesEx(rect, 1.0f, LIME);
                DATA.quad_tree.insert(Entity{
                    Entity::Type::Rect,
                    (void*) &DATA.rects[i],
                    (void*) &DATA.rects[i]
                });
            }

            for (int i=0;i<circle_objects_size;i++) {
                Circle* circle = &DATA.circles[i].c;
                DATA.quad_tree.insert(Entity{
                    Entity::Type::Circle,
                    (void*) circle,
                    (void*) &DATA.circles[i]
                });
            }

            time_start = GetTime() - time_start;
            if (DATA.quad_tree_show_time)
                infos.push_back("QuadTree Insert Time: " + std::to_string(int(time_start * 1000.0)) + "ms");
            time_start = GetTime();

            Rectangle boundary{ 0, 0, WIN_WIDTH, WIN_HEIGHT };
            // Rectangle boundary{ CIRCLE_RADIUS, CIRCLE_RADIUS, WIN_WIDTH - CIRCLE_RADIUS - CIRCLE_RADIUS, WIN_HEIGHT - CIRCLE_RADIUS - CIRCLE_RADIUS };
            // DrawRectangleLinesEx(boundary, 1.0f, MAROON);

            for (int i=0;i<circle_objects_size;i++) {
                Circle* circle = &DATA.circles[i].c;
                CircleObj* circleobj = &DATA.circles[i];
                std::vector<Entity> et;
                Rectangle range{
                    circle->x - circle->radius,
                    circle->y - circle->radius,
                    circle->radius * 2,
                    circle->radius * 2
                };

                DATA.quad_tree.range(range, et);
                if (is_physics) {
                    // Vector2 force{
                    //     (float)g_circ_rand_force(DATA.random_gen),
                    //     (float)g_circ_rand_force(DATA.random_gen),
                    // };
                    //
                    // Vector2 new_velocity{
                    //     force.x / DATA.circles[i].mass,
                    //     force.y / DATA.circles[i].mass,
                    // };
                    // DATA.circles[i].velocity = new_velocity;

                    Vector2 new_velocity = DATA.circles[i].velocity;

                    float radius = circle->radius;

                    // Rectangle boundary{ WIN_WIDTH * .5, WIN_HEIGHT * .5, 50, 50 };
                    // bool is_x_out = circle->x - radius < boundary.x || circle->x + radius > boundary.x + boundary.width;
                    // bool is_y_out = circle->y - radius < boundary.y || circle->y + radius > boundary.y + boundary.height;

                    if (circle->x - radius < boundary.x) {
                        new_velocity.x = std::abs(new_velocity.x);
                    }
                    else if (circle->x + radius > boundary.x + boundary.width){
                        new_velocity.x = -new_velocity.x;
                    }

                    if (circle->y - radius < boundary.y) {
                        new_velocity.y = std::abs(new_velocity.y);
                    }
                    else if (circle->y + radius > boundary.y + boundary.height){
                        new_velocity.y = -new_velocity.y;
                    }

                    DATA.circles[i].velocity = new_velocity;

                    // for (size_t i=0;i<et.size();i++) {
                    //     if ( et[i].data == (void*) circle )
                    //         continue;
                    //
                    //     Entity e = et[i];
                    //     switch(e.type) {
                    //         case Entity::Type::Circle:
                    //             {
                    //                 CircleObj* other = (CircleObj*) e.source;
                    //                 circleobj->resolve_collision(*other);
                    //                 other->response_collision(DATA.delta);
                    //                 circle->x += circleobj->velocity.x * DATA.delta;
                    //                 circle->y += circleobj->velocity.y * DATA.delta;
                    //             }
                    //             break;
                    //         case Entity::Rect:
                    //             {
                    //                 Rectangle* rect = (Rectangle*) e.data;
                    //                 DrawRectangleLines(rect->x, rect->y, rect->width, rect->height, GREEN);
                    //             }
                    //             break;
                    //         default:
                    //             break;
                    //     }
                    // }
                }

                circle->x += DATA.circles[i].velocity.x * DATA.delta;
                circle->y += DATA.circles[i].velocity.y * DATA.delta;

                if (DATA.viz_circles_pos) {
                    DrawText(circle_to_string(*circle).c_str(), circle->x, circle->y, 16, BLACK);
                }

                if (DATA.viz_circles_collider) {
                    DrawRectangleLinesEx(range, 1.0f, GREEN);
                }

                DrawCircle(
                    (int)circle->x,
                    (int)circle->y,
                    circle->radius,
                    (et.size() <= 1)? LIGHTGRAY:RED
                );
            }
            time_start = GetTime() - time_start;
            if (DATA.quad_tree_show_time)
                infos.push_back("QuadTree Checks Time(with draw): " + std::to_string(int(time_start * 1000.0)) + "ms");
        }

        if (DATA.viz_quad_tree_range) {
            int mousex = GetMouseX();
            int mousey = GetMouseY();
            Rectangle mouse_boundary{
                mousex - 100.0f,
                mousey - 100.0f,
                200.0f, 200.0f
            };
            DrawRectangleLinesEx(mouse_boundary, 2.0f, GREEN);

            std::vector<Entity> et_in_boundary{};
            DATA.quad_tree.range(mouse_boundary, et_in_boundary);
            for (size_t i=0;i<et_in_boundary.size();i++) {
                Entity e = et_in_boundary[i];
                switch(e.type) {
                    case Entity::Type::Circle:
                        {
                            Circle* circle = (Circle*) e.data;
                            DrawCircleLines(circle->x, circle->y, circle->radius, GREEN);
                        }
                        break;
                    case Entity::Rect:
                        {
                            Rectangle* rect = (Rectangle*) e.data;
                            DrawRectangleLines(rect->x, rect->y, rect->width, rect->height, GREEN);
                        }
                        break;
                    default:
                        break;
                }
            }

            std::string s = "Entity in Boundary: " + std::to_string(et_in_boundary.size());
            infos.push_back(s);
        }

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && circle_objects_size < OBJECTS_MAX_SIZE) {
            int mousex = GetMouseX();
            int mousey = GetMouseY();

            DATA.circles[circle_objects_size].c.x = mousex;
            DATA.circles[circle_objects_size].c.y = mousey;
            DATA.circles[circle_objects_size].c.radius = CIRCLE_RADIUS;
            float mass = 5;
            DATA.circles[circle_objects_size].velocity = Vector2{
                (float)g_circ_rand_force(DATA.random_gen) / mass,
                (float)g_circ_rand_force(DATA.random_gen) / mass,
            };

            circle_objects_size++;
        }

        if (IsKeyPressed(KEY_ONE)) {
            DATA.viz_quad_tree = !DATA.viz_quad_tree;
        }

        if (DATA.viz_quad_tree) {
            DATA.quad_tree.visualize();
            int c = DATA.quad_tree.count_entities_unique();
            std::string s = "Entity Count: " + std::to_string(c);
            infos.push_back(s);
        }

        if (IsKeyPressed(KEY_TWO)) {
            DATA.viz_circles_pos = !DATA.viz_circles_pos;
        }

        if (IsKeyPressed(KEY_THREE)) {
            DATA.viz_circles_collider = !DATA.viz_circles_collider;
        }

        if (IsKeyPressed(KEY_FOUR)) {
            DATA.viz_quad_tree_range = !DATA.viz_quad_tree_range;
        }

        if (IsKeyPressed(KEY_FIVE)) {
            DATA.quad_tree_show_time = !DATA.quad_tree_show_time;
        }

        if (IsKeyPressed(KEY_SIX)) {
            DATA.quad_tree.visualize_text();
        }

        if (!infos.empty()) {
            size_t max_length = 0;
            int pad = 10;
            int fontsize = 20;
            Color bg;
            HEXCOLOR(0xcac6caff, bg);
            for (size_t i=0;i<infos.size();i++) {
                max_length = std::max(max_length, infos[i].length());
            }
            Rectangle container{
                float (WIN_WIDTH - ( max_length * (fontsize * .5) ) - ( pad * 5 )),
                (float)pad,
                float ( max_length * (fontsize * .5) + ( pad * 4 ) ),
                float (infos.size() * (fontsize + pad))
            };
            DrawRectangleRec(container, bg);
            for (size_t i=0;i<infos.size();i++) {
                DrawText(infos[i].c_str(), container.x + 5, container.y + 5, fontsize, DARKGREEN);
                container.y += fontsize + pad;
            }
        }

        {
            Color color;
            HEXCOLOR(0xcac6caff, color);
            std::string fps_text = "FPS: " + std::to_string(GetFPS());
            std::string delta_text = "Delta: " + std::to_string(DATA.delta);
            // std::string fixed_update = "FixedUpdateDelta: " + std::to_string(DATA.fixed_update);
            DrawRectangle(5, 5, delta_text.length() * (20 * .6), 3 * 25 + 5, color);
            DrawText(fps_text.c_str(), 10, 10, 20, DARKGREEN);
            DrawText(delta_text.c_str(), 10, 30, 20, DARKGREEN);
            // DrawText(fixed_update.c_str(), 10, 60, 20, DARKGREEN);
        }

        if (DATA.fixed_update > FIXED_UPDATE_SECS) {
            DATA.fixed_update -= FIXED_UPDATE_SECS;
        }

        EndDrawing();
        DATA.quad_tree.clear();
        DATA.delta = GetTime() - frame_start_time;
        DATA.fixed_update += DATA.delta;
    }

    CloseWindow();
    return 0;
}
