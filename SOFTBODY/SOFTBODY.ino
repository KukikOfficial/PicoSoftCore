#include <Arduino.h>
#include <cmath>

static const int NUM_RING_POINTS = 16;
static const int TOTAL_POINTS = NUM_RING_POINTS + 1;
static const int CENTER_IDX = NUM_RING_POINTS;

static const float WORLD_WIDTH  = 800.0f;
static const float WORLD_HEIGHT = 600.0f;
static const float BASE_DT = 0.016f;

float GRAVITY = 500.0f;
float DRAG = 0.992f;
float BOUNCE = 0.55f;
int SOLVER_ITERATIONS = 4;
float body_radius = 55.0f;
float target_area = 0.0f;
float PRESSURE_K = 0.018f;

float time_scale = 1.0f;
float current_dt = BASE_DT;
float prev_dt = BASE_DT;

struct Point {
    float x, y;
    float old_x, old_y;
    bool pinned;
};

struct Spring {
    int p1, p2;
    float rest_len;
};

struct Obstacle {
    float x, y;
    float radius;
};

struct Segment {
    float x1, y1;
    float x2, y2;
    float thickness;
    bool is_trampoline;
};

Point points[TOTAL_POINTS];

static const int MAX_SPRINGS = NUM_RING_POINTS * 3;
Spring springs[MAX_SPRINGS];
int spring_count = 0;

static const int MAX_OBSTACLES = 32;
Obstacle obstacles[MAX_OBSTACLES];
int obstacle_count = 0;

static const int MAX_SEGMENTS = 16;
Segment segments[MAX_SEGMENTS];
int segment_count = 0;

float ext_fx = 0.0f;
float ext_fy = 0.0f;
bool is_dragging = false;
float drag_target_x = 0.0f;
float drag_target_y = 0.0f;

char rx_buf[96];
int rx_idx = 0;

void add_spring(int p1, int p2) {
    if (spring_count >= MAX_SPRINGS) return;
    float dx = points[p1].x - points[p2].x;
    float dy = points[p1].y - points[p2].y;
    springs[spring_count++] = { p1, p2, std::sqrt(dx * dx + dy * dy) };
}

void init_soft_body(float cx, float cy, float radius) {
    body_radius = radius;
    target_area = (float)M_PI * radius * radius;

    for (int i = 0; i < NUM_RING_POINTS; i++) {
        float angle = i * (2.0f * (float)M_PI / NUM_RING_POINTS);
        float x = cx + radius * std::cos(angle);
        float y = cy + radius * std::sin(angle);
        points[i] = { x, y, x, y, false };
    }
    points[CENTER_IDX] = { cx, cy, cx, cy, false };

    spring_count = 0;
    for (int i = 0; i < NUM_RING_POINTS; i++) {
        int next = (i + 1) % NUM_RING_POINTS;
        int next2 = (i + 2) % NUM_RING_POINTS;
        add_spring(i, next);
        add_spring(i, CENTER_IDX);
        add_spring(i, next2);
    }
}

void resize_soft_body(float new_radius) {
    new_radius = constrain(new_radius, 20.0f, 130.0f);
    if (body_radius > 1.0f) {
        float ratio = new_radius / body_radius;
        for (int i = 0; i < spring_count; i++) {
            springs[i].rest_len *= ratio;
        }
        body_radius = new_radius;
        target_area = (float)M_PI * new_radius * new_radius;
    }
}

void process_serial_input() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (rx_idx > 0) {
                rx_buf[rx_idx] = '\0';
                
                if (rx_buf[0] == 'R') {
                    init_soft_body(WORLD_WIDTH * 0.5f, 120.0f, body_radius);
                } 
                else if (rx_buf[0] == 'T') {
                    float ts;
                    if (sscanf(rx_buf, "T %f", &ts) == 1) time_scale = constrain(ts, 0.05f, 2.0f);
                }
                else if (rx_buf[0] == 'S') {
                    float nr;
                    if (sscanf(rx_buf, "S %f", &nr) == 1) resize_soft_body(nr);
                }
                else if (rx_buf[0] == 'X') {
                    obstacle_count = 0;
                    segment_count = 0;
                }
                else if (rx_buf[0] == '+' && rx_buf[1] == 'O') {
                    float ox, oy, orad;
                    if (sscanf(rx_buf, "+O %f %f %f", &ox, &oy, &orad) == 3) {
                        if (obstacle_count < MAX_OBSTACLES) obstacles[obstacle_count++] = { ox, oy, orad };
                    }
                }
                else if (rx_buf[0] == '+' && rx_buf[1] == 'S') {
                    // Добавление отрезка: +S x1 y1 x2 y2 thickness is_tramp
                    float x1, y1, x2, y2, th;
                    int tramp;
                    if (sscanf(rx_buf, "+S %f %f %f %f %f %d", &x1, &y1, &x2, &y2, &th, &tramp) == 6) {
                        if (segment_count < MAX_SEGMENTS) {
                            segments[segment_count++] = { x1, y1, x2, y2, th, tramp == 1 };
                        }
                    }
                }
                else if (rx_buf[0] == 'P') {
                    float g, d, b, p;
                    int iters;
                    int cnt = sscanf(rx_buf, "P %f %f %f %d %f", &g, &d, &b, &iters, &p);
                    if (cnt >= 4) {
                        GRAVITY = g;
                        DRAG = d;
                        BOUNCE = b;
                        SOLVER_ITERATIONS = constrain(iters, 1, 10);
                        if (cnt >= 5) PRESSURE_K = p;
                    }
                }
                else if (rx_buf[0] == 'C') {
                    float fx, fy, tx, ty;
                    int drag;
                    if (sscanf(rx_buf, "C %f %f %d %f %f", &fx, &fy, &drag, &tx, &ty) == 5) {
                        ext_fx = fx;
                        ext_fy = fy;
                        is_dragging = (drag == 1);
                        drag_target_x = tx;
                        drag_target_y = ty;
                    }
                }
                rx_idx = 0;
            }
        } else if (rx_idx < (int)sizeof(rx_buf) - 1) {
            rx_buf[rx_idx++] = c;
        }
    }
}

void verlet_integrate() {
    current_dt = BASE_DT * time_scale;
    float time_ratio = (prev_dt > 1e-5f) ? (current_dt / prev_dt) : 1.0f;
    prev_dt = current_dt;

    if (is_dragging) {
        float pull = 0.28f * time_scale;
        points[CENTER_IDX].x += (drag_target_x - points[CENTER_IDX].x) * pull;
        points[CENTER_IDX].y += (drag_target_y - points[CENTER_IDX].y) * pull;
    }

    for (int i = 0; i < TOTAL_POINTS; i++) {
        if (points[i].pinned) continue;

        float vx = (points[i].x - points[i].old_x) * time_ratio * DRAG;
        float vy = (points[i].y - points[i].old_y) * time_ratio * DRAG;

        points[i].old_x = points[i].x;
        points[i].old_y = points[i].y;

        points[i].x += vx + ext_fx * current_dt * current_dt;
        points[i].y += vy + (GRAVITY + ext_fy) * current_dt * current_dt;
    }
}

void solve_constraints() {
    float dt_sq_ratio = time_scale * time_scale;

    for (int iter = 0; iter < SOLVER_ITERATIONS; iter++) {
        // 1. Пружинные связи
        for (int i = 0; i < spring_count; i++) {
            Point &p1 = points[springs[i].p1];
            Point &p2 = points[springs[i].p2];

            float dx = p2.x - p1.x;
            float dy = p2.y - p1.y;
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist < 1e-4f) continue;

            float diff = (dist - springs[i].rest_len) / dist;
            float off_x = dx * 0.5f * diff;
            float off_y = dy * 0.5f * diff;

            if (!p1.pinned) { p1.x += off_x; p1.y += off_y; }
            if (!p2.pinned) { p2.x -= off_x; p2.y -= off_y; }
        }

        // 2. Давление газа
        if (PRESSURE_K > 1e-5f) {
            float cur_area = 0.0f;
            for (int i = 0; i < NUM_RING_POINTS; i++) {
                int next = (i + 1) % NUM_RING_POINTS;
                cur_area += points[i].x * points[next].y - points[next].x * points[i].y;
            }
            cur_area = 0.5f * std::abs(cur_area);

            float area_diff = target_area - cur_area;
            float pressure_force = area_diff * PRESSURE_K * dt_sq_ratio;

            for (int i = 0; i < NUM_RING_POINTS; i++) {
                int next = (i + 1) % NUM_RING_POINTS;
                float dx = points[next].x - points[i].x;
                float dy = points[next].y - points[i].y;
                float len = std::sqrt(dx * dx + dy * dy);
                if (len < 1e-4f) continue;

                float nx = dy / len;
                float ny = -dx / len;

                float pfx = nx * pressure_force * 0.5f;
                float pfy = ny * pressure_force * 0.5f;

                if (!points[i].pinned)    { points[i].x += pfx; points[i].y += pfy; }
                if (!points[next].pinned) { points[next].x += pfx; points[next].y += pfy; }
            }
        }

        // 3. Коллизии с круглыми штырями
        for (int i = 0; i < TOTAL_POINTS; i++) {
            for (int o = 0; o < obstacle_count; o++) {
                float dx = points[i].x - obstacles[o].x;
                float dy = points[i].y - obstacles[o].y;
                float dist_sq = dx * dx + dy * dy;
                float min_d = obstacles[o].radius + 5.0f;

                if (dist_sq < min_d * min_d && dist_sq > 1e-4f) {
                    float dist = std::sqrt(dist_sq);
                    float diff = (min_d - dist) / dist;
                    points[i].x += dx * diff;
                    points[i].y += dy * diff;
                }
            }
        }

        // 4. Коллизии с наклонными платформами и батутами (Segment-Point PBD)
        for (int i = 0; i < TOTAL_POINTS; i++) {
            for (int s = 0; s < segment_count; s++) {
                float sx = segments[s].x2 - segments[s].x1;
                float sy = segments[s].y2 - segments[s].y1;
                float seg_len_sq = sx * sx + sy * sy;
                if (seg_len_sq < 1e-4f) continue;

                // Проекция точки на отрезок [0, 1]
                float t = ((points[i].x - segments[s].x1) * sx + (points[i].y - segments[s].y1) * sy) / seg_len_sq;
                t = constrain(t, 0.0f, 1.0f);

                float closest_x = segments[s].x1 + t * sx;
                float closest_y = segments[s].y1 + t * sy;

                float dx = points[i].x - closest_x;
                float dy = points[i].y - closest_y;
                float dist_sq = dx * dx + dy * dy;
                float min_d = segments[s].thickness + 5.0f;

                if (dist_sq < min_d * min_d && dist_sq > 1e-4f) {
                    float dist = std::sqrt(dist_sq);
                    float diff = (min_d - dist) / dist;

                    points[i].x += dx * diff;
                    points[i].y += dy * diff;

                    // Если это неоновый батут — придаем супер-импульс
                    if (segments[s].is_trampoline && iter == 0) {
                        float nx = dx / dist;
                        float ny = dy / dist;
                        float vx = points[i].x - points[i].old_x;
                        float vy = points[i].y - points[i].old_y;
                        float v_dot_n = vx * nx + vy * ny;
                        if (v_dot_n < 0.0f) {
                            // Отражаем и ускоряем скорость отскока
                            points[i].old_x = points[i].x - (vx - 2.8f * v_dot_n * nx);
                            points[i].old_y = points[i].y - (vy - 2.8f * v_dot_n * ny);
                        }
                    }
                }
            }
        }
    }

    // 5. Границы мира
    for (int i = 0; i < TOTAL_POINTS; i++) {
        if (points[i].x < 20.0f) {
            float vx = points[i].x - points[i].old_x;
            points[i].x = 20.0f;
            points[i].old_x = points[i].x + vx * BOUNCE;
        } else if (points[i].x > WORLD_WIDTH - 20.0f) {
            float vx = points[i].x - points[i].old_x;
            points[i].x = WORLD_WIDTH - 20.0f;
            points[i].old_x = points[i].x + vx * BOUNCE;
        }

        if (points[i].y < 20.0f) {
            float vy = points[i].y - points[i].old_y;
            points[i].y = 20.0f;
            points[i].old_y = points[i].y + vy * BOUNCE;
        } else if (points[i].y > WORLD_HEIGHT - 20.0f) {
            float vy = points[i].y - points[i].old_y;
            float vx = points[i].x - points[i].old_x;
            points[i].y = WORLD_HEIGHT - 20.0f;
            points[i].old_y = points[i].y + vy * BOUNCE;
            points[i].old_x = points[i].x - vx * 0.85f;
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    init_soft_body(WORLD_WIDTH * 0.5f, 120.0f, body_radius);
}

void loop() {
    uint32_t start_time = micros();

    process_serial_input();
    verlet_integrate();
    solve_constraints();

    for (int i = 0; i < TOTAL_POINTS; i++) {
        Serial.printf("%.1f,%.1f%c", points[i].x, points[i].y, (i == TOTAL_POINTS - 1) ? '\n' : ';');
    }

    uint32_t elapsed = micros() - start_time;
    if (elapsed < 16000) {
        delayMicroseconds(16000 - elapsed);
    }
}
