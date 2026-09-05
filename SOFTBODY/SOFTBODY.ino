#include <Arduino.h>
#include <cmath>

static const int NUM_RING_POINTS = 16;
static const int TOTAL_POINTS = NUM_RING_POINTS + 1;
static const int CENTER_IDX = NUM_RING_POINTS;

static const float WORLD_WIDTH  = 800.0f;
static const float WORLD_HEIGHT = 600.0f;
static const float BASE_DT = 0.016f;

float GRAVITY = 550.0f;
float DRAG = 0.992f;
float BOUNCE = 0.55f;
int SOLVER_ITERATIONS = 4;
float body_radius = 55.0f;
float target_area = 0.0f;
float PRESSURE_K = 0.75f;

float time_scale = 1.0f;
float current_dt = BASE_DT;
float prev_dt = BASE_DT;

struct Point {
    float x, y;
    float old_x, old_y;
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

struct Fan {
    float x, y, w, h;
    float fx, fy;
};

struct Vortex {
    float x, y;
    float strength;
};

Point points[TOTAL_POINTS];

static const int MAX_SPRINGS = NUM_RING_POINTS;
Spring springs[MAX_SPRINGS];
int spring_count = 0;

static const int MAX_OBSTACLES = 32;
Obstacle obstacles[MAX_OBSTACLES];
int obstacle_count = 0;

static const int MAX_SEGMENTS = 16;
Segment segments[MAX_SEGMENTS];
int segment_count = 0;

static const int MAX_FANS = 4;
Fan fans[MAX_FANS];
int fan_count = 0;

static const int MAX_VORTICES = 4;
Vortex vortices[MAX_VORTICES];
int vortex_count = 0;

float ext_fx = 0.0f;
float ext_fy = 0.0f;
bool is_dragging = false;
float drag_target_x = 0.0f;
float drag_target_y = 0.0f;

char rx_buf[96];
int rx_idx = 0;

void init_soft_body(float cx, float cy, float radius) {
    body_radius = radius;
    target_area = 0.5f * NUM_RING_POINTS * radius * radius * std::sin(2.0f * (float)M_PI / NUM_RING_POINTS);

    for (int i = 0; i < NUM_RING_POINTS; i++) {
        float angle = i * (2.0f * (float)M_PI / NUM_RING_POINTS);
        float x = cx + radius * std::cos(angle);
        float y = cy + radius * std::sin(angle);
        points[i] = { x, y, x, y };
    }
    points[CENTER_IDX] = { cx, cy, cx, cy };

    spring_count = 0;
    for (int i = 0; i < NUM_RING_POINTS; i++) {
        int next = (i + 1) % NUM_RING_POINTS;
        float dx = points[i].x - points[next].x;
        float dy = points[i].y - points[next].y;
        springs[spring_count++] = { i, next, std::sqrt(dx * dx + dy * dy) };
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
        target_area = 0.5f * NUM_RING_POINTS * new_radius * new_radius * std::sin(2.0f * (float)M_PI / NUM_RING_POINTS);
    }
}

void process_serial_input() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (rx_idx > 0) {
                rx_buf[rx_idx] = '\0';
                
                if (rx_buf[0] == 'R') {
                    init_soft_body(WORLD_WIDTH * 0.5f, 130.0f, body_radius);
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
                    fan_count = 0;
                    vortex_count = 0;
                }
                else if (rx_buf[0] == '+' && rx_buf[1] == 'O') {
                    float ox, oy, orad;
                    if (sscanf(rx_buf, "+O %f %f %f", &ox, &oy, &orad) == 3) {
                        if (obstacle_count < MAX_OBSTACLES) obstacles[obstacle_count++] = { ox, oy, orad };
                    }
                }
                else if (rx_buf[0] == '+' && rx_buf[1] == 'S') {
                    float x1, y1, x2, y2, th;
                    int tramp;
                    if (sscanf(rx_buf, "+S %f %f %f %f %f %d", &x1, &y1, &x2, &y2, &th, &tramp) == 6) {
                        if (segment_count < MAX_SEGMENTS) {
                            segments[segment_count++] = { x1, y1, x2, y2, th, tramp == 1 };
                        }
                    }
                }
                else if (rx_buf[0] == '+' && rx_buf[1] == 'F') {
                    float fx, fy, fw, fh, f_force_x, f_force_y;
                    if (sscanf(rx_buf, "+F %f %f %f %f %f %f", &fx, &fy, &fw, &fh, &f_force_x, &f_force_y) == 6) {
                        if (fan_count < MAX_FANS) fans[fan_count++] = { fx, fy, fw, fh, f_force_x, f_force_y };
                    }
                }
                else if (rx_buf[0] == '+' && rx_buf[1] == 'V') {
                    float vx, vy, vstr;
                    if (sscanf(rx_buf, "+V %f %f %f", &vx, &vy, &vstr) == 3) {
                        if (vortex_count < MAX_VORTICES) vortices[vortex_count++] = { vx, vy, vstr };
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
                        if (cnt >= 5) PRESSURE_K = constrain(p, 0.0f, 1.0f);
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

    float cx = 0, cy = 0;
    for (int i = 0; i < NUM_RING_POINTS; i++) {
        cx += points[i].x;
        cy += points[i].y;
    }
    cx /= NUM_RING_POINTS;
    cy /= NUM_RING_POINTS;

    if (is_dragging) {
        float pull = 0.25f * time_scale;
        float shift_x = (drag_target_x - cx) * pull;
        float shift_y = (drag_target_y - cy) * pull;
        for (int i = 0; i < NUM_RING_POINTS; i++) {
            points[i].x += shift_x;
            points[i].y += shift_y;
        }
    }

    float dt_sq = current_dt * current_dt;

    for (int i = 0; i < NUM_RING_POINTS; i++) {
        float vx = (points[i].x - points[i].old_x) * time_ratio * DRAG;
        float vy = (points[i].y - points[i].old_y) * time_ratio * DRAG;

        points[i].old_x = points[i].x;
        points[i].old_y = points[i].y;

        // Вентилятор: аэродинамический подъем + центрирование в потоке
        float fan_ax = 0.0f, fan_ay = 0.0f;
        for (int f = 0; f < fan_count; f++) {
            if (points[i].x >= fans[f].x && points[i].x <= fans[f].x + fans[f].w &&
                points[i].y >= fans[f].y && points[i].y <= fans[f].y + fans[f].h) {
                // Центрирование к середине струи
                float fan_center_x = fans[f].x + fans[f].w * 0.5f;
                fan_ax += (fan_center_x - points[i].x) * 12.0f;
                // Мощный восходящий поток
                fan_ay += fans[f].fy;
            }
        }

        // Черная дыра: мощная гравитация + вихревое вращение
        float vortex_ax = 0.0f, vortex_ay = 0.0f;
        for (int v = 0; v < vortex_count; v++) {
            float dx = vortices[v].x - points[i].x;
            float dy = vortices[v].y - points[i].y;
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist > 10.0f && dist < 380.0f) {
                // Линейно-обратный закон тяги, превосходящий гравитацию в разы
                float pull = vortices[v].strength * (1.0f - dist / 380.0f) * 4200.0f;
                float nx = dx / dist;
                float ny = dy / dist;
                float swirl = pull * 0.75f;
                vortex_ax += nx * pull - ny * swirl;
                vortex_ay += ny * pull + nx * swirl;
            }
        }

        points[i].x += vx + (ext_fx + fan_ax + vortex_ax) * dt_sq;
        points[i].y += vy + (GRAVITY + ext_fy + fan_ay + vortex_ay) * dt_sq;
    }
}

void solve_constraints() {
    bool tramp_hit = false;
    float tramp_launch_nx = 0.0f;
    float tramp_launch_ny = 0.0f;

    for (int iter = 0; iter < SOLVER_ITERATIONS; iter++) {
        // 1. Оболочка (пружины)
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

            p1.x += off_x; p1.y += off_y;
            p2.x -= off_x; p2.y -= off_y;
        }

        // 2. Газовое давление (PBD Area Constraint)
        if (PRESSURE_K > 0.001f) {
            float cur_area = 0.0f;
            for (int i = 0; i < NUM_RING_POINTS; i++) {
                int next = (i + 1) % NUM_RING_POINTS;
                cur_area += points[i].x * points[next].y - points[next].x * points[i].y;
            }
            cur_area = 0.5f * cur_area;

            float gx[NUM_RING_POINTS];
            float gy[NUM_RING_POINTS];
            float sum_grad_sq = 0.0f;

            for (int i = 0; i < NUM_RING_POINTS; i++) {
                int prev = (i - 1 + NUM_RING_POINTS) % NUM_RING_POINTS;
                int next = (i + 1) % NUM_RING_POINTS;
                gx[i] = 0.5f * (points[next].y - points[prev].y);
                gy[i] = 0.5f * (points[prev].x - points[next].x);
                sum_grad_sq += gx[i] * gx[i] + gy[i] * gy[i];
            }

            if (sum_grad_sq > 1e-4f) {
                float lambda = (target_area - cur_area) / sum_grad_sq;
                float force = lambda * PRESSURE_K;
                for (int i = 0; i < NUM_RING_POINTS; i++) {
                    points[i].x += gx[i] * force;
                    points[i].y += gy[i] * force;
                }
            }
        }

        // 3. Коллизии со штырями
        for (int i = 0; i < NUM_RING_POINTS; i++) {
            for (int o = 0; o < obstacle_count; o++) {
                float dx = points[i].x - obstacles[o].x;
                float dy = points[i].y - obstacles[o].y;
                float dist_sq = dx * dx + dy * dy;
                float min_d = obstacles[o].radius + 4.0f;

                if (dist_sq < min_d * min_d && dist_sq > 1e-4f) {
                    float dist = std::sqrt(dist_sq);
                    float diff = (min_d - dist) / dist;
                    points[i].x += dx * diff;
                    points[i].y += dy * diff;
                }
            }
        }

        // 4. Коллизии с платформами и батутами
        for (int i = 0; i < NUM_RING_POINTS; i++) {
            for (int s = 0; s < segment_count; s++) {
                float sx = segments[s].x2 - segments[s].x1;
                float sy = segments[s].y2 - segments[s].y1;
                float seg_len_sq = sx * sx + sy * sy;
                if (seg_len_sq < 1e-4f) continue;

                float t = ((points[i].x - segments[s].x1) * sx + (points[i].y - segments[s].y1) * sy) / seg_len_sq;
                t = constrain(t, 0.0f, 1.0f);

                float closest_x = segments[s].x1 + t * sx;
                float closest_y = segments[s].y1 + t * sy;

                float dx = points[i].x - closest_x;
                float dy = points[i].y - closest_y;
                float dist_sq = dx * dx + dy * dy;
                float min_d = segments[s].thickness + 4.0f;

                if (dist_sq < min_d * min_d) {
                    float dist = std::sqrt(dist_sq);
                    if (dist < 1e-4f) { dist = 1e-4f; dx = 0; dy = -1.0f; }

                    // Строгая внешняя нормаль для батута (всегда направлена наружу/вверх)
                    float nx = dx / dist;
                    float ny = dy / dist;

                    if (segments[s].is_trampoline) {
                        float slen = std::sqrt(seg_len_sq);
                        float up_nx = sy / slen;
                        float up_ny = -sx / slen;
                        if (up_ny > 0.0f) { up_nx = -up_nx; up_ny = -up_ny; }
                        tramp_launch_nx = up_nx;
                        tramp_launch_ny = up_ny;
                        tramp_hit = true;
                    }

                    float overlap = min_d - dist;
                    points[i].x += nx * overlap;
                    points[i].y += ny * overlap;

                    float vx = points[i].x - points[i].old_x;
                    float vy = points[i].y - points[i].old_y;
                    float vn = vx * nx + vy * ny;

                    if (!segments[s].is_trampoline && vn < 0.0f) {
                        points[i].old_x = points[i].x - (vx - (1.0f + BOUNCE) * vn * nx);
                        points[i].old_y = points[i].y - (vy - (1.0f + BOUNCE) * vn * ny);
                    }
                }
            }
        }
    }

    // МОЩНЫЙ КАТАПУЛЬТИРУЮЩИЙ ИМПУЛЬС БАТУТА ДЛЯ ВСЕГО ШАРА
    if (tramp_hit) {
        const float LAUNCH_SPEED = 26.0f; // Взрывной катапультирующий импульс
        for (int j = 0; j < NUM_RING_POINTS; j++) {
            float vx = points[j].x - points[j].old_x;
            float vy = points[j].y - points[j].old_y;
            float vn = vx * tramp_launch_nx + vy * tramp_launch_ny;

            if (vn < LAUNCH_SPEED) {
                // Выравниваем вектор скорости всего тела вдоль нормали батута
                points[j].old_x = points[j].x - (vx - vn * tramp_launch_nx + tramp_launch_nx * LAUNCH_SPEED);
                points[j].old_y = points[j].y - (vy - vn * tramp_launch_ny + tramp_launch_ny * LAUNCH_SPEED);
            }
        }
    }

    // 5. Границы мира
    for (int i = 0; i < NUM_RING_POINTS; i++) {
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

    // 6. Центр масс
    float cx = 0, cy = 0;
    for (int i = 0; i < NUM_RING_POINTS; i++) {
        cx += points[i].x;
        cy += points[i].y;
    }
    points[CENTER_IDX].x = cx / NUM_RING_POINTS;
    points[CENTER_IDX].y = cy / NUM_RING_POINTS;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    init_soft_body(WORLD_WIDTH * 0.5f, 130.0f, body_radius);
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
