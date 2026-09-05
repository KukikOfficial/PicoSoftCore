#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <cmath>

// Пины дисплея ILI9341 (Аппаратный SPI0)
#define TFT_CS    17
#define TFT_DC    20
#define TFT_RST   21
#define BTN_PIN   15  // Кнопка смены пресетов (к GND)

// Опциональный джойстик (раскомментируйте, если подключен)
#define JOY_X_PIN 26
#define JOY_Y_PIN 27

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// Разрешение экрана в альбомной ориентации
static const float WORLD_WIDTH  = 320.0f;
static const float WORLD_HEIGHT = 240.0f;
static const float DT = 0.016f;

// Физика
static const int NUM_RING_POINTS = 16;
static const int TOTAL_POINTS = NUM_RING_POINTS + 1;
static const int CENTER_IDX = NUM_RING_POINTS;

float GRAVITY = 500.0f;
float DRAG = 0.992f;
float BOUNCE = 0.55f;
int SOLVER_ITERATIONS = 4;
float body_radius = 22.0f;
float target_area = 0.0f;
float PRESSURE_K = 0.75f;

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
Point prev_rendered_points[TOTAL_POINTS]; // Для стирания старого кадра
bool first_frame = true;

Spring springs[NUM_RING_POINTS];
int spring_count = 0;

Obstacle obstacles[16];
int obstacle_count = 0;

Segment segments[8];
int segment_count = 0;

Fan fans[2];
int fan_count = 0;

Vortex vortices[2];
int vortex_count = 0;

// Мини-игра: Баскетбол
bool hoop_active = false;
float hoop_x = 270.0f;
float hoop_y = 120.0f;
int score = 0;
float goal_fx_timer = 0.0f;

// Автономные пресеты
int current_preset = 0;
uint32_t auto_demo_timer = 0;
float fan_anim_offset = 0.0f;

void init_soft_body(float cx, float cy, float radius) {
    body_radius = radius;
    target_area = 0.5f * NUM_RING_POINTS * radius * radius * std::sin(2.0f * (float)M_PI / NUM_RING_POINTS);

    for (int i = 0; i < NUM_RING_POINTS; i++) {
        float angle = i * (2.0f * (float)M_PI / NUM_RING_POINTS);
        float x = cx + radius * std::cos(angle);
        float y = cy + radius * std::sin(angle);
        points[i] = { x, y, x, y };
        prev_rendered_points[i] = points[i];
    }
    points[CENTER_IDX] = { cx, cy, cx, cy };
    prev_rendered_points[CENTER_IDX] = points[CENTER_IDX];

    spring_count = 0;
    for (int i = 0; i < NUM_RING_POINTS; i++) {
        int next = (i + 1) % NUM_RING_POINTS;
        float dx = points[i].x - points[next].x;
        float dy = points[i].y - points[next].y;
        springs[spring_count++] = { i, next, std::sqrt(dx * dx + dy * dy) };
    }
}

void clear_world() {
    obstacle_count = 0;
    segment_count = 0;
    fan_count = 0;
    vortex_count = 0;
    hoop_active = false;
    tft.fillScreen(ILI9341_BLACK);
}

// 1. Пресет: Баскетбол
void load_preset_basketball() {
    clear_world();
    hoop_active = true;
    hoop_x = 270.0f;
    hoop_y = 120.0f;
    
    // Рампа спуска
    segments[segment_count++] = { 20.0f, 60.0f, 110.0f, 180.0f, 5.0f, false };
    // Супер-батут
    segments[segment_count++] = { 120.0f, 215.0f, 210.0f, 215.0f, 6.0f, true };
    // Штырь-рикошет сверху
    obstacles[obstacle_count++] = { 210.0f, 80.0f, 10.0f };

    init_soft_body(40.0f, 45.0f, 18.0f);
}

// 2. Пресет: Аэротруба
void load_preset_wind() {
    clear_world();
    // Направляющие стены
    segments[segment_count++] = { 100.0f, 40.0f, 100.0f, 220.0f, 5.0f, false };
    segments[segment_count++] = { 220.0f, 40.0f, 220.0f, 220.0f, 5.0f, false };
    // Мощный восходящий поток
    fans[fan_count++] = { 105.0f, 60.0f, 110.0f, 160.0f, 0.0f, -3400.0f };

    init_soft_body(160.0f, 50.0f, 20.0f);
}

// 3. Пресет: Скейт-парк
void load_preset_skate() {
    clear_world();
    segments[segment_count++] = { 20.0f, 70.0f, 110.0f, 190.0f, 5.0f, false };
    segments[segment_count++] = { 120.0f, 220.0f, 200.0f, 220.0f, 6.0f, true };
    segments[segment_count++] = { 210.0f, 190.0f, 300.0f, 90.0f, 5.0f, false };
    obstacles[obstacle_count++] = { 160.0f, 75.0f, 10.0f };

    init_soft_body(35.0f, 50.0f, 18.0f);
}

// 4. Пресет: Черная дыра
void load_preset_vortex() {
    clear_world();
    vortices[vortex_count++] = { 160.0f, 120.0f, 1.0f };
    obstacles[obstacle_count++] = { 80.0f, 60.0f, 8.0f };
    obstacles[obstacle_count++] = { 240.0f, 180.0f, 8.0f };

    init_soft_body(160.0f, 40.0f, 18.0f);
}

// 5. Пресет: Плинко
void load_preset_plinko() {
    clear_world();
    for (int r = 0; r < 4; r++) {
        int cnt = r + 3;
        float y = 70.0f + r * 35.0f;
        float sp = 42.0f;
        float sx = WORLD_WIDTH * 0.5f - ((cnt - 1) * sp) * 0.5f;
        for (int c = 0; c < cnt; c++) {
            obstacles[obstacle_count++] = { sx + c * sp, y, 7.0f };
        }
    }
    init_soft_body(160.0f, 30.0f, 18.0f);
}

void switch_preset() {
    current_preset = (current_preset + 1) % 5;
    auto_demo_timer = millis();
    score = 0;
    
    switch (current_preset) {
        case 0: load_preset_basketball(); break;
        case 1: load_preset_wind(); break;
        case 2: load_preset_skate(); break;
        case 3: load_preset_vortex(); break;
        case 4: load_preset_plinko(); break;
    }
}

// Физическая интеграция Верле
void verlet_integrate() {
    float dt_sq = DT * DT;

    // Чтение джойстика (если подключен)
    float joy_fx = 0.0f, joy_fy = 0.0f;
    int jx = analogRead(JOY_X_PIN) - 512;
    int jy = analogRead(JOY_Y_PIN) - 512;
    if (abs(jx) > 50) joy_fx = (jx / 512.0f) * 1200.0f;
    if (abs(jy) > 50) joy_fy = (jy / 512.0f) * 1200.0f;

    for (int i = 0; i < NUM_RING_POINTS; i++) {
        float vx = (points[i].x - points[i].old_x) * DRAG;
        float vy = (points[i].y - points[i].old_y) * DRAG;

        points[i].old_x = points[i].x;
        points[i].old_y = points[i].y;

        float fan_ax = 0.0f, fan_ay = 0.0f;
        for (int f = 0; f < fan_count; f++) {
            if (points[i].x >= fans[f].x && points[i].x <= fans[f].x + fans[f].w &&
                points[i].y >= fans[f].y && points[i].y <= fans[f].y + fans[f].h) {
                float center_fan_x = fans[f].x + fans[f].w * 0.5f;
                fan_ax += (center_fan_x - points[i].x) * 10.0f;
                fan_ay += fans[f].fy;
            }
        }

        float vortex_ax = 0.0f, vortex_ay = 0.0f;
        for (int v = 0; v < vortex_count; v++) {
            float dx = vortices[v].x - points[i].x;
            float dy = vortices[v].y - points[i].y;
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist > 8.0f && dist < 180.0f) {
                float pull = vortices[v].strength * (1.0f - dist / 180.0f) * 3200.0f;
                float nx = dx / dist;
                float ny = dy / dist;
                float swirl = pull * 0.7f;
                vortex_ax += nx * pull - ny * swirl;
                vortex_ay += ny * pull + nx * swirl;
            }
        }

        points[i].x += vx + (joy_fx + fan_ax + vortex_ax) * dt_sq;
        points[i].y += vy + (GRAVITY + joy_fy + fan_ay + vortex_ay) * dt_sq;
    }
}

void solve_constraints() {
    bool tramp_hit = false;
    float tramp_launch_nx = 0.0f;
    float tramp_launch_ny = 0.0f;

    for (int iter = 0; iter < SOLVER_ITERATIONS; iter++) {
        // Оболочка
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

        // Давление
        if (PRESSURE_K > 0.001f) {
            float cur_area = 0.0f;
            for (int i = 0; i < NUM_RING_POINTS; i++) {
                int next = (i + 1) % NUM_RING_POINTS;
                cur_area += points[i].x * points[next].y - points[next].x * points[i].y;
            }
            cur_area = 0.5f * cur_area;

            float gx[NUM_RING_POINTS], gy[NUM_RING_POINTS];
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

        // Штыри
        for (int i = 0; i < NUM_RING_POINTS; i++) {
            for (int o = 0; o < obstacle_count; o++) {
                float dx = points[i].x - obstacles[o].x;
                float dy = points[i].y - obstacles[o].y;
                float dist_sq = dx * dx + dy * dy;
                float min_d = obstacles[o].radius + 3.0f;

                if (dist_sq < min_d * min_d && dist_sq > 1e-4f) {
                    float dist = std::sqrt(dist_sq);
                    float diff = (min_d - dist) / dist;
                    points[i].x += dx * diff;
                    points[i].y += dy * diff;
                }
            }
        }

        // Платформы и батуты
        for (int i = 0; i < NUM_RING_POINTS; i++) {
            for (int s = 0; s < segment_count; s++) {
                float sx = segments[s].x2 - segments[s].x1;
                float sy = segments[s].y2 - segments[s].y1;
                float seg_len_sq = sx * sx + sy * sy;
                if (seg_len_sq < 1e-4f) continue;

                float t = ((points[i].x - segments[s].x1) * sx + (points[i].y - segments[s].y1) * sy) / seg_len_sq;
                t = constrain(t, 0.0f, 1.0f);

                float cx = segments[s].x1 + t * sx;
                float cy = segments[s].y1 + t * sy;
                float dx = points[i].x - cx;
                float dy = points[i].y - cy;
                float dist_sq = dx * dx + dy * dy;
                float min_d = segments[s].thickness + 3.0f;

                if (dist_sq < min_d * min_d) {
                    float dist = std::sqrt(dist_sq);
                    if (dist < 1e-4f) { dist = 1e-4f; dx = 0; dy = -1.0f; }

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

    // Катапультирующий батут для всего тела
    if (tramp_hit) {
        const float LAUNCH_SPEED = 20.0f;
        for (int j = 0; j < NUM_RING_POINTS; j++) {
            float vx = points[j].x - points[j].old_x;
            float vy = points[j].y - points[j].old_y;
            float vn = vx * tramp_launch_nx + vy * tramp_launch_ny;
            if (vn < LAUNCH_SPEED) {
                points[j].old_x = points[j].x - (vx - vn * tramp_launch_nx + tramp_launch_nx * LAUNCH_SPEED);
                points[j].old_y = points[j].y - (vy - vn * tramp_launch_ny + tramp_launch_ny * LAUNCH_SPEED);
            }
        }
    }

    // Границы экрана
    for (int i = 0; i < NUM_RING_POINTS; i++) {
        if (points[i].x < 8.0f) {
            float vx = points[i].x - points[i].old_x;
            points[i].x = 8.0f;
            points[i].old_x = points[i].x + vx * BOUNCE;
        } else if (points[i].x > WORLD_WIDTH - 8.0f) {
            float vx = points[i].x - points[i].old_x;
            points[i].x = WORLD_WIDTH - 8.0f;
            points[i].old_x = points[i].x + vx * BOUNCE;
        }

        if (points[i].y < 8.0f) {
            float vy = points[i].y - points[i].old_y;
            points[i].y = 8.0f;
            points[i].old_y = points[i].y + vy * BOUNCE;
        } else if (points[i].y > WORLD_HEIGHT - 8.0f) {
            float vy = points[i].y - points[i].old_y;
            float vx = points[i].x - points[i].old_x;
            points[i].y = WORLD_HEIGHT - 8.0f;
            points[i].old_y = points[i].y + vy * BOUNCE;
            points[i].old_x = points[i].x - vx * 0.85f;
        }
    }

    // Центр масс
    float cx = 0, cy = 0;
    for (int i = 0; i < NUM_RING_POINTS; i++) {
        cx += points[i].x;
        cy += points[i].y;
    }
    points[CENTER_IDX].x = cx / NUM_RING_POINTS;
    points[CENTER_IDX].y = cy / NUM_RING_POINTS;

    // Детекция гола в кольцо
    if (hoop_active) {
        float hx = hoop_x, hy = hoop_y;
        float prev_y = prev_rendered_points[CENTER_IDX].y;
        float cur_y = points[CENTER_IDX].y;
        if (prev_y <= hy && cur_y >= hy && abs(points[CENTER_IDX].x - hx) < 18.0f) {
            score++;
            goal_fx_timer = 1.0f;
        }
    }
}

// Отрисовка статических объектов
void render_static_world() {
    // 1. Вентиляторы
    for (int f = 0; f < fan_count; f++) {
        int fx = fans[f].x, fy = fans[f].y, fw = fans[f].w, fh = fans[f].h;
        tft.drawRect(fx, fy, fw, fh, ILI9341_DARKCYAN);
        tft.fillRect(fx, fy + fh - 4, fw, 4, ILI9341_LIGHTGREY);
        // Анимированные полосы ветра
        int off = (int)fan_anim_offset % 25;
        for (int y = fh - off; y > 0; y -= 25) {
            tft.drawFastHLine(fx + 4, fy + y, fw - 8, ILI9341_CYAN);
        }
    }

    // 2. Черные дыры
    for (int v = 0; v < vortex_count; v++) {
        int vx = vortices[v].x, vy = vortices[v].y;
        tft.drawCircle(vx, vy, 24, ILI9341_MAGENTA);
        tft.drawCircle(vx, vy, 14, ILI9341_PURPLE);
        tft.fillCircle(vx, vy, 5, ILI9341_WHITE);
    }

    // 3. Баскетбольное кольцо
    if (hoop_active) {
        int hx = hoop_x, hy = hoop_y;
        tft.fillRect(hx + 20, hy - 25, 4, 35, ILI9341_WHITE); // Щит
        tft.drawFastHLine(hx - 20, hy, 40, ILI9341_RED);      // Обод
        // Сетка
        tft.drawLine(hx - 15, hy, hx - 8, hy + 18, ILI9341_WHITE);
        tft.drawLine(hx + 15, hy, hx + 8, hy + 18, ILI9341_WHITE);
        tft.drawFastHLine(hx - 8, hy + 18, 16, ILI9341_WHITE);

        // Счет
        tft.setCursor(240, 8);
        tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
        tft.setTextSize(1);
        tft.printf("SCORE: %d", score);

        if (goal_fx_timer > 0.0f) {
            tft.setCursor(hx - 15, hy - 35);
            tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
            tft.print("GOAL!");
        }
    }

    // 4. Платформы и батуты
    for (int s = 0; s < segment_count; s++) {
        uint16_t col = segments[s].is_trampoline ? ILI9341_MAGENTA : ILI9341_LIGHTGREY;
        tft.drawLine(segments[s].x1, segments[s].y1, segments[s].x2, segments[s].y2, col);
        // Небольшая толщина
        tft.drawLine(segments[s].x1, segments[s].y1 + 1, segments[s].x2, segments[s].y2 + 1, col);
        tft.drawLine(segments[s].x1, segments[s].y1 - 1, segments[s].x2, segments[s].y2 - 1, col);
    }

    // 5. Штыри
    for (int o = 0; o < obstacle_count; o++) {
        tft.fillCircle(obstacles[o].x, obstacles[o].y, obstacles[o].radius, ILI9341_DARKGREY);
        tft.drawCircle(obstacles[o].x, obstacles[o].y, obstacles[o].radius, ILI9341_WHITE);
    }

    // Рамка арены
    tft.drawRect(2, 2, WORLD_WIDTH - 4, WORLD_HEIGHT - 4, ILI9341_NAVY);
}

// Быстрый рендеринг слайма без мерцания (Selective Redraw)
void render_soft_body() {
    // 1. Стираем старое положение слайма черным цветом
    if (!first_frame) {
        int old_cx = prev_rendered_points[CENTER_IDX].x;
        int old_cy = prev_rendered_points[CENTER_IDX].y;

        for (int i = 0; i < NUM_RING_POINTS; i++) {
            int next = (i + 1) % NUM_RING_POINTS;
            tft.fillTriangle(old_cx, old_cy,
                             prev_rendered_points[i].x, prev_rendered_points[i].y,
                             prev_rendered_points[next].x, prev_rendered_points[next].y,
                             ILI9341_BLACK);
            tft.drawLine(prev_rendered_points[i].x, prev_rendered_points[i].y,
                         prev_rendered_points[next].x, prev_rendered_points[next].y,
                         ILI9341_BLACK);
        }
    }

    // 2. Перерисовываем элементы мира (на случай, если слайм их частично затер)
    render_static_world();

    // 3. Рисуем новое положение слайма
    int cx = points[CENTER_IDX].x;
    int cy = points[CENTER_IDX].y;

    for (int i = 0; i < NUM_RING_POINTS; i++) {
        int next = (i + 1) % NUM_RING_POINTS;
        // Заливка треугольников тела
        tft.fillTriangle(cx, cy,
                         points[i].x, points[i].y,
                         points[next].x, points[next].y,
                         ILI9341_CYAN);
        // Контур
        tft.drawLine(points[i].x, points[i].y,
                     points[next].x, points[next].y,
                     ILI9341_WHITE);
    }

    // 4. Живые глазки
    int eye_spacing = max(3, (int)(body_radius * 0.25f));
    int eye_r = max(2, (int)(body_radius * 0.15f));
    tft.fillCircle(cx - eye_spacing, cy - 2, eye_r, ILI9341_WHITE);
    tft.fillCircle(cx + eye_spacing, cy - 2, eye_r, ILI9341_WHITE);
    tft.fillCircle(cx - eye_spacing, cy - 2, max(1, eye_r / 2), ILI9341_BLACK);
    tft.fillCircle(cx + eye_spacing, cy - 2, max(1, eye_r / 2), ILI9341_BLACK);

    // Сохраняем положение для стирания на следующем кадре
    for (int i = 0; i < TOTAL_POINTS; i++) {
        prev_rendered_points[i] = points[i];
    }
    first_frame = false;
}

void setup() {
    Serial.begin(115200);

    pinMode(BTN_PIN, INPUT_PULLUP);
    pinMode(JOY_X_PIN, INPUT);
    pinMode(JOY_Y_PIN, INPUT);

    // Настраиваем пины SPI0 для Pico
    SPI.setRX(16);
    SPI.setCS(TFT_CS);
    SPI.setSCK(18);
    SPI.setTX(19);

    // Запуск дисплея на максимальной стабильной частоте SPI 40 МГц
    tft.begin(40000000);
    tft.setRotation(1); // Альбомный режим (320x240)
    tft.fillScreen(ILI9341_BLACK);

    // Экран приветствия
    tft.setTextColor(ILI9341_CYAN);
    tft.setTextSize(2);
    tft.setCursor(50, 90);
    tft.println("RP2040 SOFTBODY");
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(85, 120);
    tft.println("Stand-alone Physics");
    delay(1200);

    // Загружаем первый пресет
    load_preset_basketball();
    auto_demo_timer = millis();
}

void loop() {
    uint32_t start_us = micros();

    // 1. Опрос физической кнопки (смена пресетов)
    static bool prev_btn = HIGH;
    bool btn = digitalRead(BTN_PIN);
    if (prev_btn == HIGH && btn == LOW) {
        switch_preset();
        delay(50); // антидребезг
    }
    prev_btn = btn;

    // 2. Авто-демо (переключение пресета каждые 18 секунд, если не нажимают кнопку)
    if (millis() - auto_demo_timer > 18000) {
        switch_preset();
    }

    if (goal_fx_timer > 0.0f) {
        goal_fx_timer -= DT;
    }

    fan_anim_offset += 1.5f;

    // 3. Шаг физики Верле
    verlet_integrate();
    solve_constraints();

    // 4. Отрисовка кадра прямо на дисплей
    render_soft_body();

    // 5. Стабилизация 50-60 FPS (~16-20 мс)
    uint32_t elapsed_us = micros() - start_us;
    if (elapsed_us < 16666) {
        delayMicroseconds(16666 - elapsed_us);
    }
}
