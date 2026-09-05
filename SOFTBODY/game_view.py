import sys
import math
import random
from collections import deque
import serial
import serial.tools.list_ports
import pygame

SERIAL_PORT = None
BAUD_RATE = 115200

WORLD_W, WORLD_H = 800, 600
UI_WIDTH = 260
WINDOW_W = WORLD_W + UI_WIDTH
WINDOW_H = WORLD_H

NUM_RING_POINTS = 16
CENTER_IDX = NUM_RING_POINTS

def smooth_polygon(points, iterations=2):
    pts = list(points)
    for _ in range(iterations):
        new_pts = []
        n = len(pts)
        for i in range(n):
            p0 = pts[i]
            p1 = pts[(i + 1) % n]
            new_pts.append((0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1]))
            new_pts.append((0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1]))
        pts = new_pts
    return pts

class ParticleSystem:
    def __init__(self):
        self.particles = []

    def emit(self, x, y, count=10, speed=120.0, color_override=None):
        for _ in range(count):
            angle = random.uniform(0, 2 * math.pi)
            spd = random.uniform(speed * 0.3, speed)
            c = color_override or random.choice([(100, 220, 255), (140, 240, 255), (60, 160, 230)])
            self.particles.append({
                'x': x, 'y': y,
                'vx': math.cos(angle) * spd,
                'vy': math.sin(angle) * spd,
                'life': 1.0,
                'decay': random.uniform(1.8, 2.8),
                'r': random.uniform(2.5, 4.5),
                'color': c
            })

    def update(self, dt, time_scale):
        sim_dt = dt * time_scale
        for p in self.particles[:]:
            p['x'] += p['vx'] * sim_dt
            p['y'] += p['vy'] * sim_dt
            p['vy'] += 400.0 * sim_dt
            p['life'] -= p['decay'] * sim_dt
            if p['life'] <= 0:
                self.particles.remove(p)

    def draw(self, surf):
        for p in self.particles:
            alpha_r = max(1, int(p['r'] * p['life']))
            pygame.draw.circle(surf, p['color'], (int(p['x']), int(p['y'])), alpha_r)

class Button:
    def __init__(self, rect, text, callback, color=(42, 48, 62), hover_color=(56, 65, 84)):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.callback = callback
        self.color = color
        self.hover_color = hover_color
        self.active = False

    def draw(self, surf, font):
        mouse_pos = pygame.mouse.get_pos()
        is_hover = self.rect.collidepoint(mouse_pos)
        base_color = (65, 125, 195) if self.active else (self.hover_color if is_hover else self.color)
        
        pygame.draw.rect(surf, base_color, self.rect, border_radius=5)
        pygame.draw.rect(surf, (75, 90, 115), self.rect, 1, border_radius=5)
        
        txt_surf = font.render(self.text, True, (240, 245, 255))
        surf.blit(txt_surf, txt_surf.get_rect(center=self.rect.center))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.callback()
                return True
        return False

class Slider:
    def __init__(self, x, y, w, h, min_val, max_val, val, label, fmt="%.1f", active_color=(0, 175, 215)):
        self.rect = pygame.Rect(x, y, w, h)
        self.min_val = min_val
        self.max_val = max_val
        self.val = val
        self.label = label
        self.fmt = fmt
        self.active_color = active_color
        self.dragging = False

    def draw(self, surf, font):
        txt = f"{self.label}: {self.fmt % self.val}"
        surf.blit(font.render(txt, True, (175, 185, 200)), (self.rect.x, self.rect.y - 16))
        pygame.draw.rect(surf, (38, 43, 55), self.rect, border_radius=3)
        progress = (self.val - self.min_val) / (self.max_val - self.min_val)
        fill_w = int(self.rect.w * progress)
        pygame.draw.rect(surf, self.active_color, (self.rect.x, self.rect.y, fill_w, self.rect.h), border_radius=3)
        pygame.draw.circle(surf, (225, 240, 255), (self.rect.x + fill_w, self.rect.centery), 5)

    def handle_event(self, event):
        changed = False
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos) or math.hypot(event.pos[0] - (self.rect.x + (self.val - self.min_val)/(self.max_val - self.min_val)*self.rect.w), event.pos[1] - self.rect.centery) < 10:
                self.dragging = True
                changed = self._update_val(event.pos[0])
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            changed = self._update_val(event.pos[0])
        return changed

    def _update_val(self, mx):
        old_val = self.val
        progress = max(0.0, min(1.0, (mx - self.rect.x) / self.rect.w))
        self.val = self.min_val + progress * (self.max_val - self.min_val)
        return abs(self.val - old_val) > 1e-4

def auto_detect_port():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "pico" in desc or "2e8a" in hwid:
            return p.device
    ports = list(serial.tools.list_ports.comports())
    return ports[0].device if ports else None

def main():
    global SERIAL_PORT
    if SERIAL_PORT is None:
        SERIAL_PORT = auto_detect_port()
        if not SERIAL_PORT:
            print("[ОШИБКА] Плата не найдена.")
            sys.exit(1)

    print(f"[ИНФО] Подключение к: {SERIAL_PORT}")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("RP2040 SoftBody — Skate & Trampoline Sandbox")
    clock = pygame.time.Clock()

    font_s = pygame.font.SysFont("Arial", 11)
    font_m = pygame.font.SysFont("Arial", 12, bold=True)
    font_title = pygame.font.SysFont("Arial", 14, bold=True)
    font_hud = pygame.font.SysFont("Arial", 17, bold=True)

    obstacles = []
    segments = []
    mode = ['DRAG']
    particles = ParticleSystem()
    ghost_trail = deque(maxlen=7)

    # Переменные для рисования линий
    drag_start = None

    def set_mode(m): mode[0] = m
    def reset_body(): ser.write(b"R\n")
    def clear_all():
        obstacles.clear()
        segments.clear()
        ser.write(b"X\n")

    def sync_all_world():
        ser.write(b"X\n")
        for obs in obstacles:
            ser.write(f"+O {obs['x']:.1f} {obs['y']:.1f} {obs['r']:.1f}\n".encode('ascii'))
        for s in segments:
            t_flag = 1 if s['tramp'] else 0
            ser.write(f"+S {s['x1']:.1f} {s['y1']:.1f} {s['x2']:.1f} {s['y2']:.1f} {s['th']:.1f} {t_flag}\n".encode('ascii'))

    def spawn_skate_preset():
        clear_all()
        # 1. Большая наклонная рампа
        segments.append({'x1': 50.0, 'y1': 160.0, 'x2': 280.0, 'y2': 420.0, 'th': 12.0, 'tramp': False})
        # 2. Неоновый батут-катапульта снизу
        segments.append({'x1': 320.0, 'y1': 510.0, 'x2': 500.0, 'y2': 510.0, 'th': 14.0, 'tramp': True})
        # 3. Приемная наклонная рампа справа
        segments.append({'x1': 540.0, 'y1': 430.0, 'x2': 740.0, 'y2': 220.0, 'th': 12.0, 'tramp': False})
        # 4. Несколько штырей-мишеней вверху
        obstacles.append({'x': 410.0, 'y': 220.0, 'r': 22.0})
        obstacles.append({'x': 340.0, 'y': 150.0, 'r': 18.0})
        obstacles.append({'x': 480.0, 'y': 150.0, 'r': 18.0})
        sync_all_world()

    def spawn_plinko_preset():
        clear_all()
        rows = 4
        start_y = 190
        for r in range(rows):
            count = r + 3
            y = start_y + r * 80
            spacing = 95
            start_x = WORLD_W * 0.5 - ((count - 1) * spacing) * 0.5
            for c in range(count):
                obstacles.append({'x': start_x + c * spacing, 'y': y, 'r': 18.0})
        sync_all_world()

    ui_x = WORLD_W + 15

    # Кнопки инструментов
    btn_drag   = Button((ui_x, 38, 230, 22), "[~] Захват тела (ЛКМ)", lambda: set_mode('DRAG'))
    btn_pin    = Button((ui_x, 63, 230, 22), "[+] Штырь (Круг)", lambda: set_mode('ADD_PIN'))
    btn_ramp   = Button((ui_x, 88, 230, 22), "[/] Рампа / Платформа", lambda: set_mode('ADD_RAMP'))
    btn_tramp  = Button((ui_x, 113, 230, 22), "[^] Неоновый Батут", lambda: set_mode('ADD_TRAMP'))
    btn_del    = Button((ui_x, 138, 230, 22), "[-] Удалить объект", lambda: set_mode('DEL'))

    tool_buttons = [btn_drag, btn_pin, btn_ramp, btn_tramp, btn_del]

    # Слайдеры
    slider_time     = Slider(ui_x, 185, 230, 6, 0.05, 1.5, 1.0, "Скорость времени", "%.2fx", active_color=(255, 180, 40))
    slider_body_r   = Slider(ui_x, 217, 230, 6, 25.0, 110.0, 55.0, "Размер слайма", "%.0f px")
    slider_thickness= Slider(ui_x, 249, 230, 6, 8.0, 50.0, 15.0, "Толщина / Радиус (Wheel)", "%.0f px")
    slider_pressure = Slider(ui_x, 281, 230, 6, 0.0, 0.045, 0.018, "Давление газа", "%.3f")
    slider_g        = Slider(ui_x, 313, 230, 6, 0.0, 1200.0, 500.0, "Гравитация", "%.0f")
    slider_iters    = Slider(ui_x, 345, 230, 6, 1.0, 8.0, 4.0, "Жесткость связей", "%.0f")
    slider_bounce   = Slider(ui_x, 377, 230, 6, 0.1, 0.95, 0.55, "Упругость (Bounce)", "%.2f")

    sliders = [slider_time, slider_body_r, slider_thickness, slider_pressure, slider_g, slider_iters, slider_bounce]

    # Пресеты и действия
    btn_preset1 = Button((ui_x, 410, 230, 23), "[*] Пресет: Скейт-парк", spawn_skate_preset)
    btn_preset2 = Button((ui_x, 436, 230, 23), "[*] Пресет: Плинко", spawn_plinko_preset)
    btn_clear   = Button((ui_x, 462, 230, 23), "[X] Очистить все", clear_all)
    btn_reset   = Button((ui_x, 488, 230, 23), "[R] Сбросить тело", reset_body)

    action_buttons = [btn_preset1, btn_preset2, btn_clear, btn_reset]

    def send_physics_params():
        cmd = f"P {slider_g.val:.1f} {0.992:.4f} {slider_bounce.val:.2f} {int(slider_iters.val)} {slider_pressure.val:.5f}\n"
        ser.write(cmd.encode('ascii'))

    def send_body_size():
        cmd = f"S {slider_body_r.val:.1f}\n"
        ser.write(cmd.encode('ascii'))

    def send_time_scale(ts):
        cmd = f"T {ts:.3f}\n"
        ser.write(cmd.encode('ascii'))

    running = True
    current_points = []
    prev_center_vel = (0.0, 0.0)
    last_center_pos = None

    blink_timer = 0.0
    is_blinking = False

    current_sim_time = 1.0
    last_sent_time = 1.0
    trail_surface = pygame.Surface((WORLD_W, WORLD_H), pygame.SRCALPHA)

    while running:
        dt = clock.tick(60) / 1000.0

        # Bullet Time через Shift
        keys = pygame.key.get_pressed()
        shift_pressed = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
        target_time = 0.20 if shift_pressed else slider_time.val

        current_sim_time += (target_time - current_sim_time) * min(1.0, dt * 10.0)
        if abs(current_sim_time - last_sent_time) > 0.01:
            send_time_scale(current_sim_time)
            last_sent_time = current_sim_time

        blink_timer += dt * current_sim_time
        if not is_blinking and blink_timer > random.uniform(3.0, 5.0):
            is_blinking = True
            blink_timer = 0.0
        elif is_blinking and blink_timer > 0.15:
            is_blinking = False
            blink_timer = 0.0

        particles.update(dt, current_sim_time)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                reset_body()
            elif event.type == pygame.MOUSEWHEEL:
                slider_thickness.val = max(slider_thickness.min_val, min(slider_thickness.max_val, slider_thickness.val + event.y * 2.0))

            for btn in tool_buttons + action_buttons:
                btn.handle_event(event)

            for s in sliders:
                if s.handle_event(event):
                    if s == slider_body_r: send_body_size()
                    elif s == slider_time: pass
                    else: send_physics_params()

            # Мышь на игровом поле
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                if mx < WORLD_W:
                    if mode[0] == 'ADD_PIN':
                        obstacles.append({'x': float(mx), 'y': float(my), 'r': float(slider_thickness.val)})
                        ser.write(f"+O {mx:.1f} {my:.1f} {slider_thickness.val:.1f}\n".encode('ascii'))
                    elif mode[0] in ['ADD_RAMP', 'ADD_TRAMP']:
                        drag_start = (float(mx), float(my))
                    elif mode[0] == 'DEL':
                        # Удаление круга
                        deleted = False
                        for obs in obstacles[:]:
                            if math.hypot(mx - obs['x'], my - obs['y']) <= obs['r'] + 6:
                                obstacles.remove(obs)
                                deleted = True
                                break
                        # Удаление отрезка
                        if not deleted:
                            for seg in segments[:]:
                                # Расстояние до отрезка
                                sx, sy = seg['x2'] - seg['x1'], seg['y2'] - seg['y1']
                                seg_len_sq = sx*sx + sy*sy
                                if seg_len_sq > 0:
                                    t = max(0.0, min(1.0, ((mx - seg['x1'])*sx + (my - seg['y1'])*sy) / seg_len_sq))
                                    cx, cy = seg['x1'] + t*sx, seg['y1'] + t*sy
                                    if math.hypot(mx - cx, my - cy) <= seg['th'] + 8:
                                        segments.remove(seg)
                                        deleted = True
                                        break
                        if deleted:
                            sync_all_world()

            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                if drag_start is not None and mode[0] in ['ADD_RAMP', 'ADD_TRAMP']:
                    mx, my = event.pos
                    mx = min(mx, WORLD_W - 20)
                    dist = math.hypot(mx - drag_start[0], my - drag_start[1])
                    if dist > 15.0: # Минимальная длина отрезка
                        is_t = (mode[0] == 'ADD_TRAMP')
                        seg_data = {
                            'x1': drag_start[0], 'y1': drag_start[1],
                            'x2': float(mx), 'y2': float(my),
                            'th': float(slider_thickness.val * 0.65),
                            'tramp': is_t
                        }
                        segments.append(seg_data)
                        t_flag = 1 if is_t else 0
                        ser.write(f"+S {seg_data['x1']:.1f} {seg_data['y1']:.1f} {seg_data['x2']:.1f} {seg_data['y2']:.1f} {seg_data['th']:.1f} {t_flag}\n".encode('ascii'))
                    drag_start = None

        btn_drag.active  = (mode[0] == 'DRAG')
        btn_pin.active   = (mode[0] == 'ADD_PIN')
        btn_ramp.active  = (mode[0] == 'ADD_RAMP')
        btn_tramp.active = (mode[0] == 'ADD_TRAMP')
        btn_del.active   = (mode[0] == 'DEL')

        m_buttons = pygame.mouse.get_pressed()
        m_pos = pygame.mouse.get_pos()

        fx = 0.0
        fy = 0.0
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:  fx -= 1300.0
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]: fx += 1300.0
        if keys[pygame.K_w] or keys[pygame.K_UP] or keys[pygame.K_SPACE]: fy -= 2400.0
        if keys[pygame.K_s] or keys[pygame.K_DOWN]:  fy += 1500.0

        is_dragging = 1 if (m_buttons[0] and mode[0] == 'DRAG' and m_pos[0] < WORLD_W) else 0
        cmd = f"C {fx:.1f} {fy:.1f} {is_dragging} {m_pos[0]:.1f} {m_pos[1]:.1f}\n"
        try: ser.write(cmd.encode('ascii'))
        except Exception: pass

        latest_line = None
        while ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line: latest_line = line
            except Exception: pass

        if latest_line:
            try:
                raw_pairs = latest_line.split(';')
                parsed = [tuple(map(float, p.split(','))) for p in raw_pairs]
                if len(parsed) == NUM_RING_POINTS + 1:
                    current_points = parsed
            except Exception: pass

        # Отрисовка
        screen.fill((16, 18, 24))

        if current_sim_time < 0.7:
            slowmo_alpha = int((1.0 - current_sim_time / 0.7) * 45)
            vignette_rect = pygame.Surface((WORLD_W, WORLD_H), pygame.SRCALPHA)
            pygame.draw.rect(vignette_rect, (0, 180, 255, slowmo_alpha), (0, 0, WORLD_W, WORLD_H), 12)
            screen.blit(vignette_rect, (0, 0))

        pygame.draw.rect(screen, (28, 32, 42), (20, 20, WORLD_W - 40, WORLD_H - 40), 2, border_radius=6)

        # 1. Отрисовка платформ и батутов
        for seg in segments:
            p1 = (int(seg['x1']), int(seg['y1']))
            p2 = (int(seg['x2']), int(seg['y2']))
            th = int(seg['th'])
            if seg['tramp']:
                # Неоновый батут
                pygame.draw.line(screen, (255, 40, 160), p1, p2, th * 2)
                pygame.draw.line(screen, (255, 200, 240), p1, p2, max(2, th // 2))
                pygame.draw.circle(screen, (255, 40, 160), p1, th)
                pygame.draw.circle(screen, (255, 40, 160), p2, th)
            else:
                # Обычная платформа
                pygame.draw.line(screen, (65, 78, 102), p1, p2, th * 2)
                pygame.draw.line(screen, (110, 130, 165), p1, p2, max(2, th // 2))
                pygame.draw.circle(screen, (65, 78, 102), p1, th)
                pygame.draw.circle(screen, (65, 78, 102), p2, th)

        # 2. Отрисовка круглых штырей
        for obs in obstacles:
            ox, oy, r = int(obs['x']), int(obs['y']), int(obs['r'])
            pygame.draw.circle(screen, (10, 12, 16), (ox + 3, oy + 3), r)
            pygame.draw.circle(screen, (75, 85, 105), (ox, oy), r)
            pygame.draw.circle(screen, (115, 130, 160), (ox, oy), r - 2)
            pygame.draw.circle(screen, (185, 205, 230), (ox - r//3, oy - r//3), max(2, r//5))

        # 3. Отрисовка мягкого тела
        if len(current_points) == NUM_RING_POINTS + 1:
            ring = current_points[:NUM_RING_POINTS]
            center = current_points[CENTER_IDX]

            if last_center_pos is not None and dt > 0.001:
                cur_vel = ((center[0] - last_center_pos[0])/dt, (center[1] - last_center_pos[1])/dt)
                accel_mag = math.hypot(cur_vel[0] - prev_center_vel[0], cur_vel[1] - prev_center_vel[1])
                if accel_mag > (3400.0 * current_sim_time):
                    # Если ускорение вверх колоссальное — значит удар о батут!
                    is_tramp_hit = (cur_vel[1] < -200.0)
                    col = (255, 80, 180) if is_tramp_hit else None
                    particles.emit(center[0], center[1], count=12 if is_tramp_hit else 8, speed=200.0 if is_tramp_hit else 140.0, color_override=col)
                prev_center_vel = cur_vel
            last_center_pos = center

            if is_dragging:
                pygame.draw.line(screen, (255, 220, 40), m_pos, (int(center[0]), int(center[1])), 2)
                pygame.draw.circle(screen, (255, 220, 40), m_pos, 5)

            smooth_body = smooth_polygon(ring, iterations=2)
            ghost_trail.append(smooth_body)

            if current_sim_time < 0.85 and len(ghost_trail) > 1:
                trail_surface.fill((0, 0, 0, 0))
                for i, trail_pts in enumerate(list(ghost_trail)[:-1]):
                    alpha = int((i + 1) / len(ghost_trail) * 55 * (1.0 - current_sim_time))
                    pygame.draw.polygon(trail_surface, (0, 200, 255, alpha), trail_pts)
                screen.blit(trail_surface, (0, 0))

            for pt in ring:
                pygame.draw.line(screen, (35, 65, 95), (int(center[0]), int(center[1])), (int(pt[0]), int(pt[1])), 1)

            pygame.draw.polygon(screen, (35, 145, 215), smooth_body)
            pygame.draw.polygon(screen, (130, 225, 255), smooth_body, 3)

            # Блик
            highlight_pts = []
            for i in range(len(smooth_body)//3):
                pt = smooth_body[i]
                hx = center[0] + (pt[0] - center[0]) * 0.75
                hy = center[1] + (pt[1] - center[1]) * 0.75 - 4.0
                highlight_pts.append((hx, hy))
            if len(highlight_pts) > 2:
                pygame.draw.polygon(screen, (180, 240, 255), highlight_pts)

            # Глаза
            eye_spacing = max(6, int(slider_body_r.val * 0.22))
            eye_r = max(4, int(slider_body_r.val * 0.12))
            left_eye_pos = (center[0] - eye_spacing, center[1] - 4)
            right_eye_pos = (center[0] + eye_spacing, center[1] - 4)

            look_dx = m_pos[0] - center[0]
            look_dy = m_pos[1] - center[1]
            look_dist = math.hypot(look_dx, look_dy)
            pupil_offset_x = (look_dx / look_dist * (eye_r * 0.45)) if look_dist > 1 else 0
            pupil_offset_y = (look_dy / look_dist * (eye_r * 0.45)) if look_dist > 1 else 0

            for eye_pos in [left_eye_pos, right_eye_pos]:
                if is_blinking:
                    pygame.draw.line(screen, (20, 30, 45), (eye_pos[0] - eye_r, eye_pos[1]), (eye_pos[0] + eye_r, eye_pos[1]), 2)
                else:
                    pygame.draw.circle(screen, (255, 255, 255), (int(eye_pos[0]), int(eye_pos[1])), eye_r)
                    px = int(eye_pos[0] + pupil_offset_x)
                    py = int(eye_pos[1] + pupil_offset_y)
                    pupil_r = max(2, eye_r // 2)
                    pygame.draw.circle(screen, (20, 30, 50), (px, py), pupil_r)
                    pygame.draw.circle(screen, (255, 255, 255), (px - 1, py - 1), max(1, pupil_r // 2))

        particles.draw(screen)

        # Превью рисуемой линии
        if drag_start is not None and m_pos[0] < WORLD_W:
            p_color = (255, 60, 180) if mode[0] == 'ADD_TRAMP' else (100, 220, 120)
            th_preview = max(2, int(slider_thickness.val * 0.65 * 2))
            pygame.draw.line(screen, p_color, (int(drag_start[0]), int(drag_start[1])), m_pos, th_preview)

        # Индикатор курсора
        if m_pos[0] < WORLD_W:
            if mode[0] == 'ADD_PIN':
                cur_r = int(slider_thickness.val)
                pygame.draw.circle(screen, (100, 220, 120), m_pos, cur_r, 1)
            elif mode[0] == 'DEL':
                pygame.draw.circle(screen, (230, 75, 75), m_pos, 20, 1)
                pygame.draw.line(screen, (230, 75, 75), (m_pos[0]-6, m_pos[1]-6), (m_pos[0]+6, m_pos[1]+6), 2)
                pygame.draw.line(screen, (230, 75, 75), (m_pos[0]+6, m_pos[1]-6), (m_pos[0]-6, m_pos[1]+6), 2)

        # HUD Bullet Time
        if current_sim_time < 0.65:
            hud_txt = f"BULLET TIME: {current_sim_time:.2f}x"
            hud_surf = font_hud.render(hud_txt, True, (0, 230, 255))
            screen.blit(hud_surf, (WORLD_W // 2 - hud_surf.get_width() // 2, 35))

        # UI Панель
        pygame.draw.rect(screen, (26, 29, 37), (WORLD_W, 0, UI_WIDTH, WINDOW_H))
        pygame.draw.line(screen, (45, 50, 65), (WORLD_W, 0), (WORLD_W, WINDOW_H), 2)

        screen.blit(font_title.render("КОНСТРУКТОР МИРА", True, (255, 255, 255)), (ui_x, 8))
        screen.blit(font_s.render("RP2040 Physics Sandbox", True, (115, 135, 160)), (ui_x, 24))

        pygame.draw.line(screen, (40, 44, 56), (ui_x, 167), (ui_x + 230, 167), 1)
        pygame.draw.line(screen, (40, 44, 56), (ui_x, 400), (ui_x + 230, 400), 1)

        for btn in tool_buttons: btn.draw(screen, font_m)
        for s in sliders: s.draw(screen, font_s)
        for btn in action_buttons: btn.draw(screen, font_m)

        tips = [
            "Рампа/Батут: тяните мышь для рисования",
            "Зажмите SHIFT: режим Bullet Time"
        ]
        for idx, t in enumerate(tips):
            screen.blit(font_s.render(t, True, (120, 130, 145)), (ui_x, 520 + idx * 16))

        pygame.display.flip()

    ser.close()
    pygame.quit()

if __name__ == '__main__':
    main()