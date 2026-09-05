import sys
import time
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

    def emit(self, x, y, count=10, speed=140.0, color_override=None):
        for _ in range(count):
            angle = random.uniform(0, 2 * math.pi)
            spd = random.uniform(speed * 0.3, speed)
            c = color_override or random.choice([(100, 220, 255), (140, 240, 255), (60, 160, 230)])
            self.particles.append({
                'x': x, 'y': y,
                'vx': math.cos(angle) * spd,
                'vy': math.sin(angle) * spd,
                'life': 1.0,
                'decay': random.uniform(1.6, 2.5),
                'r': random.uniform(2.5, 5.0),
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
    def __init__(self, rect, text, callback, color=(40, 46, 60), hover_color=(54, 62, 80)):
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
        
        pygame.draw.rect(surf, base_color, self.rect, border_radius=4)
        pygame.draw.rect(surf, (70, 85, 110), self.rect, 1, border_radius=4)
        
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
        surf.blit(font.render(txt, True, (170, 180, 195)), (self.rect.x, self.rect.y - 15))
        pygame.draw.rect(surf, (36, 40, 52), self.rect, border_radius=3)
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

    # Ждем перезагрузки Pico после установки соединения по USB CDC
    print("[ИНФО] Ожидание инициализации Pico...")
    time.sleep(1.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("RP2040 SoftBody — Super Trampoline & Forces")
    clock = pygame.time.Clock()

    font_s = pygame.font.SysFont("Arial", 11)
    font_m = pygame.font.SysFont("Arial", 12, bold=True)
    font_title = pygame.font.SysFont("Arial", 14, bold=True)
    font_hud = pygame.font.SysFont("Arial", 16, bold=True)
    font_score = pygame.font.SysFont("Arial", 22, bold=True)

    obstacles = []
    segments = []
    fans = []
    vortices = []
    
    hoop_active = True
    hoop_pos = (680.0, 310.0)
    score = 0
    goal_timer = 0.0

    mode = ['DRAG']
    particles = ParticleSystem()
    ghost_trail = deque(maxlen=7)
    drag_start = None

    def set_mode(m): mode[0] = m
    def reset_body(): ser.write(b"R\n")
    
    def clear_all():
        obstacles.clear()
        segments.clear()
        fans.clear()
        vortices.clear()
        ser.write(b"X\n")

    def sync_all_world():
        ser.write(b"X\n")
        for obs in obstacles:
            ser.write(f"+O {obs['x']:.1f} {obs['y']:.1f} {obs['r']:.1f}\n".encode('ascii'))
        for s in segments:
            t_flag = 1 if s['tramp'] else 0
            ser.write(f"+S {s['x1']:.1f} {s['y1']:.1f} {s['x2']:.1f} {s['y2']:.1f} {s['th']:.1f} {t_flag}\n".encode('ascii'))
        for f in fans:
            ser.write(f"+F {f['x']:.1f} {f['y']:.1f} {f['w']:.1f} {f['h']:.1f} {f['fx']:.1f} {f['fy']:.1f}\n".encode('ascii'))
        for v in vortices:
            ser.write(f"+V {v['x']:.1f} {v['y']:.1f} {v['str']:.1f}\n".encode('ascii'))

    def spawn_basketball():
        nonlocal hoop_active, hoop_pos, score
        clear_all()
        hoop_active = True
        hoop_pos = (680.0, 310.0)
        # Горка разгона
        segments.append({'x1': 50.0, 'y1': 160.0, 'x2': 240.0, 'y2': 430.0, 'th': 12.0, 'tramp': False})
        # Супер-батут
        segments.append({'x1': 250.0, 'y1': 530.0, 'x2': 470.0, 'y2': 530.0, 'th': 15.0, 'tramp': True})
        # Верхний штырь-рикошет
        obstacles.append({'x': 500.0, 'y': 200.0, 'r': 22.0})
        sync_all_world()

    def spawn_wind_tunnel():
        nonlocal hoop_active
        clear_all()
        hoop_active = False
        segments.append({'x1': 260.0, 'y1': 100.0, 'x2': 260.0, 'y2': 550.0, 'th': 10.0, 'tramp': False})
        segments.append({'x1': 540.0, 'y1': 100.0, 'x2': 540.0, 'y2': 550.0, 'th': 10.0, 'tramp': False})
        # Мощный вентилятор с силой -3800.0
        fans.append({'x': 265.0, 'y': 150.0, 'w': 270.0, 'h': 410.0, 'fx': 0.0, 'fy': -3800.0})
        sync_all_world()

    def spawn_skate_preset():
        nonlocal hoop_active
        clear_all()
        hoop_active = False
        segments.append({'x1': 50.0, 'y1': 150.0, 'x2': 260.0, 'y2': 440.0, 'th': 12.0, 'tramp': False})
        segments.append({'x1': 270.0, 'y1': 530.0, 'x2': 510.0, 'y2': 530.0, 'th': 15.0, 'tramp': True})
        segments.append({'x1': 530.0, 'y1': 450.0, 'x2': 740.0, 'y2': 230.0, 'th': 12.0, 'tramp': False})
        obstacles.append({'x': 390.0, 'y': 160.0, 'r': 22.0})
        sync_all_world()

    ui_x = WORLD_W + 15

    btn_drag   = Button((ui_x, 34, 230, 20), "[~] Захват тела", lambda: set_mode('DRAG'))
    btn_pin    = Button((ui_x, 56, 230, 20), "[+] Штырь (Круг)", lambda: set_mode('ADD_PIN'))
    btn_ramp   = Button((ui_x, 78, 230, 20), "[/] Рампа / Стена", lambda: set_mode('ADD_RAMP'))
    btn_tramp  = Button((ui_x, 100, 230, 20), "[^] Неоновый Батут", lambda: set_mode('ADD_TRAMP'))
    btn_fan    = Button((ui_x, 122, 230, 20), "[~] Вентилятор (Поток)", lambda: set_mode('ADD_FAN'))
    btn_vortex = Button((ui_x, 144, 230, 20), "[@] Черная дыра (Вихрь)", lambda: set_mode('ADD_VORTEX'))
    btn_del    = Button((ui_x, 166, 230, 20), "[-] Удалить объект", lambda: set_mode('DEL'))

    tool_buttons = [btn_drag, btn_pin, btn_ramp, btn_tramp, btn_fan, btn_vortex, btn_del]

    slider_time     = Slider(ui_x, 208, 230, 5, 0.05, 1.5, 1.0, "Скорость времени", "%.2fx", active_color=(255, 180, 40))
    slider_body_r   = Slider(ui_x, 237, 230, 5, 25.0, 110.0, 55.0, "Размер слайма", "%.0f px")
    slider_thickness= Slider(ui_x, 266, 230, 5, 8.0, 50.0, 15.0, "Толщина / Радиус", "%.0f px")
    slider_pressure = Slider(ui_x, 295, 230, 5, 0.0, 1.0, 0.75, "Давление газа (Объем)", "%.2f", active_color=(100, 240, 120))
    slider_g        = Slider(ui_x, 324, 230, 5, 0.0, 1200.0, 550.0, "Гравитация", "%.0f")
    slider_iters    = Slider(ui_x, 353, 230, 5, 1.0, 8.0, 4.0, "Жесткость оболочки", "%.0f")

    sliders = [slider_time, slider_body_r, slider_thickness, slider_pressure, slider_g, slider_iters]

    btn_preset_bball = Button((ui_x, 390, 230, 22), "[*] Пресет: Баскетбол", spawn_basketball)
    btn_preset_wind  = Button((ui_x, 414, 230, 22), "[*] Пресет: Аэротруба", spawn_wind_tunnel)
    btn_preset_skate = Button((ui_x, 438, 230, 22), "[*] Пресет: Скейт-парк", spawn_skate_preset)
    btn_clear        = Button((ui_x, 462, 230, 22), "[X] Очистить все", clear_all)
    btn_reset        = Button((ui_x, 486, 230, 22), "[R] Сбросить тело", reset_body)

    action_buttons = [btn_preset_bball, btn_preset_wind, btn_preset_skate, btn_clear, btn_reset]

    def send_physics_params():
        cmd = f"P {slider_g.val:.1f} {0.992:.4f} {0.55:.2f} {int(slider_iters.val)} {slider_pressure.val:.3f}\n"
        ser.write(cmd.encode('ascii'))

    def send_body_size():
        cmd = f"S {slider_body_r.val:.1f}\n"
        ser.write(cmd.encode('ascii'))

    def send_time_scale(ts):
        cmd = f"T {ts:.3f}\n"
        ser.write(cmd.encode('ascii'))

    running = True
    current_points = []
    prev_center_pos = None
    last_center_pos = None

    blink_timer = 0.0
    is_blinking = False

    current_sim_time = 1.0
    last_sent_time = 1.0
    fan_anim_offset = 0.0
    vortex_angle = 0.0
    trail_surface = pygame.Surface((WORLD_W, WORLD_H), pygame.SRCALPHA)

    # Инициализируем пресет баскетбола после полной загрузки платы
    spawn_basketball()

    while running:
        dt = clock.tick(60) / 1000.0

        keys = pygame.key.get_pressed()
        shift_pressed = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
        target_time = 0.20 if shift_pressed else slider_time.val

        current_sim_time += (target_time - current_sim_time) * min(1.0, dt * 10.0)
        if abs(current_sim_time - last_sent_time) > 0.01:
            send_time_scale(current_sim_time)
            last_sent_time = current_sim_time

        fan_anim_offset = (fan_anim_offset + dt * 260.0 * current_sim_time) % 40.0
        vortex_angle = (vortex_angle + dt * 5.0 * current_sim_time) % (2.0 * math.pi)

        if goal_timer > 0.0:
            goal_timer -= dt

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

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                if mx < WORLD_W:
                    if mode[0] == 'ADD_PIN':
                        obstacles.append({'x': float(mx), 'y': float(my), 'r': float(slider_thickness.val)})
                        ser.write(f"+O {mx:.1f} {my:.1f} {slider_thickness.val:.1f}\n".encode('ascii'))
                    elif mode[0] == 'ADD_VORTEX':
                        # Черная дыра со сверхсилой 1.0
                        vortices.append({'x': float(mx), 'y': float(my), 'str': 1.0})
                        ser.write(f"+V {mx:.1f} {my:.1f} 1.0\n".encode('ascii'))
                    elif mode[0] in ['ADD_RAMP', 'ADD_TRAMP', 'ADD_FAN']:
                        drag_start = (float(mx), float(my))
                    elif mode[0] == 'DEL':
                        deleted = False
                        for obs in obstacles[:]:
                            if math.hypot(mx - obs['x'], my - obs['y']) <= obs['r'] + 6:
                                obstacles.remove(obs)
                                deleted = True
                                break
                        if not deleted:
                            for v in vortices[:]:
                                if math.hypot(mx - v['x'], my - v['y']) <= 30.0:
                                    vortices.remove(v)
                                    deleted = True
                                    break
                        if not deleted:
                            for f in fans[:]:
                                if f['x'] <= mx <= f['x'] + f['w'] and f['y'] <= my <= f['y'] + f['h']:
                                    fans.remove(f)
                                    deleted = True
                                    break
                        if not deleted:
                            for seg in segments[:]:
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
                if drag_start is not None and m_pos[0] < WORLD_W:
                    mx, my = event.pos
                    mx = min(mx, WORLD_W - 20)
                    if mode[0] in ['ADD_RAMP', 'ADD_TRAMP']:
                        dist = math.hypot(mx - drag_start[0], my - drag_start[1])
                        if dist > 15.0:
                            is_t = (mode[0] == 'ADD_TRAMP')
                            seg_data = {
                                'x1': drag_start[0], 'y1': drag_start[1],
                                'x2': float(mx), 'y2': float(my),
                                'th': float(slider_thickness.val * 0.65),
                                'tramp': is_t
                            }
                            segments.append(seg_data)
                            ser.write(f"+S {seg_data['x1']:.1f} {seg_data['y1']:.1f} {seg_data['x2']:.1f} {seg_data['y2']:.1f} {seg_data['th']:.1f} {1 if is_t else 0}\n".encode('ascii'))
                    elif mode[0] == 'ADD_FAN':
                        fx = min(drag_start[0], float(mx))
                        fy = min(drag_start[1], float(my))
                        fw = abs(float(mx) - drag_start[0])
                        fh = abs(float(my) - drag_start[1])
                        if fw > 20 and fh > 20:
                            # Сила -3800.0 вверх
                            fan_data = {'x': fx, 'y': fy, 'w': fw, 'h': fh, 'fx': 0.0, 'fy': -3800.0}
                            fans.append(fan_data)
                            ser.write(f"+F {fx:.1f} {fy:.1f} {fw:.1f} {fh:.1f} 0.0 -3800.0\n".encode('ascii'))
                    drag_start = None

        btn_drag.active   = (mode[0] == 'DRAG')
        btn_pin.active    = (mode[0] == 'ADD_PIN')
        btn_ramp.active   = (mode[0] == 'ADD_RAMP')
        btn_tramp.active  = (mode[0] == 'ADD_TRAMP')
        btn_fan.active    = (mode[0] == 'ADD_FAN')
        btn_vortex.active = (mode[0] == 'ADD_VORTEX')
        btn_del.active    = (mode[0] == 'DEL')

        m_buttons = pygame.mouse.get_pressed()
        m_pos = pygame.mouse.get_pos()

        fx = 0.0
        fy = 0.0
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:  fx -= 1400.0
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]: fx += 1400.0
        if keys[pygame.K_w] or keys[pygame.K_UP] or keys[pygame.K_SPACE]: fy -= 2500.0
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

        # 1. Вентиляторы
        for f in fans:
            fx, fy, fw, fh = int(f['x']), int(f['y']), int(f['w']), int(f['h'])
            fan_surf = pygame.Surface((fw, fh), pygame.SRCALPHA)
            fan_surf.fill((0, 180, 255, 35))
            pygame.draw.rect(fan_surf, (0, 210, 255, 90), (0, 0, fw, fh), 2, border_radius=4)
            for line_y in range(int(fan_anim_offset), fh, 35):
                pygame.draw.line(fan_surf, (160, 235, 255, 120), (10, fh - line_y), (fw - 10, fh - line_y), 2)
            screen.blit(fan_surf, (fx, fy))
            pygame.draw.rect(screen, (70, 85, 110), (fx, fy + fh - 8, fw, 8), border_radius=2)

        # 2. Черные дыры
        for v in vortices:
            vx, vy = int(v['x']), int(v['y'])
            for r_ring in [45, 30, 15]:
                pygame.draw.circle(screen, (140, 50, 230), (vx, vy), r_ring, 1)
            pygame.draw.circle(screen, (210, 90, 255), (vx, vy), 9)
            pygame.draw.circle(screen, (10, 5, 20), (vx, vy), 5)
            lx = vx + int(math.cos(vortex_angle) * 42)
            ly = vy + int(math.sin(vortex_angle) * 42)
            pygame.draw.line(screen, (220, 110, 255), (vx, vy), (lx, ly), 2)

        # 3. Баскетбольное кольцо
        if hoop_active:
            hx, hy = int(hoop_pos[0]), int(hoop_pos[1])
            pygame.draw.rect(screen, (220, 225, 235), (hx + 30, hy - 45, 8, 60), border_radius=2)
            pygame.draw.rect(screen, (240, 80, 50), (hx + 28, hy - 25, 4, 25))
            pygame.draw.line(screen, (245, 90, 30), (hx - 30, hy), (hx + 30, hy), 6)
            for offset_x in [-20, -10, 0, 10, 20]:
                pygame.draw.line(screen, (220, 230, 245), (hx + offset_x, hy), (hx + offset_x * 0.5, hy + 35), 1)
            pygame.draw.line(screen, (220, 230, 245), (hx - 15, hy + 35), (hx + 15, hy + 35), 1)

            score_txt = font_score.render(f"СЧЕТ: {score}", True, (255, 215, 60))
            screen.blit(score_txt, (WORLD_W - 160, 35))

            if goal_timer > 0.0:
                goal_txt = font_score.render("ГОЛ! +1", True, (0, 255, 140))
                screen.blit(goal_txt, (hx - 30, hy - 40))

        # 4. Платформы и батуты
        for seg in segments:
            p1 = (int(seg['x1']), int(seg['y1']))
            p2 = (int(seg['x2']), int(seg['y2']))
            th = int(seg['th'])
            if seg['tramp']:
                pygame.draw.line(screen, (255, 20, 140), p1, p2, th * 2)
                pygame.draw.line(screen, (255, 220, 250), p1, p2, max(3, th // 2))
                pygame.draw.circle(screen, (255, 20, 140), p1, th)
                pygame.draw.circle(screen, (255, 20, 140), p2, th)
            else:
                pygame.draw.line(screen, (65, 78, 102), p1, p2, th * 2)
                pygame.draw.line(screen, (110, 130, 165), p1, p2, max(2, th // 2))
                pygame.draw.circle(screen, (65, 78, 102), p1, th)
                pygame.draw.circle(screen, (65, 78, 102), p2, th)

        # 5. Штыри
        for obs in obstacles:
            ox, oy, r = int(obs['x']), int(obs['y']), int(obs['r'])
            pygame.draw.circle(screen, (10, 12, 16), (ox + 3, oy + 3), r)
            pygame.draw.circle(screen, (75, 85, 105), (ox, oy), r)
            pygame.draw.circle(screen, (115, 130, 160), (ox, oy), r - 2)
            pygame.draw.circle(screen, (185, 205, 230), (ox - r//3, oy - r//3), max(2, r//5))

        # 6. Мягкое тело
        if len(current_points) == NUM_RING_POINTS + 1:
            ring = current_points[:NUM_RING_POINTS]
            center = current_points[CENTER_IDX]

            if hoop_active and prev_center_pos is not None:
                hx, hy = hoop_pos
                if prev_center_pos[1] <= hy <= center[1] and abs(center[0] - hx) < 26.0:
                    score += 1
                    goal_timer = 1.2
                    particles.emit(hx, hy, count=25, speed=240.0, color_override=(255, 215, 40))
            prev_center_pos = center

            if last_center_pos is not None and dt > 0.001:
                cur_vel_y = (center[1] - last_center_pos[1]) / dt
                if cur_vel_y < -350.0:
                    particles.emit(center[0], center[1], count=16, speed=240.0, color_override=(255, 40, 160))
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

            pygame.draw.polygon(screen, (35, 145, 215), smooth_body)
            pygame.draw.polygon(screen, (130, 225, 255), smooth_body, 3)

            if slider_pressure.val > 0.2:
                highlight_pts = []
                for i in range(len(smooth_body)//3):
                    pt = smooth_body[i]
                    hx = center[0] + (pt[0] - center[0]) * 0.75
                    hy = center[1] + (pt[1] - center[1]) * 0.75 - 4.0
                    highlight_pts.append((hx, hy))
                if len(highlight_pts) > 2:
                    pygame.draw.polygon(screen, (180, 240, 255), highlight_pts)

            eye_spacing = max(5, int(slider_body_r.val * 0.22 * min(1.0, slider_pressure.val + 0.3)))
            eye_r = max(3, int(slider_body_r.val * 0.12 * min(1.0, slider_pressure.val + 0.4)))
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

        if drag_start is not None and m_pos[0] < WORLD_W:
            if mode[0] == 'ADD_FAN':
                rx = min(drag_start[0], float(m_pos[0]))
                ry = min(drag_start[1], float(m_pos[1]))
                rw = abs(float(m_pos[0]) - drag_start[0])
                rh = abs(float(m_pos[1]) - drag_start[1])
                pygame.draw.rect(screen, (0, 200, 255), (rx, ry, rw, rh), 2)
            else:
                p_color = (255, 40, 160) if mode[0] == 'ADD_TRAMP' else (100, 220, 120)
                th_preview = max(2, int(slider_thickness.val * 0.65 * 2))
                pygame.draw.line(screen, p_color, (int(drag_start[0]), int(drag_start[1])), m_pos, th_preview)

        if m_pos[0] < WORLD_W:
            if mode[0] == 'ADD_PIN':
                cur_r = int(slider_thickness.val)
                pygame.draw.circle(screen, (100, 220, 120), m_pos, cur_r, 1)
            elif mode[0] == 'ADD_VORTEX':
                pygame.draw.circle(screen, (180, 80, 255), m_pos, 25, 1)
            elif mode[0] == 'DEL':
                pygame.draw.circle(screen, (230, 75, 75), m_pos, 20, 1)
                pygame.draw.line(screen, (230, 75, 75), (m_pos[0]-6, m_pos[1]-6), (m_pos[0]+6, m_pos[1]+6), 2)
                pygame.draw.line(screen, (230, 75, 75), (m_pos[0]+6, m_pos[1]-6), (m_pos[0]-6, m_pos[1]+6), 2)

        if current_sim_time < 0.65:
            hud_txt = f"BULLET TIME: {current_sim_time:.2f}x"
            hud_surf = font_hud.render(hud_txt, True, (0, 230, 255))
            screen.blit(hud_surf, (WORLD_W // 2 - hud_surf.get_width() // 2, 35))

        # Панель интерфейса
        pygame.draw.rect(screen, (24, 27, 35), (WORLD_W, 0, UI_WIDTH, WINDOW_H))
        pygame.draw.line(screen, (45, 50, 65), (WORLD_W, 0), (WORLD_W, WINDOW_H), 2)

        screen.blit(font_title.render("ПЕСОЧНИЦА АКТИВНОСТЕЙ", True, (255, 255, 255)), (ui_x, 8))
        screen.blit(font_s.render("RP2040 Super-Physics Core", True, (115, 135, 160)), (ui_x, 22))

        pygame.draw.line(screen, (38, 42, 54), (ui_x, 192), (ui_x + 230, 192), 1)
        pygame.draw.line(screen, (38, 42, 54), (ui_x, 375), (ui_x + 230, 375), 1)

        for btn in tool_buttons: btn.draw(screen, font_m)
        for s in sliders: s.draw(screen, font_s)
        for btn in action_buttons: btn.draw(screen, font_m)

        tips = [
            "Батут: катапультирует шар целиком к потолку!",
            "Аэротруба: левитация в потоке ветра",
            "Черная дыра: мощный космический вихрь"
        ]
        for idx, t in enumerate(tips):
            screen.blit(font_s.render(t, True, (115, 125, 140)), (ui_x, 520 + idx * 15))

        pygame.display.flip()

    ser.close()
    pygame.quit()

if __name__ == '__main__':
    main()