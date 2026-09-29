#!/usr/bin/env python3
"""Màn hình rada cho HC-SR04 gắn trên servo pan (sketch UltrasonicRadar).

ESP32 gửi qua Serial, mỗi dòng:
    <góc_độ>,<khoảng_cách_cm>\\n        (khoảng cách -1 = không có echo)

Chạy:
    python radar_display.py --port COM5
    python radar_display.py --demo          # giả lập ESP32, không cần phần cứng
"""
import argparse
import math
import os
import queue
import random
import re
import sys
import threading
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import pygame
import serial
import serial.tools.list_ports

SAMPLE_RE = re.compile(r"^\s*(\d{1,3})\s*,\s*(-?\d+(?:\.\d+)?)\s*$")

RANGE_PRESETS_CM = [50, 100, 150, 200, 300, 400]
SIGNAL_LOST_S = 2.0
TRAIL_DEG = 30

BG = (6, 16, 10)
GRID = (28, 90, 50)
GRID_TEXT = (70, 170, 100)
SWEEP = (120, 255, 150)
BLIP = (90, 255, 130)
TEXT = (200, 240, 210)
OK_COLOR = (90, 255, 130)
BAD_COLOR = (255, 90, 90)


def parse_line(line):
    """'90,123.4' -> (90, 123.4); '90,-1' -> (90, None) (không có echo); dòng khác -> None."""
    m = SAMPLE_RE.match(line)
    if not m:
        return None
    angle = int(m.group(1))
    if angle > 180:
        return None
    dist = float(m.group(2))
    return angle, (dist if dist >= 0 else None)


class SerialSource(threading.Thread):
    """Đọc Serial ở luồng riêng, đẩy vào queue: ('sample', góc, cm|None) / ('log', text) / ('error', text)."""

    def __init__(self, port, baud, events):
        super().__init__(daemon=True)
        self.port, self.baud, self.events = port, baud, events
        self.stop_flag = threading.Event()

    def run(self):
        try:
            with serial.serial_for_url(self.port, self.baud, timeout=0.2) as ser:
                buf = b""
                while not self.stop_flag.is_set():
                    chunk = ser.read(ser.in_waiting or 1)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        self._handle(raw)
                    if len(buf) > 1024:  # rác không có xuống dòng
                        buf = b""
        except serial.SerialException as e:
            self.events.put(("error", str(e)))

    def _handle(self, raw):
        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            return
        sample = parse_line(line)
        if sample:
            self.events.put(("sample", *sample))
        else:
            self.events.put(("log", line))


class DemoSource(threading.Thread):
    """Giả lập ESP32: quét 15-165° bước 2° mỗi 60 ms (giống firmware), bắn tia vào một căn phòng giả."""

    MIN_DEG, MAX_DEG, STEP_DEG, CYCLE_S = 15, 165, 2, 0.06
    WALL_Y_CM = 170

    def __init__(self, events):
        super().__init__(daemon=True)
        self.events = events
        self.stop_flag = threading.Event()

    def run(self):
        angle, direction = self.MIN_DEG, 1
        t0 = time.monotonic()
        while not self.stop_flag.is_set():
            d = self._distance(angle, time.monotonic() - t0)
            if d is None or d > 400 or random.random() < 0.03:
                self.events.put(("sample", angle, None))
            else:
                self.events.put(("sample", angle, d + random.gauss(0, 0.8)))
            time.sleep(self.CYCLE_S)
            angle += direction * self.STEP_DEG
            if angle >= self.MAX_DEG:
                angle, direction = self.MAX_DEG, -1
            elif angle <= self.MIN_DEG:
                angle, direction = self.MIN_DEG, 1

    def _distance(self, angle_deg, t):
        th = math.radians(angle_deg)
        dx, dy = math.cos(th), math.sin(th)
        hits = []
        if dy > 0.05:  # bức tường ngang phía trước
            hits.append(self.WALL_Y_CM / dy)
        for cx, cy, r in ((-60, 90, 12), (90 * math.sin(t * 0.4), 120, 10)):  # 1 cột đứng yên + 1 vật di chuyển
            b = dx * cx + dy * cy
            disc = b * b - (cx * cx + cy * cy - r * r)
            if disc >= 0 and b - math.sqrt(disc) > 0:
                hits.append(b - math.sqrt(disc))
        return min(hits) if hits else None


class RadarState:
    def __init__(self, range_cm, persist_s):
        self.range_cm = range_cm
        self.persist_s = persist_s
        self.readings = {}  # góc nguyên -> (cm | None, thời điểm nhận)
        self.angle = None
        self.dist = None
        self.last_time = None
        self.direction = 1
        self.sweep_angle = None  # góc vẽ, làm mượt về phía self.angle
        self.error = None

    def add(self, angle, dist, now):
        if self.angle is not None and angle != self.angle:
            self.direction = 1 if angle > self.angle else -1
        self.readings[angle] = (dist, now)
        self.angle, self.dist, self.last_time = angle, dist, now

    def clear(self):
        self.readings.clear()

    def update(self, dt):
        if self.angle is None:
            return
        if self.sweep_angle is None:
            self.sweep_angle = float(self.angle)
        else:
            self.sweep_angle += (self.angle - self.sweep_angle) * min(1.0, dt * 20)


def polar(cx, cy, r, angle_deg):
    a = math.radians(angle_deg)
    return cx + r * math.cos(a), cy - r * math.sin(a)


def geometry(size):
    w, h = size
    cx, cy = w / 2, h - 50
    return cx, cy, max(min(w / 2 - 60, cy - 70), 50)


def build_grid(size, range_cm, font):
    surf = pygame.Surface(size)
    surf.fill(BG)
    cx, cy, R = geometry(size)
    rings = 4
    for i in range(1, rings + 1):
        r = R * i / rings
        pygame.draw.arc(surf, GRID, pygame.Rect(cx - r, cy - r, 2 * r, 2 * r), 0, math.pi, 1)
        x, y = polar(cx, cy, r, 90)
        surf.blit(font.render(f"{range_cm * i / rings:g} cm", True, GRID_TEXT), (x + 5, y + 2))
    pygame.draw.line(surf, GRID, (cx - R, cy), (cx + R, cy), 1)
    for deg in range(0, 181, 30):
        pygame.draw.line(surf, GRID, (cx, cy), polar(cx, cy, R, deg), 1)
        label = font.render(f"{deg}°", True, GRID_TEXT)
        x, y = polar(cx, cy, R + 22, deg)
        surf.blit(label, label.get_rect(center=(x, y)))
    return surf


def draw_scene(screen, state, grid, now, fonts, source_label):
    screen.blit(grid, (0, 0))
    cx, cy, R = geometry(screen.get_size())

    # Vệt quét và điểm phát hiện ở 2 lớp riêng: draw.* ghi đè alpha chứ không trộn,
    # vẽ chung lớp thì điểm mờ sẽ khoét "lỗ" trong vệt quét.
    if state.sweep_angle is not None:
        sweep_layer = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
        for i in range(TRAIL_DEG):
            a0 = state.sweep_angle - state.direction * i
            a1 = state.sweep_angle - state.direction * (i + 1.4)
            if not (0 <= a0 <= 180 and 0 <= a1 <= 180):
                continue
            alpha = int(110 * (1 - i / TRAIL_DEG) ** 2)
            pygame.draw.polygon(sweep_layer, (*SWEEP, alpha), [(cx, cy), polar(cx, cy, R, a0), polar(cx, cy, R, a1)])
        pygame.draw.line(sweep_layer, (*SWEEP, 230), (cx, cy), polar(cx, cy, R, state.sweep_angle), 2)
        screen.blit(sweep_layer, (0, 0))

    blip_layer = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
    for angle, (dist, t) in state.readings.items():
        age = now - t
        if dist is None or dist > state.range_cm or age >= state.persist_s:
            continue
        alpha = int(255 * (1 - age / state.persist_s) ** 1.5)
        pygame.draw.circle(blip_layer, (*BLIP, alpha), polar(cx, cy, R * dist / state.range_cm, angle), 5)
    screen.blit(blip_layer, (0, 0))

    draw_hud(screen, state, now, fonts, source_label)


def connection_status(state, now):
    """(văn bản, màu) mô tả tình trạng nhận dữ liệu; state cần có .error và .last_time."""
    if state.error:
        return f"Lỗi Serial: {state.error}", BAD_COLOR
    if state.last_time is None:
        return "Đang chờ dữ liệu từ ESP32...", GRID_TEXT
    if now - state.last_time > SIGNAL_LOST_S:
        return "Mất tín hiệu", BAD_COLOR
    return "Đang nhận dữ liệu", OK_COLOR


def draw_hud(screen, state, now, fonts, source_label):
    font, small = fonts
    w, h = screen.get_size()

    if state.angle is None:
        angle_txt, dist_txt = "Góc: --", "Khoảng cách: --"
    else:
        angle_txt = f"Góc: {state.angle}°"
        dist_txt = "Không có echo" if state.dist is None else f"Khoảng cách: {state.dist:.1f} cm"
    for i, txt in enumerate((angle_txt, dist_txt, f"Tầm hiển thị: {state.range_cm} cm", f"Nguồn: {source_label}")):
        screen.blit(font.render(txt, True, TEXT), (16, 12 + i * 26))

    status, color = connection_status(state, now)
    screen.blit(font.render(status, True, color), (16, 12 + 4 * 26))

    hint = small.render("+/- hoặc cuộn chuột: đổi tầm   C: xóa   Esc: thoát", True, GRID_TEXT)
    screen.blit(hint, (w - hint.get_width() - 12, h - hint.get_height() - 10))


def load_font(size):
    return pygame.font.SysFont("segoeui,arial,tahoma", size)


def pick_port():
    # Bỏ qua cổng "Standard Serial over Bluetooth" mà Windows tự tạo sẵn (COM3, COM4...)
    ports = [p for p in serial.tools.list_ports.comports() if "bluetooth" not in p.description.lower()]
    if len(ports) == 1:
        return ports[0].device
    if not ports:
        sys.exit("Không thấy cổng COM nào. Cắm ESP32 vào, hoặc chạy thử không cần phần cứng bằng cờ --demo")
    listing = "\n".join(f"  {p.device}  {p.description}" for p in ports)
    sys.exit(f"Có nhiều cổng COM, chọn một cổng bằng --port:\n{listing}")


def main():
    ap = argparse.ArgumentParser(description="Màn hình rada HC-SR04 (nhận dữ liệu từ ESP32 qua Serial).")
    ap.add_argument("--port", help="cổng COM của ESP32, vd COM5 (bỏ trống: tự chọn nếu chỉ có 1 cổng)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--demo", action="store_true", help="giả lập ESP32, không cần phần cứng")
    ap.add_argument("--range", type=int, default=200, dest="range_cm", help="tầm hiển thị ban đầu, cm (mặc định 200)")
    ap.add_argument("--persist", type=float, default=6.0, help="số giây một điểm phát hiện mờ dần rồi biến mất (mặc định 6)")
    ap.add_argument("--mirror", action="store_true", help="đảo trái/phải nếu gắn servo ngược chiều (góc -> 180 - góc)")
    args = ap.parse_args()

    events = queue.SimpleQueue()
    if args.demo:
        source, source_label = DemoSource(events), "DEMO (giả lập)"
    else:
        port = args.port or pick_port()
        source, source_label = SerialSource(port, args.baud, events), f"{port} @ {args.baud}"
    source.start()

    ranges = sorted(set(RANGE_PRESETS_CM + [args.range_cm]))
    range_idx = ranges.index(args.range_cm)
    state = RadarState(args.range_cm, args.persist)

    pygame.init()
    pygame.display.set_caption("Rada siêu âm HC-SR04")
    screen = pygame.display.set_mode((1000, 640), pygame.RESIZABLE)
    fonts = (load_font(20), load_font(16))
    clock = pygame.time.Clock()
    grid, grid_key = None, None

    try:
        running = True
        while running:
            dt = clock.tick(60) / 1000
            now = time.monotonic()

            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False
                    elif ev.key == pygame.K_c:
                        state.clear()
                    elif ev.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                        range_idx = max(0, range_idx - 1)
                    elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        range_idx = min(len(ranges) - 1, range_idx + 1)
                elif ev.type == pygame.MOUSEWHEEL:
                    range_idx = max(0, min(len(ranges) - 1, range_idx - ev.y))
            state.range_cm = ranges[range_idx]

            while True:
                try:
                    kind, *payload = events.get_nowait()
                except queue.Empty:
                    break
                if kind == "sample":
                    angle, dist = payload
                    state.add(180 - angle if args.mirror else angle, dist, now)
                elif kind == "log":
                    print(f"[ESP32] {payload[0]}")
                elif kind == "error":
                    state.error = payload[0]
                    print(f"Lỗi Serial: {payload[0]}", file=sys.stderr)

            state.update(dt)
            key = (screen.get_size(), state.range_cm)
            if key != grid_key:
                grid, grid_key = build_grid(screen.get_size(), state.range_cm, fonts[1]), key
            draw_scene(screen, state, grid, now, fonts, source_label)
            pygame.display.flip()
    finally:
        source.stop_flag.set()
        pygame.quit()


if __name__ == "__main__":
    main()
