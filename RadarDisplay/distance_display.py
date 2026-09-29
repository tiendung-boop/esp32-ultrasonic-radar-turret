#!/usr/bin/env python3
"""Màn hình rada cho HC-SR04 đứng yên, không servo (sketch UltrasonicDistance).

ESP32 gửi qua Serial, mỗi dòng:
    <khoảng_cách_cm>\\n        (-1 = không có echo)

Chùm tia cố định hướng chính diện (90°). Mỗi lần đo để lại một điểm mờ dần dọc theo tia,
nhìn vệt điểm là biết vật đang lại gần hay ra xa.

Chạy:
    python distance_display.py --port COM5
    python distance_display.py --demo          # giả lập ESP32, không cần phần cứng
"""
import argparse
import math
import queue
import random
import re
import sys
import threading
import time
from collections import deque

from radar_display import (BAD_COLOR, BG, BLIP, GRID_TEXT, RANGE_PRESETS_CM, SWEEP, TEXT, SerialSource,
                           build_grid, connection_status, geometry, load_font, pick_port, polar)

import pygame  # sau radar_display, nơi đặt PYGAME_HIDE_SUPPORT_PROMPT trước khi import pygame

DIST_RE = re.compile(r"^\s*(-?\d+(?:\.\d+)?)\s*$")
BEAM_HALF_DEG = 7.5  # chùm tia HC-SR04 rộng khoảng 15°
WARN_COLOR = (255, 200, 80)


def parse_distance(line):
    """'123.4' -> 123.4; '-1' -> -1.0 (không có echo); dòng khác -> None."""
    m = DIST_RE.match(line)
    return float(m.group(1)) if m else None


class DistanceSerialSource(SerialSource):
    """Như SerialSource nhưng mỗi dòng chỉ có khoảng cách: đẩy ('distance', cm|None) vào queue."""

    def _handle(self, raw):
        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            return
        dist = parse_distance(line)
        if dist is None:
            self.events.put(("log", line))
        else:
            self.events.put(("distance", dist if dist >= 0 else None))


class DemoDistanceSource(threading.Thread):
    """Giả lập ESP32: một vật đi tới đi lui trước cảm biến (10-230cm), có nhiễu và thỉnh thoảng mất echo."""

    CYCLE_S = 0.06

    def __init__(self, events):
        super().__init__(daemon=True)
        self.events = events
        self.stop_flag = threading.Event()

    def run(self):
        t0 = time.monotonic()
        while not self.stop_flag.is_set():
            if random.random() < 0.03:
                self.events.put(("distance", None))
            else:
                d = 120 + 110 * math.sin((time.monotonic() - t0) * 0.5)
                self.events.put(("distance", d + random.gauss(0, 0.8)))
            time.sleep(self.CYCLE_S)


class DistanceState:
    def __init__(self, range_cm, persist_s):
        self.range_cm = range_cm
        self.persist_s = persist_s
        self.history = deque()  # (thời điểm nhận, cm | None)
        self.dist = None
        self.last_time = None
        self.error = None

    def add(self, dist, now):
        self.history.append((now, dist))
        self.dist, self.last_time = dist, now

    def prune(self, now):
        while self.history and now - self.history[0][0] >= self.persist_s:
            self.history.popleft()

    def clear(self):
        self.history.clear()


def draw_scene(screen, state, grid, now, fonts, source_label):
    screen.blit(grid, (0, 0))
    cx, cy, R = geometry(screen.get_size())

    beam = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
    pygame.draw.polygon(beam, (*SWEEP, 40), [(cx, cy), polar(cx, cy, R, 90 - BEAM_HALF_DEG), polar(cx, cy, R, 90 + BEAM_HALF_DEG)])
    pygame.draw.line(beam, (*SWEEP, 150), (cx, cy), polar(cx, cy, R, 90), 1)
    screen.blit(beam, (0, 0))

    trail = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
    for t, dist in state.history:
        if dist is None or dist > state.range_cm:
            continue
        alpha = int(255 * (1 - (now - t) / state.persist_s) ** 1.5)
        pygame.draw.circle(trail, (*BLIP, alpha), polar(cx, cy, R * dist / state.range_cm, 90), 4)
    screen.blit(trail, (0, 0))

    if state.dist is not None and state.dist <= state.range_cm and connection_status(state, now)[1] != BAD_COLOR:
        pos = polar(cx, cy, R * state.dist / state.range_cm, 90)
        pygame.draw.circle(screen, SWEEP, pos, 8)
        pygame.draw.circle(screen, BG, pos, 8, 2)

    draw_hud(screen, state, now, fonts, source_label)


def draw_hud(screen, state, now, fonts, source_label):
    big, font, small = fonts
    w, h = screen.get_size()
    status, status_color = connection_status(state, now)

    if state.last_time is None:
        big_txt = "--"
    elif state.dist is None:
        big_txt = "Không có echo"
    else:
        big_txt = f"{state.dist:.1f} cm"
    screen.blit(big.render(big_txt, True, BAD_COLOR if status_color == BAD_COLOR else TEXT), (16, 6))

    lines = [(f"Tầm hiển thị: {state.range_cm} cm", TEXT), (f"Nguồn: {source_label}", TEXT), (status, status_color)]
    for i, (txt, color) in enumerate(lines):
        screen.blit(font.render(txt, True, color), (16, 84 + i * 26))
    if state.dist is not None and state.dist > state.range_cm:
        screen.blit(font.render("Vật ngoài tầm hiển thị", True, WARN_COLOR), (16, h - 34))

    hint = small.render("+/- hoặc cuộn chuột: đổi tầm   C: xóa   Esc: thoát", True, GRID_TEXT)
    screen.blit(hint, (w - hint.get_width() - 12, h - hint.get_height() - 10))


def main():
    ap = argparse.ArgumentParser(description="Màn hình rada HC-SR04 đứng yên (nhận khoảng cách từ ESP32 qua Serial).")
    ap.add_argument("--port", help="cổng COM của ESP32, vd COM5 (bỏ trống: tự chọn nếu chỉ có 1 cổng)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--demo", action="store_true", help="giả lập ESP32, không cần phần cứng")
    ap.add_argument("--range", type=int, default=200, dest="range_cm", help="tầm hiển thị ban đầu, cm (mặc định 200)")
    ap.add_argument("--persist", type=float, default=5.0, help="số giây một điểm đo mờ dần rồi biến mất (mặc định 5)")
    args = ap.parse_args()

    events = queue.SimpleQueue()
    if args.demo:
        source, source_label = DemoDistanceSource(events), "DEMO (giả lập)"
    else:
        port = args.port or pick_port()
        source, source_label = DistanceSerialSource(port, args.baud, events), f"{port} @ {args.baud}"
    source.start()

    ranges = sorted(set(RANGE_PRESETS_CM + [args.range_cm]))
    range_idx = ranges.index(args.range_cm)
    state = DistanceState(args.range_cm, args.persist)

    pygame.init()
    pygame.display.set_caption("Rada siêu âm HC-SR04 (đứng yên)")
    screen = pygame.display.set_mode((1000, 640), pygame.RESIZABLE)
    fonts = (load_font(56), load_font(20), load_font(16))
    clock = pygame.time.Clock()
    grid, grid_key = None, None

    try:
        running = True
        while running:
            clock.tick(60)
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
                if kind == "distance":
                    state.add(payload[0], now)
                elif kind == "log":
                    print(f"[ESP32] {payload[0]}")
                elif kind == "error":
                    state.error = payload[0]
                    print(f"Lỗi Serial: {payload[0]}", file=sys.stderr)

            state.prune(now)
            key = (screen.get_size(), state.range_cm)
            if key != grid_key:
                grid, grid_key = build_grid(screen.get_size(), state.range_cm, fonts[2]), key
            draw_scene(screen, state, grid, now, fonts, source_label)
            pygame.display.flip()
    finally:
        source.stop_flag.set()
        pygame.quit()


if __name__ == "__main__":
    main()
