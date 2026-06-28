# -*- coding: utf-8 -*-
"""
Balance Lab - веб-мост (вариант A).

Читает поток с балансировочной платформы по Serial:
    PL,PP,ZL,ZP,W,balX,balY
считает координаты центра давления (ЦД) и стабильность, и раздаёт всё
в браузер по WebSocket. Заодно поднимает простой HTTP-сервер для статики
из папки web/ (страница futuristic.html и т.п.).

Логика, зависящая от упражнения/визуала, живёт в браузере. Здесь считаем
только то, что от него не зависит: геометрию ЦД и стабильность (разброс
качания за окно).

Запуск:
    python web_bridge.py                 # спросит COM-порт, откроет на :8000
    python web_bridge.py COM5            # явный порт
    python web_bridge.py COM5 9600       # порт и скорость
    python web_bridge.py --demo          # без железа: синтетический поток

Затем открой в браузере http://localhost:8000/futuristic.html
(с планшета/телефона в той же сети - http://<IP-компьютера>:8000/futuristic.html)

Зависимости: pyserial, websockets  (см. requirements.txt)
"""

import sys
import json
import math
import time
import socket
import asyncio
import argparse
import threading
import functools
import http.server

import serial
import serial.tools.list_ports
import websockets

# ---- геометрия платформы (см), как в cop_visualizer.py ----
SENSOR_HALF = 32.5
DIAG_OFFSET_CM = 0.75
_o = DIAG_OFFSET_CM / math.sqrt(2.0)
CORNER_ORDER = ["PL", "PP", "ZL", "ZP"]
CORNER_XY = {
    "PL": (-SENSOR_HALF,  SENSOR_HALF), "PP": (SENSOR_HALF,  SENSOR_HALF),
    "ZL": (-SENSOR_HALF, -SENSOR_HALF), "ZP": (SENSOR_HALF, -SENSOR_HALF),
}
SENSOR_OFFSET = {"PL": (+_o, +_o), "PP": (+_o, -_o), "ZL": (0.0, 0.0), "ZP": (0.0, 0.0)}
SENSOR_POS = {
    n: (CORNER_XY[n][0] + SENSOR_OFFSET[n][0], CORNER_XY[n][1] + SENSOR_OFFSET[n][1])
    for n in CORNER_ORDER
}

# ---- стабильность ----
SWAY_WIN   = 30      # окно отсчётов для оценки качания
SPREAD_MAX = 6.0     # см: разброс, при котором стабильность падает к 0

HTTP_PORT = 8000
WS_PORT   = 8765


def parse_line(line):
    """'PL,PP,ZL,ZP,W,balX,balY' -> (corners[4], W, balX, balY) или None."""
    try:
        nums = [float(x) for x in line.split(",")]
    except ValueError:
        return None
    if len(nums) == 7:
        return nums[0:4], nums[4], nums[5], nums[6]
    return None


def compute_cop(corners):
    """Центр давления = нагрузочно-взвешенное среднее позиций датчиков (см)."""
    s = sum(corners)
    if s <= 1e-6:
        return 0.0, 0.0
    cx = sum(l * SENSOR_POS[n][0] for n, l in zip(CORNER_ORDER, corners)) / s
    cy = sum(l * SENSOR_POS[n][1] for n, l in zip(CORNER_ORDER, corners)) / s
    return cx, cy


class StabilityTracker:
    """Стабильность 0-100 из разброса ЦД за скользящее окно."""
    def __init__(self, win=SWAY_WIN):
        self.buf = []
        self.win = win

    def push(self, x, y):
        self.buf.append((x, y))
        if len(self.buf) > self.win:
            self.buf.pop(0)

    def value(self):
        n = len(self.buf)
        if n < 3:
            return 0.0
        mx = sum(p[0] for p in self.buf) / n
        my = sum(p[1] for p in self.buf) / n
        spread = math.sqrt(sum((p[0] - mx) ** 2 + (p[1] - my) ** 2 for p in self.buf) / n)
        return max(0.0, min(100.0, 100.0 * (1.0 - spread / SPREAD_MAX)))


def pick_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("COM-порты не найдены. Подключите Nano (CH340) или запустите с --demo.")
        sys.exit(1)
    print("Доступные порты:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} - {p.description}")
    if len(ports) == 1:
        print(f"Выбран единственный порт: {ports[0].device}")
        return ports[0].device
    while True:
        val = input("Введите номер порта из списка: ").strip()
        if val.isdigit() and int(val) < len(ports):
            return ports[int(val)].device
        print("Неверный выбор, повторите.")


def make_payload(corners, W, balX, balY, tracker):
    """Собрать JSON-словарь одного отсчёта для браузера."""
    copX, copY = compute_cop(corners)
    tracker.push(copX, copY)
    return {
        "corners": [round(c, 2) for c in corners],
        "W": round(W, 2),
        "balX": round(balX, 1), "balY": round(balY, 1),
        "copX": round(copX, 2), "copY": round(copY, 2),
        "stability": round(tracker.value(), 1),
        "t": time.time(),
    }


def serial_producer(port, baud, emit, stop):
    """Поток: читает Serial, считает метрики, отдаёт через emit(dict)."""
    tracker = StabilityTracker()
    print(f"Подключение к {port} @ {baud}...")
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()
    print("Поток пошёл. Открой http://localhost:%d/futuristic.html" % HTTP_PORT)
    try:
        while not stop.is_set():
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw:
                continue
            parsed = parse_line(raw)
            if parsed is None:
                continue
            corners, W, balX, balY = parsed
            emit(make_payload(corners, W, balX, balY, tracker))
    finally:
        ser.close()
        print("Порт закрыт.")


def demo_producer(emit, stop):
    """Поток: синтетический сигнал без железа (для проверки UI)."""
    import random
    tracker = StabilityTracker()
    H = SENSOR_HALF
    t0 = time.time()
    print("ДЕМО-режим: синтетический поток. Открой http://localhost:%d/futuristic.html" % HTTP_PORT)
    while not stop.is_set():
        t = time.time() - t0
        # плавно гуляющий ЦД + шум качания
        cx = 6.0 * math.sin(t * 0.7) + random.gauss(0, 0.7)
        cy = 4.0 * math.sin(t * 0.5 + 1.0) + random.gauss(0, 0.7)
        cx = max(-H, min(H, cx)); cy = max(-H, min(H, cy))
        W = 80.0
        # раскладка веса по углам из позиции ЦД (билинейно)
        rf = (cx + H) / (2 * H); lf = 1 - rf
        ff = (cy + H) / (2 * H); bf = 1 - ff
        corners = [lf * ff * W, rf * ff * W, lf * bf * W, rf * bf * W]  # PL,PP,ZL,ZP
        left, right = corners[0] + corners[2], corners[1] + corners[3]
        front, rear = corners[0] + corners[1], corners[2] + corners[3]
        balX = (right - left) / W * 100.0
        balY = (front - rear) / W * 100.0
        emit(make_payload(corners, W, balX, balY, tracker))
        time.sleep(0.05)   # 20 Гц


def lan_ip():
    """IP в локальной сети (для подсказки про планшет)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "127.0.0.1"


def start_http(directory, port):
    handler = functools.partial(http.server.SimpleHTTPRequestHandler,
                                directory=directory)
    # глушим стандартный лог запросов, чтобы не засорять консоль
    handler.log_message = lambda *a, **k: None
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    return httpd


async def amain(args):
    import os
    web_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
    start_http(web_dir, args.http_port)

    clients = set()
    loop = asyncio.get_running_loop()
    stop = threading.Event()

    def emit(payload):
        msg = json.dumps(payload)
        # вызывается из потока-продюсера -> планируем рассылку в loop
        loop.call_soon_threadsafe(websockets.broadcast, clients, msg)

    if args.demo:
        target = functools.partial(demo_producer, emit, stop)
    else:
        port = args.port or pick_port()
        target = functools.partial(serial_producer, port, args.baud, emit, stop)
    threading.Thread(target=target, daemon=True).start()

    async def handler(ws):
        clients.add(ws)
        try:
            async for _ in ws:      # входящие сообщения не используем
                pass
        finally:
            clients.discard(ws)

    ip = lan_ip()
    print("=" * 56)
    print("  Balance Lab веб-мост запущен")
    print(f"  Локально:   http://localhost:{args.http_port}/futuristic.html")
    print(f"  В сети:     http://{ip}:{args.http_port}/futuristic.html")
    print("=" * 56)

    async with websockets.serve(handler, "0.0.0.0", args.ws_port):
        try:
            await asyncio.Future()      # работаем, пока не прервут
        finally:
            stop.set()


def main():
    ap = argparse.ArgumentParser(description="Balance Lab веб-мост (Serial -> WebSocket)")
    ap.add_argument("port", nargs="?", help="COM-порт (если не задан - спросит)")
    ap.add_argument("baud", nargs="?", type=int, default=9600, help="скорость (9600)")
    ap.add_argument("--demo", action="store_true", help="синтетический поток без железа")
    ap.add_argument("--http-port", type=int, default=HTTP_PORT)
    ap.add_argument("--ws-port", type=int, default=WS_PORT)
    args = ap.parse_args()
    try:
        asyncio.run(amain(args))
    except KeyboardInterrupt:
        print("\nОстановлено.")


if __name__ == "__main__":
    main()
