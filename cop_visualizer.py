# -*- coding: utf-8 -*-
"""
Визуализатор центра давления (Center of Pressure) для балансировочной платформы.

Читает по Serial строки от скетча balance_platform.ino:
    PL,PP,ZL,ZP,W,balX,balY

где PL..ZP - нагрузка датчиков в кг, W - суммарный вес в кг,
balX/balY - смещение в %.

Геометрия (реальная платформа):
  * платформа 0.95 x 0.95 м;
  * 4 датчика по углам, в 15 см от краёв -> образуют квадрат 0.65 x 0.65 м,
    то есть стоят в ±32.5 см от центра.
Центр давления показывается в РЕАЛЬНЫХ сантиметрах от центра платформы.
CoP физически не может выйти за квадрат опор (±32.5 см).

Показывает:
  * контур платформы (95x95 см) и квадрат опор (65x65 см);
  * красную точку центра давления в см по X (влево/вправо) и Y (назад/вперёд);
  * 4 угловых маркера в позициях датчиков, размер которых растёт с нагрузкой;
  * суммарный вес в кг, позицию CoP в см и нагрузку каждого угла.

Запуск:
    python cop_visualizer.py            # автовыбор/спросит COM-порт
    python cop_visualizer.py COM5       # явный порт
    python cop_visualizer.py COM5 9600  # порт и скорость

Зависимости: pyserial, matplotlib  (см. requirements.txt)
"""

import sys
import math
import time
import collections

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches

BAUD_DEFAULT = 9600
TRAIL_LEN = 60          # длина следа точки
ANIM_INTERVAL = 16      # мс между кадрами анимации (~60 кадров/с)
SMOOTH = 0.35           # коэф. сглаживания 0..1: больше = отзывчивее, меньше = плавнее

# --- геометрия платформы (см) ---
PLATFORM_HALF = 47.5    # половина стороны платформы 0.95 м
SENSOR_HALF   = 32.5    # датчики в 15 см от края: 47.5 - 15 = 32.5 см от центра

# Масштаб обзора (полуширина видимой области, см). Меньше = крупнее/виднее
# движение точки, но края платформы обрезаются. 36 ~= квадрат опор во весь кадр.
# Поставь больше (47.5+), чтобы видеть всю платформу, или меньше для ещё зума.
VIEW_HALF = PLATFORM_HALF + 6   # вся платформа в кадре (как было изначально)

# Усиление чувствительности: во сколько раз ВИЗУАЛЬНО увеличить смещение точки.
# 1.0 = реальное положение; >1 = мелкие сдвиги видны крупнее (точка ездит шире).
# На цифры в подписи и позиции датчиков не влияет - только на красную точку/след.
COP_GAIN = 2.5

# идеальные координаты датчиков (углов) в см: ПЛ, ПП, ЗЛ, ЗП
CORNER_XY = {
    "PL": (-SENSOR_HALF,  SENSOR_HALF),
    "PP": ( SENSOR_HALF,  SENSOR_HALF),
    "ZL": (-SENSOR_HALF, -SENSOR_HALF),
    "ZP": ( SENSOR_HALF, -SENSOR_HALF),
}
CORNER_ORDER = ["PL", "PP", "ZL", "ZP"]

# --- поправка на неровную установку датчиков ---
# Датчики стоят не точно в углах. Замеры:
#   ПЛ смещён ~0.5-1 см "влево" от диагонали ПЛ-ЗП (если смотреть из угла ПЛ)
#      -> по осям это (+X, +Y).
#   ПП смещён ~0.5-1 см "влево" от диагонали ПП-ЗЛ (если смотреть из угла ПП)
#      -> по осям это (+X, -Y).
# Смещение d см вдоль диагонали раскладывается как d/sqrt(2) по каждой оси.
# Подправь DIAG_OFFSET_CM (0.5..1.0) и знаки, если замеришь точнее.
DIAG_OFFSET_CM = 0.75
_o = DIAG_OFFSET_CM / math.sqrt(2.0)
SENSOR_OFFSET = {
    "PL": (+_o, +_o),   # влево по диагонали ПЛ-ЗП -> вправо-вперёд
    "PP": (+_o, -_o),   # влево по диагонали ПП-ЗЛ -> вправо-назад
    "ZL": (0.0, 0.0),
    "ZP": (0.0, 0.0),
}
# фактические координаты датчиков с учётом поправки (их и используем для CoP)
SENSOR_POS = {
    n: (CORNER_XY[n][0] + SENSOR_OFFSET[n][0],
        CORNER_XY[n][1] + SENSOR_OFFSET[n][1])
    for n in CORNER_ORDER
}


def pick_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("COM-порты не найдены. Подключите Nano (CH340) и перезапустите.")
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


def parse_line(line):
    """Возвращает (corners[4], W, balX, balY) или None."""
    parts = line.split(",")
    try:
        nums = [float(x) for x in parts]
    except ValueError:
        return None
    if len(nums) == 7:               # PL,PP,ZL,ZP,W,balX,balY
        return nums[0:4], nums[4], nums[5], nums[6]
    if len(nums) == 3:               # обратная совместимость: W,balX,balY
        return None, nums[0], nums[1], nums[2]
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else pick_port()
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD_DEFAULT

    print(f"Подключение к {port} @ {baud}...")
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)                    # Nano сбрасывается при открытии порта
    ser.reset_input_buffer()

    # --- настройка графика ---
    lim = VIEW_HALF
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    ax.set_title("Центр давления (CoP), см от центра платформы")
    ax.set_xlabel("X, см  (- влево ... + вправо)")
    ax.set_ylabel("Y, см  (- назад ... + вперёд)")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="gray", lw=0.8)
    ax.axvline(0, color="gray", lw=0.8)

    # контур платформы (95x95 см) и квадрат опор (65x65 см)
    ax.add_patch(patches.Rectangle(
        (-PLATFORM_HALF, -PLATFORM_HALF), 2 * PLATFORM_HALF, 2 * PLATFORM_HALF,
        fill=False, edgecolor="black", lw=1.5, label="платформа 95×95 см"))
    ax.add_patch(patches.Rectangle(
        (-SENSOR_HALF, -SENSOR_HALF), 2 * SENSOR_HALF, 2 * SENSOR_HALF,
        fill=False, edgecolor="tab:green", lw=1.0, ls="--",
        alpha=0.6, label="опоры 65×65 см"))

    # угловые маркеры (размер = нагрузка) + текст с кг, в фактических позициях
    corner_dots = {}
    corner_labels = {}
    for name in CORNER_ORDER:
        x, y = SENSOR_POS[name]
        (cd,) = ax.plot([x], [y], "o", color="tab:green", alpha=0.55,
                        markersize=12)
        corner_dots[name] = cd
        ha = "left" if x < 0 else "right"
        va = "top" if y > 0 else "bottom"
        off_x = 3 if x < 0 else -3
        off_y = -4 if y > 0 else 4
        corner_labels[name] = ax.text(x + off_x, y + off_y, "",
                                      ha=ha, va=va, fontsize=10, color="darkgreen")

    trail = collections.deque(maxlen=TRAIL_LEN)
    (trail_line,) = ax.plot([], [], "-", color="tab:blue", alpha=0.4, lw=1)
    (dot,) = ax.plot([], [], "o", color="tab:red", markersize=16)
    weight_text = ax.text(0.02, 0.98, "", transform=ax.transAxes,
                          va="top", fontsize=11,
                          bbox=dict(boxstyle="round", fc="white", alpha=0.75))

    # сглаженное (показываемое) состояние и "цель" - последние принятые данные.
    # Каждый кадр плавно подтягиваем показ к цели -> точка скользит, не прыгает.
    state = {
        "balX": 0.0, "balY": 0.0, "W": 0.0, "corners": [0.0, 0.0, 0.0, 0.0],
        "tBalX": 0.0, "tBalY": 0.0, "tW": 0.0, "tCorners": [0.0, 0.0, 0.0, 0.0],
        "have": False,
    }
    last_print = [0.0]   # для троттлинга вывода в консоль

    def update(_frame):
        # забираем все накопившиеся строки, цель = самая свежая
        got_new = False
        while ser.in_waiting > 0:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw:
                continue
            parsed = parse_line(raw)
            if parsed is not None:
                corners, W, balX, balY = parsed
                state["tBalX"], state["tBalY"], state["tW"] = balX, balY, W
                if corners is not None:
                    state["tCorners"] = corners
                state["have"] = True
                got_new = True

        if not state["have"]:
            return ()

        # экспоненциальное сглаживание показа к цели
        a = SMOOTH
        state["balX"] += a * (state["tBalX"] - state["balX"])
        state["balY"] += a * (state["tBalY"] - state["balY"])
        state["W"]    += a * (state["tW"]    - state["W"])
        for i in range(4):
            state["corners"][i] += a * (state["tCorners"][i] - state["corners"][i])

        balX, balY, W = state["balX"], state["balY"], state["W"]
        corners = state["corners"]

        # Центр давления = нагрузочно-взвешенное среднее ФАКТИЧЕСКИХ координат
        # датчиков. Так поправка на их неровную установку учитывается физически
        # точно (а не приближением через balX/balY).
        load_sum = sum(corners)
        if load_sum > 1e-6:
            copX = sum(load * SENSOR_POS[n][0]
                       for n, load in zip(CORNER_ORDER, corners)) / load_sum
            copY = sum(load * SENSOR_POS[n][1]
                       for n, load in zip(CORNER_ORDER, corners)) / load_sum
        else:
            # нет поканальных данных (старый 3-значный формат) или пусто -
            # запасной путь через проценты, без поправки на позиции
            copX = balX / 100.0 * SENSOR_HALF
            copY = balY / 100.0 * SENSOR_HALF

        # точка центра давления: визуально усиливаем смещение в COP_GAIN раз
        # (для наглядности), но держим в пределах кадра. Реальные см - в подписи.
        dispX = max(-VIEW_HALF, min(VIEW_HALF, copX * COP_GAIN))
        dispY = max(-VIEW_HALF, min(VIEW_HALF, copY * COP_GAIN))
        trail.append((dispX, dispY))
        trail_line.set_data([p[0] for p in trail], [p[1] for p in trail])
        dot.set_data([dispX], [dispY])

        # размер угловых маркеров пропорционален их нагрузке
        maxload = max(corners) if max(corners) > 0 else 1.0
        for name, load in zip(CORNER_ORDER, corners):
            size = 10 + 40 * max(load, 0) / maxload   # 10..50 px
            corner_dots[name].set_markersize(size)
            corner_labels[name].set_text(f"{name}\n{load:.2f} кг")

        txt = (f"W = {W:.2f} кг\n"
               f"CoP X = {copX:+.1f} см\n"
               f"CoP Y = {copY:+.1f} см")
        weight_text.set_text(txt)

        # печать в консоль не чаще ~2 раз в секунду и только при новых данных
        now = time.time()
        if got_new and now - last_print[0] >= 0.5:
            last_print[0] = now
            c = corners
            print(f"ПЛ={c[0]:.2f} ПП={c[1]:.2f} ЗЛ={c[2]:.2f} ЗП={c[3]:.2f} кг | "
                  f"W={W:.2f} кг | CoP X={copX:+.1f} см Y={copY:+.1f} см")
        return ()

    ax.legend(loc="lower right", fontsize=8, framealpha=0.7)
    ani = animation.FuncAnimation(fig, update, interval=ANIM_INTERVAL, blit=False,
                                  cache_frame_data=False)
    try:
        plt.show()
    finally:
        ser.close()
        print("Порт закрыт.")


if __name__ == "__main__":
    main()
