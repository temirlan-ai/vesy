# -*- coding: utf-8 -*-
"""
Геометрия балансировочной платформы v2 (единый источник для всех скриптов).

Платформа: алюминиевый лист 2 мм на раме из профтрубы 25 мм,
           800 x 600 мм, высота 56 мм.
           800 мм - влево/вправо (ось X), 600 мм - вперёд/назад (ось Y).
Датчики:   4 x MH26G4MH782, 100 кг каждый (платформа ~400 кг).

Система координат: начало - центр листа, +X вправо, +Y вперёд.

Если перемеришь датчики - правь только SENSOR_OFFSETS_MM ниже,
всё остальное (позиции, ЦД, экран) пересчитается само.
"""

# ---- размеры листа, мм ----
PLATFORM_W_MM = 800.0   # влево-вправо (X)
PLATFORM_H_MM = 600.0   # вперёд-назад (Y)

SENSOR_CAPACITY_KG = 100.0
OVERLOAD_FRAC = 0.9     # предупреждение при нагрузке > 90% номинала

CORNER_ORDER = ["PL", "PP", "ZL", "ZP"]   # порядок каналов в CSV

# ---- отступы ЦЕНТРА датчика от ближайшего угла листа, мм ----
# (dx, dy): dx - вдоль стороны 800 ("по длине"), от левой/правой кромки;
#           dy - вдоль стороны 600 ("по ширине"), от передней/задней кромки.
SENSOR_OFFSETS_MM = {
    "PL": (35.0, 37.6),   # левый передний
    "PP": (35.0, 35.0),   # правый передний
    "ZL": (36.0, 39.0),   # левый задний
    "ZP": (38.0, 36.0),   # правый задний
}

# знаки угла: (лево=-1 / право=+1, зад=-1 / перед=+1)
_CORNER_SIGN = {"PL": (-1, +1), "PP": (+1, +1), "ZL": (-1, -1), "ZP": (+1, -1)}


def _sensor_pos_mm(name):
    sx, sy = _CORNER_SIGN[name]
    dx, dy = SENSOR_OFFSETS_MM[name]
    x = sx * (PLATFORM_W_MM / 2.0 - dx)
    y = sy * (PLATFORM_H_MM / 2.0 - dy)
    return x, y


# фактические координаты датчиков, см (для расчёта ЦД)
SENSOR_POS_CM = {n: tuple(v / 10.0 for v in _sensor_pos_mm(n)) for n in CORNER_ORDER}

PLATFORM_HALF_X_CM = PLATFORM_W_MM / 20.0   # 40
PLATFORM_HALF_Y_CM = PLATFORM_H_MM / 20.0   # 30

# полуразмеры "прямоугольника опор" (средние), см
SUPPORT_HALF_X_CM = sum(abs(SENSOR_POS_CM[n][0]) for n in CORNER_ORDER) / 4.0
SUPPORT_HALF_Y_CM = sum(abs(SENSOR_POS_CM[n][1]) for n in CORNER_ORDER) / 4.0


def compute_cop(corners):
    """Центр давления (см) = нагрузочно-взвешенное среднее позиций датчиков.
    corners - нагрузки [PL, PP, ZL, ZP] в кг."""
    s = sum(corners)
    if s <= 1e-6:
        return 0.0, 0.0
    cx = sum(l * SENSOR_POS_CM[n][0] for n, l in zip(CORNER_ORDER, corners)) / s
    cy = sum(l * SENSOR_POS_CM[n][1] for n, l in zip(CORNER_ORDER, corners)) / s
    return cx, cy


def geometry_dict():
    """Геометрия для браузера (JSON)."""
    return {
        "type": "geometry",
        "platform_cm": [PLATFORM_W_MM / 10.0, PLATFORM_H_MM / 10.0],
        "sensors_cm": {n: [round(SENSOR_POS_CM[n][0], 2), round(SENSOR_POS_CM[n][1], 2)]
                       for n in CORNER_ORDER},
        "capacity_kg": SENSOR_CAPACITY_KG,
        "overload_frac": OVERLOAD_FRAC,
    }


if __name__ == "__main__":
    print(f"Платформа {PLATFORM_W_MM:.0f} x {PLATFORM_H_MM:.0f} мм")
    for n in CORNER_ORDER:
        x, y = SENSOR_POS_CM[n]
        print(f"  {n}: X={x * 10:+.1f} мм  Y={y * 10:+.1f} мм")
    print(f"Опоры ~ {2 * SUPPORT_HALF_X_CM:.1f} x {2 * SUPPORT_HALF_Y_CM:.1f} см")
