/*
 * Балансировочная платформа на 4 тензодатчиках (Center of Pressure) - v2
 * --------------------------------------------------------------------
 * Железо: Arduino Nano (ATmega328P) + 4x HX711 + 4x датчика MH26G4MH782,
 *         100 кг каждый (полный мост). Платформа ~400 кг (4 x 100 кг).
 *
 * Механика: лист алюминия 2 мм на раме из профтрубы 25 мм,
 *           800 x 600 мм, высота 56 мм. 800 - влево/вправо, 600 - вперёд/назад.
 *   Центры датчиков от углов листа (мм, по длине 800 / по ширине 600):
 *     ПЛ 35 / 37.6    ПП 35 / 35    ЗЛ 36 / 39    ЗП 38 / 36
 *   Координаты для центра давления считает ПК (platform_geometry.py);
 *   balX/balY ниже - просто доли нагрузки, без геометрии.
 *
 * Распиновка датчик -> HX711 (для каждого из 4 каналов):
 *   красный  -> E+      чёрный -> E-
 *   белый    -> A-      зелёный -> A+   (полярность сигнала некритична)
 *
 * Распиновка HX711 -> Nano:
 *   VCC -> 5V, GND -> GND (все 4 в параллель)
 *   SCK -> общий D6 на все 4 модуля
 *   DT (DOUT) у каждого свой:
 *       Канал 0  ПЛ (перед-лево)  -> D2
 *       Канал 1  ПП (перед-право) -> D3
 *       Канал 2  ЗЛ (зад-лево)    -> D4
 *       Канал 3  ЗП (зад-право)   -> D5
 *
 * Раскладка углов (вид сверху):
 *        ПЕРЕД
 *     ПЛ ----- ПП
 *     |         |
 *     ЗЛ ----- ЗП
 *        ЗАД
 *
 * Команды по Serial (9600 бод, перевод строки '\n' или '\r'):
 *   t            -> тарировать (обнулить) все 4 канала
 *   c <mass>     -> ОБЩАЯ калибровка: положи ОДИН эталон mass (в КГ) в
 *                   ЦЕНТР платформы и отправь команду. Один коэффициент на все
 *                   4 датчика так, чтобы их сумма = mass. Пример: "c 50"
 *   k<ch> <mass> -> (опц.) откалибровать ОДИН канал ch (0..3) эталоном mass (кг)
 *                   на этом углу. Пример: "k0 25"
 *   z            -> вывести текущие калибровочные коэффициенты
 *   r            -> сбросить калибровки к 1.0 (в EEPROM тоже)
 *   d            -> режим ДИАГНОСТИКИ: быстрый дамп сырых показаний каждого
 *                   канала (мгновенные кг + размах шума). Повторно 'd' - выкл.
 *
 * Постоянный вывод (CSV, для Python-визуализатора):
 *   ПЛ,ПП,ЗЛ,ЗП,W,balX,balY
 *
 *   ПЛ..ЗП - нагрузка каждого датчика, кг
 *   W      - суммарный вес, кг
 *   balX   - смещение вправо, %  (+ вправо, - влево)
 *   balY   - смещение вперёд, %  (+ вперёд, - назад)
 *
 * Перегрузка: если канал > 90% номинала (90 кг), раз в секунду печатается
 *   WARN OVERLOAD <канал> <кг>
 * (нечисловая строка - визуализаторы её пропускают).
 *
 * Библиотека: HX711_ADC (Olav Kallhovd) - https://github.com/olkal/HX711_ADC
 */

#include <HX711_ADC.h>
#include <EEPROM.h>

// ---------- пины ----------
const int DOUT_PINS[4] = {2, 3, 4, 5};   // DT каждого канала
const int SCK_PIN      = 6;              // общий тактовый на все 4 HX711

// имена углов для подписи
const char* CH_NAME[4] = {"PL", "PP", "ZL", "ZP"};

// ---------- объекты HX711 ----------
HX711_ADC LoadCell[4] = {
  HX711_ADC(DOUT_PINS[0], SCK_PIN),
  HX711_ADC(DOUT_PINS[1], SCK_PIN),
  HX711_ADC(DOUT_PINS[2], SCK_PIN),
  HX711_ADC(DOUT_PINS[3], SCK_PIN)
};

// ---------- EEPROM ----------
// 4 калибровочных коэффициента (float) подряд с адреса 0: 0,4,8,12
const int CAL_EEPROM_ADDR = 0;

float calFactor[4] = {1.0f, 1.0f, 1.0f, 1.0f};

// ---------- состояние ----------
unsigned long tLast = 0;
const unsigned long PRINT_INTERVAL = 100;    // мс между выводами (10 раз в секунду)

// порог "тишины" по каналу, чтобы шум не считался за вес (датчик 100 кг).
// Подстрой по режиму диагностики 'd': чуть выше размаха шума в покое.
const float NOISE_GATE = 0.2f;    // кг

// перегрузка датчика
const float SENSOR_CAPACITY_KG = 100.0f;
const float OVERLOAD_FRAC      = 0.9f;        // предупреждать с 90% номинала
const unsigned long WARN_INTERVAL = 1000;     // мс между предупреждениями
unsigned long tWarn = 0;

// ---------- сглаживание ----------
// Экспоненциальное скользящее среднее (EMA) по каждому каналу, чтобы
// показания не "прыгали". Чем меньше ALPHA, тем плавнее (но медленнее реакция).
const float EMA_ALPHA = 0.25f;
float wFilt[4]  = {0.0f, 0.0f, 0.0f, 0.0f};   // сглаженная нагрузка по каналам, кг
bool  emaInit   = false;                      // первый отсчёт берём как есть

// Медианный пред-фильтр: выкидывает редкие выбросы (спайки контактного шума)
// до того, как они попадут в EMA. Окно MED_N сырых отсчётов на канал.
const int MED_N = 5;
float medBuf[4][MED_N];
int   medIdx[4]   = {0, 0, 0, 0};
int   medCount[4] = {0, 0, 0, 0};

// ---------- диагностика ----------
bool  debugMode = false;                      // режим дампа сырых значений (команда 'd')
const unsigned long DEBUG_INTERVAL = 250;     // мс между строками дампа
unsigned long tDebug = 0;
float wInst[4]  = {0.0f, 0.0f, 0.0f, 0.0f};   // мгновенное (несглаженное) значение, кг
float wMin[4], wMax[4];                        // экстремумы за окно для оценки размаха шума

// -------------------------------------------------------------------

void loadCalibration() {
  for (int i = 0; i < 4; i++) {
    float v;
    EEPROM.get(CAL_EEPROM_ADDR + i * sizeof(float), v);
    // защита от пустой/битой EEPROM (NaN/inf/0 отравляют расчёт)
    if (isnan(v) || isinf(v) || v == 0.0f) v = 1.0f;
    calFactor[i] = v;
    LoadCell[i].setCalFactor(v);
  }
}

void saveCalibration(int ch) {
  EEPROM.put(CAL_EEPROM_ADDR + ch * sizeof(float), calFactor[ch]);
}

void printCalFactors() {
  Serial.print("CAL:");
  for (int i = 0; i < 4; i++) {
    Serial.print(CH_NAME[i]); Serial.print('=');
    Serial.print(calFactor[i], 4);
    if (i < 3) Serial.print(' ');
  }
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  delay(200);

  for (int i = 0; i < 4; i++) LoadCell[i].begin();

  // одновременный старт всех каналов с тарировкой
  unsigned long stabilizingTime = 3000;
  boolean doTare = true;
  byte ready = 0;
  for (int i = 0; i < 4; i++) LoadCell[i].start(stabilizingTime, false);

  // ручная общая тарировка после стабилизации (start с tare=false выше),
  // чтобы все 4 обнулились синхронно
  delay(stabilizingTime);
  for (int i = 0; i < 4; i++) {
    LoadCell[i].update();
  }

  // проверка связи с каждым модулем
  for (int i = 0; i < 4; i++) {
    if (LoadCell[i].getTareTimeoutFlag()) {
      Serial.print("TIMEOUT channel ");
      Serial.print(i);
      Serial.print(" (");
      Serial.print(CH_NAME[i]);
      Serial.println(") - check DT/SCK wiring");
    }
  }

  loadCalibration();
  tareAll();
  printCalFactors();
  Serial.println("READY");
}

void tareAll() {
  for (int i = 0; i < 4; i++) LoadCell[i].tareNoDelay();
  // ждём завершения тары на всех каналах
  bool done = false;
  unsigned long t0 = millis();
  while (!done && (millis() - t0 < 4000)) {
    done = true;
    for (int i = 0; i < 4; i++) {
      LoadCell[i].update();
      if (!LoadCell[i].getTareStatus()) done = false;
    }
  }
  // сбрасываем фильтры, чтобы после тары сглаживание стартовало с нуля
  for (int i = 0; i < 4; i++) {
    wFilt[i] = 0.0f;
    medIdx[i] = 0;
    medCount[i] = 0;
  }
  emaInit = false;
  Serial.println("TARED");
}

// ОБЩАЯ калибровка: один груз в центре платформы, один коэффициент на все 4
// датчика так, чтобы сумма их показаний равнялась знакомой массе.
// (баланс меряет соотношения нагрузок, поэтому общего коэффициента достаточно)
void calibrateCommon(float knownMass) {
  if (knownMass == 0) {
    Serial.println("CAL_BAD_ARGS");
    return;
  }
  // снимаем коэффициент -> getData() даёт "сырое" значение (raw минус тара)
  float rawSum = 0.0f;
  for (int i = 0; i < 4; i++) {
    LoadCell[i].setCalFactor(1.0f);
    LoadCell[i].refreshDataSet();
    rawSum += LoadCell[i].getData();
  }
  if (rawSum == 0.0f || isnan(rawSum) || isinf(rawSum)) {
    Serial.println("CAL_FAIL");
    // вернуть прежние коэффициенты
    for (int i = 0; i < 4; i++) LoadCell[i].setCalFactor(calFactor[i]);
    return;
  }
  float factor = rawSum / knownMass;   // raw -> кг для всех каналов одинаково
  for (int i = 0; i < 4; i++) {
    calFactor[i] = factor;
    LoadCell[i].setCalFactor(factor);
    saveCalibration(i);
  }
  Serial.print("CAL_OK common factor=");
  Serial.println(factor, 4);
}

// калибровка одного канала по эталонному грузу (кг) на этом углу
void calibrateChannel(int ch, float knownMass) {
  if (ch < 0 || ch > 3 || knownMass == 0) {
    Serial.println("CAL_BAD_ARGS");
    return;
  }
  LoadCell[ch].refreshDataSet();                 // свежий набор данных под грузом
  float newCal = LoadCell[ch].getNewCalibration(knownMass);
  if (isnan(newCal) || isinf(newCal) || newCal == 0.0f) {
    Serial.println("CAL_FAIL");
    return;
  }
  calFactor[ch] = newCal;
  LoadCell[ch].setCalFactor(newCal);
  saveCalibration(ch);
  Serial.print("CAL_OK ");
  Serial.print(CH_NAME[ch]);
  Serial.print('=');
  Serial.println(newCal, 4);
}

void handleSerial() {
  if (Serial.available() == 0) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  char c = cmd.charAt(0);

  if (c == 't') {
    tareAll();
  }
  else if (c == 'c') {
    // общая калибровка: "c <mass>"  (груз в центре)
    float mass = cmd.substring(1).toFloat();
    calibrateCommon(mass);
  }
  else if (c == 'z') {
    printCalFactors();
  }
  else if (c == 'r') {
    for (int i = 0; i < 4; i++) {
      calFactor[i] = 1.0f;
      LoadCell[i].setCalFactor(1.0f);
      saveCalibration(i);
    }
    Serial.println("CAL_RESET");
  }
  else if (c == 'k') {
    // формат: k<ch> <mass>   например "k0 25"
    int ch = cmd.charAt(1) - '0';
    float mass = cmd.substring(2).toFloat();
    calibrateChannel(ch, mass);
  }
  else if (c == 'd') {
    debugMode = !debugMode;
    if (debugMode) {
      for (int i = 0; i < 4; i++) { wMin[i] = wInst[i]; wMax[i] = wInst[i]; }
      tDebug = millis();
      Serial.println("DEBUG ON  (канал=мгновенно(размах) кг)");
    } else {
      Serial.println("DEBUG OFF");
    }
  }
}

// медиана первых n элементов массива (n <= MED_N) через сортировку копии
float medianOf(float *src, int n) {
  float t[MED_N];
  for (int i = 0; i < n; i++) t[i] = src[i];
  for (int i = 1; i < n; i++) {       // сортировка вставками (массив крошечный)
    float k = t[i];
    int j = i - 1;
    while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
    t[j + 1] = k;
  }
  return t[n / 2];
}

void loop() {
  handleSerial();

  // Обновляем каналы каждый цикл. update() возвращает true, когда HX711 выдал
  // новый отсчёт - только тогда подмешиваем его в EMA-фильтр. Так сглаживание
  // идёт непрерывно (не раз в PRINT_INTERVAL) и значения не "прыгают".
  for (int i = 0; i < 4; i++) {
    if (LoadCell[i].update()) {
      float v = LoadCell[i].getData();   // мгновенная нагрузка канала, кг
      wInst[i] = v;                      // сырое значение (для диагностики 'd')
      // следим за размахом сырого шума в окне диагностики
      if (v < wMin[i]) wMin[i] = v;
      if (v > wMax[i]) wMax[i] = v;

      // 1) медианный фильтр по последним MED_N сырым отсчётам - режет выбросы
      medBuf[i][medIdx[i]] = v;
      medIdx[i] = (medIdx[i] + 1) % MED_N;
      if (medCount[i] < MED_N) medCount[i]++;
      float vm = medianOf(medBuf[i], medCount[i]);

      // 2) EMA по медиане - финальное плавное значение
      if (!emaInit) {
        wFilt[i] = vm;                   // первый отсчёт берём как есть
      } else {
        wFilt[i] = EMA_ALPHA * vm + (1.0f - EMA_ALPHA) * wFilt[i];
      }
    }
  }
  emaInit = true;

  if (debugMode) {
    if (millis() - tDebug >= DEBUG_INTERVAL) {
      tDebug = millis();
      // мгновенное значение каждого канала и размах (max-min) за окно
      for (int i = 0; i < 4; i++) {
        Serial.print(CH_NAME[i]); Serial.print('=');
        Serial.print(wInst[i], 3);
        Serial.print('('); Serial.print(wMax[i] - wMin[i], 3); Serial.print(')');
        Serial.print(i < 3 ? "  " : "\n");
        wMin[i] = wInst[i];              // сброс окна размаха на следующий период
        wMax[i] = wInst[i];
      }
    }
    return;                              // в режиме диагностики обычный CSV не шлём
  }

  if (millis() - tLast >= PRINT_INTERVAL) {
    tLast = millis();

    float w[4];
    for (int i = 0; i < 4; i++) {
      w[i] = wFilt[i];
      // отсекаем шум возле нуля
      if (w[i] < NOISE_GATE && w[i] > -NOISE_GATE) w[i] = 0.0f;
    }

    float W = w[0] + w[1] + w[2] + w[3];

    // предупреждение о перегрузке (не чаще раза в WARN_INTERVAL)
    if (millis() - tWarn >= WARN_INTERVAL) {
      for (int i = 0; i < 4; i++) {
        if (w[i] > SENSOR_CAPACITY_KG * OVERLOAD_FRAC) {
          tWarn = millis();
          Serial.print("WARN OVERLOAD ");
          Serial.print(CH_NAME[i]); Serial.print(' ');
          Serial.println(w[i], 1);
        }
      }
    }

    // балансы считаем только при заметной нагрузке, иначе делим на шум
    float balX = 0.0f, balY = 0.0f;
    if (W > 2.0f * NOISE_GATE) {
      float left  = w[0] + w[2];   // ПЛ + ЗЛ
      float right = w[1] + w[3];   // ПП + ЗП
      float front = w[0] + w[1];   // ПЛ + ПП
      float rear  = w[2] + w[3];   // ЗЛ + ЗП
      balX = ((right - left) / W) * 100.0f;
      balY = ((front - rear) / W) * 100.0f;
    }

    // нагрузка каждого датчика и суммарный вес выводим в КГ
    Serial.print(w[0], 2); Serial.print(',');
    Serial.print(w[1], 2); Serial.print(',');
    Serial.print(w[2], 2); Serial.print(',');
    Serial.print(w[3], 2); Serial.print(',');
    Serial.print(W, 2);    Serial.print(',');
    Serial.print(balX, 1); Serial.print(',');
    Serial.println(balY, 1);
  }
}
