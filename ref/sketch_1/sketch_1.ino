#include <HX711_ADC.h>

const int HX711_dout = 2; // D4
const int HX711_sck = 6;  // D5

HX711_ADC LoadCell(HX711_dout, HX711_sck);

void setup() {
  Serial.begin(9600);
  delay(500);

  LoadCell.begin();

  unsigned long stabilizingtime = 3000;
  boolean tare = true;

  LoadCell.start(stabilizingtime, tare);

  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("HX711 timeout. Check wiring.");
    while (1);
  }

  LoadCell.setCalFactor(1.0);
  Serial.println("Ready");
}

void loop() {
  LoadCell.update();

  float value = LoadCell.getData();
  Serial.println(value);

  delay(500);
}