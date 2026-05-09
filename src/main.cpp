#include <Arduino.h>

#include "actuators.h"
#include "app_config.h"
#include "injection_controller.h"
#include "pins.h"
#include "sensors.h"

namespace {
AppConfig config = defaultConfig();
MapSensor mapSensor;
FloatSensor floatSensor;
PumpDriver pumpDriver;
InjectionController controller;

uint32_t lastLoopMs = 0;
} // namespace

void setup() {
  Serial.begin(115200);
  delay(200);

  mapSensor.begin(pins::MAP_SENSOR_ADC);
  floatSensor.begin(pins::FLOAT_SENSOR_DIGITAL);
  pumpDriver.begin(pins::PUMP_PWM);

  Serial.println("Water/Meth controller scaffold started");
}

  void loop() {
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - lastLoopMs) < config.loopPeriodMs) {
    return;
  }
  lastLoopMs = now;

  SensorReadings readings{};
  readings.mapKpa = mapSensor.readKpa();
  readings.mapValid = mapSensor.valid();
  readings.tankLow = floatSensor.isLow();

  PumpCommand command = controller.update(readings, config);
  pumpDriver.apply(command);

  // Placeholder debug output for early bring-up and wiring checks.
  Serial.print("MAP kPa: ");
  Serial.print(readings.mapKpa);
  Serial.print(" | tankLow: ");
  Serial.print(readings.tankLow ? "YES" : "NO");
  Serial.print(" | pump: ");
  Serial.print(command.enabled ? "ON" : "OFF");
  Serial.print(" @ ");
  Serial.println(command.pwm);
}
