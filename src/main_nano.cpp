#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

#include "actuators.h"
#include "app_config.h"
#include "injection_controller.h"
#include "pins.h"
#include "sensors.h"

namespace {
AppConfig config = defaultConfig();
TankBlend blend = computeTankBlend(1.5f, 0.5f);

MapSensor mapSensor;
FloatSensor floatSensor;
PumpDriver pumpDriver;
WarningOutput warningOutput;
InjectionController controller;
MCP_CAN canBus(pins::CAN_SPI_CS);

uint32_t lastLoopMs = 0;
uint32_t lastDebugMs = 0;
uint32_t lastCanHeartbeatMs = 0;
uint32_t lastCanRetryMs = 0;
float knockBiasRaw = 0.0f;
float knockEnvelopeRaw = 0.0f;
bool canOnline = false;

constexpr float kKnockAdcRefVoltage = 5.0f;
constexpr float kKnockAdcMaxCount = 1023.0f;

constexpr float kKnockBiasAlpha = 0.01f;
constexpr float kKnockEnvelopeAlpha = 0.18f;
constexpr float kKnockSoftwareGain = 1.0f;
constexpr float kKnockDetectThresholdRaw = 35.0f;
constexpr uint16_t kCanHeartbeatId = 0x300;
constexpr uint32_t kCanRetryMs = 2000;
constexpr uint32_t kCanHeartbeatMs = 1000;

inline bool elapsed(uint32_t now, uint32_t start, uint32_t intervalMs) {
  return static_cast<uint32_t>(now - start) >= intervalMs;
}

bool initCanBus() {
  pinMode(pins::CAN_SPI_INT, INPUT_PULLUP);
  SPI.begin();

  if (canBus.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN: MCP2515 init failed");
    return false;
  }

  canBus.setMode(MCP_NORMAL);
  Serial.println("CAN: MCP2515 online at 500 kbps");
  Serial.print("CAN: CS=D");
  Serial.print(pins::CAN_SPI_CS);
  Serial.print(" INT=D");
  Serial.println(pins::CAN_SPI_INT);
  return true;
}

void printCanFrame(unsigned long id, byte len, const byte *data) {
  Serial.print("CAN RX id=0x");
  Serial.print(id, HEX);
  Serial.print(" len=");
  Serial.print(len);
  Serial.print(" data=");
  for (byte i = 0; i < len; ++i) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    if (i + 1 < len) Serial.print(' ');
  }
  Serial.println();
}

void serviceCanBus(uint32_t now, bool knockDetected, int knockRaw) {
  if (!canOnline) {
    if (elapsed(now, lastCanRetryMs, kCanRetryMs)) {
      lastCanRetryMs = now;
      canOnline = initCanBus();
    }
    return;
  }

  byte framesRead = 0;
  while (digitalRead(pins::CAN_SPI_INT) == LOW && framesRead < 8) {
    unsigned long rxId = 0;
    byte len = 0;
    byte data[8] = {};

    if (canBus.readMsgBuf(&rxId, &len, data) == CAN_OK) {
      printCanFrame(rxId, len, data);
    } else {
      Serial.println("CAN: read failed");
      break;
    }
    ++framesRead;
  }

  if (elapsed(now, lastCanHeartbeatMs, kCanHeartbeatMs)) {
    lastCanHeartbeatMs = now;
    byte tx[8] = {
        0x4E, // N
        0x41, // A
        0x4E, // N
        0x4F, // O
        static_cast<byte>(knockDetected ? 1 : 0),
        static_cast<byte>(knockRaw & 0xFF),
        static_cast<byte>((knockRaw >> 8) & 0x03),
        0};

    if (canBus.sendMsgBuf(kCanHeartbeatId, 0, static_cast<byte>(sizeof(tx)), tx) != CAN_OK) {
      Serial.println("CAN: TX failed, will retry init");
      canOnline = false;
    }
  }
}
} // namespace

void setup() {
  Serial.begin(config.serialBaud);
  delay(100);

  mapSensor.begin(pins::MAP_SENSOR_ADC, config.map);
  floatSensor.begin(pins::FLOAT_SENSOR_DIGITAL,
                    config.floatActiveLow,
                    config.floatDebounceMs,
                    config.floatLowShutdownDelayMs);
  pumpDriver.beginRelay(pins::PUMP_OUT, config.relayPeriodMs);
  warningOutput.begin(pins::WARNING_LED, true);
  pinMode(pins::KNOCK_SENSOR_ADC, INPUT);

  Serial.println("Nano minimal injection firmware");
  Serial.print("Knock pin: ");
  Serial.println(pins::KNOCK_SENSOR_ADC);
  canOnline = initCanBus();
  lastCanRetryMs = millis();
}

void loop() {
  const uint32_t now = millis();
  if (!elapsed(now, lastLoopMs, config.loopPeriodMs)) return;
  lastLoopMs = now;

  const int knockRaw = analogRead(pins::KNOCK_SENSOR_ADC);
  if (knockBiasRaw <= 0.0f) {
    knockBiasRaw = static_cast<float>(knockRaw);
  } else {
    knockBiasRaw += (static_cast<float>(knockRaw) - knockBiasRaw) * kKnockBiasAlpha;
  }

  const float knockDeltaRaw = static_cast<float>(knockRaw) - knockBiasRaw;
  const float knockDeltaGained = knockDeltaRaw * kKnockSoftwareGain;
  const float knockAbsGained = fabs(knockDeltaGained);
  knockEnvelopeRaw += (knockAbsGained - knockEnvelopeRaw) * kKnockEnvelopeAlpha;
  const bool knockDetected = knockEnvelopeRaw >= kKnockDetectThresholdRaw;

  serviceCanBus(now, knockDetected, knockRaw);

  if (elapsed(now, lastDebugMs, 500)) {
    lastDebugMs = now;
    Serial.print("KNOCK raw=");
    Serial.print(knockRaw);
    Serial.print(" v=");
    Serial.print(static_cast<float>(knockRaw) * kKnockAdcRefVoltage / kKnockAdcMaxCount, 3);
    Serial.print(" biasRaw=");
    Serial.print(knockBiasRaw, 1);
    Serial.print(" biasV=");
    Serial.print(knockBiasRaw * kKnockAdcRefVoltage / kKnockAdcMaxCount, 3);
    Serial.print(" deltaRaw=");
    Serial.print(knockDeltaRaw, 1);
    Serial.print(" deltaGain=");
    Serial.print(knockDeltaGained, 1);
    Serial.print(" envGain=");
    Serial.print(knockEnvelopeRaw, 1);
    Serial.print(" gain=");
    Serial.print(kKnockSoftwareGain, 1);
    Serial.print(" detected=");
    Serial.println(knockDetected ? "YES" : "no");
  }
}
