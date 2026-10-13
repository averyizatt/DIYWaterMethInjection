#include "adc_sampler.h"

#include <Arduino.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace adc_sampler {
namespace {
constexpr uint32_t kTotalHz = knock_band::kSampleRateHz * 2UL;   // Knock is every other conversion.
constexpr size_t kFrameBytes = 512;

adc_continuous_handle_t handle = nullptr;
adc_cali_handle_t calibration[kSlowCount] = {};
adc_channel_t knockChannel = ADC_CHANNEL_0;
adc_channel_t slowChannel[kSlowCount] = {};
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

uint16_t filling[knock_band::kSamples];
uint8_t fillCount = 0;
uint16_t newest[knock_band::kSamples];
bool newestFresh = false;
uint32_t slowSum[kSlowCount] = {};
uint32_t slowCount[kSlowCount] = {};
volatile uint32_t lastDataMs = 0;
volatile uint32_t knockRate = 0;
uint32_t knockCounted = 0;
uint16_t rangeLow = 0xFFFF;   // Lowest and highest knock sample since knockRange() was last called
uint16_t rangeHigh = 0;
uint32_t rateStartMs = 0;

bool channelFor(int pin, adc_channel_t &channel) {
  adc_unit_t unit;
  return adc_continuous_io_to_channel(pin, &unit, &channel) == ESP_OK && unit == ADC_UNIT_1;
}

void collect(void *) {
  static uint8_t frame[kFrameBytes];
  for (;;) {
    uint32_t length = 0;
    if (adc_continuous_read(handle, frame, sizeof(frame), &length, 100) != ESP_OK) continue;
    const uint32_t now = millis();
    lastDataMs = now;
    if (now - rateStartMs >= 1000) {
      knockRate = static_cast<uint32_t>(static_cast<uint64_t>(knockCounted) * 1000 / (now - rateStartMs));
      knockCounted = 0;
      rateStartMs = now;
    }
    for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= length; i += SOC_ADC_DIGI_RESULT_BYTES) {
      const adc_digi_output_data_t *sample = reinterpret_cast<const adc_digi_output_data_t *>(frame + i);
      const adc_channel_t channel = static_cast<adc_channel_t>(sample->type2.channel);
      const uint16_t raw = sample->type2.data;   // 12 bits
      if (channel == knockChannel) {
        ++knockCounted;
        const uint16_t ten = raw >> 2;           // 10 bits, as the knock maths expects
        if (ten < rangeLow || ten > rangeHigh) {
          portENTER_CRITICAL(&lock);
          if (ten < rangeLow) rangeLow = ten;
          if (ten > rangeHigh) rangeHigh = ten;
          portEXIT_CRITICAL(&lock);
        }
        filling[fillCount++] = ten;
        if (fillCount == knock_band::kSamples) {
          fillCount = 0;
          portENTER_CRITICAL(&lock);
          memcpy(newest, filling, sizeof(newest));
          newestFresh = true;
          portEXIT_CRITICAL(&lock);
        }
        continue;
      }
      for (uint8_t input = 0; input < kSlowCount; ++input) {
        if (channel != slowChannel[input]) continue;
        portENTER_CRITICAL(&lock);
        slowSum[input] += raw;
        ++slowCount[input];
        portEXIT_CRITICAL(&lock);
        break;
      }
    }
  }
}
} // namespace

bool begin(int knockPin, int oilPin, int fuelPin, int prePin, int postPin) {
  const int slowPins[kSlowCount] = {oilPin, fuelPin, prePin, postPin};
  if (!channelFor(knockPin, knockChannel)) return false;
  for (uint8_t input = 0; input < kSlowCount; ++input) {
    if (!channelFor(slowPins[input], slowChannel[input])) return false;
    adc_cali_curve_fitting_config_t cali = {};
    cali.unit_id = ADC_UNIT_1;
    cali.chan = slowChannel[input];
    cali.atten = ADC_ATTEN_DB_12;
    cali.bitwidth = ADC_BITWIDTH_12;
    // Without factory calibration data the plain scale below is used instead.
    if (adc_cali_create_scheme_curve_fitting(&cali, &calibration[input]) != ESP_OK) calibration[input] = nullptr;
  }

  adc_continuous_handle_cfg_t pool = {};
  pool.max_store_buf_size = 4096;
  pool.conv_frame_size = kFrameBytes;
  if (adc_continuous_new_handle(&pool, &handle) != ESP_OK) return false;

  adc_digi_pattern_config_t pattern[8] = {};
  for (uint8_t slot = 0; slot < 8; ++slot) {
    pattern[slot].atten = ADC_ATTEN_DB_12;
    pattern[slot].unit = ADC_UNIT_1;
    pattern[slot].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    pattern[slot].channel = (slot & 1) ? slowChannel[slot / 2] : knockChannel;
  }
  adc_continuous_config_t config = {};
  config.pattern_num = 8;
  config.adc_pattern = pattern;
  config.sample_freq_hz = kTotalHz;
  config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
  if (adc_continuous_config(handle, &config) != ESP_OK || adc_continuous_start(handle) != ESP_OK) return false;

  lastDataMs = rateStartMs = millis();
  return xTaskCreatePinnedToCore(collect, "adc", 4096, nullptr, 5, nullptr, 0) == pdPASS;
}

bool latestBurst(uint16_t out[knock_band::kSamples]) {
  bool fresh;
  portENTER_CRITICAL(&lock);
  fresh = newestFresh;
  if (fresh) {
    memcpy(out, newest, sizeof(newest));
    newestFresh = false;
  }
  portEXIT_CRITICAL(&lock);
  return fresh;
}

bool knockRange(uint16_t &low, uint16_t &high) {
  portENTER_CRITICAL(&lock);
  low = rangeLow;
  high = rangeHigh;
  rangeLow = 0xFFFF;
  rangeHigh = 0;
  portEXIT_CRITICAL(&lock);
  return low <= high;
}

bool averageMv(Slow input, float &millivolts) {
  uint32_t sum, count;
  portENTER_CRITICAL(&lock);
  sum = slowSum[input];
  count = slowCount[input];
  slowSum[input] = 0;
  slowCount[input] = 0;
  portEXIT_CRITICAL(&lock);
  if (count == 0) return false;
  const int raw = static_cast<int>((sum + count / 2) / count);
  int calibrated = 0;
  if (calibration[input] && adc_cali_raw_to_voltage(calibration[input], raw, &calibrated) == ESP_OK) {
    millivolts = static_cast<float>(calibrated);
  } else {
    millivolts = raw * 3100.0f / 4095.0f;
  }
  return true;
}

uint32_t silentForMs(uint32_t nowMs) {
  const uint32_t last = lastDataMs;
  // The collecting task may have stamped a later time than the caller's.
  return static_cast<int32_t>(nowMs - last) > 0 ? nowMs - last : 0;
}

uint32_t knockSamplesPerSecond() { return knockRate; }

} // namespace adc_sampler
