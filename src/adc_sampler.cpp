#include "adc_sampler.h"

#include <Arduino.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace adc_sampler {
namespace {
constexpr uint32_t kTotalHz = kKnockRateHz * 2UL;   // Knock is every other conversion.
constexpr size_t kChunkBytes = 1024;                 // 256 conversions: 3.3 ms
constexpr uint8_t kRing = 128;                       // Frames waiting for the main loop: 64 ms
constexpr uint32_t kRateWindowMs = 2000;             // The sample rate is measured over this long
constexpr uint32_t kMinPlausibleRateHz = 8000;

adc_continuous_handle_t handle = nullptr;
adc_cali_handle_t calibration[kSlowCount] = {};
adc_cali_handle_t knockCalibration = nullptr;
constexpr float kFullScaleMv = 3100.0f;     // The ADC's range at this attenuation, without calibration
float railLow = 30.0f, railHigh = 985.0f;   // A knock sample at or beyond these counts is at a limit
adc_channel_t knockChannel = ADC_CHANNEL_0;
adc_channel_t slowChannel[kSlowCount] = {};
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

// Shared with the main loop, under the lock.
knock_dsp::Frame ring[kRing];
uint8_t ringHead = 0, ringCount = 0;
float statLow = 1e9f, statHigh = -1e9f, statBias = 0.0f;
uint32_t statRailed = 0, statSamples = 0;
uint32_t slowSum[kSlowCount] = {};
uint32_t slowCount[kSlowCount] = {};
float wantedCentre = 6000.0f, wantedBandwidth = 1500.0f;
bool bandChanged = true;
bool secondActive = false;
volatile uint32_t lastDataMs = 0;
volatile uint32_t knockRate = 0;
volatile uint32_t filterRate = kKnockRateHz;

// The collecting task's own.
knock_dsp::FrontEnd front;
uint32_t knockCounted = 0;
uint32_t rateStartMs = 0;
uint32_t offRateSum = 0;
uint8_t offRateWindows = 0;

bool channelFor(int pin, adc_channel_t &channel) {
  adc_unit_t unit;
  return adc_continuous_io_to_channel(pin, &unit, &channel) == ESP_OK && unit == ADC_UNIT_1;
}

void designFilters(uint32_t rate) {
  float centre, bandwidth;
  portENTER_CRITICAL(&lock);
  centre = wantedCentre;
  bandwidth = wantedBandwidth;
  bandChanged = false;
  portEXIT_CRITICAL(&lock);
  front.configure(static_cast<float>(rate), centre, bandwidth);
  portENTER_CRITICAL(&lock);
  secondActive = front.secondActive();
  portEXIT_CRITICAL(&lock);
  filterRate = rate;
}

// The measured rate, and the filters redesigned for it when it is steadily more than
// 1 % from what they were designed for.
void measureRate(uint32_t now) {
  if (now - rateStartMs < kRateWindowMs) return;
  const uint32_t measured = static_cast<uint32_t>(static_cast<uint64_t>(knockCounted) * 1000 / (now - rateStartMs));
  knockCounted = 0;
  rateStartMs = now;
  knockRate = measured;
  const uint32_t designed = filterRate;
  const uint32_t off = measured > designed ? measured - designed : designed - measured;
  if (measured < kMinPlausibleRateHz || off * 100 <= designed) {
    offRateWindows = 0;
    offRateSum = 0;
    return;
  }
  offRateSum += measured;
  if (++offRateWindows < 3) return;
  designFilters(offRateSum / offRateWindows);
  offRateWindows = 0;
  offRateSum = 0;
}

void collect(void *) {
  static uint8_t chunk[kChunkBytes];
  designFilters(kKnockRateHz);
  for (;;) {
    uint32_t length = 0;
    if (adc_continuous_read(handle, chunk, sizeof(chunk), &length, 100) != ESP_OK) continue;
    const uint32_t now = millis();
    lastDataMs = now;
    measureRate(now);
    if (bandChanged) designFilters(filterRate);

    // This chunk's results are gathered here and handed over in one go.
    knock_dsp::Frame frames[16];
    uint8_t frameCount = 0;
    float low = 1e9f, high = -1e9f;
    uint32_t railed = 0, samples = 0;
    uint32_t sum[kSlowCount] = {};
    uint32_t count[kSlowCount] = {};

    for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= length; i += SOC_ADC_DIGI_RESULT_BYTES) {
      const adc_digi_output_data_t *sample = reinterpret_cast<const adc_digi_output_data_t *>(chunk + i);
      const adc_channel_t channel = static_cast<adc_channel_t>(sample->type2.channel);
      const uint32_t raw = sample->type2.data;   // 12 bits
      if (channel == knockChannel) {
        const float ten = raw * 0.25f;           // 10-bit counts, with the two extra bits kept as fractions
        ++samples;
        if (ten < low) low = ten;
        if (ten > high) high = ten;
        if (ten <= railLow || ten >= railHigh) ++railed;
        knock_dsp::Frame frame;
        if (front.push(ten, frame) && frameCount < sizeof(frames) / sizeof(frames[0])) frames[frameCount++] = frame;
        continue;
      }
      for (uint8_t input = 0; input < kSlowCount; ++input) {
        if (channel != slowChannel[input]) continue;
        sum[input] += raw;
        ++count[input];
        break;
      }
    }
    knockCounted += samples;

    portENTER_CRITICAL(&lock);
    for (uint8_t n = 0; n < frameCount; ++n) {
      ring[(ringHead + ringCount) % kRing] = frames[n];
      if (ringCount < kRing) ++ringCount;
      else ringHead = (ringHead + 1) % kRing;   // Nobody is collecting: the oldest goes.
    }
    if (samples > 0) {
      if (low < statLow) statLow = low;
      if (high > statHigh) statHigh = high;
      statRailed += railed;
      statSamples += samples;
      statBias = front.bias();
    }
    for (uint8_t input = 0; input < kSlowCount; ++input) {
      slowSum[input] += sum[input];
      slowCount[input] += count[input];
    }
    portEXIT_CRITICAL(&lock);
  }
}

// Millivolts at the pin for a reading in 10-bit counts.
float toMv(float counts) {
  int calibrated = 0;
  const int raw = static_cast<int>(counts * 4.0f + 0.5f);
  if (knockCalibration && adc_cali_raw_to_voltage(knockCalibration, raw, &calibrated) == ESP_OK) return static_cast<float>(calibrated);
  return counts * kFullScaleMv / 1023.0f;
}
} // namespace

void setKnockLimitsMv(float lowMv, float highMv) {
  // In counts by the nominal scale, a little inside each limit so a clipped sample
  // counts even when this ADC reads a few percent off.
  const float margin = (highMv - lowMv) * 0.03f;
  railLow = (lowMv + margin) * 1023.0f / kFullScaleMv;
  railHigh = (highMv - margin) * 1023.0f / kFullScaleMv;
}

bool begin(int knockPin, int oilPin, int fuelPin, int prePin, int postPin) {
  const int slowPins[kSlowCount] = {oilPin, fuelPin, prePin, postPin};
  if (!channelFor(knockPin, knockChannel)) return false;
  {
    adc_cali_curve_fitting_config_t cali = {};
    cali.unit_id = ADC_UNIT_1;
    cali.chan = knockChannel;
    cali.atten = ADC_ATTEN_DB_12;
    cali.bitwidth = ADC_BITWIDTH_12;
    if (adc_cali_create_scheme_curve_fitting(&cali, &knockCalibration) != ESP_OK) knockCalibration = nullptr;
  }
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
  pool.max_store_buf_size = 8192;
  pool.conv_frame_size = kChunkBytes;
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
  // The filters run in floating point; a task that does is tied to one processor core.
  return xTaskCreatePinnedToCore(collect, "adc", 8192, nullptr, 5, nullptr, 0) == pdPASS;
}

void setKnockBand(float centreHz, float bandwidthHz) {
  portENTER_CRITICAL(&lock);
  if (centreHz != wantedCentre || bandwidthHz != wantedBandwidth) {
    wantedCentre = centreHz;
    wantedBandwidth = bandwidthHz;
    bandChanged = true;
  }
  portEXIT_CRITICAL(&lock);
}

size_t takeKnockFrames(knock_dsp::Frame *out, size_t capacity) {
  size_t taken = 0;
  portENTER_CRITICAL(&lock);
  while (ringCount > 0 && taken < capacity) {
    out[taken++] = ring[ringHead];
    ringHead = (ringHead + 1) % kRing;
    --ringCount;
  }
  portEXIT_CRITICAL(&lock);
  return taken;
}

bool takeKnockStats(KnockStats &out) {
  float low, high, bias;
  uint32_t railed, samples;
  portENTER_CRITICAL(&lock);
  low = statLow;
  high = statHigh;
  bias = statBias;
  railed = statRailed;
  samples = statSamples;
  statLow = 1e9f;
  statHigh = -1e9f;
  statRailed = 0;
  statSamples = 0;
  portEXIT_CRITICAL(&lock);
  if (samples == 0) return false;
  out.low = static_cast<uint16_t>(low < 0.0f ? 0.0f : low);
  out.high = static_cast<uint16_t>(high > 1023.0f ? 1023.0f : high + 0.5f);
  out.railed = railed;
  out.samples = samples;
  out.bias = bias;
  out.lowMv = toMv(low < 0.0f ? 0.0f : low);
  out.highMv = toMv(high > 1023.0f ? 1023.0f : high);
  out.restMv = toMv(bias);
  return true;
}

bool knockSecondActive() {
  bool active;
  portENTER_CRITICAL(&lock);
  active = secondActive;
  portEXIT_CRITICAL(&lock);
  return active;
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
uint32_t knockFilterRateHz() { return filterRate; }

} // namespace adc_sampler
