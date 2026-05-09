#pragma once

struct SensorReadings {
  float mapKpa{100.0f};
  bool mapValid{false};
  bool tankLow{true};
};

class MapSensor {
public:
  void begin(int analogPin);
  float readKpa() const;
  bool valid() const;

private:
  int pin_{-1};
  bool valid_{false};
};

class FloatSensor {
public:
  void begin(int digitalPin);
  bool isLow() const;

private:
  int pin_{-1};
};
