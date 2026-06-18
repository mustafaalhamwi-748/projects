#ifndef ADAPTIVEMAGNETOMETERSENSOR_H
#define ADAPTIVEMAGNETOMETERSENSOR_H

#include <deque>
#include <vector>
#include <cmath>
#include <algorithm>
#include "MagnetometerSensor.h"

namespace uavminedetection {

// ============================================================
// AdaptiveMagnetometerSensor — CFAR (Constant False Alarm Rate)
//
// المعادلة:
//   العتبة = Median(نافذة) + k × MAD_σ
//   مع سقف أعلى = maxThresholdRatio × saturation
//
// [مهم] لمنع "Runaway Feedback":
//   القراءات الملتقطة أثناء وضع البحث المكثف (Spiral) لا تُضاف
//   للنافذة، لأنها تمثل "إشارة هدف" لا "خلفية" — إضافتها تجعل
//   العتبة ترتفع بلا حدود وتُبقي الطائرة عالقة في الحلزون.
// ============================================================
class AdaptiveMagnetometerSensor
{
  public:
    AdaptiveMagnetometerSensor(double kFactor,
                                int    windowSize,
                                double saturation,
                                double initialThreshold);

    // updateWindow=false أثناء وضع البحث المكثف (Spiral/Intensive)
    SensorReading measure(double magneticValue, bool updateWindow = true);

    double getCurrentThreshold() const { return currentThreshold; }
    double getKFactor()          const { return kFactor; }
    int    getWindowSize()       const { return windowSize; }
    double getSaturation()       const { return saturation; }
    bool   isWarmedUp()          const
    { return (int)allReadingsWindow.size() >= warmupSize; }

  private:
    double kFactor;
    int    windowSize;
    double saturation;
    double currentThreshold;
    double initialThreshold;
    int    warmupSize;
    double maxThreshold;   // سقف أعلى لمنع الانفجار

    std::deque<double> allReadingsWindow;

    void updateThreshold();
};

} // namespace uavminedetection
#endif
