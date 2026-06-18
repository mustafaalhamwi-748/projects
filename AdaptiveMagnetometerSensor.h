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
                                double initialThreshold,
                                double lowerPercentile = 0.45,
                                double clipPercentile = 0.85,
                                double recentWeight = 0.08,
                                double zoneSplitThreshold = 350.0,
                                double zone2KBoost = 0.15);

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
    double lowerPercentile;
    double clipPercentile;
    double recentWeight;
    double zoneSplitThreshold;
    double zone2KBoost;
    int    thresholdUpdateCount = 0;

    std::deque<double> allReadingsWindow;

    double percentileFromSorted(const std::vector<double>& sorted, double p) const;
    void updateThreshold();
};

} // namespace uavminedetection
#endif
