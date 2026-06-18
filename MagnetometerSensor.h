#ifndef MAGNETOMETERSENSOR_H
#define MAGNETOMETERSENSOR_H

#include <cmath>

namespace uavminedetection {

// ============================================================
// SensorReading — نتيجة قياس الحساس المغناطيسي
// ============================================================
struct SensorReading {
    double magneticValue;   // القيمة المقاسة (nT)
    bool   isMine;          // هل تجاوزت العتبة الثابتة؟
    double confidence;      // 0.0 – 1.0 (مدى اليقين)
};

// ============================================================
// MagnetometerSensor — حساس مغناطيسي بعتبة ثابتة
//
// المنطق الفيزيائي:
//   يقارن القيمة المقاسة بعتبة ثابتة staticThreshold.
//   إذا تجاوزت القيمة العتبة → اكتشاف محتمل.
//   confidence = 0.0 عند العتبة، 1.0 عند التشبع أو أعلى.
// ============================================================
class MagnetometerSensor
{
  public:
    // threshold  : العتبة الثابتة (nT)
    // saturation : القيمة التي عندها confidence = 1.0
    MagnetometerSensor(double threshold, double saturation);

    // القياس الرئيسي
    SensorReading measure(double magneticValue) const;

    // معلومات الحالة
    double getThreshold()  const { return threshold; }
    double getSaturation() const { return saturation; }

  private:
    double threshold;
    double saturation;
};

} // namespace uavminedetection
#endif
