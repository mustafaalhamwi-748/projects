#ifndef MAGNETOMETERSENSOR_H
#define MAGNETOMETERSENSOR_H

namespace uavminedetection {

// ============================================================
// SensorReading — نتيجة قياس الحساس المغناطيسي
// ============================================================
struct SensorReading {
    double magneticValue;  // القيمة المقاسة (nT)
    bool   isMine;         // هل تجاوزت العتبة؟
    double confidence;     // 0.0 – 1.0 (مدى اليقين)
};

// ============================================================
// MagnetometerSensor — يستبدل CnnClassifier بالكامل
//
// المنطق بسيط وفيزيائي:
//   إذا magneticValue >= threshold → لغم محتمل
//   confidence تعكس مدى تجاوز العتبة
//
// لا يحتاج AI، لا يحتاج Python
// قرار رياضي بحت في سطرين
// ============================================================
class MagnetometerSensor
{
  public:
    // threshold:   العتبة بـ nT (ما فوقها يُعتبر لغماً)
    // saturation:  القيمة التي عندها confidence = 1.0
    MagnetometerSensor(double threshold, double saturation);

    // المدخل: قيمة المجال من MineField
    // المخرج: قرار + confidence
    SensorReading measure(double magneticValue) const;

    double getThreshold()   const { return threshold; }
    double getSaturation()  const { return saturation; }

  private:
    double threshold;
    double saturation;
};

} // namespace uavminedetection
#endif
