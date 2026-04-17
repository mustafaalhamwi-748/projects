#include "MagnetometerSensor.h"
#include <algorithm>

namespace uavminedetection {

MagnetometerSensor::MagnetometerSensor(double thr, double sat)
    : threshold(thr), saturation(sat)
{}

// ============================================================
// measure — القرار الفيزيائي
//
// confidence:
//   0.0  عند العتبة بالضبط (threshold)
//   0.5  عند منتصف المسافة بين العتبة والتشبع
//   1.0  عند قيمة التشبع أو أعلى (saturation)
// ============================================================
SensorReading MagnetometerSensor::measure(double magneticValue) const
{
    SensorReading r;
    r.magneticValue = magneticValue;
    r.isMine        = (magneticValue >= threshold);

    if (r.isMine) {
        // confidence تتزايد خطياً من 0 إلى 1
        double range  = saturation - threshold;
        double excess = magneticValue - threshold;
        r.confidence  = std::min(1.0, excess / range);
    } else {
        r.confidence = 0.0;
    }

    return r;
}

} // namespace uavminedetection
