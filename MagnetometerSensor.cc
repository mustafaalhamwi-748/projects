#include "MagnetometerSensor.h"
#include <algorithm>
#include <cmath>

namespace uavminedetection {

MagnetometerSensor::MagnetometerSensor(double thr, double sat)
    : threshold(thr), saturation(sat)
{}

// ============================================================
// measure — القرار الفيزيائي
//
// confidence:
//   0.0  عند العتبة بالضبط
//   1.0  عند قيمة التشبع أو أعلى
// ============================================================
SensorReading MagnetometerSensor::measure(double magneticValue) const
{
    SensorReading r;
    r.magneticValue = magneticValue;
    r.isMine        = (magneticValue >= threshold);

    if (r.isMine) {
        double range  = saturation - threshold;
        double excess = magneticValue - threshold;
        r.confidence  = (range > 0.0) ? std::min(1.0, excess / range) : 1.0;
    } else {
        r.confidence = 0.0;
    }

    return r;
}

} // namespace uavminedetection
