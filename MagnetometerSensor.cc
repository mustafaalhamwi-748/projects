#include "MagnetometerSensor.h"
#include <algorithm>
#include <numeric>
#include <cmath>

namespace uavminedetection {

// ============================================================
// Constructor
// ============================================================
MagnetometerSensor::MagnetometerSensor(double staticThr, double sat,
                                       int winSize, double k, double minThr)
    : staticThreshold(staticThr),
      saturation(sat),
      windowSize(winSize),
      kSigma(k),
      minThreshold(minThr),
      adaptiveThreshold(staticThr)   // تبدأ مساوية للثابتة حتى تمتلئ النافذة
{}

// ============================================================
// addReading — إضافة قراءة للنافذة المنزلقة
//
// المنطق:
//   1. أضف القراءة للنهاية
//   2. إذا تجاوز الحجم windowSize، احذف الأقدم من البداية
//   3. بعد امتلاء النافذة: أعد حساب العتبة التكيفية
//
// لماذا لا نحسب في كل مرة؟
//   للأداء: إعادة الحساب O(N) كل 0.5 ثانية على 10 طائرات
//   مقبولة تماماً، لكن نتجنب الحساب في مرحلة الإحماء.
// ============================================================
void MagnetometerSensor::addReading(double value)
{
    window.push_back(value);

    // الاحتفاظ فقط بآخر windowSize قراءة
    if ((int)window.size() > windowSize)
        window.pop_front();

    // أعد الحساب فقط بعد امتلاء النافذة
    if ((int)window.size() >= windowSize)
        recomputeThreshold();
}

// ============================================================
// recomputeThreshold — قلب منطق العتبة التكيفية
//
// الخطوات:
//   1. حساب المتوسط (mean)
//   2. حساب الانحراف المعياري (stdDev) — نستخدم N وليس N-1
//      لأننا نُقدّر الخلفية لا عينة إحصائية
//   3. العتبة = mean + kSigma × stdDev
//   4. تطبيق الحد الأدنى الآمن (minThreshold)
//
// مثال عملي:
//   تربة طينية: mean=400nT, stdDev=80nT, k=3.0
//   → threshold = 400 + 3×80 = 640 nT
//   (أعلى من العتبة الثابتة 250 — منطقي في تربة معدنية)
//
//   تربة رملية: mean=220nT, stdDev=10nT, k=3.0
//   → threshold = 220 + 3×10 = 250 nT
//   (تساوي العتبة الثابتة تقريباً — منطقي في تربة نظيفة)
// ============================================================
void MagnetometerSensor::recomputeThreshold()
{
    const int N = (int)window.size();

    // ── الخطوة 1: المتوسط ───────────────────────────────────
    double sum = 0.0;
    for (double v : window) sum += v;
    windowMean = sum / N;

    // ── الخطوة 2: الانحراف المعياري ─────────────────────────
    double varSum = 0.0;
    for (double v : window) {
        double diff = v - windowMean;
        varSum += diff * diff;
    }
    windowStdDev = std::sqrt(varSum / N);

    // ── الخطوة 3: العتبة التكيفية ────────────────────────────
    double computed = windowMean + kSigma * windowStdDev;

    // ── الخطوة 4: تطبيق الحد الأدنى الآمن ───────────────────
    // يضمن أن العتبة لن تنزل إلى مستوى الضوضاء الطبيعية
    // حتى في المناطق ذات الخلفية المغناطيسية المنخفضة جداً
    adaptiveThreshold = std::max(minThreshold, computed);
}

// ============================================================
// getThreshold — العتبة الفعّالة الحالية
//
// قبل امتلاء النافذة: العتبة الثابتة (مرحلة الإحماء)
// بعد امتلاء النافذة: العتبة التكيفية المحسوبة
// ============================================================
double MagnetometerSensor::getThreshold() const
{
    return isAdaptiveReady() ? adaptiveThreshold : staticThreshold;
}

// ============================================================
// measure — القرار الفيزيائي
//
// [MODIFIED]: الآن يستخدم العتبة الفعّالة (تكيفية أو ثابتة)
//             ويُدرج effectiveThreshold في النتيجة للشفافية
//
// confidence:
//   0.0  عند العتبة بالضبط
//   0.5  في منتصف المسافة بين العتبة والتشبع
//   1.0  عند قيمة التشبع أو أعلى
// ============================================================
SensorReading MagnetometerSensor::measure(double magneticValue) const
{
    SensorReading r;
    r.magneticValue      = magneticValue;
    r.effectiveThreshold = getThreshold();
    r.isMine             = (magneticValue >= r.effectiveThreshold);

    if (r.isMine) {
        double range  = saturation - r.effectiveThreshold;
        double excess = magneticValue - r.effectiveThreshold;
        // تجنب القسمة على صفر إذا كانت العتبة == التشبع
        r.confidence = (range > 0.0) ? std::min(1.0, excess / range) : 1.0;
    } else {
        r.confidence = 0.0;
    }

    return r;
}

} // namespace uavminedetection
