#include "MagnetometerSensor.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <vector>    // [FIX]: مطلوب لـ std::vector في recomputeThreshold

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
// recomputeThreshold — [MODIFIED v3]: Median + MAD (مقدّر قوي)
//
// لماذا التغيير من Mean+Sigma إلى Median+MAD؟
//   عندما تطير الطائرة بارتفاع منخفض (وضع الحلزون)، تقرأ الحساس
//   قيماً عالية جداً (2000-5000 nT) تدخل النافذة وترفع المتوسط
//   والانحراف المعياري بشكل كبير → عتبة مرتفعة تُخفي الألغام!
//
//   Median: لا يتأثر بالقيم الشاذة (outliers)
//   MAD = Median(|xi - Median|): مقاوم للتشويه أيضاً
//   معامل 1.4826: يجعل MAD متوافقاً مع σ للتوزيع الطبيعي
//
// مثال:
//   نافذة 50 قراءة: 30 خلفية@230nT + 20 لغم@3000nT
//   Mean+Sigma: mean=1340, sigma=1200, threshold=4940 nT ← كارثي
//   Median+MAD: median=230, MAD=0, threshold=230 nT     ← ممتاز!
// ============================================================
void MagnetometerSensor::recomputeThreshold()
{
    const int N = (int)window.size();

    // ── الخطوة 1: الوسيط (Median) ────────────────────────────
    std::vector<double> sorted(window.begin(), window.end());
    std::sort(sorted.begin(), sorted.end());

    double median;
    if (N % 2 == 0)
        median = (sorted[N/2 - 1] + sorted[N/2]) / 2.0;
    else
        median = sorted[N/2];

    windowMean = median;  // نحفظ الوسيط في windowMean للعرض

    // ── الخطوة 2: MAD (Median Absolute Deviation) ────────────
    std::vector<double> deviations;
    deviations.reserve(N);
    for (double v : window)
        deviations.push_back(std::abs(v - median));
    std::sort(deviations.begin(), deviations.end());

    double mad;
    if (N % 2 == 0)
        mad = (deviations[N/2 - 1] + deviations[N/2]) / 2.0;
    else
        mad = deviations[N/2];

    // 1.4826: معامل التوافق مع الانحراف المعياري للتوزيع الطبيعي
    // بدونه: MAD=σ/1.4826 → العتبة منخفضة جداً
    windowStdDev = mad * 1.4826;

    // ── الخطوة 3: العتبة = Median + k × MAD_scaled ───────────
    double computed = windowMean + kSigma * windowStdDev;
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
