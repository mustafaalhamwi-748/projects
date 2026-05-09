#ifndef MAGNETOMETERSENSOR_H
#define MAGNETOMETERSENSOR_H

#include <deque>
#include <cmath>

namespace uavminedetection {

// ============================================================
// SensorReading — نتيجة قياس الحساس المغناطيسي
// [MODIFIED]: أُضيف effectiveThreshold لمعرفة أي عتبة استُخدمت
// ============================================================
struct SensorReading {
    double magneticValue;       // القيمة المقاسة (nT)
    bool   isMine;              // هل تجاوزت العتبة الفعّالة؟
    double confidence;          // 0.0 – 1.0 (مدى اليقين)
    double effectiveThreshold;  // [NEW]: العتبة المستخدمة في هذا القياس (ثابتة أو تكيفية)
};

// ============================================================
// MagnetometerSensor — حساس مغناطيسي بعتبة تكيفية
//
// [NEW] منطق العتبة التكيفية:
//   - تحتفظ النافذة المنزلقة بآخر windowSize قراءة
//   - العتبة التكيفية = mean(window) + kSigma × stdDev(window)
//   - لا تنزل العتبة عن minThreshold (حد أمان صلب)
//   - قبل امتلاء النافذة: تُستخدم العتبة الثابتة staticThreshold
//   - بعد امتلاء النافذة: تُستخدم العتبة التكيفية
//
// المنطق الفيزيائي:
//   الطائرة تتعلم الخلفية المغناطيسية للمنطقة التي تمسحها،
//   فتصبح العتبة مرتبطة بطبيعة التربة المحلية لا برقم ثابت عالمي.
//   منطقة طينية غنية بالمعادن → عتبة أعلى تلقائياً (إنذارات كاذبة أقل)
//   منطقة رملية فقيرة       → عتبة أدنى تلقائياً (حساسية أعلى)
// ============================================================
class MagnetometerSensor
{
  public:
    // staticThreshold : العتبة الثابتة (تُستخدم قبل امتلاء النافذة)
    // saturation      : القيمة التي عندها confidence = 1.0
    // windowSize      : حجم النافذة المنزلقة (عدد القراءات المحفوظة)
    // kSigma          : معامل الانحراف المعياري (threshold = mean + k×sigma)
    // minThreshold    : الحد الأدنى المطلق للعتبة (حماية من الضوضاء المنخفضة)
    MagnetometerSensor(double staticThreshold,
                       double saturation,
                       int    windowSize    = 50,
                       double kSigma        = 3.0,
                       double minThreshold  = 200.0);

    // ── [NEW]: إضافة قراءة للنافذة المنزلقة ──────────────────
    // يجب استدعاؤها في كل دورة مسح قبل measure()
    // تُحدّث العتبة التكيفية تلقائياً بعد امتلاء النافذة
    void addReading(double magneticValue);

    // ── القياس الرئيسي ────────────────────────────────────────
    // يستخدم العتبة التكيفية إذا كانت النافذة ممتلئة،
    // والعتبة الثابتة في مرحلة الإحماء (warmup)
    SensorReading measure(double magneticValue) const;

    // ── معلومات الحالة ────────────────────────────────────────
    double getThreshold()          const;  // العتبة الفعّالة الحالية
    double getStaticThreshold()    const { return staticThreshold; }
    double getAdaptiveThreshold()  const { return adaptiveThreshold; }
    double getSaturation()         const { return saturation; }
    double getWindowMean()         const { return windowMean; }
    double getWindowStdDev()       const { return windowStdDev; }
    bool   isAdaptiveReady()       const { return (int)window.size() >= windowSize; }
    int    getWindowFill()         const { return (int)window.size(); }
    int    getWindowSize()         const { return windowSize; }

  private:
    double staticThreshold;
    double saturation;
    int    windowSize;
    double kSigma;
    double minThreshold;

    // النافذة المنزلقة
    std::deque<double> window;

    // قيم محسوبة من النافذة (تُحدَّث عند كل addReading بعد امتلاء النافذة)
    double adaptiveThreshold = 0.0;
    double windowMean        = 0.0;
    double windowStdDev      = 0.0;

    // إعادة حساب العتبة التكيفية من محتوى النافذة الحالي
    void recomputeThreshold();
};

} // namespace uavminedetection
#endif
