#include "AdaptiveMagnetometerSensor.h"

namespace uavminedetection {

AdaptiveMagnetometerSensor::AdaptiveMagnetometerSensor(
    double kFactor,
    int    windowSize,
    double saturation,
    double initialThreshold)
  : kFactor(kFactor),
    windowSize(windowSize),
    saturation(saturation),
    currentThreshold(initialThreshold),
    initialThreshold(initialThreshold),
    warmupSize(std::min(20, windowSize)),
    // سقف أمان مطلق فقط (90% من التشبع) — لا يُستخدم عادةً،
    // فقط حماية أخيرة من قيم غير منطقية.
    maxThreshold(saturation * 0.9)
{}

// ============================================================
// measure
//
// updateWindow=false أثناء Spiral/Intensive (نفس المنطق السابق).
// ============================================================
SensorReading AdaptiveMagnetometerSensor::measure(double magneticValue, bool updateWindow)
{
    if (updateWindow) {
        allReadingsWindow.push_back(magneticValue);
        if ((int)allReadingsWindow.size() > windowSize)
            allReadingsWindow.pop_front();

        if ((int)allReadingsWindow.size() >= warmupSize)
            updateThreshold();
    }

    SensorReading r;
    r.magneticValue = magneticValue;
    r.isMine        = (magneticValue >= currentThreshold);

    if (r.isMine) {
        double range  = saturation - currentThreshold;
        double excess = magneticValue - currentThreshold;
        r.confidence  = (range > 0.0) ? std::min(1.0, excess / range) : 1.0;
    } else {
        r.confidence = 0.0;
    }

    return r;
}

// ============================================================
// updateThreshold — Lower-Half Statistics
//
// المشكلة مع Median(كل النافذة)+k×MAD(كل النافذة):
//   في حقل كثيف، حتى Median قد يكون مرتفعاً لأن أكثر من نصف
//   القراءات قريبة من أجسام معدنية/ألغام.
//
// الحل: نأخذ النصف الأدنى فقط من القراءات المرتبة (lower half).
//   هذه القراءات تمثل "أبعد نقاط الطائرة عن أي هدف" ضمن
//   النافذة الحالية — وهي أقرب تقدير ممكن لـ "الخلفية الحقيقية"
//   لتلك المنطقة، بغض النظر عن كثافة الحقل الكلية.
//
//   العتبة = mean(lowerHalf) + k × std(lowerHalf)
//
// هذا يسمح للعتبة بالتمايز محلياً: في منطقة هادئة سيكون
// lowerHalf قريباً من 200nT، وفي منطقة صاخبة سيكون أعلى —
// لكن دائماً أقل من قراءات الأهداف نفسها.
// ============================================================
void AdaptiveMagnetometerSensor::updateThreshold()
{
    std::vector<double> sorted(allReadingsWindow.begin(),
                                allReadingsWindow.end());
    std::sort(sorted.begin(), sorted.end());

    int n = (int)sorted.size();
    int halfN = std::max(5, n / 2);   // النصف الأدنى (حد أدنى 5 قيم)

    double sum = 0.0;
    for (int i = 0; i < halfN; i++) sum += sorted[i];
    double mean = sum / halfN;

    double sumSq = 0.0;
    for (int i = 0; i < halfN; i++) sumSq += (sorted[i]-mean)*(sorted[i]-mean);
    double stdDev = std::sqrt(sumSq / halfN);

    double newThreshold = mean + kFactor * stdDev;
    // سقف أمان مطلق فقط
    currentThreshold = std::min(newThreshold, maxThreshold);
}

} // namespace uavminedetection
