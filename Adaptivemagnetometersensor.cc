#include "AdaptiveMagnetometerSensor.h"

namespace uavminedetection {
namespace {
// حدود حماية المعاملات لتثبيت سلوك الخوارزمية ومنع القيم المتطرفة.
constexpr double MIN_LOWER_PERCENTILE = 0.2;
constexpr double MAX_LOWER_PERCENTILE = 0.7;
constexpr double MIN_CLIP_PERCENTILE = 0.55;
constexpr double MAX_CLIP_PERCENTILE = 0.98;
constexpr double MIN_RECENT_WEIGHT = 0.01;
constexpr double MAX_RECENT_WEIGHT = 0.25;
constexpr double MIN_ZONE2_K_BOOST = 0.0;
constexpr double MAX_ZONE2_K_BOOST = 0.8;
constexpr double MIN_GUARD_BAND_NT = 5.0; // nT
constexpr double GUARD_BAND_RATIO = 0.03;
}

AdaptiveMagnetometerSensor::AdaptiveMagnetometerSensor(
    double kFactor,
    int    windowSize,
    double saturation,
    double initialThreshold,
    double lowerPercentile,
    double clipPercentile,
    double recentWeight,
    double zoneSplitThreshold,
    double zone2KBoost)
  : kFactor(kFactor),
    windowSize(windowSize),
    saturation(saturation),
    currentThreshold(initialThreshold),
    initialThreshold(initialThreshold),
    warmupSize(std::min(20, windowSize)),
    // سقف أمان مطلق فقط (90% من التشبع) — لا يُستخدم عادةً،
    // فقط حماية أخيرة من قيم غير منطقية.
    maxThreshold(saturation * 0.9),
    lowerPercentile(std::clamp(lowerPercentile, MIN_LOWER_PERCENTILE, MAX_LOWER_PERCENTILE)),
    clipPercentile(std::clamp(clipPercentile, MIN_CLIP_PERCENTILE, MAX_CLIP_PERCENTILE)),
    recentWeight(std::clamp(recentWeight, MIN_RECENT_WEIGHT, MAX_RECENT_WEIGHT)),
    zoneSplitThreshold(std::max(1.0, zoneSplitThreshold)),
    zone2KBoost(std::clamp(zone2KBoost, MIN_ZONE2_K_BOOST, MAX_ZONE2_K_BOOST))
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
double AdaptiveMagnetometerSensor::percentileFromSorted(const std::vector<double>& sorted, double p) const
{
    if (sorted.empty())
        return initialThreshold;
    p = std::clamp(p, 0.0, 1.0);
    double pos = p * (sorted.size() - 1);
    int lo = (int)std::floor(pos);
    int hi = (int)std::ceil(pos);
    if (lo == hi)
        return sorted[lo];
    double t = pos - lo;
    return sorted[lo] + t * (sorted[hi] - sorted[lo]);
}

void AdaptiveMagnetometerSensor::updateThreshold()
{
    ++thresholdUpdateCount;

    int fullN = (int)allReadingsWindow.size();
    int growthSpan = std::max(1, windowSize - warmupSize);
    // نزيد نافذة الخلفية تدريجياً حتى لا تقفز العتبة فجأة في بداية التشغيل.
    int effectiveWindow = std::min(
        fullN,
        warmupSize + std::min(growthSpan, thresholdUpdateCount / 2));

    std::vector<double> work;
    work.reserve(effectiveWindow);
    auto startIt = allReadingsWindow.end() - effectiveWindow;
    for (auto it = startIt; it != allReadingsWindow.end(); ++it)
        work.push_back(*it);

    std::vector<double> sorted(work.begin(), work.end());
    std::sort(sorted.begin(), sorted.end());

    int n = (int)work.size();
    if (n < 5) {
        currentThreshold = initialThreshold;
        return;
    }

    double backgroundCut = percentileFromSorted(sorted, lowerPercentile);
    double clipValue     = percentileFromSorted(sorted, clipPercentile);

    double wSum = 0.0;
    double weightedMean = 0.0;
    for (int i = 0; i < n; ++i) {
        double v = std::min(work[i], clipValue);
        if (v > backgroundCut)
            continue;
        double w = 1.0 + recentWeight * (i + 1);
        wSum += w;
        weightedMean += w * v;
    }
    if (wSum <= 0.0) {
        currentThreshold = std::min(initialThreshold, maxThreshold);
        return;
    }
    weightedMean /= wSum;

    double weightedVar = 0.0;
    for (int i = 0; i < n; ++i) {
        double v = std::min(work[i], clipValue);
        if (v > backgroundCut)
            continue;
        double w = 1.0 + recentWeight * (i + 1);
        double d = v - weightedMean;
        weightedVar += w * d * d;
    }
    double stdDev = std::sqrt(weightedVar / wSum);

    double zoneK = kFactor;
    if (weightedMean >= zoneSplitThreshold)
        zoneK *= (1.0 + zone2KBoost);

    double guardBand = std::max(MIN_GUARD_BAND_NT, GUARD_BAND_RATIO * weightedMean);
    double minThreshold = std::max(initialThreshold * 0.9, weightedMean + guardBand);
    double newThreshold = weightedMean + zoneK * stdDev + guardBand;

    currentThreshold = std::min(std::max(newThreshold, minThreshold), maxThreshold);
}

} // namespace uavminedetection
