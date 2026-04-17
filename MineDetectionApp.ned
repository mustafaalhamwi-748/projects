package uavminedetection;

import inet.applications.contract.IApp;

// ============================================================
// MineDetectionApp — تطبيق الكشف بالحساس المغناطيسي
//
// استُبدل: cameraRange, detectionProb, cnnConfidenceThreshold
// بـ:      magneticThreshold, magneticSaturation, confirmRadius
// ============================================================
simple MineDetectionApp like IApp
{
    parameters:
        @class(uavminedetection::MineDetectionApp);

        int    uavId                         = default(0);

        // ── معاملات المسح ──────────────────────────────────
        double scanInterval        @unit(s)  = default(0.5s);

        // ── معاملات الحساس المغناطيسي ──────────────────────
        // العتبة: ما فوقها يُعتبر لغماً (nT)
        // الخلفية الطبيعية ≈ 220 nT → العتبة = 500 nT
        double magneticThreshold             = default(500);

        // قيمة التشبع: عندها confidence = 1.0 (nT)
        double magneticSaturation            = default(2000);

        // نصف قطر تأكيد وجود لغم حقيقي (م)
        // الطائرة تبحث عن لغم ضمن هذا النطاق عند قراءة عالية
        double confirmRadius       @unit(m)  = default(12m);

        // احتمال ضوضاء مغناطيسية تتجاوز العتبة (معادن عشوائية)
        double falseAlarmProb                = default(0.003);

        // ── معاملات الشبكة ─────────────────────────────────
        int    destPort                      = default(5555);
        int    localPort                     = default(4000);

        // ── Signals ────────────────────────────────────────
        @signal[mineDetected](type=long);
        @signal[areaCoverage](type=double);
        @signal[falseAlarm](type=long);

        @statistic[mineDetected](
            title="Mine Detections";
            record=count,sum,vector);
        @statistic[areaCoverage](
            title="Area Coverage %";
            record=mean,max,vector);
        @statistic[falseAlarm](
            title="False Alarms";
            record=count,sum);

        @display("i=block/app2");

    gates:
        input  socketIn  @labels(UdpControlInfo/up);
        output socketOut @labels(UdpControlInfo/down);
}
