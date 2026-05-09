package uavminedetection;

import inet.applications.contract.IApp;

// ============================================================
// MineDetectionApp — تطبيق الكشف بالحساس المغناطيسي
//
// [MODIFIED v2]: إضافة معاملات العتبة التكيفية
//   adaptiveWindowSize   : حجم النافذة المنزلقة (عدد القراءات)
//   adaptiveKSigma       : معامل الانحراف المعياري (k في: mean + k×sigma)
//   adaptiveMinThreshold : الحد الأدنى المطلق للعتبة (nT) — حماية أمان
//
// المنطق: الطائرة تتعلم خلفية منطقتها تلقائياً ولا تعتمد
//         على عتبة واحدة ثابتة لكل البيئات.
// ============================================================
simple MineDetectionApp like IApp
{
    parameters:
        @class(uavminedetection::MineDetectionApp);

        int    uavId                         = default(0);

        // ── معاملات المسح ──────────────────────────────────
        double scanInterval        @unit(s)  = default(0.5s);

        // ── معاملات الحساس المغناطيسي (الثابتة) ────────────
        // تُستخدم في مرحلة الإحماء قبل امتلاء النافذة التكيفية
        double magneticThreshold             = default(500);

        // قيمة التشبع: عندها confidence = 1.0 (nT)
        double magneticSaturation            = default(2000);

        // ── [NEW] معاملات العتبة التكيفية ──────────────────
        // حجم النافذة المنزلقة: عدد القراءات المحفوظة لحساب الخلفية
        // قيمة أكبر → تعلم أبطأ لكن أكثر استقراراً
        // قيمة أصغر → استجابة أسرع للتغيرات البيئية
        // عند scanInterval=0.5s: windowSize=50 يعني 25 ثانية من الذاكرة
        int    adaptiveWindowSize            = default(50);

        // معامل الانحراف المعياري: threshold = mean + k × stdDev
        // k=2.0 → حساسية أعلى (إنذارات كاذبة محتملة أكثر)
        // k=3.0 → توازن جيد بين الحساسية والدقة
        // k=4.0 → حذر شديد (قد يفوّت ألغام ضعيفة الإشارة)
        double adaptiveKSigma               = default(3.0);

        // الحد الأدنى المطلق للعتبة (nT)
        // يضمن أن العتبة لن تنزل إلى مستوى الضوضاء حتى في أنظف البيئات
        // يجب أن يكون أعلى من backgroundNoise (220 nT) بهامش معقول
        double adaptiveMinThreshold         = default(200.0);

        // نصف قطر تأكيد وجود لغم حقيقي (م)
        double confirmRadius       @unit(m)  = default(12m);

        // مهلة التأكيد — إذا لم تُؤكَّد الإشارة من طائرة ثانية خلال هذه المدة
        double confirmationTimeout @unit(s)  = default(30s);

        // احتمال ضوضاء مغناطيسية تتجاوز العتبة (معادن عشوائية)
        double falseAlarmProb                = default(0.003);

        // حد أقصى لعدد إشارات الإنذار الكاذب المعروضة على الشاشة
        int falseAlarmDisplayLimit           = default(50);

        // ── معاملات الشبكة ─────────────────────────────────
        int    destPort                      = default(5555);
        int    localPort                     = default(4000);

        // ── [NEW] معاملات الضوضاء المترابطة زمنياً — AR(1) ────
        // معامل التلاشي: يحدد مدى "ذاكرة" الضوضاء
        //   0.0 = ضوضاء بيضاء مستقلة تماماً (السلوك القديم)
        //   0.5 = ارتباط متوسط (الضوضاء تتجدد كل ~2 قراءات)
        //   0.8 = ارتباط عالٍ (الضوضاء تتجدد كل ~5 قراءات) ← الافتراضي
        //   0.95= ارتباط شديد (انجراف بطيء جداً)
        // المعادلة: η(t) = alpha·η(t-1) + (1−alpha)·Uniform(−scale, +scale)
        double sensorNoiseAlpha = default(0.8);

        // سعة الابتكار العشوائي (nT)
        // يُحدّد الحد الأقصى لتغيّر الانجراف في كل خطوة
        // مع alpha=0.8: الانجراف الثابت ≈ scale (في المتوسط الإحصائي)
        // القيمة 30 nT ≈ 14% من العتبة الثابتة (250 nT) — واقعي للحساسات المغناطيسية
        double sensorNoiseScale = default(30.0);

        // ── Signals ────────────────────────────────────────
        @signal[mineDetected](type=long);
        @signal[areaCoverage](type=double);
        @signal[falseAlarm](type=long);

        // [NEW]: إشارة تتبع العتبة التكيفية عبر الزمن
        @signal[adaptiveThreshold](type=double);

        // [NEW]: إشارة تتبع الضوضاء المترابطة (انجراف الحساس)
        @signal[correlatedNoise](type=double);

        @statistic[mineDetected](
            title="Mine Detections";
            record=count,sum,vector);
        @statistic[areaCoverage](
            title="Area Coverage %";
            record=mean,max,vector);
        @statistic[falseAlarm](
            title="False Alarms";
            record=count,sum);

        // [NEW]: إحصائية العتبة التكيفية — تُظهر تطورها عبر الزمن
        @statistic[adaptiveThreshold](
            title="Adaptive Threshold (nT)";
            record=mean,min,max,vector);

        // [NEW]: إحصائية انجراف الحساس — تُظهر سلوك الضوضاء المترابطة
        @statistic[correlatedNoise](
            title="Correlated Sensor Noise (nT)";
            record=mean,min,max,vector);

        @display("i=block/app2");

    gates:
        input  socketIn  @labels(UdpControlInfo/up);
        output socketOut @labels(UdpControlInfo/down);
}
