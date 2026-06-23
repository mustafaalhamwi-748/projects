package uavminedetection;

import inet.applications.contract.IApp;

simple MineDetectionApp like IApp
{
    parameters:
        @class(uavminedetection::MineDetectionApp);

        int    uavId                         = default(0);

        double scanInterval        @unit(s)  = default(0.5s);

        double magneticThreshold             = default(250);
        double magneticSaturation            = default(2000);

        double confirmRadius       @unit(m)  = default(30m);
        double confirmationTimeout @unit(s)  = default(10s);

        double falseAlarmProb                = default(0.0001);
        int    falseAlarmDisplayLimit        = default(30);

        int    destPort                      = default(5555);
        int    localPort                     = default(5555);

        double sensorNoiseAlpha              = default(0.8);
        double sensorNoiseScale              = default(2.0);

        double returnHomeEnergyThreshold     = default(0.3);
        double energyPerMeter                = default(0.21);

        double simTimeLimit        @unit(s)  = default(600s);

        // ── [NEW] معاملات العتبة التكيفية ──────────────────────────
        // useAdaptiveThreshold: false = عتبة ثابتة (الوضع الأصلي)
        //                        true  = عتبة تكيفية (mean + k*sigma)
        bool   useAdaptiveThreshold          = default(false);

        // adaptiveK: عامل الانحراف المعياري (نطاق موصى به: 2.0 – 5.0)
        double adaptiveK                     = default(3.0);

        // adaptiveWindowSize: حجم نافذة قراءات الخلفية (موصى به: 30 – 100)
        int    adaptiveWindowSize            = default(50);

        // [NEW] adaptiveMinExcess: هامش ثقة إضافي (نانوتسلا) فوق العتبة
        // التكيفية الحالية، يُشترط لاعتبار القراءة "مرشح لغم محتمل".
        // يُستخدم فقط عندما useAdaptiveThreshold=true؛ لا تأثير له على
        // وضع العتبة الثابتة. الهدف: تصفية تجاوزات الشظايا المعدنية
        // الصغيرة الهامشية (قريبة من العتبة) دون التأثير على اكتشاف
        // الألغام الحقيقية (إشارتها أقوى بحوالي 10 مرات من إشارة
        // الشظايا عند نفس المسافة تقريباً، نتيجة كبر magneticConstant
        // مقارنة بـ debrisStrength في MineField — لذلك هامش أكبر هنا
        // لا يزال آمناً على معدل الاكتشاف).
        // [القياس التجريبي] على هذا المشروع:
        //   margin=0   → False Alarms Detected = 43  (بدون بوابة)
        //   margin=50  → False Alarms Detected = 38
        //   margin=70  → False Alarms Detected = 28  ← الأفضل المُثبت حتى الآن
        // في الحالات الثلاث: Mines Found = 26/26 (100%) بلا أي تأثير.
        // القيمة الافتراضية هنا مضبوطة على القيمة المُثبتة تجريبياً (70).
        double adaptiveMinExcess             = default(70);
        // ────────────────────────────────────────────────────────────

        @signal[mineDetected](type=long);
        @signal[areaCoverage](type=double);
        @signal[falseAlarm](type=long);
        @signal[correlatedNoise](type=double);

        @signal[timeToConfirm](type=simtime_t);
        @signal[uavsInvolved](type=long);
        @signal[confirmationConfidence](type=double);

        // [NEW] قيمة العتبة التكيفية عبر الزمن
        @signal[adaptiveThreshold](type=double);

        @statistic[mineDetected](title="Mine Detections"; record=count,sum,vector);
        @statistic[areaCoverage](title="Area Coverage %"; record=mean,max,vector);
        @statistic[falseAlarm](title="False Alarms"; record=count,sum);
        @statistic[correlatedNoise](title="Correlated Sensor Noise (nT)"; record=mean,min,max,vector);

        @statistic[timeToConfirm](title="Time to Confirm Target (s)"; record=mean,max,vector);
        @statistic[uavsInvolved](title="UAVs Involved in Confirmation"; record=histogram,mean,vector);
        @statistic[confirmationConfidence](title="Confidence at Confirmation"; record=histogram,mean,vector);

        // [NEW] تتبع تطور العتبة التكيفية عبر الزمن
        @statistic[adaptiveThreshold](title="Adaptive Threshold (nT)"; record=mean,min,max,vector);

        @display("i=block/app2");

    gates:
        input  socketIn  @labels(UdpControlInfo/up);
        output socketOut @labels(UdpControlInfo/down);
}
