#include "LawnmowerMobility.h"
#include "MineDetectionApp.h"
#include "inet/common/ModuleAccess.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/TagBase_m.h"
#include "inet/mobility/base/MovingMobilityBase.h"

#include <cmath>
#include <string>
#include <algorithm>

using namespace inet;

namespace uavminedetection {

Define_Module(MineDetectionApp);

simsignal_t MineDetectionApp::detectionSignal      = registerSignal("mineDetected");
simsignal_t MineDetectionApp::coverageSignal       = registerSignal("areaCoverage");
simsignal_t MineDetectionApp::falseAlarmSignal     = registerSignal("falseAlarm");
simsignal_t MineDetectionApp::adaptiveThreshSignal = registerSignal("adaptiveThreshold"); // [NEW]
simsignal_t MineDetectionApp::correlatedNoiseSignal= registerSignal("correlatedNoise");   // [NEW]

// ============================================================
// initialize
// [MODIFIED]: قراءة معاملات العتبة التكيفية وتمريرها للحساس
// ============================================================
void MineDetectionApp::initialize(int stage)
{
    ApplicationBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        uavId              = par("uavId");
        scanInterval       = par("scanInterval");
        magneticThreshold  = par("magneticThreshold");
        magneticSaturation = par("magneticSaturation");
        falseAlarmProb     = par("falseAlarmProb");
        falseAlarmDisplayLimit = par("falseAlarmDisplayLimit");
        confirmRadius          = par("confirmRadius");
        confirmationTimeout    = par("confirmationTimeout");
        destPort           = par("destPort");
        localPort          = par("localPort");

        // ── [NEW]: قراءة معاملات العتبة التكيفية ──────────────
        adaptiveWindowSize   = par("adaptiveWindowSize");
        adaptiveKSigma       = par("adaptiveKSigma");
        adaptiveMinThreshold = par("adaptiveMinThreshold");

        // ── [NEW]: قراءة معاملات الضوضاء المترابطة زمنياً ─────
        sensorNoiseAlpha = par("sensorNoiseAlpha");
        sensorNoiseScale = par("sensorNoiseScale");
        correlatedNoise  = 0.0;  // تبدأ الضوضاء من الصفر عند بدء المحاكاة

        // ── [MODIFIED]: إنشاء الحساس بالمعاملات التكيفية ──────
        // الآن المنشئ يأخذ 5 معاملات بدلاً من 2
        sensor = new MagnetometerSensor(
            magneticThreshold,    // العتبة الثابتة (مرحلة الإحماء)
            magneticSaturation,   // قيمة التشبع
            adaptiveWindowSize,   // [NEW] حجم النافذة
            adaptiveKSigma,       // [NEW] معامل الانحراف
            adaptiveMinThreshold  // [NEW] الحد الأدنى الآمن
        );

        WATCH(trueDetections);
        WATCH(falseAlarms);
        WATCH(duplicatesSkipped);
        WATCH(messagesSent);
        WATCH(lastMagneticValue);
        WATCH(isIntensiveMode);
        WATCH(isReturningHome);
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        mineField = check_and_cast<MineField*>(
            getSystemModule()->getSubmodule("mineField"));
        mobility = check_and_cast<IMobility*>(
            getParentModule()->getSubmodule("mobility"));

        socket.setOutputGate(gate("socketOut"));
        socket.bind(localPort);
        socket.setCallback(this);

        try {
            L3AddressResolver().tryResolve("gcs", gcsAddress);
            if (gcsAddress.isUnspecified())
                EV_WARN << "UAV[" << uavId << "] Could not resolve GCS address!\n";
            else
                EV_INFO << "UAV[" << uavId << "] GCS address resolved: "
                        << gcsAddress.str() << "\n";
        } catch (...) {
            EV_WARN << "UAV[" << uavId << "] GCS address resolution failed.\n";
        }

        initSensorVisuals();

        scanTimer       = new cMessage("scanTimer");
        intensiveTimer  = new cMessage("intensiveTimer");
        returnHomeTimer = new cMessage("returnHomeTimer");

        scheduleAt(simTime() + uniform(0, scanInterval), scanTimer);

        double returnHomeAt = 600.0 - RETURN_HOME_BEFORE;
        scheduleAt(SimTime(returnHomeAt), returnHomeTimer);
        EV_INFO << "UAV[" << uavId << "] Adaptive threshold ENABLED: "
                << "window=" << adaptiveWindowSize
                << " k=" << adaptiveKSigma
                << " minThr=" << adaptiveMinThreshold << " nT\n";
        EV_INFO << "UAV[" << uavId << "] Return-home scheduled at t="
                << returnHomeAt << "s\n";
    }
}

// ============================================================
// setFlightAltitude
// ============================================================
void MineDetectionApp::setFlightAltitude(double altMeters)
{
    auto *lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
    if (lawnmower) lawnmower->setAltitude(altMeters);
}

// ============================================================
// initiateReturnHome
// ============================================================
void MineDetectionApp::initiateReturnHome()
{
    if (isReturningHome) return;

    isReturningHome = true;

    if (scanTimer && scanTimer->isScheduled())       cancelEvent(scanTimer);
    if (intensiveTimer && intensiveTimer->isScheduled()) cancelEvent(intensiveTimer);

    if (isIntensiveMode) {
        isIntensiveMode = false;
        auto *lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
        if (lawnmower) lawnmower->stopSpiral();
    }

    setFlightAltitude(CRUISE_ALTITUDE);

    auto *lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
    if (lawnmower)
        lawnmower->goHome(GCS_X, GCS_Y, CRUISE_ALTITUDE);

    EV_INFO << "UAV[" << uavId << "] Returning to GCS at ("
            << GCS_X << ", " << GCS_Y << ") -- t=" << simTime() << "s\n";
}

// ============================================================
// initSensorVisuals
// [MODIFIED]: إضافة عنصر بصري لعرض العتبة التكيفية الحالية
// ============================================================
void MineDetectionApp::initSensorVisuals()
{
    cCanvas *canvas = getSystemModule()->getCanvas();
    Coord pos = mobility->getCurrentPosition();
    double cx  = pos.x;
    double cy  = pos.y;

    std::string ringName = "magRing_" + std::to_string(uavId);
    sensorRingFigure = new cOvalFigure(ringName.c_str());
    double R = 14.0;
    sensorRingFigure->setBounds(cFigure::Rectangle(cx-R, cy-R, 2*R, 2*R));
    sensorRingFigure->setFilled(true);
    sensorRingFigure->setFillColor(cFigure::Color(0, 100, 255));
    sensorRingFigure->setFillOpacity(0.18);
    sensorRingFigure->setLineColor(cFigure::Color(0, 100, 255));
    sensorRingFigure->setLineWidth(2);
    sensorRingFigure->setLineOpacity(0.7);
    canvas->addFigure(sensorRingFigure);

    std::string valName = "magVal_" + std::to_string(uavId);
    sensorValueFigure = new cTextFigure(valName.c_str());
    sensorValueFigure->setPosition(cFigure::Point(cx, cy - 18));
    sensorValueFigure->setText("--- nT");
    sensorValueFigure->setColor(cFigure::Color(0, 60, 180));
    sensorValueFigure->setAnchor(cFigure::ANCHOR_S);
    sensorValueFigure->setFont(cFigure::Font("", 7, cFigure::FONT_BOLD));
    canvas->addFigure(sensorValueFigure);

    // ── [NEW]: عنصر بصري لعرض العتبة التكيفية ────────────────
    // يظهر تحت الطائرة: "Thr:250" (ثابتة) أو "AThr:320" (تكيفية)
    std::string thrName = "adaptiveThr_" + std::to_string(uavId);
    adaptiveThreshFigure = new cTextFigure(thrName.c_str());
    adaptiveThreshFigure->setPosition(cFigure::Point(cx, cy + 22));
    adaptiveThreshFigure->setText("Thr:---");
    adaptiveThreshFigure->setColor(cFigure::Color(100, 100, 100));
    adaptiveThreshFigure->setAnchor(cFigure::ANCHOR_N);
    adaptiveThreshFigure->setFont(cFigure::Font("", 6, 0));
    canvas->addFigure(adaptiveThreshFigure);

    std::string barBgName = "magBarBg_" + std::to_string(uavId);
    sensorBarBg = new cRectangleFigure(barBgName.c_str());
    sensorBarBg->setBounds(cFigure::Rectangle(cx-12, cy+14, 24, 4));
    sensorBarBg->setFilled(true);
    sensorBarBg->setFillColor(cFigure::Color(200, 200, 200));
    sensorBarBg->setLineColor(cFigure::Color(150, 150, 150));
    sensorBarBg->setLineWidth(1);
    canvas->addFigure(sensorBarBg);

    std::string barFgName = "magBarFg_" + std::to_string(uavId);
    sensorBarFg = new cRectangleFigure(barFgName.c_str());
    sensorBarFg->setBounds(cFigure::Rectangle(cx-12, cy+14, 2, 4));
    sensorBarFg->setFilled(true);
    sensorBarFg->setFillColor(cFigure::Color(0, 100, 255));
    sensorBarFg->setLineWidth(1);
    canvas->addFigure(sensorBarFg);
}

// ============================================================
// getMagneticColor
// [MODIFIED]: تأخذ العتبة الفعّالة كمعامل بدلاً من الثابتة
// ============================================================
cFigure::Color MineDetectionApp::getMagneticColor(double magVal, double threshold) const
{
    double ratio = magVal / threshold;
    if (ratio < 0.8)       return cFigure::Color(0, 100, 255);
    else if (ratio < 1.0)  return cFigure::Color(255, 200, 0);
    else if (ratio < 1.5)  return cFigure::Color(255, 120, 0);
    else                   return cFigure::Color(220, 0, 0);
}

// ============================================================
// getMagneticRadius
// [MODIFIED]: تأخذ العتبة الفعّالة كمعامل بدلاً من الثابتة
// ============================================================
double MineDetectionApp::getMagneticRadius(double magVal, double threshold) const
{
    double ratio = magVal / threshold;
    return 10.0 + std::min(18.0, ratio * 12.0);
}

// ============================================================
// updateSensorVisuals
// [MODIFIED]:
//   1. استخدام sensor->getThreshold() للعتبة الفعّالة الحالية
//   2. تحديث adaptiveThreshFigure لعرض نوع العتبة وقيمتها
// ============================================================
void MineDetectionApp::updateSensorVisuals(double magVal,
                                           double uavX, double uavY) const
{
    if (!sensorRingFigure || !sensorValueFigure) return;

    // ── [MODIFIED]: استخدام العتبة الفعّالة من الحساس ──────────
    double effectiveThr = sensor->getThreshold();
    cFigure::Color color = getMagneticColor(magVal, effectiveThr);
    double R = getMagneticRadius(magVal, effectiveThr);

    sensorRingFigure->setBounds(cFigure::Rectangle(uavX-R, uavY-R, 2*R, 2*R));

    if (isReturningHome) {
        sensorRingFigure->setFillColor(cFigure::Color(150, 150, 150));
        sensorRingFigure->setLineColor(cFigure::Color(100, 100, 100));
        sensorRingFigure->setLineWidth(2);
        sensorRingFigure->setFillOpacity(0.2);
        sensorValueFigure->setText("RTH");
        sensorValueFigure->setColor(cFigure::Color(100, 100, 100));
    }
    else if (isIntensiveMode) {
        sensorRingFigure->setFillColor(cFigure::Color(255, 0, 0));
        sensorRingFigure->setLineColor(cFigure::Color(255, 0, 0));
        sensorRingFigure->setLineWidth(4);
        sensorRingFigure->setFillOpacity(0.5);
        char buf[64];
        snprintf(buf, sizeof(buf), "ALERT %.0fnT", magVal);
        sensorValueFigure->setText(buf);
        sensorValueFigure->setColor(cFigure::Color(255, 0, 0));
    }
    else {
        sensorRingFigure->setFillColor(color);
        sensorRingFigure->setLineColor(color);
        sensorRingFigure->setLineWidth(2);
        sensorRingFigure->setFillOpacity((magVal >= effectiveThr) ? 0.35 : 0.18);
        char buf[32];
        if (magVal >= effectiveThr)
            snprintf(buf, sizeof(buf), "! %.0f nT", magVal);
        else
            snprintf(buf, sizeof(buf), "%.0f nT", magVal);
        sensorValueFigure->setText(buf);
        sensorValueFigure->setColor(color);
    }

    sensorValueFigure->setPosition(cFigure::Point(uavX, uavY - 18));

    // ── [NEW]: تحديث عرض العتبة التكيفية ──────────────────────
    if (adaptiveThreshFigure) {
        char thrBuf[40];
        if (sensor->isAdaptiveReady()) {
            // النافذة ممتلئة: عرض العتبة التكيفية بلون أخضر داكن
            snprintf(thrBuf, sizeof(thrBuf), "A:%.0fnT", effectiveThr);
            adaptiveThreshFigure->setColor(cFigure::Color(0, 130, 0));
        } else {
            // مرحلة الإحماء: عرض تقدم ملء النافذة بلون رمادي
            snprintf(thrBuf, sizeof(thrBuf), "W:%d/%d",
                     sensor->getWindowFill(), sensor->getWindowSize());
            adaptiveThreshFigure->setColor(cFigure::Color(120, 120, 120));
        }
        adaptiveThreshFigure->setText(thrBuf);
        adaptiveThreshFigure->setPosition(cFigure::Point(uavX, uavY + 22));
    }

    // ── شريط التعبئة ──────────────────────────────────────────
    if (sensorBarBg && sensorBarFg) {
        sensorBarBg->setBounds(cFigure::Rectangle(uavX-12, uavY+14, 24, 4));
        // [MODIFIED]: نسبة التعبئة تعتمد على العتبة الفعّالة
        double fillRatio = std::min(1.0, magVal / (1.5 * effectiveThr));
        sensorBarFg->setBounds(cFigure::Rectangle(
            uavX-12, uavY+14, std::max(1.0, fillRatio * 24.0), 4));
        if (isReturningHome)
            sensorBarFg->setFillColor(cFigure::Color(150, 150, 150));
        else
            sensorBarFg->setFillColor(
                isIntensiveMode ? cFigure::Color(255,0,0) : color);
    }
}

// ============================================================
// checkTimeouts
// ============================================================
void MineDetectionApp::checkTimeouts()
{
    auto it = candidateMines.begin();
    while (it != candidateMines.end()) {
        if (it->firstDetectorId == uavId &&
            (simTime() - it->firstDetectedTime).dbl() > confirmationTimeout)
        {
            EV_INFO << "UAV[" << uavId << "] TIMEOUT! Auto-confirming at ("
                    << it->pos.x << "," << it->pos.y << ").\n";
            confirmTarget(it->pos, it->confidence, it->magVal);
            it = candidateMines.erase(it);
        } else {
            ++it;
        }
    }
}

// ============================================================
// confirmTarget
// ============================================================
void MineDetectionApp::confirmTarget(inet::Coord targetPos,
                                     double confidence, double magVal)
{
    sharedMemory.push_back(targetPos);

    int mineIdx = mineField->getNearestUndiscoveredMine(
        targetPos.x, targetPos.y, confirmRadius);

    if (mineIdx >= 0) {
        mineField->markDiscovered(mineIdx);
        trueDetections++;
        emit(detectionSignal, 1L);
        EV_INFO << "UAV[" << uavId << "] MINE CONFIRMED at ("
                << targetPos.x << "," << targetPos.y << ")\n";
        sendNetworkMessage("CONFIRMED_REAL", targetPos.x, targetPos.y,
                           confidence, magVal);
    } else {
        int debrisIdx = mineField->getNearestMetalDebris(
            targetPos.x, targetPos.y, confirmRadius);
        if (debrisIdx >= 0)
            mineField->markDebrisTriggered(debrisIdx);

        falseAlarms++;
        emit(falseAlarmSignal, 1L);

        if (debrisIdx >= 0) {
            const auto& db = mineField->getDebris();
            addFalseAlarmFigure(db[debrisIdx].x, db[debrisIdx].y);
        }
        sendNetworkMessage("CONFIRMED_FA", targetPos.x, targetPos.y,
                           confidence, magVal);
    }
}

// ============================================================
// performScan — قلب عملية الكشف
//
// [MODIFIED v3]: ثلاث خطوات رئيسية الآن:
//   1. rawMagVal ← قراءة الإشارة الخام من MineField
//   2. correlatedNoise ← تحديث الضوضاء المترابطة AR(1)  ← [NEW]
//   3. sensor->addReading(magVal) ← تغذية النافذة التكيفية
//   4. emit(adaptiveThreshSignal) ← تسجيل العتبة
//
// الترتيب مهم:
//   أولاً: اقرأ الخام ← قبل أي معالجة
//   ثانياً: طبّق ضوضاء الحساس ← القيمة التي "يراها" الحساس فعلاً
//   ثالثاً: غذّ النافذة بالقيمة المشوّشة ← العتبة تتكيف مع الواقع
//   رابعاً: اقرر بالعتبة الفعّالة المُحدَّثة
// ============================================================
void MineDetectionApp::performScan()
{
    if (isReturningHome) return;

    checkTimeouts();
    Coord pos = mobility->getCurrentPosition();

    for (const auto& knownPos : sharedMemory) {
        if (pos.distance(knownPos) < confirmRadius) {
            duplicatesSkipped++;
            scheduleAt(simTime() + scanInterval, scanTimer);
            return;
        }
    }

    // ── الخطوة 1: قراءة المجال المغناطيسي الخام من MineField ───
    // القيمة الخام = إشارة الألغام + ضوضاء التربة البيضاء
    double rawMagVal = mineField->getMagneticValue(pos.x, pos.y, pos.z);

    // ── [NEW] الخطوة 2: الضوضاء المترابطة زمنياً — نموذج AR(1) ─
    //
    // الفيزياء المُمثَّلة: انجراف الحساس الإلكتروني (sensor drift)
    //   - اهتزاز الطائرة يُربك الحساس باستمرار
    //   - تغيّر درجة الحرارة يُسبّب انجرافاً تدريجياً
    //   - تشويش محركات الطائرة يتغيّر ببطء
    //   كل هذه العوامل تجعل ضوضاء الحساس مترابطة زمنياً، لا مستقلة.
    //
    // المعادلة (سطر واحد):
    //   η(t) = α·η(t-1) + (1−α)·w(t)
    //   حيث w(t) ~ Uniform(-scale, +scale) هي "الابتكار العشوائي"
    //
    // التفسير المادي:
    //   α=0.8: الانجراف الحالي = 80% من السابق + 20% من الاضطراب الجديد
    //   هذا يعني الانجراف يتغيّر بالكامل تقريباً كل 5 قراءات (~2.5 ثانية)
    //   مقارنةً بضوضاء بيضاء تتغيّر كلياً كل قراءة (0.5 ثانية)
    //
    // التأثير على الكشف:
    //   ضوضاء بيضاء: قراءات مرتفعة عشوائية مستقلة → إنذارات كاذبة منفردة
    //   ضوضاء مترابطة: قراءات مرتفعة متتالية → "موجة" من الإنذارات الكاذبة
    //   هذا أكثر واقعية ويُصعّب قرار التأكيد
    double innovation  = uniform(-sensorNoiseScale, sensorNoiseScale);
    correlatedNoise    = sensorNoiseAlpha * correlatedNoise
                       + (1.0 - sensorNoiseAlpha) * innovation;

    // القيمة النهائية التي "يراها" الحساس
    // = إشارة حقيقية + ضوضاء تربة (بيضاء) + انجراف حساس (مترابط)
    double magVal = rawMagVal + correlatedNoise;
    lastMagneticValue = magVal;

    // تسجيل قيمة الانجراف للإحصاءات (للرسوم البيانية في IDE)
    emit(correlatedNoiseSignal, correlatedNoise);

    // ── [NEW] الخطوة 2: إضافة القراءة للنافذة المنزلقة ────────
    // هذا يُحدّث العتبة التكيفية تلقائياً بعد امتلاء النافذة.
    // الترتيب مقصود: نُغذّي النافذة أولاً ثم نقيس،
    // لأن القراءة الحالية جزء من بيئة الطائرة الراهنة.
    sensor->addReading(magVal);

    // ── [NEW] الخطوة 3: تسجيل العتبة الحالية كإحصائية ─────────
    // يُمكّن رسم تطور العتبة عبر الزمن في OMNeT++ IDE
    emit(adaptiveThreshSignal, sensor->getThreshold());

    // سجّل في EV عند التحول من الثابتة إلى التكيفية
    if (sensor->getWindowFill() == sensor->getWindowSize()) {
        static bool adaptiveLoggedOnce[10] = {false};
        if (uavId >= 0 && uavId < 10 && !adaptiveLoggedOnce[uavId]) {
            adaptiveLoggedOnce[uavId] = true;
            EV_INFO << "UAV[" << uavId << "] >>> ADAPTIVE THRESHOLD ACTIVE: "
                    << "mean=" << sensor->getWindowMean()
                    << " sigma=" << sensor->getWindowStdDev()
                    << " thr=" << sensor->getAdaptiveThreshold() << " nT <<<\n";
        }
    }

    // ── الخطوة 4: تحديث العناصر البصرية ─────────────────────
    updateSensorVisuals(magVal, pos.x, pos.y);

    // ── الخطوة 5: قرار الكشف بالعتبة الفعّالة ──────────────
    SensorReading reading = sensor->measure(magVal);

    if (reading.isMine) {
        bool isAlreadyCandidate = false;
        for (auto it = candidateMines.begin(); it != candidateMines.end(); ++it) {
            if (pos.distance(it->pos) < confirmRadius) {
                isAlreadyCandidate = true;
                if (it->firstDetectorId != uavId) {
                    EV_INFO << "UAV[" << uavId << "] CROSS-VALIDATED from UAV["
                            << it->firstDetectorId << "] — thr="
                            << reading.effectiveThreshold << "nT\n";
                    confirmTarget(it->pos, reading.confidence, reading.magneticValue);
                    candidateMines.erase(it);
                } else {
                    duplicatesSkipped++;
                }
                break;
            }
        }

        if (!isAlreadyCandidate) {
            EV_INFO << "UAV[" << uavId << "] NEW CANDIDATE at ("
                    << pos.x << "," << pos.y << ") magVal=" << magVal
                    << " thr=" << reading.effectiveThreshold
                    << (sensor->isAdaptiveReady() ? " [ADAPTIVE]" : " [STATIC]") << "\n";

            CandidateMine newCand = {pos, simTime(), uavId,
                                     reading.confidence, reading.magneticValue};
            candidateMines.push_back(newCand);
            sendNetworkMessage("CANDIDATE", pos.x, pos.y,
                               reading.confidence, reading.magneticValue);
        }
    }

    if (fmod(simTime().dbl(), 10.0) < scanInterval)
        emit(coverageSignal, calculateCoverage());

    scheduleAt(simTime() + scanInterval, scanTimer);
}

// ============================================================
// refreshDisplay
// [MODIFIED]: عرض معلومات العتبة التكيفية في نص الطائرة
// ============================================================
void MineDetectionApp::refreshDisplay() const
{
    if (mobility && sensorRingFigure) {
        Coord pos = mobility->getCurrentPosition();
        updateSensorVisuals(lastMagneticValue, pos.x, pos.y);
    }

    char buf[160];
    if (isReturningHome) {
        snprintf(buf, sizeof(buf),
                 "UAV%d | RTH→GCS | found=%d | FA=%d",
                 uavId, trueDetections, falseAlarms);
    }
    else if (sensor->isAdaptiveReady()) {
        // [NEW]: عرض العتبة التكيفية + الانجراف الحالي للحساس
        snprintf(buf, sizeof(buf),
                 "UAV%d | %.0fnT | Thr:%.0f(A) | η:%.0f | found=%d FA=%d | P=%zu",
                 uavId, lastMagneticValue,
                 sensor->getAdaptiveThreshold(),
                 correlatedNoise,
                 trueDetections, falseAlarms, candidateMines.size());
    }
    else {
        // مرحلة الإحماء: عرض تقدم ملء النافذة + الانجراف
        snprintf(buf, sizeof(buf),
                 "UAV%d | %.0fnT | Thr:%.0f(S) W:%d/%d | η:%.0f | found=%d FA=%d",
                 uavId, lastMagneticValue,
                 sensor->getStaticThreshold(),
                 sensor->getWindowFill(), sensor->getWindowSize(),
                 correlatedNoise,
                 trueDetections, falseAlarms);
    }
    getParentModule()->getDisplayString().setTagArg("t", 0, buf);
}

// ============================================================
// sendNetworkMessage
// ============================================================
void MineDetectionApp::sendNetworkMessage(const char* type,
                                          double x, double y,
                                          double confidence,
                                          double magneticValue)
{
    char buf[160];
    snprintf(buf, sizeof(buf),
             "%s:uav=%d,x=%.1f,y=%.1f,conf=%.2f,magVal=%.1f,t=%.2f",
             type, uavId, x, y, confidence, magneticValue, simTime().dbl());

    if (!gcsAddress.isUnspecified()) {
        auto payload1 = makeShared<BytesChunk>();
        std::vector<uint8_t> b1(buf, buf + strlen(buf));
        payload1->setBytes(b1);
        auto *pkt1 = new Packet("MineReport_GCS", payload1);
        socket.sendTo(pkt1, gcsAddress, destPort);
    }

    cModule *net = getSystemModule();
    int nUAV = net->par("numUAVs");
    for (int i = 0; i < nUAV; i++) {
        if (i == uavId) continue;
        try {
            std::string uavName = "uav[" + std::to_string(i) + "]";
            L3Address addr;
            L3AddressResolver().tryResolve(uavName.c_str(), addr);
            if (!addr.isUnspecified()) {
                auto payloadU = makeShared<BytesChunk>();
                std::vector<uint8_t> bU(buf, buf + strlen(buf));
                payloadU->setBytes(bU);
                auto *pktU = new Packet("MineReport_UAV", payloadU);
                socket.sendTo(pktU, addr, destPort);
            }
        } catch (...) {}
    }

    messagesSent++;
}

// ============================================================
// calculateCoverage — عداد تراكمي
// ============================================================
double MineDetectionApp::calculateCoverage()
{
    const int    G = 50;
    const double S = 1000.0 / G;

    Coord pos = mobility->getCurrentPosition();
    int col = (int)(pos.x / S);
    int row = (int)(pos.y / S);
    col = std::max(0, std::min(G - 1, col));
    row = std::max(0, std::min(G - 1, row));
    visitedCells.insert(col * G + row);
    return (double)visitedCells.size() / (double)(G * G) * 100.0;
}

// ============================================================
// addFalseAlarmFigure
// ============================================================
void MineDetectionApp::addFalseAlarmFigure(double x, double y)
{
    if (falseAlarmDisplayLimit >= 0 &&
        falseAlarmFigureCount >= falseAlarmDisplayLimit) return;

    cCanvas *canvas = getSystemModule()->getCanvas();
    std::string name = "fa_" + std::to_string(uavId) + "_"
                     + std::to_string(falseAlarmFigureCount++);
    auto *grp = new cGroupFigure(name.c_str());

    auto *tri = new cPolygonFigure("tri");
    tri->setPoints({{x, y-9}, {x-8, y+6}, {x+8, y+6}});
    tri->setFilled(true);
    tri->setFillColor(cFigure::Color(255, 220, 0));
    grp->addFigure(tri);

    auto *lbl = new cTextFigure("lbl");
    lbl->setPosition(cFigure::Point(x, y+2));
    lbl->setText("!");
    lbl->setColor(cFigure::Color("black"));
    lbl->setAnchor(cFigure::ANCHOR_CENTER);
    grp->addFigure(lbl);

    canvas->addFigure(grp);
}

// ============================================================
// startIntensiveSearch
// ============================================================
void MineDetectionApp::startIntensiveSearch(double cmdX, double cmdY)
{
    if (isReturningHome) return;

    Coord pos = mobility->getCurrentPosition();
    if (pos.distance(Coord(cmdX, cmdY, 0)) < 200.0) {
        isIntensiveMode = true;
        EV_INFO << "UAV[" << uavId << "] Entering SPIRAL mode at ("
                << cmdX << "," << cmdY << ") — descending to "
                << SCAN_ALTITUDE << "m\n";

        auto lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
        if (lawnmower)
            lawnmower->startSpiral(cmdX, cmdY);

        setFlightAltitude(SCAN_ALTITUDE);

        cancelEvent(intensiveTimer);
        scheduleAt(simTime() + 20.0, intensiveTimer);
    }
}

// ============================================================
// handleMessageWhenUp
// ============================================================
void MineDetectionApp::handleMessageWhenUp(cMessage *msg)
{
    if (msg == scanTimer) {
        performScan();
    }
    else if (msg == intensiveTimer) {
        isIntensiveMode = false;
        EV_INFO << "UAV[" << uavId << "] Leaving spiral mode — "
                   "climbing back to " << CRUISE_ALTITUDE << "m\n";

        auto lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
        if (lawnmower) lawnmower->stopSpiral();

        setFlightAltitude(CRUISE_ALTITUDE);
    }
    else if (msg == returnHomeTimer) {
        EV_INFO << "UAV[" << uavId << "] Return-home timer fired at t="
                << simTime() << "s — initiating RTH sequence.\n";
        initiateReturnHome();
    }
    else if (msg->arrivedOn("socketIn")) {
        socket.processMessage(msg);
    }
    else {
        delete msg;
    }
}

// ============================================================
// socketDataArrived
// ============================================================
void MineDetectionApp::socketDataArrived(UdpSocket *, Packet *pkt)
{
    auto payload = pkt->peekData<BytesChunk>();
    const auto& bytes = payload->getBytes();
    std::string msgText(reinterpret_cast<const char*>(bytes.data()),
                        bytes.size());

    if (msgText.find("CMD:INTENSIVE_SEARCH") == 0) {
        size_t xP = msgText.find(",x=");
        size_t yP = msgText.find(",y=");
        if (xP != std::string::npos && yP != std::string::npos) {
            try {
                double cmdX = std::stod(msgText.substr(xP + 3, yP - xP - 3));
                double cmdY = std::stod(msgText.substr(yP + 3));
                startIntensiveSearch(cmdX, cmdY);
            } catch(...) {}
        }
        delete pkt; return;
    }

    size_t uavPos  = msgText.find("uav=");
    size_t xPos    = msgText.find(",x=");
    size_t yPos    = msgText.find(",y=");
    size_t confPos = msgText.find(",conf=");
    size_t magPos  = msgText.find(",magVal=");
    size_t tPos    = msgText.find(",t=");

    if (uavPos != std::string::npos && xPos != std::string::npos &&
        yPos != std::string::npos && confPos != std::string::npos &&
        magPos != std::string::npos && tPos != std::string::npos)
    {
        try {
            int senderUav = std::stoi(
                msgText.substr(uavPos + 4, xPos - uavPos - 4));
            double parsedX = std::stod(
                msgText.substr(xPos + 3, yPos - xPos - 3));
            double parsedY = std::stod(
                msgText.substr(yPos + 3, confPos - yPos - 3));
            double parsedConf = std::stod(
                msgText.substr(confPos + 6, magPos - confPos - 6));
            double parsedMag  = std::stod(
                msgText.substr(magPos + 8, tPos - magPos - 8));

            Coord incomingPos(parsedX, parsedY, 0);

            if (senderUav != uavId) {
                if (msgText.find("CANDIDATE") == 0) {
                    CandidateMine cand = {incomingPos, simTime(),
                                          senderUav, parsedConf, parsedMag};
                    candidateMines.push_back(cand);
                }
                else if (msgText.find("CONFIRMED") == 0) {
                    sharedMemory.push_back(incomingPos);
                    for (auto it = candidateMines.begin();
                         it != candidateMines.end(); ) {
                        if (it->pos.distance(incomingPos) < confirmRadius)
                            it = candidateMines.erase(it);
                        else ++it;
                    }
                }
            }
        } catch (...) {}
    }
    delete pkt;
}

void MineDetectionApp::socketErrorArrived(UdpSocket *, Indication *ind)
{
    delete ind;
}

// ============================================================
// finish
// [MODIFIED]: إضافة إحصاءات العتبة التكيفية في التقرير النهائي
// ============================================================
void MineDetectionApp::finish()
{
    EV_INFO << "\n=== UAV[" << uavId << "] Final Statistics ===\n";
    EV_INFO << "True Detections   : " << trueDetections   << "\n";
    EV_INFO << "False Alarms      : " << falseAlarms       << "\n";
    EV_INFO << "Duplicates Skipped: " << duplicatesSkipped << "\n";
    EV_INFO << "Messages Sent     : " << messagesSent      << "\n";
    EV_INFO << "Returned to Home  : " << (isReturningHome ? "YES" : "NO") << "\n";

    // [NEW]: تقرير العتبة التكيفية
    if (sensor->isAdaptiveReady()) {
        EV_INFO << "Final Adaptive Threshold: "
                << sensor->getAdaptiveThreshold() << " nT"
                << " (mean=" << sensor->getWindowMean()
                << " sigma=" << sensor->getWindowStdDev() << ")\n";
    } else {
        EV_INFO << "Adaptive Threshold: NOT REACHED (window only "
                << sensor->getWindowFill() << "/" << sensor->getWindowSize()
                << " filled — static threshold used throughout)\n";
    }

    if (uavId == 0) {
        EV_INFO << "\n=== Network Summary ===\n";
        EV_INFO << "Total Mines  : " << mineField->getNumMines()        << "\n";
        EV_INFO << "Mines Found  : " << mineField->getDiscoveredCount() << "\n";
        EV_INFO << "Detection Rate: "
                << (double)mineField->getDiscoveredCount()
                   / mineField->getNumMines() * 100.0 << "%\n";
    }

    recordScalar("trueDetections",          trueDetections);
    recordScalar("falseAlarms",             falseAlarms);
    recordScalar("duplicatesSkipped",       duplicatesSkipped);
    recordScalar("returnedHome",            isReturningHome ? 1.0 : 0.0);
    // [NEW]: تسجيل القيم النهائية للعتبة التكيفية
    recordScalar("finalAdaptiveThreshold",  sensor->getAdaptiveThreshold());
    recordScalar("finalWindowMean",         sensor->getWindowMean());
    recordScalar("finalWindowStdDev",       sensor->getWindowStdDev());
    // [NEW]: تسجيل الانجراف النهائي للحساس
    recordScalar("finalCorrelatedNoise",    correlatedNoise);

    delete sensor;
    sensor = nullptr;
}

void MineDetectionApp::handleStartOperation(LifecycleOperation *)
{
    if (scanTimer)       cancelEvent(scanTimer);
    if (intensiveTimer)  cancelEvent(intensiveTimer);
    if (returnHomeTimer) cancelEvent(returnHomeTimer);

    if (!scanTimer)       scanTimer       = new cMessage("scanTimer");
    if (!intensiveTimer)  intensiveTimer  = new cMessage("intensiveTimer");
    if (!returnHomeTimer) returnHomeTimer = new cMessage("returnHomeTimer");

    scheduleAt(simTime() + uniform(0, scanInterval), scanTimer);

    double returnHomeAt = 600.0 - RETURN_HOME_BEFORE;
    if (simTime() < SimTime(returnHomeAt))
        scheduleAt(SimTime(returnHomeAt), returnHomeTimer);
}

void MineDetectionApp::handleStopOperation(LifecycleOperation *)
{
    if (scanTimer && scanTimer->isScheduled())           cancelEvent(scanTimer);
    if (intensiveTimer && intensiveTimer->isScheduled()) cancelEvent(intensiveTimer);
    if (returnHomeTimer && returnHomeTimer->isScheduled()) cancelEvent(returnHomeTimer);
    socket.close();
}

void MineDetectionApp::handleCrashOperation(LifecycleOperation *)
{
    if (scanTimer)       cancelEvent(scanTimer);
    if (intensiveTimer)  cancelEvent(intensiveTimer);
    if (returnHomeTimer) cancelEvent(returnHomeTimer);
    socket.destroy();
}

} // namespace uavminedetection
