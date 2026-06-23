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
simsignal_t MineDetectionApp::correlatedNoiseSignal= registerSignal("correlatedNoise");
simsignal_t MineDetectionApp::timeToConfirmSignal  = registerSignal("timeToConfirm");
simsignal_t MineDetectionApp::uavsInvolvedSignal   = registerSignal("uavsInvolved");
simsignal_t MineDetectionApp::confirmationConfidenceSignal = registerSignal("confirmationConfidence");
simsignal_t MineDetectionApp::adaptiveThresholdSignal      = registerSignal("adaptiveThreshold"); // [NEW]

// ============================================================
// Destructor
// ============================================================
MineDetectionApp::~MineDetectionApp()
{
    if (scanTimer)      { cancelAndDelete(scanTimer);      scanTimer      = nullptr; }
    if (intensiveTimer) { cancelAndDelete(intensiveTimer); intensiveTimer = nullptr; }
    if (statusTimer)    { cancelAndDelete(statusTimer);    statusTimer    = nullptr; }
    if (endOfSimTimer)  { cancelAndDelete(endOfSimTimer);  endOfSimTimer  = nullptr; }
    if (sensor)         { delete sensor;                   sensor         = nullptr; }
    if (adaptiveSensor) { delete adaptiveSensor;           adaptiveSensor = nullptr; } // [NEW]
}

// ============================================================
// getActiveThreshold — [NEW]
// يُعيد العتبة الحالية المستخدمة في القرار
// ============================================================
double MineDetectionApp::getActiveThreshold() const
{
    if (useAdaptiveThreshold && adaptiveSensor)
        return adaptiveSensor->getCurrentThreshold();
    return sensor ? sensor->getThreshold() : magneticThreshold;
}

// ============================================================
// initialize
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

        sensorNoiseAlpha = par("sensorNoiseAlpha");
        sensorNoiseScale = par("sensorNoiseScale");
        correlatedNoise  = 0.0;

        returnHomeEnergyThreshold = par("returnHomeEnergyThreshold");
        energyPerMeter            = par("energyPerMeter");
        simTimeLimit              = par("simTimeLimit");

        // ── [NEW] معاملات العتبة التكيفية ──
        useAdaptiveThreshold = par("useAdaptiveThreshold");
        adaptiveK            = par("adaptiveK");
        adaptiveWindowSize   = (int)par("adaptiveWindowSize");
        adaptiveMinExcess    = par("adaptiveMinExcess");   // [NEW]

        // حساس العتبة الثابتة — يُنشأ دائماً (يُستخدم للرسوم حتى في وضع التكيف)
        sensor = new MagnetometerSensor(magneticThreshold, magneticSaturation);

        // [NEW] حساس التكيف — يُنشأ فقط إذا كان الوضع مُفعَّلاً
        if (useAdaptiveThreshold)
            adaptiveSensor = new AdaptiveMagnetometerSensor(
                adaptiveK, adaptiveWindowSize,
                magneticSaturation, magneticThreshold);

        WATCH(trueDetections);
        WATCH(falseAlarms);
        WATCH(duplicatesSkipped);
        WATCH(messagesSent);
        WATCH(lastMagneticValue);
        WATCH(isIntensiveMode);
        WATCH(isReturningHome);
        WATCH(useAdaptiveThreshold);   // [NEW]
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        mineField = check_and_cast<MineField*>(
            getSystemModule()->getSubmodule("mineField"));
        mobility = check_and_cast<IMobility*>(
            getParentModule()->getSubmodule("mobility"));

        energyStorage = dynamic_cast<inet::power::IEpEnergyStorage*>(
            getParentModule()->getSubmodule("energyStorage"));

        lastEnergyPosition = mobility->getCurrentPosition();

        socket.setOutputGate(gate("socketOut"));
        socket.bind(localPort);
        socket.setCallback(this);

        try {
            L3AddressResolver().tryResolve("gcs", gcsAddress);
        } catch (...) {}

        initSensorVisuals();

        scanTimer      = new cMessage("scanTimer");
        intensiveTimer = new cMessage("intensiveTimer");
        statusTimer    = new cMessage("statusTimer");
        endOfSimTimer  = new cMessage("endOfSimTimer");

        scheduleAt(simTime() + uniform(0, scanInterval), scanTimer);
        scheduleAt(simTime() + STATUS_INTERVAL, statusTimer);

        double returnAt = simTimeLimit - 60.0;
        if (returnAt > simTime().dbl() + 65.0)
            scheduleAt(returnAt, endOfSimTimer);
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

    if (scanTimer      && scanTimer->isScheduled())      cancelEvent(scanTimer);
    if (intensiveTimer && intensiveTimer->isScheduled()) cancelEvent(intensiveTimer);
    if (statusTimer    && statusTimer->isScheduled())    cancelEvent(statusTimer);

    if (isIntensiveMode) {
        isIntensiveMode = false;
        auto *lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
        if (lawnmower) lawnmower->stopSpiral();
    }

    setFlightAltitude(CRUISE_ALTITUDE);

    auto *lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
    if (lawnmower)
        lawnmower->goHome(GCS_X, GCS_Y, CRUISE_ALTITUDE);

    if (!gcsAddress.isUnspecified()) {
        char buf[128];
        snprintf(buf, sizeof(buf), "STATUS:uav=%d,x=%.1f,y=%.1f,cov=100.0,state=RTH",
                 uavId, mobility->getCurrentPosition().x, mobility->getCurrentPosition().y);
        auto payload = makeShared<BytesChunk>();
        std::vector<uint8_t> bytes(buf, buf + strlen(buf));
        payload->setBytes(bytes);
        auto *pkt = new Packet("StatusReport", payload);
        socket.sendTo(pkt, gcsAddress, destPort);
    }
}

// ============================================================
// initSensorVisuals
// ============================================================
void MineDetectionApp::initSensorVisuals()
{
    cCanvas *canvas = getSystemModule()->getCanvas();
    Coord pos = mobility->getCurrentPosition();
    double cx = pos.x, cy = pos.y;

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

    double dashX = -150;
    double dashY = 50 + uavId * 50;

    std::string valName = "magVal_" + std::to_string(uavId);
    sensorValueFigure = new cTextFigure(valName.c_str());
    sensorValueFigure->setPosition(cFigure::Point(dashX, dashY));
    sensorValueFigure->setText("---");
    sensorValueFigure->setColor(cFigure::Color(0, 60, 180));
    sensorValueFigure->setAnchor(cFigure::ANCHOR_NW);
    sensorValueFigure->setFont(cFigure::Font("", 9, cFigure::FONT_BOLD));
    canvas->addFigure(sensorValueFigure);

    std::string thrName = "thr_" + std::to_string(uavId);
    threshFigure = new cTextFigure(thrName.c_str());
    threshFigure->setPosition(cFigure::Point(dashX, dashY + 16));
    threshFigure->setText("---");
    threshFigure->setColor(cFigure::Color(100, 100, 100));
    threshFigure->setAnchor(cFigure::ANCHOR_NW);
    threshFigure->setFont(cFigure::Font("", 8, 0));
    canvas->addFigure(threshFigure);
}

cFigure::Color MineDetectionApp::getMagneticColor(double magVal, double thr) const
{
    double ratio = magVal / thr;
    if      (ratio < 0.8) return cFigure::Color(0, 100, 255);
    else if (ratio < 1.0) return cFigure::Color(255, 200, 0);
    else if (ratio < 1.5) return cFigure::Color(255, 120, 0);
    else                  return cFigure::Color(220, 0, 0);
}

double MineDetectionApp::getMagneticRadius(double magVal, double thr) const
{
    double ratio = magVal / thr;
    return 10.0 + std::min(18.0, ratio * 12.0);
}

// ============================================================
// updateSensorVisuals — [MODIFIED] يعرض العتبة الفعّالة
// ============================================================
void MineDetectionApp::updateSensorVisuals(double magVal, double uavX, double uavY) const
{
    if (!sensorRingFigure || !sensorValueFigure) return;

    double activeThr = getActiveThreshold();   // [NEW] ثابتة أو تكيفية
    cFigure::Color color = getMagneticColor(magVal, activeThr);
    double R = getMagneticRadius(magVal, activeThr);

    sensorRingFigure->setBounds(cFigure::Rectangle(uavX-R, uavY-R, 2*R, 2*R));

    char valBuf[64];
    if (isReturningHome) {
        sensorRingFigure->setFillColor(cFigure::Color(150, 150, 150));
        sensorRingFigure->setLineColor(cFigure::Color(100, 100, 100));
        sensorRingFigure->setLineWidth(2);
        sensorRingFigure->setFillOpacity(0.2);
        snprintf(valBuf, sizeof(valBuf), "UAV[%d]: RTH", uavId);
        sensorValueFigure->setColor(cFigure::Color(100, 100, 100));
    }
    else if (isIntensiveMode) {
        sensorRingFigure->setFillColor(cFigure::Color(255, 0, 0));
        sensorRingFigure->setLineColor(cFigure::Color(255, 0, 0));
        sensorRingFigure->setLineWidth(4);
        sensorRingFigure->setFillOpacity(0.5);
        snprintf(valBuf, sizeof(valBuf), "UAV[%d]: ALERT %.0fnT", uavId, magVal);
        sensorValueFigure->setColor(cFigure::Color(255, 0, 0));
    }
    else {
        sensorRingFigure->setFillColor(color);
        sensorRingFigure->setLineColor(color);
        sensorRingFigure->setLineWidth(2);
        sensorRingFigure->setFillOpacity((magVal >= activeThr) ? 0.35 : 0.18);

        if (magVal >= activeThr)
            snprintf(valBuf, sizeof(valBuf), "UAV[%d]: ! %.0f nT", uavId, magVal);
        else
            snprintf(valBuf, sizeof(valBuf), "UAV[%d]: %.0f nT", uavId, magVal);

        sensorValueFigure->setColor(color);
    }

    sensorValueFigure->setText(valBuf);

    // [NEW] يعرض نوع العتبة ويُلوِّنها
    if (threshFigure) {
        char thrBuf[80];
        if (useAdaptiveThreshold && adaptiveSensor) {
            const char* warmupTag = adaptiveSensor->isWarmedUp() ? "" : " [WU]";
            snprintf(thrBuf, sizeof(thrBuf),
                     "AThr(k=%.1f): %.0fnT%s | F:%d",
                     adaptiveK, activeThr, warmupTag, trueDetections);
            threshFigure->setColor(cFigure::Color(0, 120, 60));   // أخضر داكن للتمييز
        } else {
            snprintf(thrBuf, sizeof(thrBuf),
                     "Thr: %.0fnT | Found: %d", activeThr, trueDetections);
            threshFigure->setColor(cFigure::Color(0, 80, 0));
        }
        threshFigure->setText(thrBuf);
    }

    if (sensorBarBg && sensorBarFg) {
        sensorBarBg->setBounds(cFigure::Rectangle(uavX-12, uavY+14, 24, 4));
        double fillRatio = std::min(1.0, magVal / (1.5 * activeThr));
        sensorBarFg->setBounds(cFigure::Rectangle(
            uavX-12, uavY+14, std::max(1.0, fillRatio * 24.0), 4));
        if (isReturningHome)
            sensorBarFg->setFillColor(cFigure::Color(150, 150, 150));
        else
            sensorBarFg->setFillColor(
                isIntensiveMode ? cFigure::Color(255, 0, 0) : color);
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
            emit(timeToConfirmSignal,  simTime() - it->firstDetectedTime);
            emit(uavsInvolvedSignal,   1L);
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
void MineDetectionApp::confirmTarget(inet::Coord targetPos, double confidence, double magVal)
{
    sharedMemory.push_back(targetPos);
    emit(confirmationConfidenceSignal, confidence);

    int mineIdx = mineField->getNearestUndiscoveredMine(
        targetPos.x, targetPos.y, confirmRadius);

    if (mineIdx < 0)
        mineIdx = mineField->getNearestUndiscoveredMine(
            targetPos.x, targetPos.y, confirmRadius * 1.8);

    if (mineIdx >= 0) {
        mineField->markDiscovered(mineIdx);
        trueDetections++;
        emit(detectionSignal, 1L);
        sendNetworkMessage("CONFIRMED_REAL", targetPos.x, targetPos.y, confidence, magVal);
        return;
    }

    int debrisIdx = mineField->getNearestMetalDebris(
        targetPos.x, targetPos.y, confirmRadius);

    if (debrisIdx >= 0) {
        mineField->markDebrisTriggered(debrisIdx);
        falseAlarms++;
        emit(falseAlarmSignal, 1L);
        const auto& db = mineField->getDebris();
        addFalseAlarmFigure(db[debrisIdx].x, db[debrisIdx].y);
        sendNetworkMessage("CONFIRMED_FA", targetPos.x, targetPos.y, confidence, magVal);
        return;
    }

    int anyIdx = mineField->getNearestMetalDebrisAny(
        targetPos.x, targetPos.y, confirmRadius);
    if (anyIdx >= 0) {
        duplicatesSkipped++;
        return;
    }

    duplicatesSkipped++;
}

// ============================================================
// performScan — [MODIFIED] يختار الحساس حسب useAdaptiveThreshold
// ============================================================
void MineDetectionApp::performScan()
{
    // ── فحص الطاقة ──
    if (!isReturningHome && energyStorage) {
        Coord currentPos = mobility->getCurrentPosition();
        double dist = currentPos.distance(lastEnergyPosition);
        if (dist > 0) totalEnergyConsumed += dist * energyPerMeter;
        lastEnergyPosition = currentPos;

        double nominalCapacity = energyStorage->getNominalEnergyCapacity().get();
        double remainingRatio  = (nominalCapacity - totalEnergyConsumed) / nominalCapacity;

        if (remainingRatio <= returnHomeEnergyThreshold) {
            initiateReturnHome();
            return;
        }
    }

    if (isReturningHome) return;

    checkTimeouts();
    Coord pos = mobility->getCurrentPosition();

    // ── إنتاج القراءة المغناطيسية ──
    double rawMagVal  = mineField->getMagneticValue(pos.x, pos.y, pos.z);
    double innovation = uniform(-sensorNoiseScale, sensorNoiseScale);
    correlatedNoise   = sensorNoiseAlpha * correlatedNoise + (1.0 - sensorNoiseAlpha) * innovation;
    double magVal     = rawMagVal + correlatedNoise;
    lastMagneticValue = magVal;

    emit(correlatedNoiseSignal, correlatedNoise);

    // ── [NEW] إرسال إشارة العتبة التكيفية لمتابعتها عبر الوقت ──
    if (useAdaptiveThreshold && adaptiveSensor)
        emit(adaptiveThresholdSignal, adaptiveSensor->getCurrentThreshold());

    // ── فحص التكرار ──
    for (const auto& knownPos : sharedMemory) {
        if (pos.distance(knownPos) < confirmRadius) {
            duplicatesSkipped++;
            scheduleAt(simTime() + scanInterval, scanTimer);
            return;
        }
    }

    // ── وضع المسح الحلزوني ──
    if (isIntensiveMode) {
        if (magVal > maxSpiralMagVal) {
            maxSpiralMagVal = magVal;
            maxSpiralPos    = pos;
        }

        // [MODIFIED] نستخدم العتبة الفعّالة بدل sensor->getThreshold() مباشرة
        double activeThr = getActiveThreshold();
        if (magVal > (activeThr * spiralStopRatio)) {
            if (intensiveTimer && intensiveTimer->isScheduled())
                cancelEvent(intensiveTimer);

            isIntensiveMode = false;
            auto lawnmower  = dynamic_cast<LawnmowerMobility*>(mobility);
            if (lawnmower) lawnmower->stopSpiral();

            confirmTarget(maxSpiralPos, 1.0, maxSpiralMagVal);
            sendNetworkMessage("SPIRAL_PEAK", maxSpiralPos.x, maxSpiralPos.y,
                               1.0, maxSpiralMagVal);
            setFlightAltitude(CRUISE_ALTITUDE);
        }
    }

    updateSensorVisuals(magVal, pos.x, pos.y);

    // ── [MODIFIED] اختيار الحساس الفعّال ──
    SensorReading reading;
    if (useAdaptiveThreshold && adaptiveSensor)
        // [FIX] أثناء وضع البحث المكثف (Spiral)، لا تُضاف القراءة
        // لنافذة الخلفية، لأنها تمثل إشارة هدف لا خلفية —
        // إضافتها كانت تُسبب انفجار العتبة (Runaway Feedback).
        reading = adaptiveSensor->measure(magVal, !isIntensiveMode);
    else
        reading = sensor->measure(magVal);            // العتبة الثابتة

    if (reading.isMine) {
        // ── [NEW] بوابة هامش الثقة — تُفعَّل فقط في الوضع التكيفي ──
        //
        // المشكلة: العتبة التكيفية تتبع الخلفية المحلية بدقة (CFAR)،
        // فأي تجاوز بسيط جداً — حتى من شظية معدنية صغيرة (مسمار/سلك) —
        // يُسجَّل كمرشح فوراً بنفس معاملة اللغم الحقيقي. هذا لا يحدث
        // بنفس الحدة مع العتبة الثابتة لأن هامشها عادة أكبر من الخلفية.
        //
        // الحل: نشترط تجاوز العتبة التكيفية الحالية بمقدار لا يقل عن
        // adaptiveMinExcess نانوتسلا قبل قبول القراءة كمرشح. إشارة
        // اللغم الحقيقي عند نفس مدى الكشف أقوى بكثير من إشارة الشظايا
        // الصغيرة، فهي تتجاوز هذا الهامش بسهولة — لذلك معدل اكتشاف
        // الألغام لا يتأثر، بينما تُستبعد التجاوزات الهامشية القريبة
        // من خط العتبة (مصدر معظم الإنذارات الكاذبة في الوضع التكيفي).
        //
        // [مهم] هذه البوابة معطّلة تماماً عندما useAdaptiveThreshold=false،
        // فلا تؤثر بأي شكل على سلوك العتبة الثابتة.
        bool passesConfidenceGate = true;
        if (useAdaptiveThreshold && adaptiveSensor) {
            double excess = magVal - adaptiveSensor->getCurrentThreshold();
            passesConfidenceGate = (excess >= adaptiveMinExcess);
        }

        if (passesConfidenceGate) {
        bool isAlreadyCandidate = false;
        for (auto it = candidateMines.begin(); it != candidateMines.end(); ++it) {
            if (pos.distance(it->pos) < confirmRadius) {
                isAlreadyCandidate = true;
                if (it->firstDetectorId != uavId) {
                    emit(timeToConfirmSignal, simTime() - it->firstDetectedTime);
                    emit(uavsInvolvedSignal,  2L);
                    confirmTarget(it->pos, reading.confidence, reading.magneticValue);
                    candidateMines.erase(it);
                } else {
                    duplicatesSkipped++;
                }
                break;
            }
        }

        if (!isAlreadyCandidate) {
            int anyIdx = mineField->getNearestMetalDebrisAny(pos.x, pos.y, confirmRadius);
            if (anyIdx >= 0 && mineField->getDebris()[anyIdx].triggered) {
                duplicatesSkipped++;
            } else {
                CandidateMine newCand = {pos, simTime(), uavId,
                                        reading.confidence, reading.magneticValue};
                candidateMines.push_back(newCand);
                sendNetworkMessage("CANDIDATE", pos.x, pos.y,
                                   reading.confidence, reading.magneticValue);
            }
        }
        }
    }

    if (fmod(simTime().dbl(), 10.0) < scanInterval)
        emit(coverageSignal, calculateCoverage());

    scheduleAt(simTime() + scanInterval, scanTimer);
}

// ============================================================
// broadcastStatus
// ============================================================
void MineDetectionApp::broadcastStatus()
{
    if (isReturningHome || gcsAddress.isUnspecified()) return;

    Coord pos       = mobility->getCurrentPosition();
    double coverage = calculateCoverage();

    char buf[128];
    snprintf(buf, sizeof(buf), "STATUS:uav=%d,x=%.1f,y=%.1f,cov=%.2f,state=%s",
             uavId, pos.x, pos.y, coverage,
             isIntensiveMode ? "SPIRAL" : "SCAN");

    auto payload = makeShared<BytesChunk>();
    std::vector<uint8_t> bytes(buf, buf + strlen(buf));
    payload->setBytes(bytes);
    auto *pkt = new Packet("StatusReport", payload);
    socket.sendTo(pkt, gcsAddress, destPort);
}

// ============================================================
// refreshDisplay
// ============================================================
void MineDetectionApp::refreshDisplay() const
{
    if (mobility && sensorRingFigure) {
        Coord pos = mobility->getCurrentPosition();
        updateSensorVisuals(lastMagneticValue, pos.x, pos.y);
    }
    getParentModule()->getDisplayString().setTagArg("t", 0, "");
}

// ============================================================
// sendNetworkMessage
// ============================================================
void MineDetectionApp::sendNetworkMessage(const char* type, double x, double y,
                                          double confidence, double magneticValue)
{
    char buf[160];
    snprintf(buf, sizeof(buf),
             "%s:uav=%d,x=%.1f,y=%.1f,conf=%.2f,magVal=%.1f,t=%.2f",
             type, uavId, x, y, confidence, magneticValue, simTime().dbl());

    if (!gcsAddress.isUnspecified()) {
        auto payload = makeShared<BytesChunk>();
        std::vector<uint8_t> b(buf, buf + strlen(buf));
        payload->setBytes(b);
        socket.sendTo(new Packet("MineReport_GCS", payload), gcsAddress, destPort);
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
                socket.sendTo(new Packet("MineReport_UAV", payloadU), addr, destPort);
            }
        } catch (...) {}
    }
    messagesSent++;
}

// ============================================================
// calculateCoverage
// ============================================================
double MineDetectionApp::calculateCoverage()
{
    const int    G = 50;
    const double S = 1000.0 / G;

    Coord pos = mobility->getCurrentPosition();
    int col = std::max(0, std::min(G-1, (int)(pos.x / S)));
    int row = std::max(0, std::min(G-1, (int)(pos.y / S)));
    visitedCells.insert(col * G + row);
    return (double)visitedCells.size() / (G * G) * 100.0;
}

// ============================================================
// addFalseAlarmFigure
// ============================================================
void MineDetectionApp::addFalseAlarmFigure(double x, double y)
{
    if (falseAlarmDisplayLimit >= 0 && falseAlarmFigureCount >= falseAlarmDisplayLimit) return;

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
        maxSpiralMagVal = -1.0;
        maxSpiralPos    = pos;

        auto lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
        if (lawnmower) lawnmower->startSpiral(cmdX, cmdY);

        setFlightAltitude(SCAN_ALTITUDE);

        cancelEvent(intensiveTimer);
        scheduleAt(simTime() + 20.0, intensiveTimer);
    }
}

// ============================================================
// redirectToArea
// ============================================================
void MineDetectionApp::redirectToArea(double targetX, double targetY)
{
    if (isReturningHome) return;

    auto lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
    if (lawnmower) {
        if (isIntensiveMode) {
            isIntensiveMode = false;
            if (intensiveTimer && intensiveTimer->isScheduled())
                cancelEvent(intensiveTimer);
            lawnmower->stopSpiral();
            setFlightAltitude(CRUISE_ALTITUDE);
        }
        lawnmower->goHome(targetX, targetY, CRUISE_ALTITUDE);
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
    else if (msg == endOfSimTimer) {
        if (!isReturningHome) {
            EV_INFO << "UAV[" << uavId << "]: End-of-simulation RTH triggered at t="
                    << simTime() << "\n";
            initiateReturnHome();
        }
    }
    else if (msg == intensiveTimer) {
        isIntensiveMode = false;
        auto lawnmower  = dynamic_cast<LawnmowerMobility*>(mobility);
        if (lawnmower) lawnmower->stopSpiral();
        setFlightAltitude(CRUISE_ALTITUDE);
    }
    else if (msg == statusTimer) {
        broadcastStatus();
        scheduleAt(simTime() + STATUS_INTERVAL, statusTimer);
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
    std::string msgText(reinterpret_cast<const char*>(bytes.data()), bytes.size());

    if (msgText.find("CMD:INTENSIVE_SEARCH") == 0) {
        size_t xP = msgText.find(",x="), yP = msgText.find(",y=");
        if (xP != std::string::npos && yP != std::string::npos) {
            try {
                double cmdX = std::stod(msgText.substr(xP + 3, yP - xP - 3));
                double cmdY = std::stod(msgText.substr(yP + 3));
                startIntensiveSearch(cmdX, cmdY);
            } catch (...) {}
        }
        delete pkt; return;
    }
    else if (msgText.find("CMD:REDIRECT") == 0) {
        size_t xP = msgText.find(",x="), yP = msgText.find(",y=");
        if (xP != std::string::npos && yP != std::string::npos) {
            try {
                double tgtX = std::stod(msgText.substr(xP + 3, yP - xP - 3));
                double tgtY = std::stod(msgText.substr(yP + 3));
                redirectToArea(tgtX, tgtY);
            } catch (...) {}
        }
        delete pkt; return;
    }
    else if (msgText.find("CMD:CANCEL_SPIRAL") == 0) {
        if (isIntensiveMode) {
            isIntensiveMode = false;
            if (intensiveTimer && intensiveTimer->isScheduled())
                cancelEvent(intensiveTimer);
            auto lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
            if (lawnmower) lawnmower->stopSpiral();
            setFlightAltitude(CRUISE_ALTITUDE);
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
        yPos   != std::string::npos && confPos != std::string::npos &&
        magPos != std::string::npos && tPos    != std::string::npos)
    {
        try {
            int    senderUav  = std::stoi(msgText.substr(uavPos + 4, xPos - uavPos - 4));
            double parsedX    = std::stod(msgText.substr(xPos + 3, yPos - xPos - 3));
            double parsedY    = std::stod(msgText.substr(yPos + 3, confPos - yPos - 3));
            double parsedConf = std::stod(msgText.substr(confPos + 6, magPos - confPos - 6));
            double parsedMag  = std::stod(msgText.substr(magPos + 8, tPos - magPos - 8));

            Coord incomingPos(parsedX, parsedY, 0);

            if (senderUav != uavId) {
                if (msgText.find("CANDIDATE") == 0) {
                    CandidateMine cand = {incomingPos, simTime(), senderUav, parsedConf, parsedMag};
                    candidateMines.push_back(cand);
                }
                else if (msgText.find("CONFIRMED") == 0) {
                    sharedMemory.push_back(incomingPos);

                    if (msgText.find("CONFIRMED_FA") == 0) {
                        int dIdx = mineField->getNearestMetalDebrisAny(
                            incomingPos.x, incomingPos.y, confirmRadius);
                        if (dIdx >= 0 && !mineField->getDebris()[dIdx].triggered)
                            mineField->markDebrisTriggered(dIdx);
                    }

                    for (auto it = candidateMines.begin(); it != candidateMines.end(); ) {
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
// finish — [MODIFIED] يُسجِّل إحصاءات إضافية للعتبة التكيفية
// ============================================================
void MineDetectionApp::finish()
{
    EV_INFO << "\n=== UAV[" << uavId << "] Final Statistics ===\n";
    EV_INFO << "Threshold Mode    : "
            << (useAdaptiveThreshold ? "ADAPTIVE" : "FIXED") << "\n";
    EV_INFO << "True Detections   : " << trueDetections   << "\n";
    EV_INFO << "False Alarms      : " << falseAlarms       << "\n";
    EV_INFO << "Duplicates Skipped: " << duplicatesSkipped << "\n";
    EV_INFO << "Messages Sent     : " << messagesSent      << "\n";
    EV_INFO << "Returned to Home  : " << (isReturningHome ? "YES" : "NO") << "\n";

    if (useAdaptiveThreshold && adaptiveSensor) {
        EV_INFO << "Final Adaptive Thr: " << adaptiveSensor->getCurrentThreshold()
                << " nT  (k=" << adaptiveK
                << ", window=" << adaptiveWindowSize
                << ", minExcess=" << adaptiveMinExcess << ")\n";
        EV_INFO << "Warmed Up         : "
                << (adaptiveSensor->isWarmedUp() ? "YES" : "NO") << "\n";
    } else {
        EV_INFO << "Static Threshold  : " << magneticThreshold << " nT\n";
    }

    if (uavId == 0) {
        EV_INFO << "\n=== Network Summary ===\n";
        EV_INFO << "Total Mines  : " << mineField->getNumMines()        << "\n";
        EV_INFO << "Mines Found  : " << mineField->getDiscoveredCount() << "\n";
        EV_INFO << "Detection Rate: "
                << (double)mineField->getDiscoveredCount()
                   / mineField->getNumMines() * 100.0 << "%\n";
    }

    // ── Scalars الأصلية ──
    recordScalar("trueDetections",    trueDetections);
    recordScalar("falseAlarms",       falseAlarms);
    recordScalar("duplicatesSkipped", duplicatesSkipped);
    recordScalar("returnedHome",      isReturningHome ? 1.0 : 0.0);
    recordScalar("finalCorrelatedNoise", correlatedNoise);

    // ── [NEW] Scalars للمقارنة بين الوضعين ──
    recordScalar("useAdaptiveThreshold", useAdaptiveThreshold ? 1.0 : 0.0);
    recordScalar("staticThreshold",      magneticThreshold);

    if (useAdaptiveThreshold && adaptiveSensor)
        recordScalar("finalAdaptiveThreshold",
                     adaptiveSensor->getCurrentThreshold());
    else
        recordScalar("finalAdaptiveThreshold", magneticThreshold);
}

// ============================================================
// Lifecycle operations
// ============================================================
void MineDetectionApp::handleStartOperation(LifecycleOperation *)
{
    if (scanTimer)      cancelEvent(scanTimer);
    if (intensiveTimer) cancelEvent(intensiveTimer);
    if (statusTimer)    cancelEvent(statusTimer);
    if (endOfSimTimer)  cancelEvent(endOfSimTimer);

    if (!scanTimer)      scanTimer      = new cMessage("scanTimer");
    if (!intensiveTimer) intensiveTimer = new cMessage("intensiveTimer");
    if (!statusTimer)    statusTimer    = new cMessage("statusTimer");
    if (!endOfSimTimer)  endOfSimTimer  = new cMessage("endOfSimTimer");

    scheduleAt(simTime() + uniform(0, scanInterval), scanTimer);
    scheduleAt(simTime() + STATUS_INTERVAL, statusTimer);

    double returnAt = simTimeLimit - 60.0;
    if (returnAt > simTime().dbl() + 65.0)
        scheduleAt(returnAt, endOfSimTimer);
}

void MineDetectionApp::handleStopOperation(LifecycleOperation *)
{
    if (scanTimer      && scanTimer->isScheduled())      cancelEvent(scanTimer);
    if (intensiveTimer && intensiveTimer->isScheduled()) cancelEvent(intensiveTimer);
    if (statusTimer    && statusTimer->isScheduled())    cancelEvent(statusTimer);
    if (endOfSimTimer  && endOfSimTimer->isScheduled())  cancelEvent(endOfSimTimer);
    socket.close();
}

void MineDetectionApp::handleCrashOperation(LifecycleOperation *)
{
    if (scanTimer      && scanTimer->isScheduled())      cancelEvent(scanTimer);
    if (intensiveTimer && intensiveTimer->isScheduled()) cancelEvent(intensiveTimer);
    if (statusTimer    && statusTimer->isScheduled())    cancelEvent(statusTimer);
    if (endOfSimTimer  && endOfSimTimer->isScheduled())  cancelEvent(endOfSimTimer);
    socket.destroy();
}

} // namespace uavminedetection
