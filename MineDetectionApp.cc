#include "LawnmowerMobility.h"
#include "MineDetectionApp.h"
#include "inet/common/ModuleAccess.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/TagBase_m.h"
// [NEW]: مطلوب لتعديل constraintArea والارتفاع في وقت التشغيل
#include "inet/mobility/base/MovingMobilityBase.h"

#include <cmath>
#include <string>
#include <algorithm>

using namespace inet;

namespace uavminedetection {

Define_Module(MineDetectionApp);

simsignal_t MineDetectionApp::detectionSignal  = registerSignal("mineDetected");
simsignal_t MineDetectionApp::coverageSignal   = registerSignal("areaCoverage");
simsignal_t MineDetectionApp::falseAlarmSignal = registerSignal("falseAlarm");

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
        confirmRadius      = par("confirmRadius");
        destPort           = par("destPort");
        localPort          = par("localPort");

        sensor = new MagnetometerSensor(magneticThreshold, magneticSaturation);

        WATCH(trueDetections);
        WATCH(falseAlarms);
        WATCH(duplicatesSkipped);
        WATCH(messagesSent);
        WATCH(lastMagneticValue);
        WATCH(isIntensiveMode);
        WATCH(isReturningHome);   // [NEW]
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        mineField = check_and_cast<MineField*>(
            getSystemModule()->getSubmodule("mineField"));
        mobility = check_and_cast<IMobility*>(
            getParentModule()->getSubmodule("mobility"));

        socket.setOutputGate(gate("socketOut"));
        socket.bind(localPort);
        socket.setCallback(this);

        // حل عنوان GCS مرة واحدة
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

        scanTimer      = new cMessage("scanTimer");
        intensiveTimer = new cMessage("intensiveTimer");
        returnHomeTimer= new cMessage("returnHomeTimer");   // [NEW]

        scheduleAt(simTime() + uniform(0, scanInterval), scanTimer);

        // ── [NEW]: جدولة العودة إلى القاعدة عند t=550s (50 ثانية قبل النهاية) ──
        double returnHomeAt = 600.0 - RETURN_HOME_BEFORE; // = 550.0
        scheduleAt(SimTime(returnHomeAt), returnHomeTimer);
        EV_INFO << "UAV[" << uavId << "] Return-home scheduled at t="
                << returnHomeAt << "s\n";
    }
}

// ============================================================
// setFlightAltitude
// ──────────────────────────────────────────────────────────────
// يستدعي LawnmowerMobility::setAltitude() التي تغيّر المتغير
// الداخلي altitude مباشرة — هذا هو الحل الصحيح لأن تغيير par()
// وحده لا يؤثر على المتغير الداخلي المخزَّن منذ initialize().
// ============================================================
void MineDetectionApp::setFlightAltitude(double altMeters)
{
    auto *lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
    if (lawnmower) lawnmower->setAltitude(altMeters);
}

// ============================================================
// [NEW] initiateReturnHome
// ──────────────────────────────────────────────────────────────
// يُفعَّل عند t=550s: توقف عن المسح والتوجه نحو GCS
// ============================================================
void MineDetectionApp::initiateReturnHome()
{
    if (isReturningHome) return;  // تجنب الاستدعاء المزدوج

    isReturningHome = true;

    // إلغاء كل المؤقتات النشطة
    if (scanTimer && scanTimer->isScheduled())
        cancelEvent(scanTimer);
    if (intensiveTimer && intensiveTimer->isScheduled())
        cancelEvent(intensiveTimer);

    // الخروج من وضع المسح الحلزوني إن كان نشطاً
    if (isIntensiveMode) {
        isIntensiveMode = false;
        auto *lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
        if (lawnmower) lawnmower->stopSpiral();
    }

    // العودة إلى الارتفاع الطبيعي قبل التوجه
    setFlightAltitude(CRUISE_ALTITUDE);

    // توجيه الطائرة نحو GCS عبر Spiral مبسّط باتجاه النقطة الهدف
    // الأنسب: نستخدم startSpiral مع نقطة GCS ليتقارب إليها الحلزون
    // أو ببساطة نجعل LawnmowerMobility يتجه مباشرة لـ GCS
    auto *lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
    if (lawnmower) {
        lawnmower->goHome(GCS_X, GCS_Y, CRUISE_ALTITUDE);
    }

    EV_INFO << "UAV[" << uavId << "] Returning to GCS at ("
            << GCS_X << ", " << GCS_Y << ") -- t=" << simTime() << "s\n";
}

// ============================================================
// initSensorVisuals
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

cFigure::Color MineDetectionApp::getMagneticColor(double magVal) const
{
    double ratio = magVal / magneticThreshold;
    if (ratio < 0.8)       return cFigure::Color(0, 100, 255);
    else if (ratio < 1.0)  return cFigure::Color(255, 200, 0);
    else if (ratio < 1.5)  return cFigure::Color(255, 120, 0);
    else                   return cFigure::Color(220, 0, 0);
}

double MineDetectionApp::getMagneticRadius(double magVal) const
{
    double ratio = magVal / magneticThreshold;
    return 10.0 + std::min(18.0, ratio * 12.0);
}

void MineDetectionApp::updateSensorVisuals(double magVal,
                                           double uavX, double uavY) const
{
    if (!sensorRingFigure || !sensorValueFigure) return;

    cFigure::Color color = getMagneticColor(magVal);
    double R = getMagneticRadius(magVal);

    sensorRingFigure->setBounds(cFigure::Rectangle(uavX-R, uavY-R, 2*R, 2*R));

    if (isReturningHome) {
        // [NEW]: حالة العودة — حلقة رمادية وخط متقطع
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
        sensorRingFigure->setFillOpacity((magVal >= magneticThreshold) ? 0.35 : 0.18);
        char buf[32];
        if (magVal >= magneticThreshold)
            snprintf(buf, sizeof(buf), "! %.0f nT", magVal);
        else
            snprintf(buf, sizeof(buf), "%.0f nT", magVal);
        sensorValueFigure->setText(buf);
        sensorValueFigure->setColor(color);
    }

    sensorValueFigure->setPosition(cFigure::Point(uavX, uavY - 18));

    if (sensorBarBg && sensorBarFg) {
        sensorBarBg->setBounds(cFigure::Rectangle(uavX-12, uavY+14, 24, 4));
        double fillRatio = std::min(1.0, magVal / (1.5 * magneticThreshold));
        sensorBarFg->setBounds(cFigure::Rectangle(
            uavX-12, uavY+14, std::max(1.0, fillRatio * 24.0), 4));
        if (isReturningHome)
            sensorBarFg->setFillColor(cFigure::Color(150, 150, 150));
        else
            sensorBarFg->setFillColor(isIntensiveMode ? cFigure::Color(255,0,0) : color);
    }
}

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

void MineDetectionApp::performScan()
{
    // لا مسح إذا كانت الطائرة في طريق العودة
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

    double magVal = mineField->getMagneticValue(pos.x, pos.y);
    lastMagneticValue = magVal;
    updateSensorVisuals(magVal, pos.x, pos.y);

    SensorReading reading = sensor->measure(magVal);

    if (reading.isMine) {
        bool isAlreadyCandidate = false;
        for (auto it = candidateMines.begin(); it != candidateMines.end(); ++it) {
            if (pos.distance(it->pos) < confirmRadius) {
                isAlreadyCandidate = true;
                if (it->firstDetectorId != uavId) {
                    EV_INFO << "UAV[" << uavId << "] CROSS-VALIDATED from UAV["
                            << it->firstDetectorId << "]\n";
                    confirmTarget(it->pos, reading.confidence, reading.magneticValue);
                    candidateMines.erase(it);
                } else {
                    duplicatesSkipped++;
                }
                break;
            }
        }

        if (!isAlreadyCandidate) {
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

void MineDetectionApp::refreshDisplay() const
{
    if (mobility && sensorRingFigure) {
        Coord pos = mobility->getCurrentPosition();
        updateSensorVisuals(lastMagneticValue, pos.x, pos.y);
    }

    char buf[120];
    if (isReturningHome) {
        snprintf(buf, sizeof(buf),
                 "UAV%d | RTH → GCS | found=%d | FA=%d",
                 uavId, trueDetections, falseAlarms);
    } else {
        snprintf(buf, sizeof(buf),
                 "UAV%d | %.0f nT | found=%d | FA=%d | Pend=%zu",
                 uavId, lastMagneticValue, trueDetections,
                 falseAlarms, candidateMines.size());
    }
    getParentModule()->getDisplayString().setTagArg("t", 0, buf);
}

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

double MineDetectionApp::calculateCoverage()
{
    const int G = 50;
    const double AREA = 1000.0;
    int cov = 0;
    cModule *net = getSystemModule();
    int nUAV = net->par("numUAVs");
    const double sensorCovRange = 10.0;

    for (int i = 0; i < G; i++) {
        for (int j = 0; j < G; j++) {
            double px = (i + 0.5) * (AREA / G);
            double py = (j + 0.5) * (AREA / G);
            for (int u = 0; u < nUAV; u++) {
                IMobility *mob = check_and_cast<IMobility*>(
                    net->getSubmodule("uav", u)->getSubmodule("mobility"));
                if (mob->getCurrentPosition().distance(Coord(px, py, 0))
                        <= sensorCovRange) {
                    cov++; break;
                }
            }
        }
    }
    return (double)cov / (G * G) * 100.0;
}

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
// ── [MODIFIED]: يُضيف تغيير الارتفاع إلى 30م عند الدخول
//               في المسح الحلزوني، والعودة إلى 80م عند الخروج
// ============================================================
void MineDetectionApp::startIntensiveSearch(double cmdX, double cmdY)
{
    // تجاهل الأمر إذا كانت الطائرة تعود للقاعدة
    if (isReturningHome) return;

    Coord pos = mobility->getCurrentPosition();
    if (pos.distance(Coord(cmdX, cmdY, 0)) < 200.0) {
        isIntensiveMode = true;
        EV_INFO << "UAV[" << uavId << "] Entering SPIRAL mode at ("
                << cmdX << "," << cmdY << ") — descending to "
                << SCAN_ALTITUDE << "m\n";

        // ── [FIX]: ابدأ الحلزون أولاً لحفظ savedPosition على z=80م
        //          ثم اخفض الارتفاع — هكذا عند stopSpiral() تعود z إلى 80م ──
        auto lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
        if (lawnmower)
            lawnmower->startSpiral(cmdX, cmdY);

        setFlightAltitude(SCAN_ALTITUDE);  // ينزل إلى 30م بعد حفظ savedPosition

        cancelEvent(intensiveTimer);
        scheduleAt(simTime() + 20.0, intensiveTimer);
    }
}

// ============================================================
// handleMessageWhenUp
// ── [MODIFIED]: إضافة معالجة returnHomeTimer
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

        // ── [NEW]: العودة إلى الارتفاع الطبيعي 80م ──
        setFlightAltitude(CRUISE_ALTITUDE);
    }
    else if (msg == returnHomeTimer) {
        // ── [NEW]: حان وقت العودة إلى القاعدة ──
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

void MineDetectionApp::finish()
{
    EV_INFO << "\n=== UAV[" << uavId << "] Final Statistics ===\n";
    EV_INFO << "True Detections   : " << trueDetections   << "\n";
    EV_INFO << "False Alarms      : " << falseAlarms       << "\n";
    EV_INFO << "Duplicates Skipped: " << duplicatesSkipped << "\n";
    EV_INFO << "Messages Sent     : " << messagesSent      << "\n";
    EV_INFO << "Returned to Home  : " << (isReturningHome ? "YES" : "NO") << "\n";

    if (uavId == 0) {
        EV_INFO << "\n=== Network Summary ===\n";
        EV_INFO << "Total Mines  : " << mineField->getNumMines()       << "\n";
        EV_INFO << "Mines Found  : " << mineField->getDiscoveredCount()<< "\n";
        EV_INFO << "Detection Rate: "
                << (double)mineField->getDiscoveredCount()
                   / mineField->getNumMines() * 100.0 << "%\n";
    }

    recordScalar("trueDetections",   trueDetections);
    recordScalar("falseAlarms",      falseAlarms);
    recordScalar("duplicatesSkipped",duplicatesSkipped);
    recordScalar("returnedHome",     isReturningHome ? 1.0 : 0.0);
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
    if (scanTimer && scanTimer->isScheduled())       cancelEvent(scanTimer);
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
