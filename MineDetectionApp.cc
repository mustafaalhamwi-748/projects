#include "LawnmowerMobility.h"
#include "MineDetectionApp.h"
#include "inet/common/ModuleAccess.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/TagBase_m.h"

#include <cmath>
#include <string>
#include <algorithm>

using namespace inet;

namespace uavminedetection {

Define_Module(MineDetectionApp);

simsignal_t MineDetectionApp::detectionSignal  = registerSignal("mineDetected");
simsignal_t MineDetectionApp::coverageSignal   = registerSignal("areaCoverage");
simsignal_t MineDetectionApp::falseAlarmSignal = registerSignal("falseAlarm");

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
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        mineField = check_and_cast<MineField*>(getSystemModule()->getSubmodule("mineField"));
        mobility = check_and_cast<IMobility*>(getParentModule()->getSubmodule("mobility"));

        socket.setOutputGate(gate("socketOut"));
        socket.bind(localPort + uavId);
        socket.setCallback(this);

        initSensorVisuals();

        scanTimer = new cMessage("scanTimer");
        intensiveTimer = new cMessage("intensiveTimer"); // إنشاء مؤقت حالة التأهب

        scheduleAt(simTime() + uniform(0, scanInterval), scanTimer);
    }
}

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

void MineDetectionApp::updateSensorVisuals(double magVal, double uavX, double uavY) const
{
    if (!sensorRingFigure || !sensorValueFigure) return;

    cFigure::Color color = getMagneticColor(magVal);
    double R = getMagneticRadius(magVal);

    sensorRingFigure->setBounds(cFigure::Rectangle(uavX-R, uavY-R, 2*R, 2*R));

    // ── التعديل البصري عند الاستجابة للـ GCS ──
    if (isIntensiveMode) {
        sensorRingFigure->setFillColor(cFigure::Color(255, 0, 0)); // لون الخطر (أحمر ناصع)
        sensorRingFigure->setLineColor(cFigure::Color(255, 0, 0));
        sensorRingFigure->setLineWidth(4); // سماكة أكبر
        sensorRingFigure->setFillOpacity(0.5);

        char buf[64];
        snprintf(buf, sizeof(buf), "🔴 ALERT %.0fnT", magVal);
        sensorValueFigure->setText(buf);
        sensorValueFigure->setColor(cFigure::Color(255, 0, 0));
    } else {
        // الحالة الطبيعية
        sensorRingFigure->setFillColor(color);
        sensorRingFigure->setLineColor(color);
        sensorRingFigure->setLineWidth(2);
        sensorRingFigure->setFillOpacity((magVal >= magneticThreshold) ? 0.35 : 0.18);

        char buf[32];
        if (magVal >= magneticThreshold) snprintf(buf, sizeof(buf), "⚠ %.0f nT", magVal);
        else snprintf(buf, sizeof(buf), "%.0f nT", magVal);
        sensorValueFigure->setText(buf);
        sensorValueFigure->setColor(color);
    }
    // ──────────────────────────────────────────

    sensorValueFigure->setPosition(cFigure::Point(uavX, uavY - 18));

    if (sensorBarBg && sensorBarFg) {
        sensorBarBg->setBounds(cFigure::Rectangle(uavX-12, uavY+14, 24, 4));
        double fillRatio = std::min(1.0, magVal / (1.5 * magneticThreshold));
        sensorBarFg->setBounds(cFigure::Rectangle(uavX-12, uavY+14, std::max(1.0, fillRatio * 24.0), 4));
        sensorBarFg->setFillColor(isIntensiveMode ? cFigure::Color(255,0,0) : color);
    }
}

void MineDetectionApp::checkTimeouts()
{
    auto it = candidateMines.begin();
    while (it != candidateMines.end()) {
        if (it->firstDetectorId == uavId && (simTime() - it->firstDetectedTime).dbl() > confirmationTimeout) {
            EV_INFO << "UAV[" << uavId << "] TIMEOUT! Auto-confirming target at (" << it->pos.x << "," << it->pos.y << ").\n";
            confirmTarget(it->pos, it->confidence, it->magVal);
            it = candidateMines.erase(it);
        } else {
            ++it;
        }
    }
}

void MineDetectionApp::confirmTarget(inet::Coord targetPos, double confidence, double magVal)
{
    sharedMemory.push_back(targetPos);

    int mineIdx = mineField->getNearestUndiscoveredMine(targetPos.x, targetPos.y, confirmRadius);
    if (mineIdx >= 0) {
        mineField->markDiscovered(mineIdx);
        trueDetections++;
        emit(detectionSignal, 1L);
        EV_INFO << "UAV[" << uavId << "] MINE CONFIRMED at (" << targetPos.x << "," << targetPos.y << ")\n";
        sendNetworkMessage("CONFIRMED_REAL", targetPos.x, targetPos.y, confidence, magVal);
    } else {
        int debrisIdx = mineField->getNearestMetalDebris(targetPos.x, targetPos.y, confirmRadius);
        if (debrisIdx >= 0) {
            mineField->markDebrisTriggered(debrisIdx);
            falseAlarms++;
            emit(falseAlarmSignal, 1L);
            const auto& db = mineField->getDebris();
            addFalseAlarmFigure(db[debrisIdx].x, db[debrisIdx].y);
            EV_INFO << "UAV[" << uavId << "] FALSE ALARM CONFIRMED at (" << targetPos.x << "," << targetPos.y << ")\n";
            sendNetworkMessage("CONFIRMED_FA", targetPos.x, targetPos.y, confidence, magVal);
        } else {
            sendNetworkMessage("CONFIRMED_FA", targetPos.x, targetPos.y, confidence, magVal);
        }
    }
}

void MineDetectionApp::performScan()
{
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

                // ── هنا نستفيد من قيم الطائرة الأولى المخزنة بشكل صحيح! ──
                if (it->firstDetectorId != uavId) {
                    EV_INFO << "UAV[" << uavId << "] CROSS-VALIDATED target from UAV[" << it->firstDetectorId
                            << "] (Original Conf: " << it->confidence << ")\n";
                    confirmTarget(it->pos, reading.confidence, reading.magneticValue);
                    candidateMines.erase(it);
                } else {
                    duplicatesSkipped++;
                }
                break;
            }
        }

        if (!isAlreadyCandidate) {
            CandidateMine newCand = {pos, simTime(), uavId, reading.confidence, reading.magneticValue};
            candidateMines.push_back(newCand);
            sendNetworkMessage("CANDIDATE", pos.x, pos.y, reading.confidence, reading.magneticValue);
            EV_INFO << "UAV[" << uavId << "] FOUND CANDIDATE at (" << pos.x << "," << pos.y << "). Waiting for confirmation.\n";
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
    char buf[100];
    snprintf(buf, sizeof(buf), "UAV%d | %.0f nT | found=%d | FA=%d | Pend=%zu",
             uavId, lastMagneticValue, trueDetections, falseAlarms, candidateMines.size());
    getParentModule()->getDisplayString().setTagArg("t", 0, buf);
}

void MineDetectionApp::sendNetworkMessage(const char* type, double x, double y, double confidence, double magneticValue)
{
    char buf[160];
    snprintf(buf, sizeof(buf), "%s:uav=%d,x=%.1f,y=%.1f,conf=%.2f,magVal=%.1f,t=%.2f",
             type, uavId, x, y, confidence, magneticValue, simTime().dbl());

    auto payload = makeShared<BytesChunk>();
    std::vector<uint8_t> bytes(buf, buf + strlen(buf));
    payload->setBytes(bytes);
    auto *pkt = new Packet("MineReport", payload);

    L3Address dest;
    L3AddressResolver().tryResolve("gcs", dest);
    if (!dest.isUnspecified()) socket.sendTo(pkt->dup(), dest, destPort);

    L3Address broadcast("255.255.255.255");
    socket.sendTo(pkt, broadcast, destPort);

    messagesSent++;
}

double MineDetectionApp::calculateCoverage()
{
    const int G = 50; const double AREA = 1000.0;
    int cov = 0; cModule *net = getSystemModule();
    int nUAV = net->par("numUAVs");
    const double sensorCovRange = 10.0;

    for (int i = 0; i < G; i++) {
        for (int j = 0; j < G; j++) {
            double px = (i + 0.5) * (AREA / G); double py = (j + 0.5) * (AREA / G);
            for (int u = 0; u < nUAV; u++) {
                IMobility *mob = check_and_cast<IMobility*>(net->getSubmodule("uav", u)->getSubmodule("mobility"));
                if (mob->getCurrentPosition().distance(Coord(px, py, 0)) <= sensorCovRange) {
                    cov++; break;
                }
            }
        }
    }
    return (double)cov / (G*G) * 100.0;
}

void MineDetectionApp::addFalseAlarmFigure(double x, double y)
{
    if (falseAlarmDisplayLimit >= 0 && falseAlarmFigureCount >= falseAlarmDisplayLimit) return;
    cCanvas *canvas = getSystemModule()->getCanvas();
    std::string name = "fa_" + std::to_string(uavId) + "_" + std::to_string(falseAlarmFigureCount++);
    auto *grp = new cGroupFigure(name.c_str());

    auto *tri = new cPolygonFigure("tri");
    tri->setPoints({{x, y-9}, {x-8, y+6}, {x+8, y+6}});
    tri->setFilled(true); tri->setFillColor(cFigure::Color(255, 220, 0));
    grp->addFigure(tri);

    auto *lbl = new cTextFigure("lbl");
    lbl->setPosition(cFigure::Point(x, y+2)); lbl->setText("!");
    lbl->setColor(cFigure::Color("black")); lbl->setAnchor(cFigure::ANCHOR_CENTER);
    grp->addFigure(lbl);
    canvas->addFigure(grp);
}

// ── الاستجابة لأمر المحطة الأرضية (التأهب) ──
void MineDetectionApp::startIntensiveSearch(double cmdX, double cmdY)
{
    Coord pos = mobility->getCurrentPosition();
    // إذا كانت الطائرة قريبة (ضمن 200م)، تبدأ المسح الحلزوني!
    if (pos.distance(Coord(cmdX, cmdY, 0)) < 200.0) {
        isIntensiveMode = true;
        EV_INFO << "UAV[" << uavId << "] Entering SPIRAL ALERT mode at (" << cmdX << "," << cmdY << ")\n";

        // استدعاء دالة الحلزون في نموذج الحركة
        auto lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
        if (lawnmower) {
            lawnmower->startSpiral(cmdX, cmdY);
        }

        cancelEvent(intensiveTimer);
        scheduleAt(simTime() + 20.0, intensiveTimer); // حلزون لمدة 20 ثانية
    }
}

void MineDetectionApp::handleMessageWhenUp(cMessage *msg)
{
    if (msg == scanTimer) {
        performScan();
    }
    else if (msg == intensiveTimer) {
        isIntensiveMode = false;
        EV_INFO << "UAV[" << uavId << "] Leaving spiral mode, resuming normal scan.\n";

        // إيقاف الحلزون والعودة للمسح العادي
        auto lawnmower = dynamic_cast<LawnmowerMobility*>(mobility);
        if (lawnmower) {
            lawnmower->stopSpiral();
        }
    }
    else if (msg->arrivedOn("socketIn")) {
        socket.processMessage(msg);
    }
    else {
        delete msg;
    }
}

void MineDetectionApp::socketDataArrived(UdpSocket *, Packet *pkt)
{
    auto payload = pkt->peekData<BytesChunk>();
    const auto& bytes = payload->getBytes();
    std::string msgText(reinterpret_cast<const char*>(bytes.data()), bytes.size());

    // ── 1. معالجة أوامر المحطة الأرضية (حل المشكلة 2) ──
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
        delete pkt;
        return;
    }

    // ── 2. معالجة رسائل الطائرات واستخراج القيم الدقيقة (حل المشكلة 1) ──
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
            int senderUav = std::stoi(msgText.substr(uavPos + 4, xPos - uavPos - 4));
            double parsedX = std::stod(msgText.substr(xPos + 3, yPos - xPos - 3));
            double parsedY = std::stod(msgText.substr(yPos + 3, confPos - yPos - 3));

            // استخراج القيم الحقيقية بدقة!
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
                    for (auto it = candidateMines.begin(); it != candidateMines.end(); ) {
                        if (it->pos.distance(incomingPos) < confirmRadius) it = candidateMines.erase(it);
                        else ++it;
                    }
                }
            }
        } catch (...) {}
    }
    delete pkt;
}

void MineDetectionApp::socketErrorArrived(UdpSocket *, Indication *ind) { delete ind; }

void MineDetectionApp::finish()
{
    EV_INFO << "\n=== UAV[" << uavId << "] Final Statistics ===\n";
    EV_INFO << "True Detections   : " << trueDetections << "\n";
    EV_INFO << "False Alarms      : " << falseAlarms << "\n";
    EV_INFO << "Duplicates Skipped: " << duplicatesSkipped << "\n";
    EV_INFO << "Messages Sent     : " << messagesSent << "\n";

    if (uavId == 0) {
        EV_INFO << "\n=== Network Summary ===\n";
        EV_INFO << "Total Mines     : " << mineField->getNumMines() << "\n";
        EV_INFO << "Mines Found     : " << mineField->getDiscoveredCount() << "\n";
        EV_INFO << "Detection Rate  : " << (double)mineField->getDiscoveredCount()/mineField->getNumMines()*100.0 << "%\n";
    }

    recordScalar("trueDetections", trueDetections);
    recordScalar("falseAlarms", falseAlarms);
    recordScalar("duplicatesSkipped", duplicatesSkipped);
}

void MineDetectionApp::handleStartOperation(LifecycleOperation *) {
    if (!scanTimer) scanTimer = new cMessage("scanTimer");
    scheduleAt(simTime() + uniform(0, scanInterval), scanTimer);
}
void MineDetectionApp::handleStopOperation(LifecycleOperation *) {
    cancelEvent(scanTimer);
    cancelEvent(intensiveTimer);
    socket.close();
}
void MineDetectionApp::handleCrashOperation(LifecycleOperation *) {
    cancelEvent(scanTimer);
    cancelEvent(intensiveTimer);
    socket.destroy();
}

} // namespace uavminedetection
