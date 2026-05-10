#include "GcsApp.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/packet/Packet.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/networklayer/common/L3Address.h"

namespace uavminedetection {

Define_Module(GcsApp);

simsignal_t GcsApp::globalCoverageSignal = registerSignal("globalCoverage");

// ── الهادم لتنظيف آمن للذاكرة ──
GcsApp::~GcsApp()
{
    if (coordinationTimer) {
        cancelAndDelete(coordinationTimer);
        coordinationTimer = nullptr;
    }
}

void GcsApp::initialize(int stage)
{
    ApplicationBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        localPort = par("localPort");
        destPort  = par("destPort");
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        socket.setOutputGate(gate("socketOut"));
        socket.bind(localPort);
        socket.setBroadcast(true);
        socket.setCallback(this);

        coordinationTimer = new cMessage("coordinationTimer");
        scheduleAt(simTime() + 10.0, coordinationTimer);

        EV_INFO << "Smart GCS Initialized. Listening on port " << localPort << "\n";
    }
}

void GcsApp::handleMessageWhenUp(cMessage *msg)
{
    if (msg == coordinationTimer) {
        coordinateSwarm();
        scheduleAt(simTime() + 10.0, coordinationTimer);
    }
    else if (msg->arrivedOn("socketIn")) {
        socket.processMessage(msg);
    }
    else delete msg;
}

bool GcsApp::isDuplicate(const std::vector<inet::Coord>& map,
                         const inet::Coord& pos, double threshold) const
{
    for (const auto& m : map)
        if (m.distance(pos) < threshold)
            return true;
    return false;
}

void GcsApp::socketDataArrived(UdpSocket *, Packet *pkt)
{
    auto payload = pkt->peekData<BytesChunk>();
    std::string msgText(
        reinterpret_cast<const char*>(payload->getBytes().data()),
        payload->getBytes().size());

    if (msgText.find("STATUS:uav=") == 0) {
        size_t uavPos = msgText.find("uav=");
        size_t xPos   = msgText.find(",x=");
        size_t yPos   = msgText.find(",y=");
        size_t covPos = msgText.find(",cov=");
        size_t statePos= msgText.find(",state=");

        if (uavPos != std::string::npos && xPos != std::string::npos &&
            covPos != std::string::npos && statePos != std::string::npos)
        {
            try {
                int uavId = std::stoi(msgText.substr(uavPos + 4, xPos - uavPos - 4));
                double x = std::stod(msgText.substr(xPos + 3, yPos - xPos - 3));
                double y = std::stod(msgText.substr(yPos + 3, covPos - yPos - 3));
                double cov = std::stod(msgText.substr(covPos + 5, statePos - covPos - 5));
                std::string state = msgText.substr(statePos + 7);

                swarmStatus[uavId] = {Coord(x, y, 0), cov, state, simTime()};

                const int G = 50;
                const double S = 1000.0 / G;
                int col = std::max(0, std::min(G - 1, (int)(x / S)));
                int row = std::max(0, std::min(G - 1, (int)(y / S)));
                globalVisitedCells.insert(col * G + row);
                emit(globalCoverageSignal, (double)globalVisitedCells.size() / (G * G) * 100.0);

                if (state == "SPIRAL") {
                    Coord currentPos(x, y, 0);
                    if (isDuplicate(realMineMap, currentPos, 50.0) || isDuplicate(falseAlarmMap, currentPos, 50.0)) {
                        EV_INFO << "GCS: UAV[" << uavId << "] is performing SPIRAL in an already confirmed area. Sending CANCEL_SPIRAL.\n";
                        sendCancelSpiral(uavId);
                    }
                }

            } catch (...) {}
        }
        delete pkt;
        return;
    }

    bool isRealMine  = (msgText.find("CONFIRMED_REAL") != std::string::npos) ||
                       (msgText.find("SPIRAL_PEAK")    != std::string::npos);
    bool isFalseAlarm= (msgText.find("CONFIRMED_FA")   != std::string::npos);

    if (!isRealMine && !isFalseAlarm) { delete pkt; return; }

    size_t xPos    = msgText.find(",x=");
    size_t yPos    = msgText.find(",y=");
    size_t confPos = msgText.find(",conf=");

    if (xPos == std::string::npos || yPos == std::string::npos ||
        confPos == std::string::npos) {
        delete pkt; return;
    }

    try {
        double x = std::stod(msgText.substr(xPos + 3, yPos  - xPos  - 3));
        double y = std::stod(msgText.substr(yPos + 3, confPos - yPos - 3));
        Coord newPos(x, y, 0);

        if (isRealMine) {
            if (!isDuplicate(realMineMap, newPos)) {
                realMineMap.push_back(newPos);
                EV_INFO << "GCS: REAL Mine #" << realMineMap.size()
                        << " at (" << x << ", " << y << ")\n";
                if (msgText.find("SPIRAL_PEAK") == std::string::npos) {
                    sendCommandToSwarm(x, y);
                }
            }
        } else {
            if (!isDuplicate(falseAlarmMap, newPos)) {
                falseAlarmMap.push_back(newPos);
                EV_INFO << "GCS: False Alarm #" << falseAlarmMap.size()
                        << " at (" << x << ", " << y << ")\n";
            }
        }
    } catch (...) {
        EV_WARN << "GCS: Failed to parse message coordinates.\n";
    }

    delete pkt;
}

void GcsApp::coordinateSwarm()
{
    for (auto const& pair : swarmStatus) {
        int uavId = pair.first;
        const UavStatus& status = pair.second;

        if (status.state == "RTH" && status.coveragePercent < 90.0) {
            EV_INFO << "GCS: Detected UAV[" << uavId << "] returning home early with only "
                    << status.coveragePercent << "% coverage!\n";

            int bestHelper = -1;
            double minDistance = 999999.0;

            for (auto const& hPair : swarmStatus) {
                int helperId = hPair.first;
                const UavStatus& hStatus = hPair.second;

                if (helperId != uavId && hStatus.state == "SCAN") {
                    double dist = hStatus.lastPos.distance(status.lastPos);
                    if (dist < minDistance) {
                        minDistance = dist;
                        bestHelper = helperId;
                    }
                }
            }

            if (bestHelper != -1) {
                EV_INFO << "GCS: Dispatching UAV[" << bestHelper << "] to assist in area of UAV["
                        << uavId << "].\n";

                char buf[128];
                snprintf(buf, sizeof(buf), "CMD:REDIRECT,x=%.1f,y=%.1f", status.lastPos.x, status.lastPos.y);

                try {
                    std::string uavName = "uav[" + std::to_string(bestHelper) + "]";
                    L3Address addr;
                    L3AddressResolver().tryResolve(uavName.c_str(), addr);
                    if (!addr.isUnspecified()) {
                        auto payload = makeShared<BytesChunk>();
                        std::vector<uint8_t> bytes(buf, buf + strlen(buf));
                        payload->setBytes(bytes);
                        auto *pkt = new Packet("RedirectCmd", payload);
                        socket.sendTo(pkt, addr, destPort);
                    }
                } catch (...) {}

                swarmStatus[uavId].coveragePercent = 100.0;
            }
        }
    }
}

void GcsApp::sendCancelSpiral(int uavId)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "CMD:CANCEL_SPIRAL");

    try {
        std::string uavName = "uav[" + std::to_string(uavId) + "]";
        L3Address addr;
        L3AddressResolver().tryResolve(uavName.c_str(), addr);
        if (!addr.isUnspecified()) {
            auto payload = makeShared<BytesChunk>();
            std::vector<uint8_t> bytes(buf, buf + strlen(buf));
            payload->setBytes(bytes);
            auto *pkt = new Packet("CancelSpiralCmd", payload);
            socket.sendTo(pkt, addr, destPort);
        }
    } catch (...) {}
}

void GcsApp::sendCommandToSwarm(double x, double y)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "CMD:INTENSIVE_SEARCH,x=%.1f,y=%.1f", x, y);

    cModule *net = getParentModule()->getParentModule();
    int nUAV = net->par("numUAVs");
    int sent = 0;

    for (int i = 0; i < nUAV; i++) {
        try {
            std::string uavName = "uav[" + std::to_string(i) + "]";
            L3Address addr;
            L3AddressResolver().tryResolve(uavName.c_str(), addr);
            if (!addr.isUnspecified()) {
                auto payload = makeShared<BytesChunk>();
                std::vector<uint8_t> bytes(buf, buf + strlen(buf));
                payload->setBytes(bytes);
                auto *pkt = new Packet("GcsCommand", payload);
                socket.sendTo(pkt, addr, destPort);
                sent++;
            }
        } catch (...) {}
    }

    EV_INFO << "GCS: Sent INTENSIVE_SEARCH to " << sent
            << " UAVs at (" << x << ", " << y << ")\n";
}

void GcsApp::refreshDisplay() const
{
    char buf[120];
    snprintf(buf, sizeof(buf),
             "Real Mines: %zu | False Alarms: %zu",
             realMineMap.size(), falseAlarmMap.size());

    getParentModule()->getDisplayString().setTagArg("t", 0, buf);
    getParentModule()->getDisplayString().setTagArg("i", 1, "red");
}

void GcsApp::finish()
{
    EV_INFO << "\n=== GCS Final Statistics ===\n";
    EV_INFO << "Total Real Mines Confirmed : " << realMineMap.size()   << "\n";
    EV_INFO << "Total False Alarms         : " << falseAlarmMap.size() << "\n";

    recordScalar("gcs_realMines",   (double)realMineMap.size());
    recordScalar("gcs_falseAlarms", (double)falseAlarmMap.size());
}

void GcsApp::handleStartOperation(LifecycleOperation *op)
{
    if (coordinationTimer) cancelEvent(coordinationTimer);
    if (!coordinationTimer) coordinationTimer = new cMessage("coordinationTimer");
    scheduleAt(simTime() + 10.0, coordinationTimer);
}

void GcsApp::handleStopOperation(LifecycleOperation *op)
{
    if (coordinationTimer && coordinationTimer->isScheduled()) cancelEvent(coordinationTimer);
    socket.close();
}

void GcsApp::handleCrashOperation(LifecycleOperation *op)
{
    if (coordinationTimer && coordinationTimer->isScheduled()) cancelEvent(coordinationTimer);
    socket.destroy();
}

} // namespace uavminedetection
