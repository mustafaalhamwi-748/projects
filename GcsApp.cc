#include "GcsApp.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/packet/Packet.h"

namespace uavminedetection {

Define_Module(GcsApp);

void GcsApp::initialize(int stage)
{
    ApplicationBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        localPort = par("localPort");
        destPort = par("destPort");
        totalFalseAlarms = 0;
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        socket.setOutputGate(gate("socketOut"));
        socket.bind(localPort);
        socket.setBroadcast(true);
        socket.setCallback(this);

        EV_INFO << "Smart GCS Initialized. Listening on port " << localPort << "\n";
    }
}

void GcsApp::handleMessageWhenUp(cMessage *msg)
{
    if (msg->arrivedOn("socketIn")) socket.processMessage(msg);
    else delete msg;
}

void GcsApp::socketDataArrived(UdpSocket *, Packet *pkt)
{
    auto payload = pkt->peekData<BytesChunk>();
    std::string msgText(reinterpret_cast<const char*>(payload->getBytes().data()), payload->getBytes().size());

    bool isRealMine = (msgText.find("CONFIRMED_REAL") != std::string::npos);
    bool isFalseAlarm = (msgText.find("CONFIRMED_FA") != std::string::npos);

    if (isRealMine || isFalseAlarm) {
        size_t xPos = msgText.find(",x=");
        size_t yPos = msgText.find(",y=");
        size_t confPos = msgText.find(",conf");

        if (xPos != std::string::npos && yPos != std::string::npos) {
            try {
                double x = std::stod(msgText.substr(xPos + 3, yPos - xPos - 3));
                double y = std::stod(msgText.substr(yPos + 3, confPos - yPos - 3));
                Coord newMine(x, y, 0);

                bool alreadyExists = false;
                for (const auto& m : globalMineMap) {
                    if (m.distance(newMine) < 50.0) { alreadyExists = true; break; }
                }

                if (!alreadyExists) {
                    globalMineMap.push_back(newMine);

                    if (isRealMine) {
                        EV_INFO << "GCS: REAL Mine Registered at (" << x << ", " << y << ")\n";
                    } else {
                        totalFalseAlarms++;
                        EV_INFO << "GCS: False Alarm Registered at (" << x << ", " << y << ")\n";
                    }

                    sendCommandToSwarm(x, y);
                }
            } catch (...) {}
        }
    }
    delete pkt;
}

void GcsApp::sendCommandToSwarm(double x, double y)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "CMD:INTENSIVE_SEARCH,x=%.1f,y=%.1f", x, y);
    auto payload = makeShared<BytesChunk>();
    std::vector<uint8_t> bytes(buf, buf + strlen(buf));
    payload->setBytes(bytes);
    auto *pkt = new Packet("GcsCommand", payload);

    L3Address dest("255.255.255.255");
    socket.sendTo(pkt, dest, destPort);
}

void GcsApp::refreshDisplay() const
{
    char buf[100];
    int realMinesCount = globalMineMap.size() - totalFalseAlarms;
    snprintf(buf, sizeof(buf), "Real Mines: %d | False Alarms: %d", realMinesCount, totalFalseAlarms);

    getParentModule()->getDisplayString().setTagArg("t", 0, buf);
    getParentModule()->getDisplayString().setTagArg("i", 1, "red");
}

void GcsApp::finish()
{
    int realMinesCount = globalMineMap.size() - totalFalseAlarms;
    EV_INFO << "\n=== GCS Final Statistics ===\n";
    EV_INFO << "Total Real Mines: " << realMinesCount << "\n";
    EV_INFO << "Total False Alarms (Debris): " << totalFalseAlarms << "\n";
}

} // namespace uavminedetection
