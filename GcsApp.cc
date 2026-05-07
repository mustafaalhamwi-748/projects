#include "GcsApp.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/packet/Packet.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/networklayer/common/L3Address.h"

namespace uavminedetection {

Define_Module(GcsApp);

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

        EV_INFO << "Smart GCS Initialized. Listening on port " << localPort << "\n";
    }
}

void GcsApp::handleMessageWhenUp(cMessage *msg)
{
    if (msg->arrivedOn("socketIn")) socket.processMessage(msg);
    else delete msg;
}

// ============================================================
// مساعد — فحص التكرار
// ============================================================
bool GcsApp::isDuplicate(const std::vector<inet::Coord>& map,
                         const inet::Coord& pos, double threshold) const
{
    for (const auto& m : map)
        if (m.distance(pos) < threshold)
            return true;
    return false;
}

// ============================================================
// socketDataArrived
// [تم التصحيح]:
//   1. خريطتان منفصلتان — realMineMap و falseAlarmMap
//   2. فحص التكرار على الخريطة الصحيحة فقط
//   3. الأوامر ترسل على نفس البورت 5555
// ============================================================
void GcsApp::socketDataArrived(UdpSocket *, Packet *pkt)
{
    auto payload = pkt->peekData<BytesChunk>();
    std::string msgText(
        reinterpret_cast<const char*>(payload->getBytes().data()),
        payload->getBytes().size());

    // تجاهل الرسائل التي لا تهم GCS
    bool isRealMine  = (msgText.find("CONFIRMED_REAL") != std::string::npos);
    bool isFalseAlarm= (msgText.find("CONFIRMED_FA")   != std::string::npos);

    if (!isRealMine && !isFalseAlarm) { delete pkt; return; }

    // استخراج الإحداثيات
    // صيغة الرسالة: TYPE:uav=N,x=X.X,y=Y.Y,conf=C.CC,magVal=M.M,t=T.TT
    size_t xPos    = msgText.find(",x=");
    size_t yPos    = msgText.find(",y=");
    size_t confPos = msgText.find(",conf=");   // [تم التصحيح]: كان ",conf" بدون "="
    // magPos حُذف — confPos يكفي لتحديد نهاية y في السطر 81

    if (xPos == std::string::npos || yPos == std::string::npos ||
        confPos == std::string::npos) {
        delete pkt; return;
    }

    try {
        double x = std::stod(msgText.substr(xPos + 3, yPos  - xPos  - 3));
        double y = std::stod(msgText.substr(yPos + 3, confPos - yPos - 3));
        Coord newPos(x, y, 0);

        if (isRealMine) {
            // [تم التصحيح]: تحقق من التكرار في خريطة الألغام الحقيقية فقط
            if (!isDuplicate(realMineMap, newPos)) {
                realMineMap.push_back(newPos);
                EV_INFO << "GCS: REAL Mine #" << realMineMap.size()
                        << " at (" << x << ", " << y << ")\n";
                sendCommandToSwarm(x, y);
            }
        } else {
            // [تم التصحيح]: تحقق من التكرار في خريطة الإنذارات الكاذبة فقط
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

// ============================================================
// sendCommandToSwarm
// [تم التصحيح الجذري]:
//   المشكلة القديمة: broadcast 255.255.255.255 لا يصل في INET WiFi
//   الحل: unicast مباشر لكل طائرة بعنوانها الحقيقي
// ============================================================
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

// ============================================================
// refreshDisplay
// [تم التصحيح]: يعرض أعداداً صحيحة من الخريطتين المنفصلتين
// ============================================================
void GcsApp::refreshDisplay() const
{
    char buf[120];
    snprintf(buf, sizeof(buf),
             "Real Mines: %zu | False Alarms: %zu",
             realMineMap.size(), falseAlarmMap.size());

    getParentModule()->getDisplayString().setTagArg("t", 0, buf);
    getParentModule()->getDisplayString().setTagArg("i", 1, "red");
}

// ============================================================
// finish
// ============================================================
void GcsApp::finish()
{
    EV_INFO << "\n=== GCS Final Statistics ===\n";
    EV_INFO << "Total Real Mines Confirmed : " << realMineMap.size()   << "\n";
    EV_INFO << "Total False Alarms         : " << falseAlarmMap.size() << "\n";

    recordScalar("gcs_realMines",   (double)realMineMap.size());
    recordScalar("gcs_falseAlarms", (double)falseAlarmMap.size());
}

} // namespace uavminedetection
