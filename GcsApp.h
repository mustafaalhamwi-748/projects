#ifndef __UAVMINEDETECTION_GCSAPP_H
#define __UAVMINEDETECTION_GCSAPP_H

#include <vector>
#include <string>
#include <map>
#include <set>
#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "inet/common/geometry/common/Coord.h"

using namespace inet;

namespace uavminedetection {

struct UavStatus {
    inet::Coord lastPos;
    double coveragePercent;
    std::string state;      // "SCAN", "SPIRAL", "RTH"
    simtime_t lastUpdate;
};

class GcsApp : public ApplicationBase, public UdpSocket::ICallback
{
  public:
    virtual ~GcsApp(); // الهادم لتنظيف الذاكرة بأمان

  protected:
    int localPort;
    int destPort;
    UdpSocket socket;

    std::vector<inet::Coord> realMineMap;
    std::vector<inet::Coord> falseAlarmMap;

    std::map<int, UavStatus> swarmStatus;

    cMessage *coordinationTimer = nullptr;

    std::set<int> globalVisitedCells;
    static simsignal_t globalCoverageSignal;

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void refreshDisplay() const override;
    virtual void finish() override;

    virtual void socketDataArrived(UdpSocket *socket, Packet *packet) override;
    virtual void socketErrorArrived(UdpSocket *socket, Indication *indication) override { delete indication; }
    virtual void socketClosed(UdpSocket *socket) override {}

    virtual void handleStartOperation(LifecycleOperation *op) override;
    virtual void handleStopOperation(LifecycleOperation *op) override;
    virtual void handleCrashOperation(LifecycleOperation *op) override;

    void sendCommandToSwarm(double x, double y);
    void coordinateSwarm();

    void sendCancelSpiral(int uavId);

    bool isDuplicate(const std::vector<inet::Coord>& map, const inet::Coord& pos, double threshold = 50.0) const;
};

} // namespace uavminedetection
#endif
