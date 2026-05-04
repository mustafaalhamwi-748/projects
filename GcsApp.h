#ifndef __UAVMINEDETECTION_GCSAPP_H
#define __UAVMINEDETECTION_GCSAPP_H

#include <vector>
#include <string>
#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "inet/common/geometry/common/Coord.h"

using namespace inet;

namespace uavminedetection {

class GcsApp : public ApplicationBase, public UdpSocket::ICallback
{
  protected:
    int localPort;
    int destPort;
    UdpSocket socket;

    // [تم التصحيح: خريطتان منفصلتان بدلاً من خريطة واحدة]
    std::vector<inet::Coord> realMineMap;      // الألغام الحقيقية فقط
    std::vector<inet::Coord> falseAlarmMap;    // الإنذارات الكاذبة فقط

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void refreshDisplay() const override;
    virtual void finish() override;

    virtual void socketDataArrived(UdpSocket *socket, Packet *packet) override;
    virtual void socketErrorArrived(UdpSocket *socket, Indication *indication) override { delete indication; }
    virtual void socketClosed(UdpSocket *socket) override {}

    virtual void handleStartOperation(LifecycleOperation *op) override {}
    virtual void handleStopOperation(LifecycleOperation *op) override { socket.close(); }
    virtual void handleCrashOperation(LifecycleOperation *op) override { socket.destroy(); }

    void sendCommandToSwarm(double x, double y);

    // مساعد: هل الموقع موجود مسبقاً في الخريطة المعطاة؟
    bool isDuplicate(const std::vector<inet::Coord>& map, const inet::Coord& pos, double threshold = 50.0) const;
};

} // namespace uavminedetection
#endif
