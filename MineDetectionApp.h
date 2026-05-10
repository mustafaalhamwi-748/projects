#ifndef __UAVMINEDETECTION_MINEDETECTIONAPP_H
#define __UAVMINEDETECTION_MINEDETECTIONAPP_H

#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "inet/mobility/contract/IMobility.h"
#include "inet/common/geometry/common/Coord.h"
#include "inet/environment/contract/IPhysicalEnvironment.h"
#include "inet/common/packet/Packet.h"
#include "inet/networklayer/common/L3Address.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/power/contract/IEpEnergyStorage.h"

#include <omnetpp.h>
#include <set>

#include "MineField.h"
#include "MagnetometerSensor.h"

using namespace omnetpp;
using namespace inet;

namespace uavminedetection {

struct CandidateMine {
    inet::Coord pos;
    simtime_t firstDetectedTime;
    int firstDetectorId;
    double confidence;
    double magVal;
};

class MineDetectionApp : public ApplicationBase, public UdpSocket::ICallback
{
  public:
    virtual ~MineDetectionApp(); // الهادم لتنظيف الذاكرة بأمان

  protected:
    int uavId = -1;
    double scanInterval = 0.5;

    double magneticThreshold  = 0;
    double magneticSaturation = 0;
    double falseAlarmProb     = 0;
    int    falseAlarmDisplayLimit = 30;
    double confirmRadius = 25.0;

    int    adaptiveWindowSize   = 50;
    double adaptiveKSigma       = 3.0;
    double adaptiveMinThreshold = 200.0;

    double sensorNoiseAlpha = 0.8;
    double sensorNoiseScale = 30.0;
    double correlatedNoise  = 0.0;

    inet::power::IEpEnergyStorage *energyStorage = nullptr;
    double returnHomeEnergyThreshold = 0.3;

    int destPort  = -1;
    int localPort = -1;
    UdpSocket socket;

    inet::L3Address gcsAddress;

    MineField         *mineField = nullptr;
    MagnetometerSensor *sensor   = nullptr;
    IMobility         *mobility  = nullptr;

    cMessage *scanTimer       = nullptr;
    cMessage *intensiveTimer  = nullptr;
    cMessage *statusTimer     = nullptr;

    bool isIntensiveMode  = false;
    bool isReturningHome  = false;

    double maxSpiralMagVal = -1.0;
    inet::Coord maxSpiralPos;
    double spiralStopRatio = 1.2;

    static constexpr double GCS_X              = 500.0;
    static constexpr double GCS_Y              = 950.0;
    static constexpr double CRUISE_ALTITUDE    = 80.0;
    // [FIX]: 30م→50م: عند 30م حطام WIRE/CAN/TOOL يعطي 277-442nT → إنذارات كاذبة كثيرة
    //        عند 50م: حطام أقصاه 160nT (آمن) | ألغام 900-1800nT (مرئية جداً)
    static constexpr double SCAN_ALTITUDE      = 50.0;
    static constexpr double STATUS_INTERVAL    = 5.0;
    static constexpr double RETURN_HOME_BEFORE = 50.0;

    std::vector<inet::Coord>   sharedMemory;
    std::vector<CandidateMine> candidateMines;

    std::set<int> visitedCells;

    int    trueDetections   = 0;
    int    falseAlarms      = 0;
    int    duplicatesSkipped= 0;
    int    messagesSent     = 0;
    double lastMagneticValue= 0.0;
    int    falseAlarmFigureCount = 0;

    double confirmationTimeout = 30.0;

    cOvalFigure     *sensorRingFigure  = nullptr;
    cTextFigure     *sensorValueFigure = nullptr;
    cRectangleFigure *sensorBarBg      = nullptr;
    cRectangleFigure *sensorBarFg      = nullptr;
    cTextFigure     *adaptiveThreshFigure = nullptr;

    static simsignal_t detectionSignal;
    static simsignal_t coverageSignal;
    static simsignal_t falseAlarmSignal;
    static simsignal_t adaptiveThreshSignal;
    static simsignal_t correlatedNoiseSignal;
    static simsignal_t timeToConfirmSignal;
    static simsignal_t uavsInvolvedSignal;
    static simsignal_t confirmationConfidenceSignal;

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void finish() override;
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void refreshDisplay() const override;

    virtual void handleStartOperation(LifecycleOperation *op) override;
    virtual void handleStopOperation(LifecycleOperation *op) override;
    virtual void handleCrashOperation(LifecycleOperation *op) override;

    virtual void socketDataArrived(UdpSocket *socket, Packet *packet) override;
    virtual void socketErrorArrived(UdpSocket *socket, Indication *indication) override;
    virtual void socketClosed(UdpSocket *socket) override {}

    void performScan();
    void broadcastStatus();
    void sendNetworkMessage(const char* type, double x, double y,
                            double confidence, double magneticValue);
    double calculateCoverage();

    void initSensorVisuals();
    void updateSensorVisuals(double magVal, double uavX, double uavY) const;

    cFigure::Color getMagneticColor(double magVal, double threshold) const;
    double         getMagneticRadius(double magVal, double threshold) const;

    void addFalseAlarmFigure(double x, double y);

    void checkTimeouts();
    void confirmTarget(inet::Coord targetPos, double confidence, double magVal);
    void startIntensiveSearch(double cmdX, double cmdY);
    void redirectToArea(double targetX, double targetY);

    void initiateReturnHome();
    void setFlightAltitude(double altitudeMeters);
};

} // namespace uavminedetection
#endif
