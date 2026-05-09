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
  protected:
    int uavId = -1;
    double scanInterval = 0.5;

    // ── معاملات الحساس الثابتة ─────────────────────────────
    double magneticThreshold  = 0;   // عتبة ثابتة (مرحلة الإحماء)
    double magneticSaturation = 0;
    double falseAlarmProb     = 0;
    int    falseAlarmDisplayLimit = 30;
    double confirmRadius = 25.0;

    // ── [NEW]: معاملات العتبة التكيفية ──────────────────────
    int    adaptiveWindowSize   = 50;    // حجم النافذة المنزلقة
    double adaptiveKSigma       = 3.0;  // معامل الانحراف المعياري
    double adaptiveMinThreshold = 200.0; // الحد الأدنى المطلق (nT)

    // ── [NEW]: الضوضاء المترابطة زمنياً — نموذج AR(1) ──────────
    // الفيزياء: انجراف الحساس الإلكتروني (drift) لا يتغيّر فجأة
    // بين قراءة وأخرى، بل يتطور ببطء مثل سير عشوائي مُخمَّد.
    //
    // AR(1): η(t) = α·η(t-1) + (1−α)·w(t)
    //   α → 1: تغيّر بطيء جداً (انجراف طويل الأمد)
    //   α → 0: ضوضاء بيضاء مستقلة (السلوك القديم)
    //   α = 0.8: الانجراف يُجدَّد كل ~5 قراءات تقريباً
    double sensorNoiseAlpha = 0.8;   // معامل التلاشي [0, 1)
    double sensorNoiseScale = 30.0;  // سعة الابتكار (nT)
    double correlatedNoise  = 0.0;   // حالة الضوضاء الحالية (تُحدَّث في كل مسح)

    int destPort  = -1;
    int localPort = -1;
    UdpSocket socket;

    inet::L3Address gcsAddress;

    MineField         *mineField = nullptr;
    MagnetometerSensor *sensor   = nullptr;
    IMobility         *mobility  = nullptr;

    cMessage *scanTimer       = nullptr;
    cMessage *intensiveTimer  = nullptr;
    cMessage *returnHomeTimer = nullptr;

    // حالة الطائرة
    bool isIntensiveMode  = false;
    bool isReturningHome  = false;

    static constexpr double RETURN_HOME_BEFORE = 50.0;
    static constexpr double GCS_X              = 500.0;
    static constexpr double GCS_Y              = 950.0;
    static constexpr double CRUISE_ALTITUDE    = 80.0;
    static constexpr double SCAN_ALTITUDE      = 30.0;

    std::vector<inet::Coord>   sharedMemory;
    std::vector<CandidateMine> candidateMines;

    // العداد التراكمي للتغطية
    std::set<int> visitedCells;

    // الإحصاءات
    int    trueDetections   = 0;
    int    falseAlarms      = 0;
    int    duplicatesSkipped= 0;
    int    messagesSent     = 0;
    double lastMagneticValue= 0.0;
    int    falseAlarmFigureCount = 0;

    double confirmationTimeout = 30.0;

    // العناصر البصرية
    cOvalFigure     *sensorRingFigure  = nullptr;
    cTextFigure     *sensorValueFigure = nullptr;
    cRectangleFigure *sensorBarBg      = nullptr;
    cRectangleFigure *sensorBarFg      = nullptr;

    // [NEW]: نص يعرض حالة العتبة التكيفية فوق الطائرة
    cTextFigure     *adaptiveThreshFigure = nullptr;

    // الإشارات (Signals)
    static simsignal_t detectionSignal;
    static simsignal_t coverageSignal;
    static simsignal_t falseAlarmSignal;
    static simsignal_t adaptiveThreshSignal;   // [NEW]
    static simsignal_t correlatedNoiseSignal;  // [NEW]: تتبع الضوضاء المترابطة

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
    void sendNetworkMessage(const char* type, double x, double y,
                            double confidence, double magneticValue);
    double calculateCoverage();

    void initSensorVisuals();
    void updateSensorVisuals(double magVal, double uavX, double uavY) const;

    // [MODIFIED]: تأخذ الآن effectiveThreshold من الحساس لا من المتغير الثابت
    cFigure::Color getMagneticColor(double magVal, double threshold) const;
    double         getMagneticRadius(double magVal, double threshold) const;

    void addFalseAlarmFigure(double x, double y);

    void checkTimeouts();
    void confirmTarget(inet::Coord targetPos, double confidence, double magVal);
    void startIntensiveSearch(double cmdX, double cmdY);

    void initiateReturnHome();
    void setFlightAltitude(double altitudeMeters);
};

} // namespace uavminedetection
#endif
