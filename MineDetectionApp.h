#ifndef __UAVMINEDETECTION_MINEDETECTIONAPP_H
#define __UAVMINEDETECTION_MINEDETECTIONAPP_H

#include <vector>
#include <string>
#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "inet/mobility/contract/IMobility.h"
#include "MagnetometerSensor.h"
#include "MineField.h"

using namespace inet;

namespace uavminedetection {

class MineDetectionApp : public ApplicationBase,
                         public UdpSocket::ICallback
{
  protected:
    // ── معاملات التطبيق ────────────────────────
    int    uavId;
    double scanInterval;
    double magneticThreshold;
    double magneticSaturation;
    double falseAlarmProb;
    double confirmRadius;
    int    destPort;
    int    localPort;

    // ── الحساس والمرجع لحقل الألغام ───────────
    MagnetometerSensor *sensor    = nullptr;
    MineField          *mineField = nullptr;

    // ── حالة التطبيق ───────────────────────────
    UdpSocket  socket;
    cMessage  *scanTimer = nullptr;
    IMobility *mobility  = nullptr;

    // ── آخر قيمة مقاسة (للتصور المرئي) ────────
    double lastMagneticValue = 0.0;

    // ── figures التصور المرئي للتردد المغناطيسي ─
    // دائرة خارجية: تمثل نطاق الحساس، لونها يعكس القيمة
    cOvalFigure *sensorRingFigure  = nullptr;
    // رقم قيمة المجال (nT)
    cTextFigure *sensorValueFigure = nullptr;
    // شريط تقدم بصري للقيمة
    cRectangleFigure *sensorBarBg  = nullptr;  // خلفية الشريط
    cRectangleFigure *sensorBarFg  = nullptr;  // ملء الشريط

    // ── figures الإنذارات الكاذبة ──────────────
    int falseAlarmFigureCount = 0;

    // ── إحصاءات ────────────────────────────────
    int trueDetections    = 0;
    int falseAlarms       = 0;
    int duplicatesSkipped = 0;
    int messagesSent      = 0;

    // ── Signals ─────────────────────────────────
    static simsignal_t detectionSignal;
    static simsignal_t coverageSignal;
    static simsignal_t falseAlarmSignal;

  protected:
    virtual int  numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage)          override;
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void finish()                       override;
    virtual void refreshDisplay() const         override;

    // ── دورة المسح ─────────────────────────────
    void performScan();
    void sendDetectionReport(double x, double y,
                             double confidence,
                             double magneticValue);
    double calculateCoverage();

    // ── التصور المرئي للتردد المغناطيسي ────────
    void   initSensorVisuals();
    void   updateSensorVisuals(double magVal,
                               double uavX, double uavY) const;
    cFigure::Color getMagneticColor(double magVal) const;
    double         getMagneticRadius(double magVal) const;

    // ── مساعدات Canvas ─────────────────────────
    void addFalseAlarmFigure(double x, double y);

    // ── UdpSocket callbacks ─────────────────────
    virtual void socketDataArrived(UdpSocket *socket,
                                   Packet *packet)         override;
    virtual void socketErrorArrived(UdpSocket *socket,
                                    Indication *indication) override;
    virtual void socketClosed(UdpSocket *socket)           override {}

    // ── دورة حياة التطبيق ──────────────────────
    virtual void handleStartOperation(LifecycleOperation *op) override;
    virtual void handleStopOperation(LifecycleOperation *op)  override;
    virtual void handleCrashOperation(LifecycleOperation *op) override;
};

} // namespace uavminedetection
#endif
