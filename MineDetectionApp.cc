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

// ── Signals ──────────────────────────────────────────────────
simsignal_t MineDetectionApp::detectionSignal  =
    registerSignal("mineDetected");
simsignal_t MineDetectionApp::coverageSignal   =
    registerSignal("areaCoverage");
simsignal_t MineDetectionApp::falseAlarmSignal =
    registerSignal("falseAlarm");

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

        sensor = new MagnetometerSensor(magneticThreshold,
                                        magneticSaturation);

        WATCH(trueDetections);
        WATCH(falseAlarms);
        WATCH(duplicatesSkipped);
        WATCH(messagesSent);
        WATCH(lastMagneticValue);
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {

        mineField = check_and_cast<MineField*>(
            getSystemModule()->getSubmodule("mineField"));

        mobility = check_and_cast<IMobility*>(
            getParentModule()->getSubmodule("mobility"));

        socket.setOutputGate(gate("socketOut"));
        socket.bind(localPort + uavId);
        socket.setCallback(this);

        // إنشاء العناصر البصرية للتردد المغناطيسي
        initSensorVisuals();

        scanTimer = new cMessage("scanTimer");
        scheduleAt(simTime() + uniform(0, scanInterval), scanTimer);

        EV_INFO << "UAV[" << uavId
                << "] MagSensor + Visuals ready — threshold="
                << magneticThreshold << "nT\n";
    }
}

// ============================================================
// initSensorVisuals — إنشاء العناصر البصرية مرة واحدة
//
// لكل طائرة:
//   1. دائرة ملونة تعكس قوة المجال المغناطيسي
//   2. نص يعرض القيمة الرقمية (nT)
//   3. شريط تقدم يوضح نسبة القيمة من العتبة
// ============================================================
void MineDetectionApp::initSensorVisuals()
{
    cCanvas *canvas = getSystemModule()->getCanvas();

    Coord pos = mobility->getCurrentPosition();
    double cx  = pos.x;
    double cy  = pos.y;

    // ── 1. الدائرة الملونة ─────────────────────────────────
    // تنكمش وتتمدد وتغير لونها حسب قيمة المجال
    std::string ringName = "magRing_" + std::to_string(uavId);
    sensorRingFigure = new cOvalFigure(ringName.c_str());
    double R = 14.0;  // نصف قطر ابتدائي
    sensorRingFigure->setBounds(
        cFigure::Rectangle(cx-R, cy-R, 2*R, 2*R));
    sensorRingFigure->setFilled(true);
    sensorRingFigure->setFillColor(cFigure::Color(0, 100, 255));
    sensorRingFigure->setFillOpacity(0.18);
    sensorRingFigure->setLineColor(cFigure::Color(0, 100, 255));
    sensorRingFigure->setLineWidth(2);
    sensorRingFigure->setLineOpacity(0.7);
    canvas->addFigure(sensorRingFigure);

    // ── 2. نص قيمة المجال ──────────────────────────────────
    // يظهر فوق الطائرة: "220 nT"
    std::string valName = "magVal_" + std::to_string(uavId);
    sensorValueFigure = new cTextFigure(valName.c_str());
    sensorValueFigure->setPosition(cFigure::Point(cx, cy - 18));
    sensorValueFigure->setText("--- nT");
    sensorValueFigure->setColor(cFigure::Color(0, 60, 180));
    sensorValueFigure->setAnchor(cFigure::ANCHOR_S);
    sensorValueFigure->setFont(cFigure::Font("", 7, cFigure::FONT_BOLD));
    canvas->addFigure(sensorValueFigure);

    // ── 3. شريط التقدم ─────────────────────────────────────
    // شريط صغير تحت الطائرة يُظهر نسبة المجال من العتبة
    // الخلفية (رمادي)
    std::string barBgName = "magBarBg_" + std::to_string(uavId);
    sensorBarBg = new cRectangleFigure(barBgName.c_str());
    sensorBarBg->setBounds(cFigure::Rectangle(cx-12, cy+14, 24, 4));
    sensorBarBg->setFilled(true);
    sensorBarBg->setFillColor(cFigure::Color(200, 200, 200));
    sensorBarBg->setLineColor(cFigure::Color(150, 150, 150));
    sensorBarBg->setLineWidth(1);
    canvas->addFigure(sensorBarBg);

    // الملء (يتمدد مع ارتفاع القيمة)
    std::string barFgName = "magBarFg_" + std::to_string(uavId);
    sensorBarFg = new cRectangleFigure(barFgName.c_str());
    sensorBarFg->setBounds(cFigure::Rectangle(cx-12, cy+14, 2, 4));
    sensorBarFg->setFilled(true);
    sensorBarFg->setFillColor(cFigure::Color(0, 100, 255));
    sensorBarFg->setLineWidth(1);
    canvas->addFigure(sensorBarFg);
}

// ============================================================
// getMagneticColor — لون الدائرة حسب قيمة المجال
//
// منخفض (< 80% من العتبة)    → أزرق   (آمن، بعيد عن الألغام)
// متوسط (80% - 100% عتبة)   → أصفر   (تحذير، يقترب)
// فوق العتبة (100% - 150%)  → برتقالي (تأهب، قريب جداً)
// عالي جداً  (> 150% عتبة)  → أحمر   (فوق لغم مباشرة)
// ============================================================
cFigure::Color MineDetectionApp::getMagneticColor(double magVal) const
{
    double ratio = magVal / magneticThreshold;

    if (ratio < 0.8)
        return cFigure::Color(0, 100, 255);      // أزرق: آمن
    else if (ratio < 1.0)
        return cFigure::Color(255, 200, 0);      // أصفر: تحذير
    else if (ratio < 1.5)
        return cFigure::Color(255, 120, 0);      // برتقالي: تأهب
    else
        return cFigure::Color(220, 0, 0);        // أحمر: لغم!
}

// ============================================================
// getMagneticRadius — حجم الدائرة حسب قيمة المجال
// كلما ارتفعت القيمة كلما اتسعت الدائرة (تصور توسع التردد)
// ============================================================
double MineDetectionApp::getMagneticRadius(double magVal) const
{
    double ratio = magVal / magneticThreshold;
    // نصف قطر بين 10 و 28 متراً حسب النسبة
    double R = 10.0 + std::min(18.0, ratio * 12.0);
    return R;
}

// ============================================================
// updateSensorVisuals — تحديث العناصر البصرية في كل قياس
// ============================================================
void MineDetectionApp::updateSensorVisuals(double magVal,
                                            double uavX,
                                            double uavY) const
{
    if (!sensorRingFigure || !sensorValueFigure) return;

    // ── تحديث الدائرة ──────────────────────────────────────
    cFigure::Color color = getMagneticColor(magVal);
    double R = getMagneticRadius(magVal);

    sensorRingFigure->setBounds(
        cFigure::Rectangle(uavX-R, uavY-R, 2*R, 2*R));
    sensorRingFigure->setFillColor(color);
    sensorRingFigure->setLineColor(color);

    // تحديث الشفافية: أكثر وضوحاً عند الاقتراب من لغم
    double opacity = (magVal >= magneticThreshold) ? 0.35 : 0.18;
    sensorRingFigure->setFillOpacity(opacity);

    // ── تحديث النص ─────────────────────────────────────────
    char buf[32];
    if (magVal >= magneticThreshold)
        snprintf(buf, sizeof(buf), "⚠ %.0f nT", magVal);
    else
        snprintf(buf, sizeof(buf), "%.0f nT", magVal);

    sensorValueFigure->setPosition(
        cFigure::Point(uavX, uavY - 18));
    sensorValueFigure->setText(buf);
    sensorValueFigure->setColor(color);

    // ── تحديث شريط التقدم ──────────────────────────────────
    if (sensorBarBg && sensorBarFg) {
        const double barW   = 24.0;
        const double barH   = 4.0;
        const double barX   = uavX - 12;
        const double barY   = uavY + 14;

        sensorBarBg->setBounds(
            cFigure::Rectangle(barX, barY, barW, barH));

        // نسبة الملء = magVal / (1.5 * threshold) بحد أقصى 100%
        double fillRatio = std::min(1.0,
            magVal / (1.5 * magneticThreshold));
        double fillW = std::max(1.0, fillRatio * barW);

        sensorBarFg->setBounds(
            cFigure::Rectangle(barX, barY, fillW, barH));
        sensorBarFg->setFillColor(color);
    }
}

// ============================================================
// performScan — دورة المسح الرئيسية
// ============================================================
void MineDetectionApp::performScan()
{
    Coord pos = mobility->getCurrentPosition();

    // ── قياس المجال المغناطيسي ──────────────────────────────
    double magVal = mineField->getMagneticValue(pos.x, pos.y);
    lastMagneticValue = magVal;

    // ── تحديث التصور المرئي فوراً ───────────────────────────
    updateSensorVisuals(magVal, pos.x, pos.y);

    // ── قرار الحساس ────────────────────────────────────────
    SensorReading reading = sensor->measure(magVal);

    if (reading.isMine) {
        int mineIdx = mineField->getNearestUndiscoveredMine(
            pos.x, pos.y, confirmRadius);

        if (mineIdx >= 0) {
            // ✅ اكتشاف حقيقي
            mineField->markDiscovered(mineIdx);
            trueDetections++;
            emit(detectionSignal, 1L);

            sendDetectionReport(pos.x, pos.y,
                                reading.confidence,
                                reading.magneticValue);

            EV_INFO << "UAV[" << uavId
                    << "] ✅ MINE CONFIRMED at ("
                    << pos.x << "," << pos.y
                    << ") magVal=" << magVal
                    << " conf=" << reading.confidence << "\n";
        } else {
            // تحقق: مكتشف مسبقاً أم إنذار كاذب؟
            const auto& mines = mineField->getMines();
            bool alreadyKnown = false;
            for (const auto& m : mines) {
                double d = sqrt(pow(pos.x-m.x,2)+pow(pos.y-m.y,2));
                if (d < confirmRadius && m.discovered) {
                    alreadyKnown = true;
                    break;
                }
            }
            if (alreadyKnown) {
                duplicatesSkipped++;
            } else {
                falseAlarms++;
                emit(falseAlarmSignal, 1L);
                addFalseAlarmFigure(pos.x, pos.y);
            }
        }
    }

    // ── ضوضاء مغناطيسية عشوائية ────────────────────────────
    if (uniform(0, 1) < falseAlarmProb) {
        double noiseSpike = magneticThreshold
            + uniform(0, magneticSaturation - magneticThreshold);
        SensorReading nr = sensor->measure(noiseSpike);
        if (nr.isMine) {
            falseAlarms++;
            emit(falseAlarmSignal, 1L);
            double fpX = std::max(10.0,
                std::min(990.0, pos.x + uniform(-20, 20)));
            double fpY = std::max(10.0,
                std::min(990.0, pos.y + uniform(-20, 20)));
            addFalseAlarmFigure(fpX, fpY);
        }
    }

    emit(coverageSignal, calculateCoverage());
    scheduleAt(simTime() + scanInterval, scanTimer);
}

// ============================================================
// refreshDisplay — يُستدعى من Qtenv تلقائياً
// ============================================================
void MineDetectionApp::refreshDisplay() const
{
    // تحديث موقع العناصر البصرية مع حركة الطائرة
    if (mobility && sensorRingFigure) {
        Coord pos = mobility->getCurrentPosition();
        updateSensorVisuals(lastMagneticValue, pos.x, pos.y);
    }

    // تحديث تسمية الطائرة
    char buf[80];
    snprintf(buf, sizeof(buf),
             "UAV%d | %.0f nT | found=%d | FA=%d",
             uavId, lastMagneticValue,
             trueDetections, falseAlarms);
    getParentModule()->getDisplayString().setTagArg("t", 0, buf);
}

// ============================================================
// sendDetectionReport
// ============================================================
void MineDetectionApp::sendDetectionReport(double x, double y,
                                            double confidence,
                                            double magneticValue)
{
    char buf[160];
    snprintf(buf, sizeof(buf),
             "MINE:uav=%d,x=%.1f,y=%.1f,"
             "conf=%.2f,magVal=%.1f,t=%.2f",
             uavId, x, y,
             confidence, magneticValue, simTime().dbl());

    auto payload = makeShared<BytesChunk>();
    std::vector<uint8_t> bytes(buf, buf + strlen(buf));
    payload->setBytes(bytes);

    auto *pkt = new Packet("MineDetectionReport", payload);

    L3Address dest;
    L3AddressResolver().tryResolve("gcs", dest);

    if (!dest.isUnspecified()) {
        socket.sendTo(pkt, dest, destPort);
        messagesSent++;
        EV_INFO << "Sent to GCS: " << buf << "\n";
    } else {
        EV_WARN << "UAV[" << uavId
                << "] Cannot resolve GCS address!\n";
        delete pkt;
    }
}

// ============================================================
// calculateCoverage
// ============================================================
double MineDetectionApp::calculateCoverage()
{
    const int    G    = 50;
    const double AREA = 1000.0;
    int          cov  = 0;
    cModule     *net  = getSystemModule();
    int          nUAV = net->par("numUAVs");
    const double sensorCovRange = 10.0;

    for (int i = 0; i < G; i++) {
        for (int j = 0; j < G; j++) {
            double px = (i + 0.5) * (AREA / G);
            double py = (j + 0.5) * (AREA / G);
            for (int u = 0; u < nUAV; u++) {
                IMobility *mob = check_and_cast<IMobility*>(
                    net->getSubmodule("uav", u)
                       ->getSubmodule("mobility"));
                Coord p = mob->getCurrentPosition();
                if (sqrt(pow(p.x-px,2)+pow(p.y-py,2))
                        <= sensorCovRange) {
                    cov++; break;
                }
            }
        }
    }
    return (double)cov / (G*G) * 100.0;
}

// ============================================================
// addFalseAlarmFigure
// ============================================================
void MineDetectionApp::addFalseAlarmFigure(double x, double y)
{
    if (falseAlarmDisplayLimit >= 0 &&
        falseAlarmFigureCount >= falseAlarmDisplayLimit)
        return;

    cCanvas    *canvas = getSystemModule()->getCanvas();
    std::string name   = "fa_" + std::to_string(uavId)
                       + "_" + std::to_string(falseAlarmFigureCount++);

    auto *grp = new cGroupFigure(name.c_str());

    auto *tri = new cPolygonFigure("tri");
    tri->setPoints({{x, y-9}, {x-8, y+6}, {x+8, y+6}});
    tri->setFilled(true);
    tri->setFillColor(cFigure::Color(255, 220, 0));
    tri->setLineColor(cFigure::Color(180, 130, 0));
    tri->setLineWidth(1);
    grp->addFigure(tri);

    auto *lbl = new cTextFigure("lbl");
    lbl->setPosition(cFigure::Point(x, y+2));
    lbl->setText("!");
    lbl->setColor(cFigure::Color("black"));
    lbl->setAnchor(cFigure::ANCHOR_CENTER);
    lbl->setFont(cFigure::Font("", 7, cFigure::FONT_BOLD));
    grp->addFigure(lbl);

    canvas->addFigure(grp);
}

// ============================================================
// handleMessageWhenUp
// ============================================================
void MineDetectionApp::handleMessageWhenUp(cMessage *msg)
{
    if (msg == scanTimer)
        performScan();
    else if (msg->arrivedOn("socketIn"))
        socket.processMessage(msg);
    else
        delete msg;
}

void MineDetectionApp::socketDataArrived(UdpSocket *, Packet *pkt)
{
    EV_INFO << "UAV[" << uavId
            << "] received: " << pkt->getName() << "\n";
    delete pkt;
}

void MineDetectionApp::socketErrorArrived(UdpSocket *,
                                           Indication *ind)
{
    EV_WARN << "UDP error on UAV[" << uavId << "]\n";
    delete ind;
}

// ============================================================
// finish
// ============================================================
void MineDetectionApp::finish()
{
    EV_INFO << "\n=== UAV[" << uavId
            << "] Final Statistics ===\n";
    EV_INFO << "True Detections   : " << trueDetections    << "\n";
    EV_INFO << "False Alarms      : " << falseAlarms        << "\n";
    EV_INFO << "Duplicates Skipped: " << duplicatesSkipped  << "\n";
    EV_INFO << "Messages Sent     : " << messagesSent       << "\n";

    if (uavId == 0) {
        int total = mineField->getNumMines();
        int found = mineField->getDiscoveredCount();
        EV_INFO << "\n=== Network Summary ===\n";
        EV_INFO << "Total Mines   : " << total << "\n";
        EV_INFO << "Found Mines   : " << found << "\n";
        EV_INFO << "Detection Rate: "
                << (double)found/total*100.0 << "%\n";
    }

    recordScalar("trueDetections",    trueDetections);
    recordScalar("falseAlarms",       falseAlarms);
    recordScalar("duplicatesSkipped", duplicatesSkipped);
    recordScalar("messagesSent",      messagesSent);
}

// ============================================================
// دورة حياة التطبيق
// ============================================================
void MineDetectionApp::handleStartOperation(LifecycleOperation *)
{
    if (!scanTimer) scanTimer = new cMessage("scanTimer");
    scheduleAt(simTime() + uniform(0, scanInterval), scanTimer);
}

void MineDetectionApp::handleStopOperation(LifecycleOperation *)
{
    cancelEvent(scanTimer);
    socket.close();
}

void MineDetectionApp::handleCrashOperation(LifecycleOperation *)
{
    cancelEvent(scanTimer);
    socket.destroy();
}

} // namespace uavminedetection
