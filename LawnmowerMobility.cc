#include "LawnmowerMobility.h"
#include <omnetpp.h>
#include "inet/common/INETMath.h"
#include "inet/common/InitStages.h"

namespace uavminedetection {

Define_Module(LawnmowerMobility);

void LawnmowerMobility::initialize(int stage)
{
    LineSegmentsMobilityBase::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL) {
        speed = par("speed");
        originalSpeed = speed;
        x1 = par("x1"); y1 = par("y1");
        x2 = par("x2"); y2 = par("y2");
        rowCount = par("rowCount");
        altitude = par("altitude");
        step = 0;

        int uavId = getParentModule()->getIndex();
        if (uavId % 2 != 0) {
            double temp = y1; y1 = y2; y2 = temp;
        }
    }
}

void LawnmowerMobility::setInitialPosition()
{
    lastPosition = inet::Coord(x1, y1, altitude);
}

void LawnmowerMobility::setTargetPosition()
{
    // ── 1. وضع المسح الحلزوني (Spiral Mode) ──
    if (isSpiralMode) {
        // زيادة الزاوية ونصف القطر تدريجياً لعمل شكل حلزوني
        currentAngle += M_PI / 4; // 45 درجة في كل خطوة
        currentRadius += 3.0;     // الاتساع بمقدار 3 أمتار

        double nextX = spiralCenter.x + currentRadius * cos(currentAngle);
        double nextY = spiralCenter.y + currentRadius * sin(currentAngle);

        // منع الطائرة من الخروج خارج الحقل
        nextX = std::max(0.0, std::min(1000.0, nextX));
        nextY = std::max(0.0, std::min(1000.0, nextY));

        targetPosition = inet::Coord(nextX, nextY, altitude);
        inet::Coord delta = targetPosition - lastPosition;
        nextChange = omnetpp::simTime() + delta.length() / speed;
        return;
    }

    // ── 2. وضع المسح العادي (Lawnmower Mode) ──
    int sign;
    inet::Coord positionDelta = inet::Coord::ZERO;

    switch (step % 4) {
        case 0: positionDelta.x = x2 - x1; break;
        case 1:
        case 3:
            sign = (step / (2 * rowCount)) % 2 ? -1 : 1;
            positionDelta.y = (y2 - y1) / rowCount * sign; break;
        case 2: positionDelta.x = x1 - x2; break;
    }

    step++;
    targetPosition = lastPosition + positionDelta;
    targetPosition.z = altitude;
    nextChange = omnetpp::simTime() + positionDelta.length() / speed;
}

// ── دوال الاستدعاء من التطبيق ──
void LawnmowerMobility::startSpiral(double centerX, double centerY)
{
    if (!isSpiralMode) {
        savedStep = step; // حفظ الخطوة الحالية للعودة إليها لاحقاً
    }
    isSpiralMode = true;
    spiralCenter = inet::Coord(centerX, centerY, altitude);
    currentAngle = 0;
    currentRadius = 5.0; // البدء من قطر 5 متر
    speed = originalSpeed * 0.5; // إبطاء السرعة للنصف أثناء الحلزون لدقة أكبر!
}

void LawnmowerMobility::stopSpiral()
{
    isSpiralMode = false;
    speed = originalSpeed; // استعادة السرعة الطبيعية
    step = savedStep;      // إكمال المسح من حيث توقفت
}

} // namespace uavminedetection
