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
        speed        = par("speed");
        originalSpeed= speed;
        x1           = par("x1");
        y1           = par("y1");
        x2           = par("x2");
        y2           = par("y2");
        rowCount     = par("rowCount");
        altitude     = par("altitude");
        step         = 0;

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
    if (isHomeMode) {
        targetPosition = homeTarget;
        inet::Coord delta = targetPosition - lastPosition;
        double dist = delta.length();

        if (dist < 5.0) {
            targetPosition = homeTarget;
            nextChange = omnetpp::simTime() + 9999.0;
        } else {
            nextChange = omnetpp::simTime() + dist / speed;
        }
        return;
    }

    if (isSpiralMode) {
        currentAngle  += M_PI / 4;
        currentRadius += 3.0;

        double nextX = spiralCenter.x + currentRadius * cos(currentAngle);
        double nextY = spiralCenter.y + currentRadius * sin(currentAngle);

        nextX = std::max(0.0, std::min(1000.0, nextX));
        nextY = std::max(0.0, std::min(1000.0, nextY));

        targetPosition = inet::Coord(nextX, nextY, altitude);
        inet::Coord delta = targetPosition - lastPosition;
        nextChange = omnetpp::simTime() + delta.length() / speed;
        return;
    }

    int sign;
    inet::Coord positionDelta = inet::Coord::ZERO;

    switch (step % 4) {
        case 0: positionDelta.x = x2 - x1; break;
        case 1:
        case 3:
            sign = (step / (2 * rowCount)) % 2 ? -1 : 1;
            positionDelta.y = (y2 - y1) / rowCount * sign;
            break;
        case 2: positionDelta.x = x1 - x2; break;
    }

    step++;
    targetPosition    = lastPosition + positionDelta;
    targetPosition.z  = altitude;
    nextChange        = omnetpp::simTime() + positionDelta.length() / speed;
}

void LawnmowerMobility::startSpiral(double centerX, double centerY)
{
    if (!isSpiralMode) {
        savedStep     = step;
        savedPosition = targetPosition;
    }

    lastPosition = getCurrentPosition();
    lastUpdate   = omnetpp::simTime();

    isSpiralMode  = true;
    spiralCenter  = inet::Coord(centerX, centerY, altitude);
    currentAngle  = 0;
    currentRadius = 5.0;
    speed         = originalSpeed * 0.5;

    setTargetPosition();
}

void LawnmowerMobility::stopSpiral()
{
    isSpiralMode = false;
    speed        = originalSpeed;
    step         = savedStep;

    lastPosition = savedPosition;
    lastUpdate   = omnetpp::simTime();

    inet::Coord positionDelta = inet::Coord::ZERO;
    switch (step % 4) {
        case 0: positionDelta.x = x2 - x1; break;
        case 1:
        case 3: {
            int sign = (step / (2 * rowCount)) % 2 ? -1 : 1;
            positionDelta.y = (y2 - y1) / rowCount * sign;
            break;
        }
        case 2: positionDelta.x = x1 - x2; break;
    }
    step++;
    targetPosition   = lastPosition + positionDelta;
    targetPosition.z = altitude;
    nextChange       = omnetpp::simTime() + positionDelta.length() / speed;
}

void LawnmowerMobility::setAltitude(double newAlt)
{
    altitude = newAlt;
    par("constraintAreaMinZ").setDoubleValue(newAlt);
    par("constraintAreaMaxZ").setDoubleValue(newAlt);

    if (!isSpiralMode && !isHomeMode)
        lastPosition.z = newAlt;

    if (isSpiralMode)
        spiralCenter.z = newAlt;
}

void LawnmowerMobility::goHome(double homeX, double homeY, double homeAltitude)
{
    lastPosition = getCurrentPosition();
    lastUpdate   = omnetpp::simTime();

    isSpiralMode = false;
    speed        = originalSpeed * 1.2;
    altitude = homeAltitude;

    homeTarget  = inet::Coord(homeX, homeY, homeAltitude);
    isHomeMode  = true;

    targetPosition = homeTarget;
    inet::Coord delta = homeTarget - lastPosition;
    double dist = delta.length();

    if (dist > 1.0)
        nextChange = omnetpp::simTime() + dist / speed;
    else
        nextChange = omnetpp::simTime() + 9999.0;
}

} // namespace uavminedetection
