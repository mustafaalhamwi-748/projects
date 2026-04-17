#include "LawnmowerMobility.h"

#include <omnetpp.h>

#include "inet/common/INETMath.h"

namespace uavminedetection {

Define_Module(LawnmowerMobility);

void LawnmowerMobility::initialize(int stage)
{
    LineSegmentsMobilityBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        speed = par("speed");
        x1 = par("x1");
        y1 = par("y1");
        x2 = par("x2");
        y2 = par("y2");
        rowCount = par("rowCount");
        altitude = par("altitude");
        step = 0;

        if (speed <= 0)
            throw cRuntimeError("LawnmowerMobility: speed must be > 0");
        if (rowCount <= 0)
            throw cRuntimeError("LawnmowerMobility: rowCount must be > 0");
    }
}

void LawnmowerMobility::setInitialPosition()
{
    lastPosition = inet::Coord(x1, y1, altitude);
}

void LawnmowerMobility::setTargetPosition()
{
    int sign;
    inet::Coord positionDelta = inet::Coord::ZERO;

    switch (step % 4) {
        case 0:
            positionDelta.x = x2 - x1;
            break;

        case 1:
        case 3:
            sign = (step / (2 * rowCount)) % 2 ? -1 : 1;
            positionDelta.y = (y2 - y1) / rowCount * sign;
            break;

        case 2:
            positionDelta.x = x1 - x2;
            break;
    }

    step++;
    targetPosition = lastPosition + positionDelta;
    targetPosition.z = altitude;
    nextChange = simTime() + positionDelta.length() / speed;
}

} // namespace uavminedetection
