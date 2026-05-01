#ifndef LAWNMOWERMOBILITY_H
#define LAWNMOWERMOBILITY_H

#include "inet/mobility/base/LineSegmentsMobilityBase.h"

namespace uavminedetection {

class LawnmowerMobility : public inet::LineSegmentsMobilityBase
{
  protected:
    double speed = 0;
    double x1 = 0;
    double y1 = 0;
    double x2 = 0;
    double y2 = 0;
    int rowCount = 0;
    double altitude = 0;
    int step = 0;

  protected:
    virtual void initialize(int stage) override;
    virtual void setInitialPosition() override;
    virtual void setTargetPosition() override;
};

} // namespace uavminedetection

#endif
