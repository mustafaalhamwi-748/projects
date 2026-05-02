#ifndef LAWNMOWERMOBILITY_H
#define LAWNMOWERMOBILITY_H

#include "inet/mobility/base/LineSegmentsMobilityBase.h"

namespace uavminedetection {

class LawnmowerMobility : public inet::LineSegmentsMobilityBase
{
  protected:
    double speed = 0;
    double originalSpeed = 0; // لحفظ السرعة الأصلية
    double x1 = 0;
    double y1 = 0;
    double x2 = 0;
    double y2 = 0;
    int rowCount = 0;
    double altitude = 0;
    int step = 0;

    // ── متغيرات المسح الحلزوني ──
    bool isSpiralMode = false;
    inet::Coord spiralCenter;
    double currentAngle = 0;
    double currentRadius = 0;
    inet::Coord savedPosition; // لحفظ موقع الطائرة قبل الحلزون
    int savedStep = 0;

  protected:
    virtual void initialize(int stage) override;
    virtual void setInitialPosition() override;
    virtual void setTargetPosition() override;

  public:
    // دوال للتحكم من الخارج
    void startSpiral(double centerX, double centerY);
    void stopSpiral();
};

} // namespace uavminedetection

#endif
