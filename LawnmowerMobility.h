#ifndef LAWNMOWERMOBILITY_H
#define LAWNMOWERMOBILITY_H

#include "inet/mobility/base/LineSegmentsMobilityBase.h"

namespace uavminedetection {

class LawnmowerMobility : public inet::LineSegmentsMobilityBase
{
  protected:
    double speed = 0;
    double originalSpeed = 0;
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
    inet::Coord savedPosition;
    int savedStep = 0;

    // ── [NEW] متغيرات العودة إلى القاعدة ──
    bool isHomeMode = false;       // هل الطائرة في وضع RTH؟
    inet::Coord homeTarget;        // موضع GCS (500, 950, 80)

  protected:
    virtual void initialize(int stage) override;
    virtual void setInitialPosition() override;
    virtual void setTargetPosition() override;

  public:
    void startSpiral(double centerX, double centerY);
    void stopSpiral();

    // [NEW]: تغيير الارتفاع فورياً بتحديث المتغير الداخلي مباشرة
    void setAltitude(double newAlt);

    // [NEW]: التوجه المباشر نحو نقطة القاعدة
    void goHome(double homeX, double homeY, double homeAltitude);
};

} // namespace uavminedetection

#endif
