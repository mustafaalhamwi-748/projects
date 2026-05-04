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

// ============================================================
// setTargetPosition
// ── [MODIFIED]: أولوية Home > Spiral > Lawnmower
// ============================================================
void LawnmowerMobility::setTargetPosition()
{
    // ── 1. وضع العودة إلى القاعدة (RTH) ──────────────────────
    if (isHomeMode) {
        // الطائرة تتجه مباشرة نحو GCS بخط مستقيم
        targetPosition = homeTarget;
        inet::Coord delta = targetPosition - lastPosition;
        double dist = delta.length();

        if (dist < 5.0) {
            // وصلنا — نبقى عند القاعدة
            targetPosition = homeTarget;
            nextChange = omnetpp::simTime() + 9999.0; // انتظار لا نهائي
        } else {
            nextChange = omnetpp::simTime() + dist / speed;
        }
        return;
    }

    // ── 2. وضع المسح الحلزوني (Spiral) ───────────────────────
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

    // ── 3. وضع المسح الطبيعي (Lawnmower) ─────────────────────
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

// ============================================================
// startSpiral
// ============================================================
void LawnmowerMobility::startSpiral(double centerX, double centerY)
{
    if (!isSpiralMode) {
        savedStep     = step;
        savedPosition = lastPosition;
    }
    isSpiralMode  = true;
    spiralCenter  = inet::Coord(centerX, centerY, altitude);
    currentAngle  = 0;
    currentRadius = 5.0;
    speed         = originalSpeed * 0.5;
}

// ============================================================
// stopSpiral
// ============================================================
void LawnmowerMobility::stopSpiral()
{
    isSpiralMode = false;
    speed        = originalSpeed;
    step         = savedStep;
    lastPosition = savedPosition;

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
    nextChange       = omnetpp::simTime()
                     + positionDelta.length() / speed;
}

// ============================================================
// [NEW] setAltitude
// ──────────────────────────────────────────────────────────────
// الحل الصحيح لتغيير الارتفاع فعلياً:
//   par("altitude").setDoubleValue() وحده لا يكفي لأن altitude
//   مخزَّن كـ member variable يُقرأ مرة واحدة في initialize()
//   ولا يتحدث تلقائياً لاحقاً.
//   هنا نغيّر المتغير الداخلي مباشرة → يؤثر فوراً على
//   setTargetPosition() وstartSpiral() وكل حسابات Z اللاحقة.
// ============================================================
void LawnmowerMobility::setAltitude(double newAlt)
{
    altitude = newAlt;  // ← التحديث الفعلي الوحيد المهم

    // تحديث constraintArea للتوافق مع INET internals
    par("constraintAreaMinZ").setDoubleValue(newAlt);
    par("constraintAreaMaxZ").setDoubleValue(newAlt);

    // تحديث lastPosition.z فوراً في الوضع الطبيعي فقط
    // (Spiral وHome يحسبان Z من spiralCenter/homeTarget — لا تدخل)
    if (!isSpiralMode && !isHomeMode)
        lastPosition.z = newAlt;

    // [FIX]: في وضع الحلزون، حدّث مركز الحلزون ليطابق الارتفاع الجديد
    // بدون هذا السطر، ستطير الطائرة حلزونياً على ارتفاع 80م رغم طلب 30م
    if (isSpiralMode)
        spiralCenter.z = newAlt;

    EV_INFO << "LawnmowerMobility: altitude updated to " << newAlt << "m\n";
}

// ============================================================
// [NEW] goHome
// ──────────────────────────────────────────────────────────────
// يُفعَّل من MineDetectionApp عند t=550s
//
// المنطق:
//   1. إيقاف Spiral إذا كان نشطاً
//   2. ضبط homeTarget على موضع GCS بالارتفاع المطلوب
//   3. تفعيل isHomeMode → setTargetPosition ستتجه مباشرة
//   4. تحديث altitude المتغير ليُستخدم في targetPosition.z
// ============================================================
void LawnmowerMobility::goHome(double homeX, double homeY, double homeAltitude)
{
    // إيقاف أي وضع سابق
    isSpiralMode = false;
    speed        = originalSpeed * 1.2; // أسرع قليلاً في العودة

    // تحديث الارتفاع
    altitude = homeAltitude;

    // ضبط الهدف
    homeTarget  = inet::Coord(homeX, homeY, homeAltitude);
    isHomeMode  = true;

    // تحديث فوري للهدف الحالي
    targetPosition = homeTarget;
    inet::Coord delta = homeTarget - lastPosition;
    double dist = delta.length();
    if (dist > 1.0)
        nextChange = omnetpp::simTime() + dist / speed;
    else
        nextChange = omnetpp::simTime() + 9999.0;

    EV_INFO << "LawnmowerMobility: goHome() → ("
            << homeX << ", " << homeY << ", " << homeAltitude
            << ") at speed=" << speed << " m/s\n";
}

} // namespace uavminedetection
