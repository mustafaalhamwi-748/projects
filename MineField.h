#ifndef MINEFIELD_H
#define MINEFIELD_H

#include <vector>
#include <omnetpp.h>

using namespace omnetpp;

namespace uavminedetection {

// ============================================================
// بنية اللغم الحقيقي
// ============================================================
struct MinePos {
    double x, y;
    bool   discovered;
};

// ============================================================
// نوع القطعة المعدنية العشوائية
// ============================================================
enum MetalType {
    NAIL,       // مسمار صغير
    WIRE,       // سلك
    CAN,        // علبة معدنية
    TOOL_PART   // بقايا معدات زراعية
};

// ============================================================
// بنية القطعة المعدنية العشوائية (50 قطعة)
// تُسبب إنذارات كاذبة عند الاكتشاف
// ============================================================
struct MetalDebris {
    double   x, y;
    MetalType type;
    double   magneticStrength; // قوة مغناطيسية أقل من اللغم
    bool     triggered;        // هل أطلقت إنذاراً من قبل؟
};

// ============================================================
// MineField — حقل الألغام + القطع المعدنية العشوائية
//
// الطائرات لا تعرف مسبقاً:
//   - مواضع الألغام الحقيقية
//   - مواضع القطع المعدنية
// تكتشف فقط بالقياس المغناطيسي
// ============================================================
class MineField : public cSimpleModule
{
  public:
    // ── واجهة الطائرات ──────────────────────────
    double getMagneticValue(double uavX, double uavY) const;

    // يُعيد index أقرب لغم حقيقي غير مكتشف ضمن radius
    int    getNearestUndiscoveredMine(double x, double y,
                                      double radius) const;

    // يُعيد index أقرب قطعة معدنية ضمن radius
    // يُعيد -1 إذا لا يوجد
    int    getNearestMetalDebris(double x, double y,
                                 double radius) const;

    // تسجيل اكتشافات
    void   markDiscovered(int index);
    void   markDebrisTriggered(int index);

    // ── استعلامات ──────────────────────────────
    int    getNumMines()        const { return (int)mines.size(); }
    int    getDiscoveredCount() const;
    int    getNumDebris()       const { return (int)debris.size(); }
    const  std::vector<MinePos>&    getMines()  const { return mines; }
    const  std::vector<MetalDebris>& getDebris() const { return debris; }

  protected:
    virtual void initialize()              override;
    virtual void handleMessage(cMessage *) override {}
    virtual void refreshDisplay() const    override;

  private:
    std::vector<MinePos>        mines;
    std::vector<MetalDebris>    debris;

    double magneticConstant;
    double backgroundNoise;
    double noiseVariation;

    // رسومات Canvas
    std::vector<cGroupFigure*>  mineFigures;
    std::vector<cGroupFigure*>  debrisFigures;

    void createMineVisuals();
    void createDebrisVisuals();
    void drawFarmBackground();
    void addLegend();
};

} // namespace uavminedetection
#endif
