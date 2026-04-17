#ifndef MINEFIELD_H
#define MINEFIELD_H

#include <vector>
#include <omnetpp.h>

using namespace omnetpp;

namespace uavminedetection {

// ============================================================
// بنية اللغم — تعرفها المحاكاة فقط، لا تعرفها الطائرات
// ============================================================
struct MinePos {
    double x, y;       // الإحداثيات الحقيقية (سر المحاكاة)
    bool   discovered; // هل اكتشفته طائرة ما؟
};

// ============================================================
// MineField — وحدة مستقلة تمثل حقل الألغام
//
// الطائرات لا تعرف الألغام مسبقاً
// تستطيع فقط:
//   1. سؤال getMagneticValue(x,y) → رقم nT
//   2. عند قراءة عالية: سؤال getNearestMineIndex(x,y,r)
//   3. عند تأكيد: markDiscovered(index)
// ============================================================
class MineField : public cSimpleModule
{
  public:
    // ── واجهة الطائرات ──────────────────────────
    // يُعيد قيمة المجال المغناطيسي عند موقع الطائرة (nT)
    double getMagneticValue(double uavX, double uavY) const;

    // يُعيد index أقرب لغم غير مكتشف ضمن radius
    // يُعيد -1 إذا لا يوجد → إنذار كاذب من ضوضاء
    int    getNearestUndiscoveredMine(double x, double y, double radius) const;

    // يُسجّل اكتشاف لغم بعد تأكيده
    void   markDiscovered(int index);

    // ── استعلامات إحصائية ──────────────────────
    int    getNumMines()        const { return (int)mines.size(); }
    int    getDiscoveredCount() const;
    const  std::vector<MinePos>& getMines() const { return mines; }

  protected:
    virtual void initialize()              override;
    virtual void handleMessage(cMessage *) override {}   // لا رسائل
    virtual void refreshDisplay() const    override;

  private:
    std::vector<MinePos>        mines;
    double                      magneticConstant;
    double                      backgroundNoise;
    double                      noiseVariation;

    // رسومات Canvas
    std::vector<cGroupFigure*>  mineFigures;
    void createMineVisuals();
    void addLegend();
};

} // namespace uavminedetection
#endif
