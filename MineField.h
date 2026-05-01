#ifndef MINEFIELD_H
#define MINEFIELD_H

#include <vector>
#include <omnetpp.h>

#ifdef WITH_OSG
#include <omnetpp/cosgcanvas.h>
#include <osg/Group>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Shape>
#include <osg/Material>
#include <osg/StateSet>
#include <osg/MatrixTransform>
#endif

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
    double   magneticStrength;
    bool     triggered;
};

// ============================================================
// MineField — حقل الألغام + القطع المعدنية العشوائية
// ============================================================
class MineField : public cSimpleModule
{
  public:
    double getMagneticValue(double uavX, double uavY) const;
    int    getNearestUndiscoveredMine(double x, double y, double radius) const;
    int    getNearestMetalDebris(double x, double y, double radius) const;
    void   markDiscovered(int index);
    void   markDebrisTriggered(int index);

    int    getNumMines()        const { return (int)mines.size(); }
    int    getDiscoveredCount() const;
    int    getNumDebris()       const { return (int)debris.size(); }
    const  std::vector<MinePos>&     getMines()  const { return mines; }
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

    // رسومات Canvas 2D
    std::vector<cGroupFigure*>  mineFigures;
    std::vector<cGroupFigure*>  debrisFigures;

    void createMineVisuals();
    void createDebrisVisuals();
    void drawFarmBackground();
    void addLegend();

#ifdef WITH_OSG
    // رسومات OSG 3D
    std::vector<osg::ref_ptr<osg::MatrixTransform>> mineOsgNodes;
    std::vector<osg::ref_ptr<osg::MatrixTransform>> debrisOsgNodes;

    void createMineOsgVisuals();
    void createDebrisOsgVisuals();
    osg::ref_ptr<osg::MatrixTransform> makeSphere(double x, double y, double z,
                                                   double radius,
                                                   float r, float g, float b);
    osg::ref_ptr<osg::MatrixTransform> makeBox(double x, double y, double z,
                                               double size,
                                               float r, float g, float b);
#endif
};

} // namespace uavminedetection
#endif
