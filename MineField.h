#ifndef MINEFIELD_H
#define MINEFIELD_H

#include <vector>
#include <map>
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
#include <osg/Geometry>
#include <osg/Array>
#include <osg/PrimitiveSet>
#endif

using namespace omnetpp;

namespace uavminedetection {

// ============================================================
// MinePos — بيانات اللغم الواحد
// depth: عمق الدفن بالمتر (0.05 – 0.40 م)
// ============================================================
struct MinePos {
    double x, y;
    double depth;       // عمق الدفن (متر)
    bool   discovered;
};

enum MetalType { NAIL, WIRE, CAN, TOOL_PART };

struct MetalDebris {
    double    x, y;
    MetalType type;
    double    magneticStrength;
    bool      triggered;
};

class MineField : public cSimpleModule
{
  public:
    // ============================================================
    // getMagneticValue — [MODIFIED v3]: يقبل uavId للضوضاء المترابطة
    //
    // كل طائرة تحتفظ بحالة ضوضائها السابقة منفصلة عن البقية.
    // هذا يعكس الواقع: طائرتان في نفس المنطقة تقرآن نفس الإشارة
    // الحقيقية لكن كل حساس له تاريخه الخاص من الضوضاء.
    // ============================================================
    double getMagneticValue(double uavX, double uavY, double uavZ) const;

    int    getNearestUndiscoveredMine(double x, double y, double radius) const;
    int    getNearestMetalDebris(double x, double y, double radius) const;
    void   markDiscovered(int index);
    void   markDebrisTriggered(int index);

    int    getNumMines()        const { return (int)mines.size(); }
    int    getDiscoveredCount() const;
    int    getNumDebris()       const { return (int)debris.size(); }
    const  std::vector<MinePos>&     getMines()  const { return mines; }
    const  std::vector<MetalDebris>& getDebris() const { return debris; }

    double getMinDepth()        const { return minDepth; }
    double getMaxDepth()        const { return maxDepth; }
    double getSoilAttenuation() const { return soilAttenuation; }
    double getNoiseCorrelation()const { return noiseCorrelation; }

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

    // معاملات عمق الدفن
    double minDepth;
    double maxDepth;
    double soilAttenuation;

    // ── [NEW]: معامل الضوضاء المترابطة زمنياً ─────────────────
    // نموذج AR(1): noise[t] = α × noise[t-1] + (1-α) × white[t]
    // α=0.0 → ضوضاء بيضاء مستقلة (السلوك القديم)
    // α=0.7 → ترابط معتدل (موصى به)
    // α=0.95→ ضوضاء تتغير ببطء شديد
    double noiseCorrelation;

    // ── [NEW]: حالة الضوضاء الحالية لكل طائرة (مُعلَّم mutable
    //    لأن getMagneticValue تحتاج تعديله وهي معرَّفة const)
    mutable std::map<int, double> prevNoise;

    std::vector<cGroupFigure*>  mineFigures;
    std::vector<cGroupFigure*>  debrisFigures;

    void createMineVisuals();
    void createDebrisVisuals();
    void drawFarmBackground();
    void addLegend();
    cFigure::Color getMineColorByDepth(double depth) const;

#ifdef WITH_OSG
    std::vector<osg::ref_ptr<osg::MatrixTransform>> mineOsgNodes;
    std::vector<osg::ref_ptr<osg::MatrixTransform>> debrisOsgNodes;

    osg::ref_ptr<osg::Group> getOrCreateOsgScene();
    void addGroundPlane(osg::ref_ptr<osg::Group> scene);
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
