#include "MineField.h"
#include <cmath>
#include <string>
#include <algorithm>

namespace uavminedetection {

Define_Module(MineField);

void MineField::initialize()
{
    magneticConstant = par("magneticConstant");
    backgroundNoise  = par("backgroundNoise");
    noiseVariation   = par("noiseVariation");

    struct { double x, y; } minePos[] = {
        {  80,  120}, { 220,  75}, { 380, 160}, { 530,  90},
        { 670, 190}, { 820, 110}, { 950, 210}, { 140, 310},
        { 290, 370}, { 460, 290}, { 610, 340}, { 760, 400},
        { 930, 320}, { 100, 520}, { 260, 550}, { 420, 490},
        { 580, 560}, { 740, 510}, { 890, 600}, { 180, 730},
        { 340, 760}, { 510, 820}, { 680, 770}, { 840, 840}
    };
    int nm = (int)(sizeof(minePos) / sizeof(minePos[0]));
    mines.clear();
    for (int i = 0; i < nm; i++) {
        MinePos m;
        m.x = minePos[i].x;
        m.y = minePos[i].y;
        m.discovered = false;
        mines.push_back(m);
    }

    struct { double x, y; MetalType t; double str; } debrisPos[] = {
        { 45,  200, NAIL,      15.0}, {165,  430, NAIL,      12.0},
        {310,  180, NAIL,      18.0}, {480,  650, NAIL,      14.0},
        {620,  430, NAIL,      16.0}, {750,  280, NAIL,      13.0},
        {880,  710, NAIL,      17.0}, {120,  860, NAIL,      11.0},
        {390,  940, NAIL,      15.0}, {710,  590, NAIL,      14.0},
        {830,  460, NAIL,      13.0}, {960,  780, NAIL,      16.0},
        {200,  650, NAIL,      12.0}, {550,  150, NAIL,      15.0},
        {670,  880, NAIL,      18.0},
        { 70,  350, WIRE,      25.0}, {230,  500, WIRE,      22.0},
        {400,  720, WIRE,      28.0}, {560,  310, WIRE,      24.0},
        {720,  640, WIRE,      26.0}, {890,  380, WIRE,      23.0},
        {150,  740, WIRE,      27.0}, {470,  880, WIRE,      21.0},
        {800,  150, WIRE,      25.0}, {340,  430, WIRE,      24.0},
        {110,  600, CAN,       40.0}, {280,  250, CAN,       38.0},
        {440,  580, CAN,       42.0}, {590,  720, CAN,       39.0},
        {760,  490, CAN,       41.0}, {920,  650, CAN,       37.0},
        {350,  870, CAN,       43.0}, {640,  200, CAN,       40.0},
        {190,  420, CAN,       38.0}, {780,  780, CAN,       41.0},
        { 55,  780, TOOL_PART, 35.0}, {245,  130, TOOL_PART, 32.0},
        {490,  400, TOOL_PART, 38.0}, {630,  560, TOOL_PART, 34.0},
        {870,  250, TOOL_PART, 36.0}, {130,  320, TOOL_PART, 33.0},
        {510,  700, TOOL_PART, 37.0}, {720,  900, TOOL_PART, 35.0},
        {380,  320, TOOL_PART, 32.0}, {850,  580, TOOL_PART, 36.0},
        {200,  900, TOOL_PART, 34.0}, {450,  200, TOOL_PART, 33.0},
        {700,  350, TOOL_PART, 37.0}, {950,  490, TOOL_PART, 35.0},
        { 90,  470, TOOL_PART, 38.0}
    };
    int nd = (int)(sizeof(debrisPos) / sizeof(debrisPos[0]));
    debris.clear();
    for (int i = 0; i < nd; i++) {
        MetalDebris d;
        d.x = debrisPos[i].x;
        d.y = debrisPos[i].y;
        d.type = debrisPos[i].t;
        d.magneticStrength = debrisPos[i].str;
        d.triggered = false;
        debris.push_back(d);
    }

    drawFarmBackground();
    createMineVisuals();
    createDebrisVisuals();
    addLegend();

#ifdef WITH_OSG
    createMineOsgVisuals();
    createDebrisOsgVisuals();
#endif

    EV_INFO << "MineField: " << nm << " mines + " << nd << " metal debris placed.\n";
}

double MineField::getMagneticValue(double uavX, double uavY) const
{
    double noise = backgroundNoise
                 + uniform(-noiseVariation / 2.0, noiseVariation / 2.0);
    double mineEffect = 0.0;
    for (const auto& mine : mines) {
        double dx = uavX - mine.x, dy = uavY - mine.y;
        double e  = magneticConstant / (dx*dx + dy*dy + 1.0);
        if (e > mineEffect) mineEffect = e;
    }
    double debrisEffect = 0.0;
    for (const auto& d : debris) {
        double dx = uavX - d.x, dy = uavY - d.y;
        double e = (magneticConstant * d.magneticStrength / 500.0) / (dx*dx + dy*dy + 1.0);
        if (e > debrisEffect) debrisEffect = e;
    }
    return noise + std::max(mineEffect, debrisEffect);
}

int MineField::getNearestUndiscoveredMine(double x, double y, double radius) const
{
    int best = -1; double bestDist = radius;
    for (int i = 0; i < (int)mines.size(); i++) {
        if (mines[i].discovered) continue;
        double d = sqrt(pow(x-mines[i].x,2) + pow(y-mines[i].y,2));
        if (d < bestDist) { bestDist = d; best = i; }
    }
    return best;
}

int MineField::getNearestMetalDebris(double x, double y, double radius) const
{
    int best = -1; double bestDist = radius;
    for (int i = 0; i < (int)debris.size(); i++) {
        if (debris[i].triggered) continue;
        double d = sqrt(pow(x-debris[i].x,2) + pow(y-debris[i].y,2));
        if (d < bestDist) { bestDist = d; best = i; }
    }
    return best;
}

void MineField::markDiscovered(int index)
{
    if (index >= 0 && index < (int)mines.size()) mines[index].discovered = true;
}

void MineField::markDebrisTriggered(int index)
{
    if (index >= 0 && index < (int)debris.size()) debris[index].triggered = true;
}

int MineField::getDiscoveredCount() const
{
    int cnt = 0;
    for (const auto& m : mines) if (m.discovered) cnt++;
    return cnt;
}

void MineField::drawFarmBackground() {}

void MineField::createMineVisuals()
{
    cCanvas *canvas = getSystemModule()->getCanvas();
    const double R=7.0, Rsp=3.0;
    const double spOff[4][2] = {{0,-R-Rsp},{0,R+Rsp},{R+Rsp,0},{-R-Rsp,0}};
    mineFigures.clear();
    for (size_t i = 0; i < mines.size(); i++) {
        double cx=mines[i].x, cy=mines[i].y;
        auto *grp = new cGroupFigure(("mine_"+std::to_string(i)).c_str());
        auto *body = new cOvalFigure("body");
        body->setBounds(cFigure::Rectangle(cx-R,cy-R,2*R,2*R));
        body->setFilled(true);
        body->setFillColor(cFigure::Color(220,0,0));
        body->setLineColor(cFigure::Color(120,0,0));
        body->setLineWidth(2);
        grp->addFigure(body);
        for (int s=0;s<4;s++) {
            auto *sp = new cOvalFigure(("sp"+std::to_string(s)).c_str());
            sp->setBounds(cFigure::Rectangle(cx+spOff[s][0]-Rsp,cy+spOff[s][1]-Rsp,2*Rsp,2*Rsp));
            sp->setFilled(true); sp->setFillColor(cFigure::Color(220,0,0));
            sp->setLineColor(cFigure::Color(120,0,0)); sp->setLineWidth(1);
            grp->addFigure(sp);
        }
        auto *lbl = new cTextFigure("label");
        lbl->setPosition(cFigure::Point(cx,cy));
        lbl->setText(("M"+std::to_string(i+1)).c_str());
        lbl->setColor(cFigure::Color("white"));
        lbl->setAnchor(cFigure::ANCHOR_CENTER);
        lbl->setFont(cFigure::Font("",6,cFigure::FONT_BOLD));
        grp->addFigure(lbl);
        canvas->addFigure(grp);
        mineFigures.push_back(grp);
    }
}

void MineField::createDebrisVisuals()
{
    cCanvas *canvas = getSystemModule()->getCanvas();
    debrisFigures.clear();
    for (size_t i=0;i<debris.size();i++) {
        double cx=debris[i].x, cy=debris[i].y, sz=4.0;
        auto *grp = new cGroupFigure(("debris_"+std::to_string(i)).c_str());
        auto *sq  = new cRectangleFigure("sq");
        sq->setBounds(cFigure::Rectangle(cx-sz,cy-sz,2*sz,2*sz));
        sq->setFilled(true);
        switch(debris[i].type) {
            case NAIL:      sq->setFillColor(cFigure::Color(160,160,160)); break;
            case WIRE:      sq->setFillColor(cFigure::Color(140,140,100)); break;
            case CAN:       sq->setFillColor(cFigure::Color(180,130, 80)); break;
            case TOOL_PART: sq->setFillColor(cFigure::Color(120,100, 80)); break;
        }
        sq->setLineColor(cFigure::Color(80,80,80)); sq->setLineWidth(1);
        grp->addFigure(sq);
        canvas->addFigure(grp);
        debrisFigures.push_back(grp);
    }
}

void MineField::addLegend()
{
    cCanvas *canvas = getSystemModule()->getCanvas();
    auto *lg = new cGroupFigure("legend");
    const double LX=1310,LY=650,LW=260,LH=120,ROW=24;
    auto *bg = new cRectangleFigure("bg");
    bg->setBounds(cFigure::Rectangle(LX,LY,LW,LH));
    bg->setFilled(true); bg->setFillColor(cFigure::Color(245,245,245));
    bg->setFillOpacity(0.95); bg->setLineColor(cFigure::Color(80,80,80)); bg->setLineWidth(1);
    lg->addFigure(bg);
    auto *ttl = new cTextFigure("ttl");
    ttl->setPosition(cFigure::Point(LX+LW/2,LY+12)); ttl->setText("MAD System Legend");
    ttl->setColor(cFigure::Color(30,30,30)); ttl->setAnchor(cFigure::ANCHOR_CENTER);
    ttl->setFont(cFigure::Font("",9,cFigure::FONT_BOLD));
    lg->addFigure(ttl);
    struct { uint8_t fr,fg,fb; bool isRect; const char *txt; } rows[] = {
        {220,0,0,false,"Real Mine (undiscovered)"},
        {0,210,0,false,"Real Mine (discovered by MAD)"},
        {255,220,0,false,"False Alarm (debris detected)"},
        {160,160,160,true,"Metal Debris (nails/wires/cans)"}
    };
    for (int r=0;r<4;r++) {
        double iy=LY+30+r*ROW, ty=iy+9;
        if (rows[r].isRect) {
            auto *sq=new cRectangleFigure(("ld"+std::to_string(r)).c_str());
            sq->setBounds(cFigure::Rectangle(LX+8,iy,14,14)); sq->setFilled(true);
            sq->setFillColor(cFigure::Color(rows[r].fr,rows[r].fg,rows[r].fb));
            sq->setLineColor(cFigure::Color(60,60,60)); sq->setLineWidth(1);
            lg->addFigure(sq);
        } else {
            auto *dot=new cOvalFigure(("ld"+std::to_string(r)).c_str());
            dot->setBounds(cFigure::Rectangle(LX+8,iy,14,14)); dot->setFilled(true);
            dot->setFillColor(cFigure::Color(rows[r].fr,rows[r].fg,rows[r].fb));
            dot->setLineColor(cFigure::Color(60,60,60)); dot->setLineWidth(1);
            lg->addFigure(dot);
        }
        auto *txt=new cTextFigure(("lt"+std::to_string(r)).c_str());
        txt->setPosition(cFigure::Point(LX+28,ty)); txt->setText(rows[r].txt);
        txt->setColor(cFigure::Color(30,30,30)); txt->setAnchor(cFigure::ANCHOR_W);
        txt->setFont(cFigure::Font("",8,0));
        lg->addFigure(txt);
    }
    canvas->addFigure(lg);
}

void MineField::refreshDisplay() const
{
    for (size_t i=0;i<mineFigures.size()&&i<mines.size();i++) {
        if (!mineFigures[i]) continue;
        cFigure::Color fill = mines[i].discovered ? cFigure::Color(0,210,0) : cFigure::Color(220,0,0);
        cFigure::Color line = mines[i].discovered ? cFigure::Color(0,100,0) : cFigure::Color(120,0,0);
        for (int j=0;j<mineFigures[i]->getNumFigures();j++) {
            auto *oval=dynamic_cast<cOvalFigure*>(mineFigures[i]->getFigure(j));
            if (oval) { oval->setFillColor(fill); oval->setLineColor(line); }
        }
    }
#ifdef WITH_OSG
    for (size_t i=0;i<mineOsgNodes.size()&&i<mines.size();i++) {
        if (!mineOsgNodes[i]) continue;
        osg::Geode *geode=dynamic_cast<osg::Geode*>(mineOsgNodes[i]->getChild(0));
        if (!geode||geode->getNumDrawables()==0) continue;
        osg::ShapeDrawable *sd=dynamic_cast<osg::ShapeDrawable*>(geode->getDrawable(0));
        if (!sd) continue;
        sd->setColor(mines[i].discovered
            ? osg::Vec4(0.0f,0.85f,0.0f,1.0f)
            : osg::Vec4(0.9f,0.0f,0.0f,1.0f));
    }
#endif
}

// ============================================================
// OSG 3D
// ============================================================
#ifdef WITH_OSG

// ──────────────────────────────────────────────────────────────
// [FIX]: getOrCreateOsgScene
//
// الخطأ القديم: setOsgCanvas() غير موجودة في cModule API
//
// الحل الصحيح:
//   - getOsgCanvas() فقط — لا setOsgCanvas أبداً
//   - إذا أرجع nullptr → OSG غير مدعوم → نتوقف بهدوء
//   - إذا كان المشهد فارغاً → ننشئ Group ونضعه بـ setScene()
//     (setScene موجودة في cOsgCanvas وليس في cModule)
// ──────────────────────────────────────────────────────────────
osg::ref_ptr<osg::Group> MineField::getOrCreateOsgScene()
{
    cOsgCanvas *osgCanvas = getSystemModule()->getOsgCanvas();
    if (!osgCanvas) {
        EV_WARN << "MineField: getOsgCanvas() returned null."
                   " Build OMNeT++ with OSG support (WITH_OSG=yes).\n";
        return nullptr;
    }

    osg::ref_ptr<osg::Group> scene =
        dynamic_cast<osg::Group*>(osgCanvas->getScene());

    if (!scene) {
        scene = new osg::Group();
        osgCanvas->setScene(scene);   // setScene موجودة في cOsgCanvas ✓
    }

    // تفعيل الإضاءة
    osg::ref_ptr<osg::StateSet> ss = scene->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    ss->setMode(GL_LIGHT0,   osg::StateAttribute::ON);

    // أرضية خضراء (مرة واحدة فقط)
    if (scene->getNumChildren() == 0)
        addGroundPlane(scene);

    return scene;
}

void MineField::addGroundPlane(osg::ref_ptr<osg::Group> scene)
{
    if (!scene) return;
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
    osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array(4);
    (*v)[0]=osg::Vec3(0,0,0); (*v)[1]=osg::Vec3(1000,0,0);
    (*v)[2]=osg::Vec3(1000,1000,0); (*v)[3]=osg::Vec3(0,1000,0);
    geom->setVertexArray(v);
    osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array(1);
    (*c)[0]=osg::Vec4(0.55f,0.76f,0.45f,1.0f);
    geom->setColorArray(c);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    osg::ref_ptr<osg::Vec3Array> n = new osg::Vec3Array(1);
    (*n)[0]=osg::Vec3(0,0,1);
    geom->setNormalArray(n);
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(geom);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    scene->addChild(geode);
}

osg::ref_ptr<osg::MatrixTransform> MineField::makeSphere(
    double x, double y, double z, double radius, float r, float g, float b)
{
    osg::ref_ptr<osg::ShapeDrawable> sd =
        new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,0,0),(float)radius));
    sd->setColor(osg::Vec4(r,g,b,1.0f));
    sd->setUseDisplayList(false);
    sd->setUseVertexBufferObjects(true);
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(sd);
    osg::ref_ptr<osg::Material> mat = new osg::Material();
    mat->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(r,g,b,1.0f));
    mat->setSpecular(osg::Material::FRONT_AND_BACK,osg::Vec4(0.8f,0.8f,0.8f,1.0f));
    mat->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(r*0.3f,g*0.3f,b*0.3f,1.0f));
    mat->setShininess(osg::Material::FRONT_AND_BACK,64.0f);
    geode->getOrCreateStateSet()->setAttribute(mat);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::ON);
    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform();
    mt->setMatrix(osg::Matrix::translate(x,y,z));
    mt->addChild(geode);
    return mt;
}

osg::ref_ptr<osg::MatrixTransform> MineField::makeBox(
    double x, double y, double z, double size, float r, float g, float b)
{
    osg::ref_ptr<osg::ShapeDrawable> sd =
        new osg::ShapeDrawable(new osg::Box(osg::Vec3(0,0,0),(float)size));
    sd->setColor(osg::Vec4(r,g,b,1.0f));
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(sd);
    osg::ref_ptr<osg::Material> mat = new osg::Material();
    mat->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(r,g,b,1.0f));
    mat->setSpecular(osg::Material::FRONT_AND_BACK,osg::Vec4(0.5f,0.5f,0.5f,1.0f));
    mat->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(r*0.3f,g*0.3f,b*0.3f,1.0f));
    mat->setShininess(osg::Material::FRONT_AND_BACK,32.0f);
    geode->getOrCreateStateSet()->setAttribute(mat);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::ON);
    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform();
    mt->setMatrix(osg::Matrix::translate(x,y,z));
    mt->addChild(geode);
    return mt;
}

void MineField::createMineOsgVisuals()
{
    osg::ref_ptr<osg::Group> scene = getOrCreateOsgScene();
    if (!scene) return;
    mineOsgNodes.clear();
    for (size_t i=0;i<mines.size();i++) {
        auto node = makeSphere(mines[i].x, mines[i].y, 8.0, 8.0, 0.9f,0.0f,0.0f);
        scene->addChild(node);
        mineOsgNodes.push_back(node);
    }
    EV_INFO << "MineField OSG: " << mines.size() << " mine spheres added.\n";
}

void MineField::createDebrisOsgVisuals()
{
    cOsgCanvas *osgCanvas = getSystemModule()->getOsgCanvas();
    if (!osgCanvas) return;
    osg::ref_ptr<osg::Group> scene =
        dynamic_cast<osg::Group*>(osgCanvas->getScene());
    if (!scene) return;
    debrisOsgNodes.clear();
    for (size_t i=0;i<debris.size();i++) {
        float r,g,b;
        switch(debris[i].type) {
            case NAIL:      r=0.63f;g=0.63f;b=0.63f; break;
            case WIRE:      r=0.55f;g=0.55f;b=0.39f; break;
            case CAN:       r=0.71f;g=0.51f;b=0.31f; break;
            case TOOL_PART: r=0.47f;g=0.39f;b=0.31f; break;
            default:        r=0.5f; g=0.5f; b=0.5f;  break;
        }
        auto node = makeBox(debris[i].x, debris[i].y, 2.5, 5.0, r,g,b);
        scene->addChild(node);
        debrisOsgNodes.push_back(node);
    }
    EV_INFO << "MineField OSG: " << debris.size() << " debris boxes added.\n";
}

#endif // WITH_OSG

} // namespace uavminedetection
