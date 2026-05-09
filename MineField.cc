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

    // [NEW]: قراءة معاملات العمق
    minDepth        = par("minDepth");
    maxDepth        = par("maxDepth");
    soilAttenuation = par("soilAttenuation");

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

    // [NEW]: عمق عشوائي لكل لغم
    for (int i = 0; i < nm; i++) {
        MinePos m;
        m.x          = minePos[i].x;
        m.y          = minePos[i].y;
        m.depth      = uniform(minDepth, maxDepth);
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
        d.x = debrisPos[i].x; d.y = debrisPos[i].y;
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

    // طباعة إحصاءات الأعماق
    double minD=maxDepth, maxD=minDepth, sumD=0.0;
    int shallow=0, medium=0, deep=0;
    for (const auto& m : mines) {
        if (m.depth < minD) minD=m.depth;
        if (m.depth > maxD) maxD=m.depth;
        sumD += m.depth;
        if      (m.depth < 0.15) shallow++;
        else if (m.depth < 0.30) medium++;
        else                      deep++;
    }
    EV_INFO << "MineField: " << nm << " mines + " << nd << " debris placed.\n"
            << "  Depth range : " << (minD*100) << "-" << (maxD*100) << " cm"
            << " | avg=" << (sumD/mines.size()*100) << " cm\n"
            << "  Shallow(<15cm)=" << shallow
            << " Medium(15-30cm)=" << medium
            << " Deep(>30cm)=" << deep << "\n"
            << "  SoilAttenuation=" << soilAttenuation << "/m\n"
            << "  Signal@5cm=" << (int)(exp(-soilAttenuation*0.05)*100)
            << "%, @20cm=" << (int)(exp(-soilAttenuation*0.20)*100)
            << "%, @40cm=" << (int)(exp(-soilAttenuation*0.40)*100) << "%\n";
}

// ============================================================
// getMagneticValue — [MODIFIED]: عمق الدفن + تخميد التربة
//
// التأثير المزدوج:
//   1. المسافة الهندسية: dz = uavZ + mine.depth
//      اللغم عند z=-depth، الطائرة عند z=+uavZ
//   2. تخميد التربة (الأهم): exp(-soilAttenuation × depth)
//      soilAttenuation=5.0/م:
//        depth=5cm  → 78% من الإشارة الأصلية
//        depth=20cm → 37% من الإشارة الأصلية
//        depth=40cm → 14% من الإشارة الأصلية
// ============================================================
double MineField::getMagneticValue(double uavX, double uavY, double uavZ) const
{
    double noise = backgroundNoise
                 + uniform(-noiseVariation / 2.0, noiseVariation / 2.0);

    // تأثير الألغام مع العمق والتخميد
    double mineEffect = 0.0;
    for (const auto& mine : mines) {
        double dx = uavX - mine.x;
        double dy = uavY - mine.y;
        // [NEW]: المسافة الرأسية تشمل عمق الدفن
        double dz = uavZ + mine.depth;
        double dist2 = dx*dx + dy*dy + dz*dz + 1.0;

        // [NEW]: تخميد التربة - أسي مع العمق
        double soilFactor = std::exp(-soilAttenuation * mine.depth);

        double e = (magneticConstant * soilFactor) / dist2;
        if (e > mineEffect) mineEffect = e;
    }

    // تأثير الحطام المعدني - على السطح (لا عمق، لا تخميد)
    double debrisEffect = 0.0;
    for (const auto& d : debris) {
        double dx = uavX - d.x;
        double dy = uavY - d.y;
        double dz = uavZ;   // الحطام على السطح
        double dist2 = dx*dx + dy*dy + dz*dz + 1.0;
        double e = (magneticConstant * d.magneticStrength / 500.0) / dist2;
        if (e > debrisEffect) debrisEffect = e;
    }

    return noise + mineEffect + debrisEffect;
}

int MineField::getNearestUndiscoveredMine(double x, double y, double radius) const
{
    int best=-1; double bestDist=radius;
    for (int i=0; i<(int)mines.size(); i++) {
        if (mines[i].discovered) continue;
        double d=std::sqrt(std::pow(x-mines[i].x,2)+std::pow(y-mines[i].y,2));
        if (d<bestDist) { bestDist=d; best=i; }
    }
    return best;
}

int MineField::getNearestMetalDebris(double x, double y, double radius) const
{
    int best=-1; double bestDist=radius;
    for (int i=0; i<(int)debris.size(); i++) {
        if (debris[i].triggered) continue;
        double d=std::sqrt(std::pow(x-debris[i].x,2)+std::pow(y-debris[i].y,2));
        if (d<bestDist) { bestDist=d; best=i; }
    }
    return best;
}

void MineField::markDiscovered(int index)
{
    if (index>=0 && index<(int)mines.size()) mines[index].discovered=true;
}

void MineField::markDebrisTriggered(int index)
{
    if (index>=0 && index<(int)debris.size()) debris[index].triggered=true;
}

int MineField::getDiscoveredCount() const
{
    int cnt=0; for (const auto& m:mines) if (m.discovered) cnt++; return cnt;
}

// [NEW]: لون اللغم يعكس عمقه
cFigure::Color MineField::getMineColorByDepth(double depth) const
{
    if      (depth < 0.15) return cFigure::Color(255,  30,  30); // أحمر فاتح (ضحل)
    else if (depth < 0.30) return cFigure::Color(200,  50,   0); // أحمر برتقالي (متوسط)
    else                   return cFigure::Color(140,   0,   0); // أحمر داكن (عميق)
}

void MineField::drawFarmBackground() {}

// [MODIFIED]: التسمية تعرض العمق + اللون يعكس قابلية الاكتشاف
void MineField::createMineVisuals()
{
    cCanvas *canvas = getSystemModule()->getCanvas();
    const double R=7.0, Rsp=3.0;
    const double spOff[4][2]={{0,-R-Rsp},{0,R+Rsp},{R+Rsp,0},{-R-Rsp,0}};
    mineFigures.clear();

    for (size_t i=0; i<mines.size(); i++) {
        double cx=mines[i].x, cy=mines[i].y;
        cFigure::Color bodyColor = getMineColorByDepth(mines[i].depth);
        cFigure::Color lineColor(
            (int)(bodyColor.red*0.55),
            (int)(bodyColor.green*0.55),
            (int)(bodyColor.blue*0.55));

        auto *grp = new cGroupFigure(("mine_"+std::to_string(i)).c_str());

        auto *body = new cOvalFigure("body");
        body->setBounds(cFigure::Rectangle(cx-R,cy-R,2*R,2*R));
        body->setFilled(true);
        body->setFillColor(bodyColor);
        body->setLineColor(lineColor);
        body->setLineWidth(2);
        grp->addFigure(body);

        for (int s=0;s<4;s++) {
            auto *sp=new cOvalFigure(("sp"+std::to_string(s)).c_str());
            sp->setBounds(cFigure::Rectangle(
                cx+spOff[s][0]-Rsp,cy+spOff[s][1]-Rsp,2*Rsp,2*Rsp));
            sp->setFilled(true);
            sp->setFillColor(bodyColor);
            sp->setLineColor(lineColor);
            sp->setLineWidth(1);
            grp->addFigure(sp);
        }

        // [NEW]: تسمية تعرض رقم اللغم وعمقه — مثال: "M3 (12cm)"
        char lblTxt[24];
        snprintf(lblTxt, sizeof(lblTxt), "M%zu\n%.0fcm", i+1, mines[i].depth*100.0);
        auto *lbl = new cTextFigure("label");
        lbl->setPosition(cFigure::Point(cx,cy));
        lbl->setText(lblTxt);
        lbl->setColor(cFigure::Color("white"));
        lbl->setAnchor(cFigure::ANCHOR_CENTER);
        lbl->setFont(cFigure::Font("",5,cFigure::FONT_BOLD));
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
        double cx=debris[i].x,cy=debris[i].y,sz=4.0;
        auto *grp=new cGroupFigure(("debris_"+std::to_string(i)).c_str());
        auto *sq=new cRectangleFigure("sq");
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

// [MODIFIED]: أضيفت فئات اللون للعمق في Legend
void MineField::addLegend()
{
    cCanvas *canvas = getSystemModule()->getCanvas();
    const double LX=1020,LY=300,LW=165,LH=240;
    auto *lg=new cGroupFigure("legend");
    auto *bg=new cRectangleFigure("bg");
    bg->setBounds(cFigure::Rectangle(LX,LY,LW,LH));
    bg->setFilled(true);
    bg->setFillColor(cFigure::Color(245,245,220));
    bg->setLineColor(cFigure::Color(100,100,100));
    bg->setLineWidth(1);
    lg->addFigure(bg);
    auto *title=new cTextFigure("title");
    title->setPosition(cFigure::Point(LX+LW/2,LY+12));
    title->setText("Legend");
    title->setColor(cFigure::Color(30,30,30));
    title->setAnchor(cFigure::ANCHOR_CENTER);
    title->setFont(cFigure::Font("",9,cFigure::FONT_BOLD));
    lg->addFigure(title);

    struct { bool isRect; int fr,fg,fb; const char* txt; } rows[]={
        {false,255, 30, 30,"Mine shallow (<15cm)"},
        {false,200, 50,  0,"Mine medium (15-30cm)"},
        {false,140,  0,  0,"Mine deep (>30cm)"},
        {false,  0,210,  0,"Mine (discovered)"},
        {true, 160,160,160,"Nail"},
        {true, 140,140,100,"Wire"},
        {true, 180,130, 80,"Can"},
        {true, 120,100, 80,"Tool part"},
    };
    int nrows=(int)(sizeof(rows)/sizeof(rows[0]));
    for (int r=0;r<nrows;r++) {
        double iy=LY+28+r*24,ty=iy+7;
        if (rows[r].isRect) {
            auto *sq=new cRectangleFigure(("ld"+std::to_string(r)).c_str());
            sq->setBounds(cFigure::Rectangle(LX+8,iy,14,14));
            sq->setFilled(true);
            sq->setFillColor(cFigure::Color(rows[r].fr,rows[r].fg,rows[r].fb));
            sq->setLineColor(cFigure::Color(60,60,60)); sq->setLineWidth(1);
            lg->addFigure(sq);
        } else {
            auto *dot=new cOvalFigure(("ld"+std::to_string(r)).c_str());
            dot->setBounds(cFigure::Rectangle(LX+8,iy,14,14));
            dot->setFilled(true);
            dot->setFillColor(cFigure::Color(rows[r].fr,rows[r].fg,rows[r].fb));
            dot->setLineColor(cFigure::Color(60,60,60)); dot->setLineWidth(1);
            lg->addFigure(dot);
        }
        auto *txt=new cTextFigure(("lt"+std::to_string(r)).c_str());
        txt->setPosition(cFigure::Point(LX+28,ty));
        txt->setText(rows[r].txt);
        txt->setColor(cFigure::Color(30,30,30));
        txt->setAnchor(cFigure::ANCHOR_W);
        txt->setFont(cFigure::Font("",8,0));
        lg->addFigure(txt);
    }
    canvas->addFigure(lg);
}

// [MODIFIED]: اللون في refreshDisplay يعكس العمق أيضاً
void MineField::refreshDisplay() const
{
    for (size_t i=0;i<mineFigures.size()&&i<mines.size();i++) {
        if (!mineFigures[i]) continue;
        cFigure::Color fill, line;
        if (mines[i].discovered) {
            fill=cFigure::Color(0,210,0);
            line=cFigure::Color(0,100,0);
        } else {
            fill=getMineColorByDepth(mines[i].depth);
            line=cFigure::Color(
                (int)(fill.red*0.55),
                (int)(fill.green*0.55),
                (int)(fill.blue*0.55));
        }
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
        if (mines[i].discovered) {
            sd->setColor(osg::Vec4(0.0f,0.85f,0.0f,1.0f));
        } else {
            double r1=(mines[i].depth-minDepth)/(maxDepth-minDepth+1e-6);
            float  rc=(float)(0.9-0.4*r1);
            sd->setColor(osg::Vec4(rc,0.0f,0.0f,1.0f));
        }
    }
#endif
}

#ifdef WITH_OSG
osg::ref_ptr<osg::Group> MineField::getOrCreateOsgScene()
{
    cOsgCanvas *osgCanvas=getSystemModule()->getOsgCanvas();
    if (!osgCanvas) { EV_WARN<<"MineField: getOsgCanvas() null.\n"; return nullptr; }
    osg::ref_ptr<osg::Group> scene=dynamic_cast<osg::Group*>(osgCanvas->getScene());
    if (!scene) { scene=new osg::Group(); osgCanvas->setScene(scene); }
    osg::ref_ptr<osg::StateSet> ss=scene->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING,osg::StateAttribute::ON);
    ss->setMode(GL_LIGHT0,osg::StateAttribute::ON);
    if (scene->getNumChildren()==0) addGroundPlane(scene);
    return scene;
}
void MineField::addGroundPlane(osg::ref_ptr<osg::Group> scene)
{
    if (!scene) return;
    osg::ref_ptr<osg::Geometry> geom=new osg::Geometry();
    osg::ref_ptr<osg::Vec3Array> v=new osg::Vec3Array(4);
    (*v)[0]=osg::Vec3(0,0,0);(*v)[1]=osg::Vec3(1000,0,0);
    (*v)[2]=osg::Vec3(1000,1000,0);(*v)[3]=osg::Vec3(0,1000,0);
    geom->setVertexArray(v);
    osg::ref_ptr<osg::Vec4Array> c=new osg::Vec4Array(1);
    (*c)[0]=osg::Vec4(0.55f,0.76f,0.45f,1.0f);
    geom->setColorArray(c); geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    osg::ref_ptr<osg::Vec3Array> n=new osg::Vec3Array(1);
    (*n)[0]=osg::Vec3(0,0,1);
    geom->setNormalArray(n); geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));
    osg::ref_ptr<osg::Geode> geode=new osg::Geode(); geode->addDrawable(geom);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    scene->addChild(geode);
}
osg::ref_ptr<osg::MatrixTransform> MineField::makeSphere(
    double x,double y,double z,double radius,float r,float g,float b)
{
    osg::ref_ptr<osg::ShapeDrawable> sd=
        new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,0,0),(float)radius));
    sd->setColor(osg::Vec4(r,g,b,1.0f));
    sd->setUseDisplayList(false); sd->setUseVertexBufferObjects(true);
    osg::ref_ptr<osg::Geode> geode=new osg::Geode(); geode->addDrawable(sd);
    osg::ref_ptr<osg::Material> mat=new osg::Material();
    mat->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(r,g,b,1.0f));
    mat->setSpecular(osg::Material::FRONT_AND_BACK,osg::Vec4(0.8f,0.8f,0.8f,1.0f));
    mat->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(r*0.3f,g*0.3f,b*0.3f,1.0f));
    mat->setShininess(osg::Material::FRONT_AND_BACK,64.0f);
    geode->getOrCreateStateSet()->setAttribute(mat);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::ON);
    osg::ref_ptr<osg::MatrixTransform> mt=new osg::MatrixTransform();
    mt->setMatrix(osg::Matrix::translate(x,y,z)); mt->addChild(geode);
    return mt;
}
osg::ref_ptr<osg::MatrixTransform> MineField::makeBox(
    double x,double y,double z,double size,float r,float g,float b)
{
    osg::ref_ptr<osg::ShapeDrawable> sd=
        new osg::ShapeDrawable(new osg::Box(osg::Vec3(0,0,0),(float)size));
    sd->setColor(osg::Vec4(r,g,b,1.0f));
    osg::ref_ptr<osg::Geode> geode=new osg::Geode(); geode->addDrawable(sd);
    osg::ref_ptr<osg::Material> mat=new osg::Material();
    mat->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(r,g,b,1.0f));
    mat->setSpecular(osg::Material::FRONT_AND_BACK,osg::Vec4(0.5f,0.5f,0.5f,1.0f));
    mat->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(r*0.3f,g*0.3f,b*0.3f,1.0f));
    mat->setShininess(osg::Material::FRONT_AND_BACK,32.0f);
    geode->getOrCreateStateSet()->setAttribute(mat);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::ON);
    osg::ref_ptr<osg::MatrixTransform> mt=new osg::MatrixTransform();
    mt->setMatrix(osg::Matrix::translate(x,y,z)); mt->addChild(geode);
    return mt;
}
// [MODIFIED]: حجم الكرة يعكس قوة إشارة اللغم (الأكبر = أسهل اكتشافاً)
void MineField::createMineOsgVisuals()
{
    osg::ref_ptr<osg::Group> scene=getOrCreateOsgScene();
    if (!scene) return;
    mineOsgNodes.clear();
    for (size_t i=0;i<mines.size();i++) {
        double signalStr=std::exp(-soilAttenuation*mines[i].depth);
        double radius=4.0+8.0*signalStr;
        double depthRatio=(mines[i].depth-minDepth)/(maxDepth-minDepth+1e-6);
        float rc=(float)(0.9-0.4*depthRatio);
        auto node=makeSphere(mines[i].x,mines[i].y,8.0,radius,rc,0.0f,0.0f);
        scene->addChild(node);
        mineOsgNodes.push_back(node);
    }
}
void MineField::createDebrisOsgVisuals()
{
    cOsgCanvas *osgCanvas=getSystemModule()->getOsgCanvas();
    if (!osgCanvas) return;
    osg::ref_ptr<osg::Group> scene=dynamic_cast<osg::Group*>(osgCanvas->getScene());
    if (!scene) return;
    debrisOsgNodes.clear();
    for (size_t i=0;i<debris.size();i++) {
        float r,g,b;
        switch(debris[i].type) {
            case NAIL:      r=0.63f;g=0.63f;b=0.63f; break;
            case WIRE:      r=0.55f;g=0.55f;b=0.39f; break;
            case CAN:       r=0.71f;g=0.51f;b=0.31f; break;
            case TOOL_PART: r=0.47f;g=0.39f;b=0.31f; break;
            default:        r=0.5f; g=0.5f; b=0.5f;
        }
        auto node=makeBox(debris[i].x,debris[i].y,2.5,5.0,r,g,b);
        scene->addChild(node);
        debrisOsgNodes.push_back(node);
    }
}
#endif

} // namespace uavminedetection
