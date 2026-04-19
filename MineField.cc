#include "MineField.h"
#include <cmath>
#include <string>
#include <algorithm>

namespace uavminedetection {

Define_Module(MineField);

// ============================================================
// initialize
// ============================================================
void MineField::initialize()
{
    magneticConstant = par("magneticConstant");
    backgroundNoise  = par("backgroundNoise");
    noiseVariation   = par("noiseVariation");

    // ── 24 لغماً حقيقياً بمواضع استراتيجية ───────────────
    // موزعة على كامل المنطقة 1000×1000
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

    // ── 50 قطعة معدنية عشوائية ────────────────────────────
    // بقايا معدات زراعية، مسامير، أسلاك، علب
    // قوتها المغناطيسية أضعف من الألغام
    struct { double x, y; MetalType t; double str; } debrisPos[] = {
        // مسامير صغيرة (nail) — قوة منخفضة جداً
        { 45,  200, NAIL,      15.0},
        {165,  430, NAIL,      12.0},
        {310,  180, NAIL,      18.0},
        {480,  650, NAIL,      14.0},
        {620,  430, NAIL,      16.0},
        {750,  280, NAIL,      13.0},
        {880,  710, NAIL,      17.0},
        {120,  860, NAIL,      11.0},
        {390,  940, NAIL,      15.0},
        {710,  590, NAIL,      14.0},
        {830,  460, NAIL,      13.0},
        {960,  780, NAIL,      16.0},
        {200,  650, NAIL,      12.0},
        {550,  150, NAIL,      15.0},
        {670,  880, NAIL,      18.0},

        // أسلاك (wire) — قوة منخفضة
        { 70,  350, WIRE,      25.0},
        {230,  500, WIRE,      22.0},
        {400,  720, WIRE,      28.0},
        {560,  310, WIRE,      24.0},
        {720,  640, WIRE,      26.0},
        {890,  380, WIRE,      23.0},
        {150,  740, WIRE,      27.0},
        {470,  880, WIRE,      21.0},
        {800,  150, WIRE,      25.0},
        {340,  430, WIRE,      24.0},

        // علب معدنية (can) — قوة متوسطة
        {110,  600, CAN,       40.0},
        {280,  250, CAN,       38.0},
        {440,  580, CAN,       42.0},
        {590,  720, CAN,       39.0},
        {760,  490, CAN,       41.0},
        {920,  650, CAN,       37.0},
        {350,  870, CAN,       43.0},
        {640,  200, CAN,       40.0},
        {190,  420, CAN,       38.0},
        {780,  780, CAN,       41.0},

        // بقايا معدات زراعية (tool_part) — قوة متوسطة
        { 55,  780, TOOL_PART, 35.0},
        {245,  130, TOOL_PART, 32.0},
        {490,  400, TOOL_PART, 38.0},
        {630,  560, TOOL_PART, 34.0},
        {870,  250, TOOL_PART, 36.0},
        {130,  320, TOOL_PART, 33.0},
        {510,  700, TOOL_PART, 37.0},
        {720,  900, TOOL_PART, 35.0},
        {380,  320, TOOL_PART, 32.0},
        {850,  580, TOOL_PART, 36.0},
        {200,  900, TOOL_PART, 34.0},
        {450,  200, TOOL_PART, 33.0},
        {700,  350, TOOL_PART, 37.0},
        {950,  490, TOOL_PART, 35.0},
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

    // رسم البيئة
    drawFarmBackground();
    createMineVisuals();
    createDebrisVisuals();
    addLegend();

    EV_INFO << "MineField: " << nm << " mines + "
            << nd << " metal debris placed.\n";
}

// ============================================================
// getMagneticValue — قانون MAD مع تأثير القطع المعدنية
//
// يأخذ MAX بدلاً من SUM لتجنب التراكم الكاذب
// الألغام: تأثير k/d² (أقوى)
// القطع المعدنية: تأثير أضعف بكثير
// ============================================================
double MineField::getMagneticValue(double uavX, double uavY) const
{
    double noise = backgroundNoise
                 + uniform(-noiseVariation / 2.0, noiseVariation / 2.0);

    // ── تأثير الألغام الحقيقية (MAX) ───────────────────────
    double mineEffect = 0.0;
    for (const auto& mine : mines) {
        double dx = uavX - mine.x;
        double dy = uavY - mine.y;
        double d2 = dx*dx + dy*dy;
        double e  = magneticConstant / (d2 + 1.0);
        if (e > mineEffect) mineEffect = e;
    }

    // ── تأثير القطع المعدنية (MAX) ─────────────────────────
    // أضعف بكثير من الألغام لكن قد ترفع القيمة قليلاً
    double debrisEffect = 0.0;
    for (const auto& d : debris) {
        double dx = uavX - d.x;
        double dy = uavY - d.y;
        double d2 = dx*dx + dy*dy;
        // القطع المعدنية لها ثابت أصغر بكثير من الألغام
        double debrisK = magneticConstant * (d.magneticStrength / 500.0);
        double e = debrisK / (d2 + 1.0);
        if (e > debrisEffect) debrisEffect = e;
    }

    // الإجمالي = ضوضاء + أقوى تأثير (لغم أو قطعة معدنية)
    return noise + std::max(mineEffect, debrisEffect);
}

// ============================================================
// getNearestUndiscoveredMine
// ============================================================
int MineField::getNearestUndiscoveredMine(double x, double y,
                                           double radius) const
{
    int    best     = -1;
    double bestDist = radius;

    for (int i = 0; i < (int)mines.size(); i++) {
        if (mines[i].discovered) continue;
        double d = sqrt(pow(x - mines[i].x, 2) +
                        pow(y - mines[i].y, 2));
        if (d < bestDist) {
            bestDist = d;
            best     = i;
        }
    }
    return best;
}

// ============================================================
// getNearestMetalDebris — أقرب قطعة معدنية ضمن radius
// ============================================================
int MineField::getNearestMetalDebris(double x, double y,
                                      double radius) const
{
    int    best     = -1;
    double bestDist = radius;

    for (int i = 0; i < (int)debris.size(); i++) {
        double d = sqrt(pow(x - debris[i].x, 2) +
                        pow(y - debris[i].y, 2));
        if (d < bestDist) {
            bestDist = d;
            best     = i;
        }
    }
    return best;
}

void MineField::markDiscovered(int index)
{
    if (index >= 0 && index < (int)mines.size())
        mines[index].discovered = true;
}

void MineField::markDebrisTriggered(int index)
{
    if (index >= 0 && index < (int)debris.size())
        debris[index].triggered = true;
}

int MineField::getDiscoveredCount() const
{
    int cnt = 0;
    for (const auto& m : mines)
        if (m.discovered) cnt++;
    return cnt;
}

// ============================================================
// drawFarmBackground — رسم بيئة الأرض الزراعية
// خطوط حراثة أفقية + لون بني للتربة
// ============================================================
void MineField::drawFarmBackground()
{
    cCanvas *canvas = getSystemModule()->getCanvas();

    // ── تسمية المنطقة ──────────────────────────────────────
    auto *areaLbl = new cTextFigure("areaLabel");
    areaLbl->setPosition(cFigure::Point(500, 15));
    areaLbl->setText("Agricultural Field — 1km x 1km | MAD Mine Detection");
    areaLbl->setColor(cFigure::Color(60, 40, 10));
    areaLbl->setAnchor(cFigure::ANCHOR_N);
    areaLbl->setFont(cFigure::Font("", 9, cFigure::FONT_BOLD));
    canvas->addFigure(areaLbl);
}

// ============================================================
// createMineVisuals — رسم الألغام الحقيقية (أحمر)
// ============================================================
void MineField::createMineVisuals()
{
    cCanvas      *canvas = getSystemModule()->getCanvas();
    const double  R      = 7.0;
    const double  Rsp    = 3.0;
    const double  spOff[4][2] = {
        { 0, -R-Rsp}, { 0, R+Rsp}, { R+Rsp, 0}, {-R-Rsp, 0}
    };

    mineFigures.clear();

    for (size_t i = 0; i < mines.size(); i++) {
        double cx = mines[i].x;
        double cy = mines[i].y;

        auto *grp = new cGroupFigure(
            ("mine_" + std::to_string(i)).c_str());

        // الجسم الرئيسي
        auto *body = new cOvalFigure("body");
        body->setBounds(cFigure::Rectangle(cx-R, cy-R, 2*R, 2*R));
        body->setFilled(true);
        body->setFillColor(cFigure::Color(220, 0, 0));
        body->setLineColor(cFigure::Color(120, 0, 0));
        body->setLineWidth(2);
        grp->addFigure(body);

        // الزوائد الأربع
        for (int s = 0; s < 4; s++) {
            auto *sp = new cOvalFigure(
                ("sp" + std::to_string(s)).c_str());
            sp->setBounds(cFigure::Rectangle(
                cx + spOff[s][0] - Rsp,
                cy + spOff[s][1] - Rsp,
                2*Rsp, 2*Rsp));
            sp->setFilled(true);
            sp->setFillColor(cFigure::Color(220, 0, 0));
            sp->setLineColor(cFigure::Color(120, 0, 0));
            sp->setLineWidth(1);
            grp->addFigure(sp);
        }

        // التسمية
        auto *lbl = new cTextFigure("label");
        lbl->setPosition(cFigure::Point(cx, cy));
        lbl->setText(("M" + std::to_string(i+1)).c_str());
        lbl->setColor(cFigure::Color("white"));
        lbl->setAnchor(cFigure::ANCHOR_CENTER);
        lbl->setFont(cFigure::Font("", 6, cFigure::FONT_BOLD));
        grp->addFigure(lbl);

        canvas->addFigure(grp);
        mineFigures.push_back(grp);
    }
}

// ============================================================
// createDebrisVisuals — رسم القطع المعدنية العشوائية
// كل نوع له شكل مختلف (مربع صغير رمادي)
// ============================================================
void MineField::createDebrisVisuals()
{
    cCanvas *canvas = getSystemModule()->getCanvas();
    debrisFigures.clear();

    for (size_t i = 0; i < debris.size(); i++) {
        double cx = debris[i].x;
        double cy = debris[i].y;

        auto *grp = new cGroupFigure(
            ("debris_" + std::to_string(i)).c_str());

        // شكل القطعة المعدنية: مربع صغير رمادي
        double sz = 4.0;
        auto *sq = new cRectangleFigure("sq");
        sq->setBounds(cFigure::Rectangle(cx-sz, cy-sz, 2*sz, 2*sz));
        sq->setFilled(true);

        // لون حسب النوع
        switch (debris[i].type) {
            case NAIL:
                sq->setFillColor(cFigure::Color(160, 160, 160)); // رمادي
                break;
            case WIRE:
                sq->setFillColor(cFigure::Color(140, 140, 100)); // رمادي مصفر
                break;
            case CAN:
                sq->setFillColor(cFigure::Color(180, 130, 80));  // بني
                break;
            case TOOL_PART:
                sq->setFillColor(cFigure::Color(120, 100, 80));  // بني داكن
                break;
        }
        sq->setLineColor(cFigure::Color(80, 80, 80));
        sq->setLineWidth(1);
        grp->addFigure(sq);

        canvas->addFigure(grp);
        debrisFigures.push_back(grp);
    }

    EV_INFO << "MineField: " << debrisFigures.size()
            << " metal debris drawn.\n";
}

// ============================================================
// addLegend — وسيلة إيضاح شاملة
// ============================================================
void MineField::addLegend()
{
    cCanvas *canvas = getSystemModule()->getCanvas();
    auto    *lg     = new cGroupFigure("legend");

    // الخلفية — خارج حدود المنطقة (يمين الحقل)
    auto *bg = new cRectangleFigure("bg");
    bg->setBounds(cFigure::Rectangle(1030, 50, 310, 115));
    bg->setFilled(true);
    bg->setFillColor(cFigure::Color(245, 240, 225));
    bg->setFillOpacity(0.92);
    bg->setLineColor(cFigure::Color(100, 80, 40));
    bg->setLineWidth(2);
    lg->addFigure(bg);

    // العنوان
    auto *ttl = new cTextFigure("ttl");
    ttl->setPosition(cFigure::Point(1185, 62));
    ttl->setText("MAD System Legend — Agricultural Field");
    ttl->setColor(cFigure::Color(60, 40, 10));
    ttl->setAnchor(cFigure::ANCHOR_CENTER);
    ttl->setFont(cFigure::Font("", 8, cFigure::FONT_BOLD));
    lg->addFigure(ttl);

    // الصفوف
    struct {
        int    y1, y2;
        bool   isRect;
        uint8_t fr,fg,fb;
        const char *txt;
    } rows[] = {
        { 78,  85, false, 220,  0,  0,
            "Real Mine (undiscovered)"},
        { 96, 103, false,   0,210,  0,
            "Real Mine (discovered by MAD)"},
        {114, 121, false, 255,220,  0,
            "False Alarm (metal debris detected)"},
        {132, 139, true,  160,160,160,
            "Metal Debris (nails/wires/cans/tools)"}
    };

    for (int r = 0; r < 4; r++) {
        if (rows[r].isRect) {
            auto *sq = new cRectangleFigure(
                ("ld" + std::to_string(r)).c_str());
            sq->setBounds(cFigure::Rectangle(1037, rows[r].y1, 12, 12));
            sq->setFilled(true);
            sq->setFillColor(
                cFigure::Color(rows[r].fr, rows[r].fg, rows[r].fb));
            sq->setLineColor(cFigure::Color(80, 80, 80));
            lg->addFigure(sq);
        } else {
            auto *dot = new cOvalFigure(
                ("ld" + std::to_string(r)).c_str());
            dot->setBounds(
                cFigure::Rectangle(1037, rows[r].y1, 12, 12));
            dot->setFilled(true);
            dot->setFillColor(
                cFigure::Color(rows[r].fr, rows[r].fg, rows[r].fb));
            dot->setLineColor(cFigure::Color(80, 80, 80));
            lg->addFigure(dot);
        }

        auto *txt = new cTextFigure(
            ("lt" + std::to_string(r)).c_str());
        txt->setPosition(cFigure::Point(1055, rows[r].y2));
        txt->setText(rows[r].txt);
        txt->setColor(cFigure::Color(40, 30, 10));
        txt->setAnchor(cFigure::ANCHOR_W);
        txt->setFont(cFigure::Font("", 8, 0));
        lg->addFigure(txt);
    }

    canvas->addFigure(lg);
}

// ============================================================
// refreshDisplay
// ============================================================
void MineField::refreshDisplay() const
{
    // تحديث ألوان الألغام: أحمر → أخضر عند الاكتشاف
    for (size_t i = 0;
         i < mineFigures.size() && i < mines.size(); i++)
    {
        if (!mineFigures[i]) continue;

        cFigure::Color fill = mines[i].discovered
            ? cFigure::Color(0, 210, 0)
            : cFigure::Color(220, 0, 0);
        cFigure::Color line = mines[i].discovered
            ? cFigure::Color(0, 100, 0)
            : cFigure::Color(120, 0, 0);

        for (int j = 0; j < mineFigures[i]->getNumFigures(); j++) {
            auto *oval = dynamic_cast<cOvalFigure*>(
                mineFigures[i]->getFigure(j));
            if (oval) {
                oval->setFillColor(fill);
                oval->setLineColor(line);
            }
        }
    }
}

} // namespace uavminedetection
