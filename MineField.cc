#include "MineField.h"
#include <cmath>
#include <string>
#include <algorithm>

namespace uavminedetection {

Define_Module(MineField);

// ============================================================
// initialize — وضع الألغام ورسمها
// ============================================================
void MineField::initialize()
{
    magneticConstant = par("magneticConstant");
    backgroundNoise  = par("backgroundNoise");
    noiseVariation   = par("noiseVariation");

    // ── 20 لغماً موزعاً على منطقة 1000×1000 ──────────────
    // هذه هي الحقيقة — الطائرات لا تعرفها مسبقاً
    struct { double x, y; } pos[] = {
        {  80,  120}, { 220,  75}, { 380, 160}, { 530,  90},
        { 670, 190}, { 820, 110}, { 950, 210}, { 140, 310},
        { 290, 370}, { 460, 290}, { 610, 340}, { 760, 400},
        { 930, 320}, { 100, 520}, { 260, 550}, { 420, 490},
        { 580, 560}, { 740, 510}, { 890, 600}, { 180, 730},
        { 340, 760}, { 510, 820}, { 680, 770}, { 840, 840}
    };

    int n = (int)(sizeof(pos) / sizeof(pos[0]));
    mines.clear();
    for (int i = 0; i < n; i++) {
        MinePos m;
        m.x = pos[i].x;
        m.y = pos[i].y;
        m.discovered = false;
        mines.push_back(m);
    }

    createMineVisuals();
    addLegend();

    EV_INFO << "MineField: " << n
            << " mines placed secretly in 1000x1000m area.\n";
}

// ============================================================
// getMagneticValue — القانون الفيزيائي
//
// القيمة الكلية = ضوضاء الخلفية + مجموع تأثير كل الألغام
// تأثير لغم واحد = k / (d² + 1)
//
// المعادلة مشتقة من قانون التناقص مع المسافة للمجال المغناطيسي
// عند d=0  → تأثير أقصى = k
// عند d=10 → k/101  ← عند العتبة 500: k=50500
// عند d=15 → k/226  ← تحت العتبة
// ============================================================
double MineField::getMagneticValue(double uavX, double uavY) const
{
    // ضوضاء الخلفية المغناطيسية للتربة
    double value = backgroundNoise
                 + uniform(-noiseVariation / 2.0, noiseVariation / 2.0);

    // مجموع تأثير كل الألغام
    for (const auto& mine : mines) {
        double dx = uavX - mine.x;
        double dy = uavY - mine.y;
        double d2 = dx*dx + dy*dy;
        // القانون الفيزيائي: يتناقص مع مربع المسافة
        value += magneticConstant / (d2 + 1.0);
    }

    return value;
}

// ============================================================
// getNearestUndiscoveredMine
// يُستدعى فقط عند قراءة عالية — الطائرة لا تعرف المواقع مسبقاً
// ============================================================
int MineField::getNearestUndiscoveredMine(double x, double y,
                                           double radius) const
{
    int    best     = -1;
    double bestDist = radius;

    for (int i = 0; i < (int)mines.size(); i++) {
        if (mines[i].discovered) continue;   // مكتشف مسبقاً
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
// markDiscovered — تُستدعى عند تأكيد اكتشاف لغم
// ============================================================
void MineField::markDiscovered(int index)
{
    if (index >= 0 && index < (int)mines.size())
        mines[index].discovered = true;
}

int MineField::getDiscoveredCount() const
{
    int cnt = 0;
    for (const auto& m : mines)
        if (m.discovered) cnt++;
    return cnt;
}

// ============================================================
// createMineVisuals — رسم الألغام على Canvas
// ============================================================
void MineField::createMineVisuals()
{
    cCanvas       *canvas = getSystemModule()->getCanvas();
    const double   R      = 7.0;
    const double   Rsp    = 3.0;
    const double   spOff[4][2] = {
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

    EV_INFO << "MineField: " << mineFigures.size()
            << " mine figures drawn on canvas.\n";
}

// ============================================================
// addLegend
// ============================================================
void MineField::addLegend()
{
    cCanvas *canvas = getSystemModule()->getCanvas();
    auto    *lg     = new cGroupFigure("legend");

    // الخلفية
    auto *bg = new cRectangleFigure("bg");
    bg->setBounds(cFigure::Rectangle(5, 1010, 230, 72));
    bg->setFilled(true);
    bg->setFillColor(cFigure::Color(245, 245, 245));
    bg->setFillOpacity(0.88);
    bg->setLineColor(cFigure::Color(100, 100, 100));
    bg->setLineWidth(1);
    lg->addFigure(bg);

    // العنوان
    auto *ttl = new cTextFigure("ttl");
    ttl->setPosition(cFigure::Point(120, 1022));
    ttl->setText("Magnetometer Legend");
    ttl->setColor(cFigure::Color("black"));
    ttl->setAnchor(cFigure::ANCHOR_CENTER);
    ttl->setFont(cFigure::Font("", 8, cFigure::FONT_BOLD));
    lg->addFigure(ttl);

    // الصفوف
    struct {
        int    y1, y2;
        uint8_t fr,fg,fb, lr,lg_,lb;
        const char *txt;
    } rows[] = {
        {1030,1037,  220,  0,  0,  120,  0,  0,
            "Undiscovered Mine"},
        {1048,1055,    0,210,  0,    0,100,  0,
            "Discovered Mine (Magnetometer)"},
        {1066,1073,  255,220,  0,  180,130,  0,
            "False Alarm (Noise Spike)"}
    };

    for (int r = 0; r < 3; r++) {
        auto *dot = new cOvalFigure(
            ("ld" + std::to_string(r)).c_str());
        dot->setBounds(cFigure::Rectangle(12, rows[r].y1, 12, 12));
        dot->setFilled(true);
        dot->setFillColor(
            cFigure::Color(rows[r].fr, rows[r].fg, rows[r].fb));
        dot->setLineColor(
            cFigure::Color(rows[r].lr, rows[r].lg_, rows[r].lb));
        lg->addFigure(dot);

        auto *txt = new cTextFigure(
            ("lt" + std::to_string(r)).c_str());
        txt->setPosition(cFigure::Point(30, rows[r].y2));
        txt->setText(rows[r].txt);
        txt->setColor(cFigure::Color("black"));
        txt->setAnchor(cFigure::ANCHOR_W);
        txt->setFont(cFigure::Font("", 8, 0));
        lg->addFigure(txt);
    }

    canvas->addFigure(lg);
}

// ============================================================
// refreshDisplay — تحديث ألوان الألغام (أحمر → أخضر عند الاكتشاف)
// ============================================================
void MineField::refreshDisplay() const
{
    for (size_t i = 0;
         i < mineFigures.size() && i < mines.size(); i++)
    {
        if (!mineFigures[i]) continue;

        cFigure::Color fill = mines[i].discovered
            ? cFigure::Color(0, 210, 0)    // أخضر: مكتشف
            : cFigure::Color(220, 0, 0);   // أحمر: غير مكتشف

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
