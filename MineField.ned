package uavminedetection;

// ============================================================
// MineField — وحدة مستقلة تحتوي حقل الألغام
//
// تعرف مواضع الألغام لكن لا تُخبر الطائرات بها
// تُزوّد الطائرات فقط بقيمة المجال المغناطيسي
// ============================================================
simple MineField
{
    parameters:
        @class(uavminedetection::MineField);

        // ثابت المجال المغناطيسي (nT·m²)
        // عند d=10م: تأثير = 50000/101 ≈ 495 nT
        double magneticConstant = default(50000);

        // ضوضاء الخلفية الطبيعية للتربة (nT)
        double backgroundNoise  = default(220);

        // تذبذب الضوضاء ± (nT)
        double noiseVariation   = default(50);

        @display("p=1100,260;i=misc/globe;is=s");
}
