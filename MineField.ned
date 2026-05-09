package uavminedetection;

// ============================================================
// MineField — وحدة مستقلة تحتوي حقل الألغام
//
// [MODIFIED v3]: إضافة معامل نوع التربة وشكل/اتجاه اللغم
// ============================================================
simple MineField
{
    parameters:
        @class(uavminedetection::MineField);

        // ثابت المجال المغناطيسي (nT·m²)
        double magneticConstant = default(50000);

        // ضوضاء الخلفية الطبيعية للتربة (nT)
        double backgroundNoise  = default(220);

        // تذبذب الضوضاء ± (nT)
        double noiseVariation   = default(50);

        // معاملات عمق الدفن
        double minDepth         = default(0.05);
        double maxDepth         = default(0.40);

        // معامل تخميد التربة (1/متر)
        double soilAttenuation  = default(5.0);

        // [NEW] معامل نوع التربة الثابت
        double soilTypeFactor   = default(1.0);

        @display("p=1100,260;i=misc/globe;is=s");
}