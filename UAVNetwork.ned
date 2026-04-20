package uavminedetection;

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedCanvasVisualizer;
import inet.node.inet.AdhocHost;

// ============================================================
// UAVMineDetection — شبكة 10 طائرات + GCS + حقل ألغام سري
//
// المنطقة: 1000×1000 متر
// الألغام: 24 لغماً بإحداثيات يعرفها MineField فقط
// الطائرات: تكتشف الألغام فقط عند مرورها فوقها
// ============================================================
network UAVMineDetection
{
    parameters:
        int    numUAVs        = default(10);
        double areaX @unit(m) = default(1000m);
        double areaY @unit(m) = default(1000m);

        // 1000px للمنطقة + 100px للـ legend في الأسفل
        @display("bgb=1000,1000;bgc=#d8f0d8");

    submodules:
        // ── وحدات البنية التحتية ───────────────────────────
        visualizer: IntegratedCanvasVisualizer {
            @display("p=1100,50");
        }
        radioMedium: Ieee80211ScalarRadioMedium {
            @display("p=1100,130");
        }
        configurator: Ipv4NetworkConfigurator {
            @display("p=1100,210");
        }

        // ── حقل الألغام السري ─────────────────────────────
        // الطائرات لا تعرف محتواه — تسأله فقط عن قيمة المجال
        mineField: MineField {
            @display("p=1100,290");
        }

        // ── الطائرات المسيّرة ──────────────────────────────
        uav[numUAVs]: AdhocHost {
            @display("p=100,100;i=misc/drone");
        }

        // ── محطة التحكم الأرضية ────────────────────────────
        gcs: AdhocHost {
            @display("p=500,1000;i=device/antennatower;is=l;t=Ground Control Station");
        }
}