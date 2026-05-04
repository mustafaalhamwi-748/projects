package uavminedetection;

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedOsgVisualizer;
import uavminedetection.UAVNode;

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

        @display("bgb=1000,1000;bgc=#d8f0d8");

    submodules:
        // ── وحدات البنية التحتية ───────────────────────────
        visualizer: IntegratedOsgVisualizer {
            @display("p=1100,50");
        }
        radioMedium: Ieee80211ScalarRadioMedium {
            @display("p=1100,130");
        }
        // ── [تم التصحيح]: configurator مع مسارات ثابتة ──────
        // addStaticRoutes يضمن أن كل عقدة تعرف طريقها لبقية العقد
        configurator: Ipv4NetworkConfigurator {
            parameters:
                addStaticRoutes  = true;
                addDefaultRoutes = false;
                addSubnetRoutes  = false;
                optimizeRoutes   = false;
            @display("p=1100,210");
        }

        // ── حقل الألغام السري ─────────────────────────────
        mineField: MineField {
            @display("p=1100,290");
        }

        // ── الطائرات المسيّرة — تستخدم UAVNode الآن ──────
        // [تم التصحيح: AdhocHost → UAVNode]
        uav[numUAVs]: UAVNode {
            @display("p=100,100");
        }

        // ── محطة التحكم الأرضية ────────────────────────────
        gcs: UAVNode {
            @display("p=500,1000;i=device/antennatower;is=l;t=Ground Control Station");
        }
}