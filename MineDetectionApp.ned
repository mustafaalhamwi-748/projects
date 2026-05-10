package uavminedetection;

import inet.applications.contract.IApp;

simple MineDetectionApp like IApp
{
    parameters:
        @class(uavminedetection::MineDetectionApp);

        int    uavId                         = default(0);

        double scanInterval        @unit(s)  = default(0.5s);

        double magneticThreshold             = default(500);
        double magneticSaturation            = default(2000);

        int    adaptiveWindowSize            = default(50);
        double adaptiveKSigma               = default(3.0);
        double adaptiveMinThreshold         = default(200.0);

        double confirmRadius       @unit(m)  = default(12m);
        double confirmationTimeout @unit(s)  = default(30s);

        double falseAlarmProb                = default(0.003);
        int falseAlarmDisplayLimit           = default(50);

        int    destPort                      = default(5555);
        int    localPort                     = default(4000);

        double sensorNoiseAlpha = default(0.8);
        double sensorNoiseScale = default(30.0);
        double returnHomeEnergyThreshold = default(0.3);

        @signal[mineDetected](type=long);
        @signal[areaCoverage](type=double);
        @signal[falseAlarm](type=long);
        @signal[adaptiveThreshold](type=double);
        @signal[correlatedNoise](type=double);

        // ── [NEW]: إشارات الإحصاءات البحثية الجديدة ─────────────
        @signal[timeToConfirm](type=simtime_t);
        @signal[uavsInvolved](type=long);
        @signal[confirmationConfidence](type=double);

        @statistic[mineDetected](title="Mine Detections"; record=count,sum,vector);
        @statistic[areaCoverage](title="Area Coverage %"; record=mean,max,vector);
        @statistic[falseAlarm](title="False Alarms"; record=count,sum);
        @statistic[adaptiveThreshold](title="Adaptive Threshold (nT)"; record=mean,min,max,vector);
        @statistic[correlatedNoise](title="Correlated Sensor Noise (nT)"; record=mean,min,max,vector);

        // ── [NEW]: تسجيل الإحصاءات الجديدة في النتائج ───────────
        @statistic[timeToConfirm](title="Time to Confirm Target (s)"; record=mean,max,vector);
        @statistic[uavsInvolved](title="UAVs Involved in Confirmation"; record=histogram,mean,vector);
        @statistic[confirmationConfidence](title="Confidence at Confirmation"; record=histogram,mean,vector);

        @display("i=block/app2");

    gates:
        input  socketIn  @labels(UdpControlInfo/up);
        output socketOut @labels(UdpControlInfo/down);
}