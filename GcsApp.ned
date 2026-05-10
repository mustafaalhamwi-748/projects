package uavminedetection;

import inet.applications.contract.IApp;

simple GcsApp like IApp
{
    parameters:
        int localPort = default(5000);
        int destPort = default(5000);
        @class(uavminedetection::GcsApp);
        @display("i=block/app");

        // [NEW]: إشارة التغطية الشاملة التراكمية لكل السرب
        @signal[globalCoverage](type=double);
        @statistic[globalCoverage](title="Global Swarm Area Coverage %"; record=mean,max,vector);

    gates:
        input socketIn;
        output socketOut;
}