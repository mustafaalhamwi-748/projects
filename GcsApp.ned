package uavminedetection;

import inet.applications.contract.IApp;

simple GcsApp like IApp
{
    parameters:
        int localPort = default(5000);
        int destPort = default(5000);
        @class(uavminedetection::GcsApp);
        @display("i=block/app");

    gates:
        input socketIn;
        output socketOut;
}