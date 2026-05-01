package uavminedetection;

import inet.mobility.base.MovingMobilityBase;

//
// LawnmowerMobility — حركة تغطية بخطوط متوازية (Lawnmower)
//
simple LawnmowerMobility extends MovingMobilityBase
{
    parameters:
        double speed @unit(mps) = default(12mps);
        double x1 @unit(m);
        double y1 @unit(m);
        double x2 @unit(m);
        double y2 @unit(m);
        int rowCount;
        double altitude @unit(m) = default(80m);
        @class(uavminedetection::LawnmowerMobility);
}