package uavminedetection;

import inet.node.inet.AdhocHost;

module UAVNode extends AdhocHost
{
    parameters:
        @display("i=misc/drone");
        int uavId = default(0);
}
