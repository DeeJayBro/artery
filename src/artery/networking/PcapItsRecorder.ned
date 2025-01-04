package artery.networking;

// taken from https://gitlab.com/Matk3z/artery/-/blob/master/src/artery/networking/platelet/PcapItsRecorder.ned
simple PcapItsRecorder
{
    parameters:
        @class(PcapItsRecorder);
        @display("i=block/process;is=s");
        string outputFile = default("");
        string moduleNamePatterns = default("wlan[*]"); // space-separated list of sibling module names to listen on
        string sendingSignalNames = default("packetSentToLower"); // space-separated list of outbound packet signals to subscribe to
        string receivingSignalNames = default("packetReceivedFromLower"); // space-separated list of inbound packet signals to subscribe to
}
