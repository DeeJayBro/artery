#ifndef PCAP_ITS_RECORDER_H
#define PCAP_ITS_RECORDER_H

#include <omnetpp.h>
#include <artery/networking/GeoNetPacket.h>


// taken from https://gitlab.com/Matk3z/artery/-/blob/master/src/artery/networking/platelet/PcapItsRecorder.h
using namespace omnetpp;
namespace artery {

struct pcap_packet_header {
  uint8_t to_addr[6];
  uint8_t from_addr[6];
  uint8_t  protocol[2] = { 0x89, 0x47 };
};
class PcapItsRecorder : public cSimpleModule, protected cListener {
  public:
    PcapItsRecorder();
  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details) override;
        typedef std::map<simsignal_t, bool> SignalList;
    SignalList signalList;
};
}


#endif
