#include "PcapItsRecorder.h"

#include <vanetza/geonet/basic_header.hpp>
#include<omnetpp.h>
#include "inet/linklayer/ieee80211/mac/Ieee80211Frame_m.h"
#include <pcap/pcap.h>
#include <vanetza/common/byte_buffer_source.hpp>
#include <vanetza/geonet/pdu.hpp>
#include <vanetza/geonet/pdu_conversion.hpp>
#include <vanetza/geonet/pdu_variant.hpp>
#include <vanetza/geonet/common_header.hpp>
#include <vanetza/common/byte_buffer.hpp>
#include <boost/iostreams/stream.hpp>
#include <vanetza/btp/header_conversion.hpp>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/security/secured_message.hpp>

// adapted from https://gitlab.com/Matk3z/artery/-/blob/master/src/artery/networking/platelet/PcapItsRecorder.cc

using namespace vanetza;

namespace artery {
PcapItsRecorder::PcapItsRecorder(): cSimpleModule() {

}

Define_Module(PcapItsRecorder);

    void PcapItsRecorder::initialize()
    {
        EV << "Initializing PcapItsRecorder module" << std::endl;
        const char *outputFile = par("outputFile");
        
        if (!strcmp(outputFile, "")) {
            return;
        }

        signalList.clear();

        {
            cStringTokenizer signalTokenizer(par("sendingSignalNames"));

            while (signalTokenizer.hasMoreTokens())
                signalList[registerSignal(signalTokenizer.nextToken())] = true;
        }

        {
            cStringTokenizer signalTokenizer(par("receivingSignalNames"));

            while (signalTokenizer.hasMoreTokens())
                signalList[registerSignal(signalTokenizer.nextToken())] = false;
        }

        const char *moduleNames = par("moduleNamePatterns");
        cStringTokenizer moduleTokenizer(moduleNames);

        while (moduleTokenizer.hasMoreTokens()) {

            bool found = false;
            std::string mname(moduleTokenizer.nextToken());
            bool isAllIndex = (mname.length() > 3) && mname.rfind("[*]") == mname.length() - 3;

            if (isAllIndex)
                mname.replace(mname.length() - 3, 3, "");

            for (cModule::SubmoduleIterator i(getParentModule()); !i.end(); i++) {
                cModule *submod = *i;
                if (0 == strcmp(isAllIndex ? submod->getName() : submod->getFullName(), mname.c_str())) {
                    found = true;

                    for (auto & elem : signalList) {
                        if (!submod->isSubscribed(elem.first, this)) {
                            submod->subscribe(elem.first, this);
                            EV << "PcapRecorder " << getFullPath() << " subscribed to "
                            << submod->getFullPath() << ":" << getSignalName(elem.first) << endl;
                        }
                    }
                }
            }
        }
        FILE* pcap_fd = fopen(outputFile, "wb");
        pcap_t *handle = pcap_open_dead_with_tstamp_precision(1, 1 << 16, PCAP_TSTAMP_PRECISION_NANO);

        //Initialize pcap file
        pcap_dumper_t *dumper = pcap_dump_fopen(handle, pcap_fd);
        pcap_dump_close(dumper);

    }

    void PcapItsRecorder::handleMessage(cMessage * msg)
    {

    }

    void PcapItsRecorder::finish()
    {
    }

    void serialize_bit_vector(OutputArchive& ar, std::vector<unsigned char, std::allocator<unsigned char>> bit_vector) {
        for (auto iter = bit_vector.begin(); iter < bit_vector.end(); iter++) {
            ar << *iter;
        }
    }

    void serialize_layer(OutputArchive ar, ChunkPacket chunk_packet, OsiLayer osi_layer) {
        convertible::byte_buffer * layer_buffer = chunk_packet.layer(osi_layer).ptr();
        
        ByteBuffer layer_byte_buffer;
        layer_buffer->convert(layer_byte_buffer);
        serialize_bit_vector(ar, layer_byte_buffer);
    }

    ByteBuffer serialize_packet(ChunkPacket packet) {
        
        vanetza::ByteBuffer buf;
        vanetza::byte_buffer_sink sink(buf);
        boost::iostreams::stream_buffer<vanetza::byte_buffer_sink> stream(sink);
        OutputArchive ar(stream);

        geonet::ExtendedPdu<geonet::ShbHeader> pdu = *dynamic_cast<geonet::ExtendedPdu<geonet::ShbHeader>*>(vanetza::geonet::pdu_cast(packet.layer(OsiLayer::Network)));

        // All headers need to be declared as const before serializing to allow template deduction.
        const geonet::BasicHeader& basic_header = pdu.basic();
        const security::SecuredMessage& secured_header = *pdu.secured();
        const geonet::CommonHeader& common_header = pdu.common();
        const geonet::ShbHeader& extended_header = pdu.extended();

        vanetza::geonet::serialize(basic_header, ar);
        if (pdu.secured()) vanetza::security::serialize(ar, secured_header);
        vanetza::geonet::serialize(common_header, ar);
        vanetza::geonet::serialize(extended_header, ar);

        serialize_layer(ar, packet, OsiLayer::Transport);
        serialize_layer(ar, packet, OsiLayer::Application);

        stream.close();
        return buf;
    }

    void write_packet(const char* pcap_filename, uint8_t *packet_buffer, size_t size) {

        // This implementation currently opens a file on every new write
        pcap_t *handle = pcap_open_dead_with_tstamp_precision(1, 1 << 16, PCAP_TSTAMP_PRECISION_NANO);
        pcap_dumper_t *dumper = pcap_dump_open_append(handle, pcap_filename);
        
        auto current_time = simTime();
        struct timeval ts1 = {current_time.inUnit(omnetpp::SimTimeUnit::SIMTIME_S) , current_time.remainderForUnit(omnetpp::SimTimeUnit::SIMTIME_S).inUnit(omnetpp::SimTimeUnit::SIMTIME_NS)};

        struct pcap_pkthdr pcap_hdr;
        pcap_hdr.ts = ts1;
        pcap_hdr.caplen = size + 14;
        pcap_hdr.len = pcap_hdr.caplen;

        pcap_dump((u_char *)dumper, &pcap_hdr, packet_buffer);
        pcap_dump_close(dumper);
    }

    void PcapItsRecorder::receiveSignal(cComponent * source, simsignal_t signalID, cObject * obj, cObject * details)
    {
        const char *outputFile = par("outputFile");
        
        if (!strcmp(outputFile, "")) {
            return;
        }
        const char *className = obj->getClassName();
        inet::ieee80211::Ieee80211DataFrameWithSNAP packet = *dynamic_cast<inet::ieee80211::Ieee80211DataFrameWithSNAP *>(obj);
        auto from_address = packet.getTransmitterAddress();
        auto to_address = packet.getReceiverAddress();

        GeoNetPacket *geonet_packet = dynamic_cast<GeoNetPacket *>(packet.getEncapsulatedPacket());
        std::unique_ptr<vanetza::PacketVariant> packet_variant = std::move(*geonet_packet).extractPayload();
        vanetza::ChunkPacket chunk_packet = boost::get<vanetza::ChunkPacket>(*packet_variant);

        ByteBuffer buffer = serialize_packet(chunk_packet);
        size_t size = buffer.size();
        
        pcap_packet_header packet_header;

        from_address.getAddressBytes(packet_header.from_addr);
        to_address.getAddressBytes(packet_header.to_addr);

        uint8_t *packet_buffer = (uint8_t *)malloc(size + 14);

        memcpy(packet_buffer, &packet_header, sizeof(pcap_packet_header));
        memcpy(packet_buffer + sizeof(pcap_packet_header), &buffer[0], size);

        write_packet(outputFile, packet_buffer, size);
        free(packet_buffer);
    }
}


