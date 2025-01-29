#include "artery/application/misbehavior/TrustService.h"
#include "artery/utility/InitStages.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/envmod/TraCIEnvironmentModelObject.h"
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>

namespace artery {

Define_Module(TrustService);

namespace
{

omnetpp::simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");

} // namespace

int TrustService::numInitStages() const
{
    return InitStages::Total;
}

void TrustService::initialize(int stage) {
    if (stage == InitStages::Prepare) {
        ItsG5Service::initialize();
        mPositionProvider = &getFacilities().get_const<PositionProvider>();
        mEnvironmentModel = &getFacilities().get_const<LocalEnvironmentModel>();

        
    } else if (stage == InitStages::Self) {
        subscribe(scSignalCamReceived);
    }
}

void TrustService::receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t signal,
        omnetpp::cObject* obj, omnetpp::cObject*) {
            Enter_Method("receiveSignal");
            
            if(signal == scSignalCamReceived) {
                auto *ca = dynamic_cast<CaObject *>(obj);
                const auto& cam = ca->asn1();

                const auto& trackedObj = filterBySensorCategory(mEnvironmentModel->allObjects(), "Radar");
                for (const auto &i : trackedObj)
                {
                    const auto obj_ptr = i.first.lock();
                    if (!obj_ptr) {
                        continue; /*< objects remain in tracking briefly after leaving simulation */
                    }

                    auto traci_obj_ptr = std::dynamic_pointer_cast<TraCIEnvironmentModelObject>(obj_ptr);
                        if (traci_obj_ptr) {
                            const auto& vd = traci_obj_ptr->getVehicleData();
                            if(vd.getStationId() == cam->header.stationID) {
                                //vehicle in cam is percieved by local sensors
                                EV << "vehicle " << vd.getStationId() << " exists in data and in the world" << endl;
                            }
                    }
                }
                
            }
}
        

} // namespace artery