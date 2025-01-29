#ifndef ARTERY_MISBEHAVIORCASERVICE_H_
#define ARTERY_MISBEHAVIORCASERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/networking/PositionProvider.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/misbehavior/attack/util/AttackTypes.h"
#include "traci/API.h"
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>

namespace artery
{

using namespace omnetpp;

class MisbehaviorCaService : public ItsG5Service
{
    protected:
        int numInitStages() const override;
        void initialize(int stage) override;
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;
        void trigger() override;
        void sendCam(uint16_t genDeltaTime);
        void visualizeCamPosition(vanetza::asn1::Cam cam);
        bool checkTriggeringConditions(const omnetpp::SimTime&);
		bool checkHeadingDelta() const;
		bool checkPositionDelta() const;
		bool checkSpeedDelta() const;
		omnetpp::SimTime genCamDcc();
    private:
        const PositionProvider* mPositionProvider = nullptr;
        const LocalEnvironmentModel* mEnvironmentModel = nullptr;
        const VehicleDataProvider* mVehicleDataProvider = nullptr;
        const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
        const Timer* mTimer = nullptr;
        ChannelNumber mPrimaryChannel = channel::CCH;

        std::shared_ptr<const traci::API> mTraciAPI = nullptr;

        std::string mVehicleId;
        attackTypes::AttackTypes mAttackType;
        Latitude_t mConstLatitude;
        Longitude_t mConstLongitude;
        std::list<std::string> activePoIs;
        bool mVisualizeCam;

        omnetpp::SimTime mGenCamMin;
		omnetpp::SimTime mGenCamMax;
		omnetpp::SimTime mGenCam;
		unsigned mGenCamLowDynamicsCounter;
		unsigned mGenCamLowDynamicsLimit;
		Position mLastCamPosition;
		vanetza::units::Velocity mLastCamSpeed;
		vanetza::units::Angle mLastCamHeading;
		omnetpp::SimTime mLastCamTimestamp;
		omnetpp::SimTime mLastLowCamTimestamp;
		vanetza::units::Angle mHeadingDelta;
		vanetza::units::Length mPositionDelta;
		vanetza::units::Velocity mSpeedDelta;
		bool mDccRestriction;
		bool mFixedRate;
};

} // namespace artery

#endif /* ARTERY_MISBEHAVIORCASERVICE_H_ */