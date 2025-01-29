#include "artery/application/misbehavior/attack/MisbehaviorCaService.h"
#include "artery/application/CaService.h"
#include "artery/application/CaObject.h"
#include "artery/utility/InitStages.h"
#include "artery/envmod/TraCIEnvironmentModelObject.h"
#include "artery/traci/VehicleController.h"
#include "artery/utility/simtime_cast.h"
#include <boost/units/cmath.hpp>
#include <vanetza/btp/ports.hpp>
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
#include <artery/traci/Cast.h>
#include <boost/units/systems/si/prefixes.hpp>
#include <vanetza/facilities/cam_functions.hpp>

//inspired by a4md

namespace artery {

Define_Module(MisbehaviorCaService);

namespace
{

omnetpp::simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
auto microdegree = vanetza::units::degree * boost::units::si::micro;
auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;
static const auto scLowFrequencyContainerInterval = std::chrono::milliseconds(500);

} // namespace

// copied from CaService.cc
template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}

SpeedValue_t buildSpeedValue(const vanetza::units::Velocity& v)
{
	static const vanetza::units::Velocity lower { 0.0 * boost::units::si::meter_per_second };
	static const vanetza::units::Velocity upper { 163.82 * boost::units::si::meter_per_second };

	SpeedValue_t speed = SpeedValue_unavailable;
	if (v >= upper) {
		speed = 16382; // see CDD A.74 (TS 102 894 v1.2.1)
	} else if (v >= lower) {
		speed = round(v, centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
	}
	return speed;
}

int MisbehaviorCaService::numInitStages() const
{
    return InitStages::Total;
}

void MisbehaviorCaService::initialize(int stage) {
    if (stage == InitStages::Prepare) {
        ItsG5Service::initialize();
        mPositionProvider = &getFacilities().get_const<PositionProvider>();
        mEnvironmentModel = &getFacilities().get_const<LocalEnvironmentModel>();
        mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
        mTimer = &getFacilities().get_const<Timer>();
        mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();

        auto vehicleController = &getFacilities().get_const<traci::VehicleController>();
        mVehicleId = vehicleController->getId();

        mTraciAPI = vehicleController->getTraCI();
        
    } else if (stage == InitStages::Self) {
        // avoid unreasonable high elapsed time values for newly inserted vehicles
        mLastCamTimestamp = simTime();

        // first generated CAM shall include the low frequency container
        mLastLowCamTimestamp = mLastCamTimestamp - artery::simtime_cast(scLowFrequencyContainerInterval);

        // generation rate boundaries
        mGenCamMin = par("minInterval");
        mGenCamMax = par("maxInterval");
        mGenCam = mGenCamMax;

        // vehicle dynamics thresholds
        mHeadingDelta = vanetza::units::Angle { par("headingDelta").doubleValue() * vanetza::units::degree };
        mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
        mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;

        mDccRestriction = par("withDccRestriction");
        mFixedRate = par("fixedRate");

        // look up primary channel for CA
        mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::CA);


        // misbehavior specific initializations
        mVisualizeCam = par("camLocationVisualizer");
        mTraciAPI->vehicle.setColor(mVehicleId, libsumo::TraCIColor(255, 0, 0));

        auto netBoundary = mTraciAPI->simulation.getNetBoundary();
        auto maxPosX = std::max(netBoundary.value.front().x,netBoundary.value.back().x);
        auto maxPosY = std::max(netBoundary.value.front().y,netBoundary.value.back().y);
        auto minPosX = std::min(netBoundary.value.front().x,netBoundary.value.back().x);
        auto minPosY = std::min(netBoundary.value.front().y,netBoundary.value.back().y);
        libsumo::TraCIPosition maxPos;
        maxPos.x = maxPosX;
        maxPos.y = maxPosY;
        auto traciMaxGeo = mTraciAPI->convertGeo(maxPos);
        libsumo::TraCIPosition minPos;
        minPos.x = minPosX;
        minPos.y = minPosY;
        auto traciMinGeo = mTraciAPI->convertGeo(minPos);

        artery::GeoPosition constGeo;
        constGeo.latitude = uniform(traciMaxGeo.latitude, traciMinGeo.latitude) * boost::units::degree::degree;
        constGeo.longitude = uniform(traciMaxGeo.longitude, traciMinGeo.longitude) * boost::units::degree::degree;

        mConstLatitude = round(constGeo.latitude, microdegree) * Latitude_oneMicrodegreeNorth;
        mConstLongitude = round(constGeo.longitude, microdegree) * Longitude_oneMicrodegreeEast;

        mAttackType = attackTypes::AttackTypes((int) par("staticAttackType"));
        par("AttackType").setStringValue(attackTypes::AttackNames[mAttackType]);

        subscribe(scSignalCamReceived);
    }
}

void MisbehaviorCaService::receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t signal,
        omnetpp::cObject* obj, omnetpp::cObject*) {
            Enter_Method("receiveSignal");
            
            if(signal == scSignalCamReceived) {
                auto *ca = dynamic_cast<CaObject *>(obj);
                const auto& cam = ca->asn1();
                
            }
}

void MisbehaviorCaService::trigger() {
    Enter_Method("trigger");
    uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
    if (mAttackType == attackTypes::Disruptive || mAttackType == attackTypes::DataReplay ||
            mAttackType == attackTypes::DoS || mAttackType == attackTypes::DoSRandom ||
            mAttackType == attackTypes::DoSDisruptive || mAttackType == attackTypes::GridSybil ||
            mAttackType == attackTypes::DataReplaySybil || mAttackType == attackTypes::DoSRandomSybil ||
            mAttackType == attackTypes::DoSDisruptiveSybil || checkTriggeringConditions(simTime())) {
            sendCam(genDeltaTimeMod);
        }
}

void MisbehaviorCaService::sendCam(uint16_t genDeltaTime) {
    auto cam = createCooperativeAwarenessMessage(*mVehicleDataProvider, genDeltaTime);

    //modify cam depending on attack type
    // taken from a4md
    switch (mAttackType) {
            case attackTypes::Benign:
                break;
            case attackTypes::ConstPos: {
                cam->cam.camParameters.basicContainer.referencePosition.latitude =
                        mConstLatitude;
                cam->cam.camParameters.basicContainer.referencePosition.longitude =
                        mConstLongitude;
                break;
            }
            /*case attackTypes::ConstPosOffset: {
                cam->cam.camParameters.basicContainer.referencePosition.latitude =
                        (round(mVehicleDataProvider->latitude(), microdegree) +
                         AttackConstantPositionOffsetLatitudeMicrodegrees) *
                        Latitude_oneMicrodegreeNorth;
                cam->cam.camParameters.basicContainer.referencePosition.longitude =
                        (round(mVehicleDataProvider->longitude(), microdegree) +
                         AttackConstantPositionOffsetLongitudeMicrodegrees) *
                        Longitude_oneMicrodegreeEast;
                break;
            }
            case attackTypes::RandomPos: {
                long attackLatitude =
                        (long) (uniform(-F2MDParameters::attackParameters.AttackRandomPositionMinLatitude,
                                        F2MDParameters::attackParameters.AttackRandomPositionMaxLatitude) * 1000000);
                long attackLongitude =
                        (long) (uniform(-F2MDParameters::attackParameters.AttackRandomPositionMinLongitude,
                                        F2MDParameters::attackParameters.AttackRandomPositionMaxLongitude) * 1000000);
                cam->cam.camParameters.basicContainer.referencePosition.latitude =
                        attackLatitude * Latitude_oneMicrodegreeNorth;
                cam->cam.camParameters.basicContainer.referencePosition.longitude =
                        attackLongitude * Longitude_oneMicrodegreeEast;
                break;
            }
            case attackTypes::RandomPosOffset: {
                long attackLatitudeOffset = (long) (
                        uniform(-F2MDParameters::attackParameters.AttackRandomPositionOffsetMaxLatitudeOffset,
                                F2MDParameters::attackParameters.AttackRandomPositionOffsetMaxLatitudeOffset) *
                        1000000);
                long attackLongitudeOffset = (long) (
                        uniform(-F2MDParameters::attackParameters.AttackRandomPositionOffsetMaxLongitudeOffset,
                                F2MDParameters::attackParameters.AttackRandomPositionOffsetMaxLongitudeOffset) *
                        1000000);
                cam->cam.camParameters.basicContainer.referencePosition.latitude =
                        (round(mVehicleDataProvider->latitude(), microdegree) + attackLatitudeOffset) *
                        Latitude_oneMicrodegreeNorth;
                cam->cam.camParameters.basicContainer.referencePosition.longitude =
                        (round(mVehicleDataProvider->longitude(), microdegree) + attackLongitudeOffset) *
                        Longitude_oneMicrodegreeEast;
                break;
            }
            case attackTypes::ConstSpeed: {
                cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = AttackConstantSpeedValue;
                break;
            }
            case attackTypes::ConstSpeedOffset: {
                vanetza::units::Velocity speed;
                if (mVehicleDataProvider->speed() + attackConstantSpeedOffsetValue <
                    0 * boost::units::si::meter_per_second) {
                    speed = 0 * boost::units::si::meter_per_second;
                } else {
                    speed = mVehicleDataProvider->speed() + attackConstantSpeedOffsetValue;
                }
                cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue =
                        buildSpeedValue(speed);
                break;
            }
            case attackTypes::RandomSpeed: {
                double randomSpeed = uniform(F2MDParameters::attackParameters.AttackRandomSpeedMin,
                                             F2MDParameters::attackParameters.AttackRandomSpeedMax);
                cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = buildSpeedValue(
                        randomSpeed * boost::units::si::meter_per_second);
                break;
            }
            case attackTypes::RandomSpeedOffset: {
                double speed = std::min(0.0, uniform(-F2MDParameters::attackParameters.AttackRandomSpeedOffsetMax,
                                                     F2MDParameters::attackParameters.AttackRandomSpeedOffsetMax));
                cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = buildSpeedValue(
                        mVehicleDataProvider->speed() +
                        (speed * boost::units::si::meter_per_second));
                break;
            }
            case attackTypes::EventualStop: {
                if (attackEventualStopHasStopped) {
                    cam->cam.camParameters.basicContainer.referencePosition = attackEventualStopPosition;
                    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = 0;
                    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = 0;
                } else {
                    if (F2MDParameters::attackParameters.AttackEventualStopProbabilityThreshold > uniform(0, 1)) {
                        attackEventualStopHasStopped = true;
                        attackEventualStopPosition = cam->cam.camParameters.basicContainer.referencePosition;
                        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = 0;
                        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = 0;
                    } else {
                    }
                }
                break;
            }
            case attackTypes::Disruptive: {
                if (disruptiveMessageQueue.size() >= F2MDParameters::attackParameters.AttackDisruptiveMinimumReceived) {
                    int index = (int) uniform(0, (double) disruptiveMessageQueue.size());
                    auto it = disruptiveMessageQueue.begin();
                    std::advance(it, index);
                    message = *it;
                    disruptiveMessageQueue.erase(it);
                    message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                            mTimer->getTimeFor(mVehicleDataProvider->updated()));
                    message->header.stationID = mVehicleDataProvider->getStationId();
                } else {
                    message = vanetza::asn1::Cam();
                }
                break;
            }
            case attackTypes::DataReplay: {
                message = getNextReplayCam();
                if (message->header.messageID != 2) {
                    break;
                } else {
                    message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                            mTimer->getTimeFor(mVehicleDataProvider->updated()));
                    message->header.stationID = mVehicleDataProvider->getStationId();
                }
                break;
            }
            case attackTypes::StaleMessages: {
                staleMessageQueue.push(message);
                if (staleMessageQueue.size() >= F2MDParameters::attackParameters.AttackStaleDelayCount) {
                    cam = staleMessageQueue.front();
                    staleMessageQueue.pop();
                    cam->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                            mTimer->getTimeFor(mVehicleDataProvider->updated()));
                } else {
                    cam = vanetza::asn1::Cam();
                }
                break;
            }
            case attackTypes::DoS: {
                cam->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(mTimer->getTimeFor(simTime()));
                break;
            }
            case attackTypes::DoSRandom: {
                cam->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(mTimer->getTimeFor(simTime()));
                long attackLatitude =
                        (long) (uniform(-F2MDParameters::attackParameters.AttackRandomPositionMinLatitude,
                                        F2MDParameters::attackParameters.AttackRandomPositionMaxLatitude) * 1000000);
                long attackLongitude =
                        (long) (uniform(-F2MDParameters::attackParameters.AttackRandomPositionMinLongitude,
                                        F2MDParameters::attackParameters.AttackRandomPositionMaxLongitude) * 1000000);
                cam->cam.camParameters.basicContainer.referencePosition.latitude =
                        attackLatitude * Latitude_oneMicrodegreeNorth;
                cam->cam.camParameters.basicContainer.referencePosition.longitude =
                        attackLongitude * Longitude_oneMicrodegreeEast;
                double randomSpeed = uniform(F2MDParameters::attackParameters.AttackRandomSpeedMin,
                                             F2MDParameters::attackParameters.AttackRandomSpeedMax);
                cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = buildSpeedValue(
                        randomSpeed * boost::units::si::meter_per_second);
                break;
            }
            case attackTypes::DoSDisruptive: {
                if (!receivedMessages.empty()) {
                    auto it = receivedMessages.begin();
                    int index = (int) uniform(0, (double) receivedMessages.size());
                    std::advance(it, index);
                    if (!it->second.empty()) {
                        message = it->second.front();
                        it->second.pop_front();
                        if (it->second.empty()) {
                            receivedMessages.erase(it->first);
                        }
                        message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                                mTimer->getTimeFor(mVehicleDataProvider->updated()));
                        message->header.stationID = mVehicleDataProvider->getStationId();
                    } else {
                        receivedMessages.erase(it->first);
                        message = vanetza::asn1::Cam();
                    }
                } else {
                    message = vanetza::asn1::Cam();
                }
                break;
            }
            case attackTypes::GridSybil: {
                int offsetIndex;
                if (F2MDParameters::attackParameters.AttackGridSybilSelfSybil) {
                    offsetIndex = attackGridSybilCurrentVehicleIndex;
                } else {
                    message = getNextReplayCam();
                    if (message->header.messageID != 2) {
                        message = vanetza::asn1::Cam();
                        break;
                    } else {
                        offsetIndex = attackGridSybilCurrentVehicleIndex + 1;
                    }
                }
                BasicVehicleContainerHighFrequency &hfc =
                        message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
                double width = hfc.vehicleWidth == VehicleWidth_unavailable ?
                               mVehicleController->getLength().value() :
                               (double) hfc.vehicleWidth / 10;
                double length = hfc.vehicleLength.vehicleLengthValue == VehicleLengthValue_unavailable ?
                                mVehicleController->getLength().value() :
                                (double) hfc.vehicleLength.vehicleLengthValue / 10;
                double offsetX = -((double) offsetIndex / 2) *
                                 (width + attackGridSybilActualDistanceX) +
                                 uniform(-attackGridSybilActualDistanceX / 10, attackGridSybilActualDistanceX / 10);
                double offsetY = -((double) (offsetIndex % 2)) *
                                 (length + attackGridSybilActualDistanceY) +
                                 uniform(-attackGridSybilActualDistanceY / 10, attackGridSybilActualDistanceY / 10);

                ReferencePosition_t &referencePosition = message->cam.camParameters.basicContainer.referencePosition;
                Position originalPosition = convertReferencePosition(referencePosition, mSimulationBoundary, mTraciAPI);

                Position relativePosition = Position(offsetX, offsetY);
                double currentHeadingAngle = (double) hfc.heading.headingValue / 10.0;
                double newAngle = currentHeadingAngle + calculateHeadingAngle(relativePosition);
                newAngle = 360 - std::fmod(newAngle, 360);

                double offsetDistance = sqrt(pow(offsetX, 2) + pow(offsetY, 2));
                double relativeX = offsetDistance * sin(newAngle * PI / 180);
                double relativeY = offsetDistance * cos(newAngle * PI / 180);
                Position sybilPosition = Position(originalPosition.x.value() + relativeX,
                                                  originalPosition.y.value() + relativeY);
                if (getDistanceToNearestRoad(mGlobalEnvironmentModel, sybilPosition) >
                    F2MDParameters::attackParameters.AttackGridSybilMaxDistanceFromRoad) {
                    message = vanetza::asn1::Cam();
                    break;
                }
                setPositionWithJitter(referencePosition, sybilPosition, mSimulationBoundary, mTraciAPI, getRNG(0));

                if (hfc.speed.speedConfidence != SpeedConfidence_unavailable) {
                    long speedConfidence = hfc.speed.speedConfidence;
                    hfc.speed.speedValue = intuniform(std::max(0, (int) (hfc.speed.speedValue - speedConfidence)),
                                                      std::min(16382,
                                                               (int) (hfc.speed.speedValue + speedConfidence)));
                }
                if (hfc.longitudinalAcceleration.longitudinalAccelerationConfidence !=
                    AccelerationConfidence_unavailable) {
                    long accelerationConfidence = hfc.longitudinalAcceleration.longitudinalAccelerationConfidence;
                    hfc.longitudinalAcceleration.longitudinalAccelerationValue =
                            intuniform(std::max(-160,
                                                (int) (hfc.longitudinalAcceleration.longitudinalAccelerationValue -
                                                       accelerationConfidence)),
                                       std::min(160,
                                                (int) (hfc.longitudinalAcceleration.longitudinalAccelerationValue +
                                                       accelerationConfidence)));
                }
                if (hfc.heading.headingConfidence != HeadingConfidence_unavailable) {
                    long headingConfidence = hfc.heading.headingConfidence;
                    long newHeading = intuniform((int) (hfc.heading.headingValue - headingConfidence),
                                                 (int) (hfc.heading.headingValue + headingConfidence));
                    hfc.heading.headingValue = (newHeading + 3600) % 3600;
                }

                double steeringAngle = std::fmod(attackGridSybilLastHeadingAngle - currentHeadingAngle, 360);
                steeringAngle = steeringAngle > 180 ? 360 - steeringAngle : steeringAngle;
                attackGridSybilLastHeadingAngle = currentHeadingAngle;
                if (steeringAngle > 5 && attackGridSybilCurrentVehicleIndex > 0) {
                    message = vanetza::asn1::Cam();
                    break;
                }

                attackGridSybilCurrentVehicleIndex = ++attackGridSybilCurrentVehicleIndex % attackGridSybilVehicleCount;
                message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                        mTimer->getCurrentTime());
                message->header.stationID = mPseudonyms[mPseudonymIndex++];
                mPseudonymIndex %= attackGridSybilVehicleCount;
                break;
            }
            case attackTypes::DataReplaySybil: {
                if (attackDataReplayCurrentStationId == -1) {
                    auto it = receivedMessages.begin();
                    if (it != receivedMessages.end()) {
                        uint32_t mostReceivedStationId = -1;
                        unsigned long maxSize = 0;
                        for (; it != receivedMessages.end(); ++it) {
                            if (receivedMessages[it->first].size() > maxSize) {
                                maxSize = receivedMessages[it->first].size();
                                mostReceivedStationId = it->first;
                            }
                        }
                        attackDataReplayCurrentStationId = mostReceivedStationId;
                        message = receivedMessages[attackDataReplayCurrentStationId].front();
                        receivedMessages[attackDataReplayCurrentStationId].pop_front();
                        message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                                mTimer->getTimeFor(mVehicleDataProvider->updated()));
                        message->header.stationID = mPseudonyms.front();
                    } else {
                        message = vanetza::asn1::Cam();
                    }
                } else {
                    if (!receivedMessages[attackDataReplayCurrentStationId].empty()) {
                        message = receivedMessages[attackDataReplayCurrentStationId].front();
                        receivedMessages[attackDataReplayCurrentStationId].pop_front();
                        message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                                mTimer->getTimeFor(mVehicleDataProvider->updated()));
                        message->header.stationID = mPseudonyms.front();
                    } else {
                        receivedMessages.erase(attackDataReplayCurrentStationId);
                        attackDataReplayCurrentStationId = -1;
                        message = vanetza::asn1::Cam();
                        mPseudonymIndex = ++mPseudonymIndex % attackGridSybilVehicleCount;
                    }
                }
                break;
            }
            case attackTypes::DoSRandomSybil: {
                cam->header.stationID = mPseudonyms[mPseudonymIndex++];
                mPseudonymIndex %= attackGridSybilVehicleCount;
                cam->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(mTimer->getTimeFor(simTime()));
                long attackLatitude =
                        (long) (uniform(-F2MDParameters::attackParameters.AttackRandomPositionMinLatitude,
                                        F2MDParameters::attackParameters.AttackRandomPositionMaxLatitude) * 1000000);
                long attackLongitude =
                        (long) (uniform(-F2MDParameters::attackParameters.AttackRandomPositionMinLongitude,
                                        F2MDParameters::attackParameters.AttackRandomPositionMaxLongitude) * 1000000);
                cam->cam.camParameters.basicContainer.referencePosition.latitude =
                        attackLatitude * Latitude_oneMicrodegreeNorth;
                cam->cam.camParameters.basicContainer.referencePosition.longitude =
                        attackLongitude * Longitude_oneMicrodegreeEast;
                double randomSpeed = uniform(F2MDParameters::attackParameters.AttackRandomSpeedMin,
                                             F2MDParameters::attackParameters.AttackRandomSpeedMax);
                cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = buildSpeedValue(
                        randomSpeed * boost::units::si::meter_per_second);
                break;
            }
            case attackTypes::DoSDisruptiveSybil: {
                if (!receivedMessages.empty()) {
                    auto it = receivedMessages.begin();
                    int index = (int) uniform(0, (double) receivedMessages.size());
                    std::advance(it, index);
                    if (!it->second.empty()) {
                        message = it->second.front();
                        it->second.pop_front();
                        if (it->second.empty()) {
                            receivedMessages.erase(it->first);
                        }
                        message->cam.generationDeltaTime = (uint16_t) countTaiMilliseconds(
                                mTimer->getTimeFor(mVehicleDataProvider->updated()));
                        message->header.stationID = mPseudonyms[mPseudonymIndex++];
                        mPseudonymIndex %= attackGridSybilVehicleCount;
                    } else {
                        receivedMessages.erase(it->first);
                        cam = vanetza::asn1::Cam();
                    }
                } else {
                    cam = vanetza::asn1::Cam();
                }
                break;
            }
            case attackTypes::FakeReport: {
                break;
            }*/
            default:
                break;
    }

    // save values only after misbehavior induction to have realistic cam sending behaviour
    mLastCamPosition = mVehicleDataProvider->position();
	mLastCamSpeed = mVehicleDataProvider->speed();
	mLastCamHeading = mVehicleDataProvider->heading();
	mLastCamTimestamp = simTime();

    // send code copied from CaService.cc
    using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::CAM;
	request.gn.its_aid = aid::CA;
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    if(mVisualizeCam) {
        visualizeCamPosition(cam);
    }
	CaObject obj(std::move(cam));
	//emit(scSignalCamSent, &obj);

	using CamByteBuffer = convertible::byte_buffer_impl<asn1::Cam>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new CamByteBuffer(obj.shared_ptr()) };
	payload->layer(OsiLayer::Application) = std::move(buffer);
	this->request(request, std::move(payload));
}

void MisbehaviorCaService::visualizeCamPosition(vanetza::asn1::Cam cam) {

        std::vector<libsumo::TraCIColor> colors = {libsumo::TraCIColor(255, 0, 255, 255),
                                                   libsumo::TraCIColor(207, 255, 0, 255),
                                                   libsumo::TraCIColor(255, 155, 155, 255),
                                                   libsumo::TraCIColor(0, 140, 255, 255),
                                                   libsumo::TraCIColor(0, 255, 162, 255)};
        libsumo::TraCIColor color = libsumo::TraCIColor(255, 0, 255, 255);
        int maxActivePoIs = par("camLocationVisualizerMaxLength");
        std::string poiId = {
                mVehicleId + "_CAM_" + std::to_string(cam->header.stationID) + "_" +
                std::to_string((uint16_t) countTaiMilliseconds(mTimer->getCurrentTime()))};
        /*if (mAttackType == attackTypes::GridSybil && attackGridSybilVehicleCount <= 5) {
            color = colors[(attackGridSybilCurrentVehicleIndex + (attackGridSybilVehicleCount - 1)) %
                           attackGridSybilVehicleCount];
            maxActivePoIs = attackGridSybilVehicleCount;
            poiId = {mVehicleController->getVehicleId() + "_CAM_" + std::to_string(cam->header.stationID) + "_" +
                     std::to_string((uint16_t) countTaiMilliseconds(
                             mTimer->getCurrentTime()))};
        }*/
        traci::TraCIGeoPosition traciGeoPosition = {
                (double) cam->cam.camParameters.basicContainer.referencePosition.longitude / 10000000.0,
                (double) cam->cam.camParameters.basicContainer.referencePosition.latitude / 10000000.0};
        traci::TraCIPosition traciPosition = mTraciAPI->convert2D(traciGeoPosition);
        mTraciAPI->poi.add(poiId, traciPosition.x, traciPosition.y, color,
                           poiId, 5, "", 0,
                           0, 0);
        activePoIs.push_back(poiId);
        if (activePoIs.size() > maxActivePoIs) {
            mTraciAPI->poi.remove(activePoIs.front());
            activePoIs.pop_front();
        }
        if (mAttackType != attackTypes::GridSybil) {
            int alphaStep = 185 / maxActivePoIs;
            int currentAlpha = 80;
            for (const auto &poi: activePoIs) {
                mTraciAPI->poi.setColor(poi, libsumo::TraCIColor(255, 0, 255, currentAlpha));
                currentAlpha += alphaStep;
            }
        }
    }

    // copied from CaService.cc
bool MisbehaviorCaService::checkTriggeringConditions(const SimTime& T_now)
{
	// provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
	SimTime& T_GenCam = mGenCam;
	const SimTime& T_GenCamMin = mGenCamMin;
	const SimTime& T_GenCamMax = mGenCamMax;
	const SimTime T_GenCamDcc = mDccRestriction ? genCamDcc() : T_GenCamMin;
	const SimTime T_elapsed = T_now - mLastCamTimestamp;

	if (T_elapsed >= T_GenCamDcc) {
		if (mFixedRate) {
			return true;
		} else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
			return true;
			T_GenCam = std::min(T_elapsed, T_GenCamMax); /*< if middleware update interval is too long */
			mGenCamLowDynamicsCounter = 0;
		} else if (T_elapsed >= T_GenCam) {
			return true;
			if (++mGenCamLowDynamicsCounter >= mGenCamLowDynamicsLimit) {
				T_GenCam = T_GenCamMax;
			}
		}
	}
    return false;
}

bool MisbehaviorCaService::checkHeadingDelta() const
{
	return !vanetza::facilities::similar_heading(mLastCamHeading, mVehicleDataProvider->heading(), mHeadingDelta);
}

bool MisbehaviorCaService::checkPositionDelta() const
{
	return (distance(mLastCamPosition, mVehicleDataProvider->position()) > mPositionDelta);
}

bool MisbehaviorCaService::checkSpeedDelta() const
{
	return abs(mLastCamSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
}

SimTime MisbehaviorCaService::genCamDcc()
{
	// network interface may not be ready yet during initialization, so look it up at this later point
	auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for CA's primary channel %i", mPrimaryChannel);
	}

	static const vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
	vanetza::Clock::duration interval = trc->interval(ca_tx);
	SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
	return std::min(mGenCamMax, std::max(mGenCamMin, dcc));
}
        

} // namespace artery