#ifndef ARTERY_TRUSTSERVICE_H_
#define ARTERY_TRUSTSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/networking/PositionProvider.h"
#include "artery/envmod/LocalEnvironmentModel.h"

namespace artery
{

using namespace omnetpp;

class TrustService : public ItsG5Service
{
    protected:
        int numInitStages() const override;
        void initialize(int stage) override;
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;
    private:
        const PositionProvider* mPositionProvider = nullptr;
        const LocalEnvironmentModel* mEnvironmentModel = nullptr;
};

} // namespace artery

#endif /* ARTERY_TRUSTSERVICE_H_ */