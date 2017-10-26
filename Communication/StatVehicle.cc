//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "StatVehicle.h"

Define_Module(StatVehicle);

void StatVehicle::initialize(int stage)
{
    BaseApplLayer::initialize(stage);

    if (stage==0) {

        //Getting pointer to access RSU coordinates
        baseMob = FindModule<BaseMobility*>::findSubModule(getParentModule());

        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        mac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(
                getParentModule());
        assert(mac);

        myId = getParentModule()->getId();

        //read parameters
        headerLength = par("headerLength").longValue();
        sendBeacons = par("sendBeacons").boolValue();
        beaconLengthBits = par("beaconLengthBits").longValue();
        beaconPriority = par("beaconPriority").longValue();
        beaconInterval =  par("beaconInterval");

        dataLengthBits = par("dataLengthBits").longValue();
        dataOnSch = par("dataOnSch").boolValue();
        dataPriority = par("dataPriority").longValue();

        wsaInterval = par("wsaInterval").doubleValue();
        communicateWhileParked = par("communicateWhileParked").boolValue();
        currentOfferedServiceId = -1;

        isParked = false;


        findHost()->subscribe(mobilityStateChangedSignal, this);
        findHost()->subscribe(parkingStateChangedSignal, this);

        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendWSAEvt = new cMessage("wsa evt", SEND_WSA_EVT);

        generatedBSMs = 0;
        generatedWSAs = 0;
        generatedWSMs = 0;
        receivedBSMs = 0;
        receivedWSAs = 0;
        receivedWSMs = 0;

        //Simulation parameters
        densityMovVeh = par("densityMovVeh").doubleValue();
        densityStatVeh = par("densityStatVeh").doubleValue();
        isFirstVehicle = par("isFirstVehicle").boolValue();
        hops = par("hops").longValue();

        //GPS Parameters
        //TODO change to a parameter in .ned and .ini files
        gpsModule = new GPS(0.0,2.1);

        //RSSI parameters
        alpha = 2.0;
        pTX = 20.0;


    }
    else if (stage == 1) {
        //simulate asynchronous channel access

        if (dataOnSch == true && !mac->isChannelSwitchingActive()) {
            dataOnSch = false;
            std::cerr << "App wants to send data on SCH but MAC doesn't use any SCH. Sending all data on CCH" << std::endl;
        }
        simtime_t firstBeacon = simTime();

        if (par("avoidBeaconSynchronization").boolValue() == true) {

            simtime_t randomOffset = dblrand() * beaconInterval;
            firstBeacon = simTime() + randomOffset;

            if (mac->isChannelSwitchingActive() == true) {
                if ( beaconInterval.raw() % (mac->getSwitchingInterval().raw()*2)) {
                    std::cerr << "The beacon interval (" << beaconInterval << ") is smaller than or not a multiple of  one synchronization interval (" << 2*mac->getSwitchingInterval() << "). "
                            << "This means that beacons are generated during SCH intervals" << std::endl;
                }
                firstBeacon = computeAsynchronousSendingTime(beaconInterval, type_CCH);
            }

            if (sendBeacons) {
                scheduleAt(firstBeacon, sendBeaconEvt);
            }
        }
    }

}

void StatVehicle::handleSelfMsg(cMessage* msg){
    switch (msg->getKind()) {
        case SEND_BEACON_EVT: {
            BasicSafetyMessage* bsm = new BasicSafetyMessage();
            InsertBeaconInformation(bsm);
            populateWSM(bsm);
            sendDown(bsm);
            scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
            break;
        }
        case SEND_WSA_EVT:   {
            WaveServiceAdvertisment* wsa = new WaveServiceAdvertisment();
            populateWSM(wsa);
            sendDown(wsa);
            scheduleAt(simTime() + wsaInterval, sendWSAEvt);
            break;
        }
        default: {
            if (msg)
                DBG_APP << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
            break;
        }
    }

}

void StatVehicle::onBSM(BasicSafetyMessage* bsm){
    //If the message already was received then do nothing
    if (AlreadyReceivedMSG(bsm)){
        return;
    }

    bsm->setHops(bsm->getHops()-1);

    listMSGIDs.push_back(bsm->getSerial());

    UpdateNodesList(bsm);

    //Update Statistics about this beacon
    delaySum += simTime().dbl() - bsm->getTimestamp().dbl();

    if(VerifyTTL(bsm)){
        //Forwarding...
        sendDelayedDown(bsm->dup(), beaconInterval + uniform(0.01,0.02));//TODO thsi can influency on delay drastically
    }

    //this bsm will be deleted on handleLowerMessage
}

void StatVehicle::InsertBeaconInformation(BasicSafetyMessage *bsm){
    Coord coord;
    bsm->setHops(hops);
    bsm->setSenderGPSPos(gpsModule->getPosition());
    bsm->setSenderRealPos(realPos);
}

