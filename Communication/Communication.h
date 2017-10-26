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

#ifndef __SBRC2018_COMMUNICATION_H_
#define __SBRC2018_COMMUNICATION_H_

#include <map>
#include "veins/base/modules/BaseApplLayer.h"
#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/messages/WaveShortMessage_m.h"
#include "veins/modules/messages/WaveServiceAdvertisement_m.h"
#include "veins/modules/messages/BasicSafetyMessage_m.h"
#include "veins/base/connectionManager/ChannelAccess.h"
#include "veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::AnnotationManager;
using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;


//Utils libraries
#include <exception>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <limits>
#include <fstream>


//Localization includes
#include <Types/Types.h>
#include <localization/GPS/GPS.h>
#include <localization/Multilateration/Multilateration.h>
#include <localization/DeadReckoning/DeadReckoning.h>
#include <localization/Projections/Projection.h>
#include <localization/Outage/Outage.h>
#include <Filters/Filters.h>
#include <localization/MapMatching/MapMatching.h>
#include <localization/OutagesServer/Outages.h>

using namespace Localization;

//#define DBG_APP std::cerr << "[" << simTime().raw() << "] " << getParentModule()->getFullPath() << " "

#ifndef DBG_APP
#define DBG_APP EV
#endif

/**
 * @brief
 * WAVE application layer base class.
 *
 * @author David Eckhoff
 *
 * @ingroup applLayer
 *
 * @see BaseWaveApplLayer
 * @see Mac1609_4
 * @see PhyLayer80211p
 * @see Decider80211p
 */


/*Struct to save localization parameters*/

class Communication : public BaseApplLayer {

    public:
        ~Communication();
        virtual void initialize(int stage);
        virtual void finish();

        virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);

        enum WaveApplMessageKinds {
            SEND_BEACON_EVT,
            SEND_WSA_EVT
        };
    protected:

        static const simsignalwrap_t mobilityStateChangedSignal;
        static const simsignalwrap_t parkingStateChangedSignal;

        /** @brief handle messages from below and calls the onWSM, onBSM, and onWSA functions accordingly */
        virtual void handleLowerMsg(cMessage* msg);

        /** @brief handle self messages */
        virtual void handleSelfMsg(cMessage* msg);

        /** @brief sets all the necessary fields in the WSM, BSM, or WSA. */
        virtual void populateWSM(WaveShortMessage*  wsm, int rcvId=0, int serial=0);

        /** @brief this function is called upon receiving a WaveShortMessage */
        virtual void onWSM(WaveShortMessage* wsm) { };

        /** @brief this function is called upon receiving a BasicSafetyMessage, also referred to as a beacon  */
        virtual void onBSM(BasicSafetyMessage* bsm);

        /** @brief this function is called upon receiving a WaveServiceAdvertisement */
        virtual void onWSA(WaveServiceAdvertisment* wsa) { };

        /** @brief this function is called every time the vehicle receives a position update signal */
        virtual void handlePositionUpdate(cObject* obj);

        /** @brief this function is called every time the vehicle parks or starts moving again */
        virtual void handleParkingUpdate(cObject* obj);

        /** @brief This will start the periodic advertising of the new service on the CCH
         *
         *  @param channel the channel on which the service is provided
         *  @param serviceId a service ID to be used with the service
         *  @param serviceDescription a literal description of the service
         */
        virtual void startService(Channels::ChannelNumber channel, int serviceId, std::string serviceDescription);

        /** @brief stopping the service and advertising for it */
        virtual void stopService();

        /** @brief compute a point in time that is guaranteed to be in the correct channel interval plus a random offset
         *
         * @param interval the interval length of the periodic message
         * @param chantype the type of channel, either type_CCH or type_SCH
         */
        virtual simtime_t computeAsynchronousSendingTime(simtime_t interval, t_channel chantype);

        /**
         * @brief overloaded for error handling and stats recording purposes
         *
         * @param msg the message to be sent. Must be a WSM/BSM/WSA
         */
        virtual void sendDown(cMessage* msg);

        /**
         * @brief overloaded for error handling and stats recording purposes
         *
         * @param msg the message to be sent. Must be a WSM/BSM/WSA
         * @param delay the delay for the message
         */
        virtual void sendDelayedDown(cMessage* msg, simtime_t delay);

        /**
         * @brief helper function for error handling and stats recording purposes
         *
         * @param msg the message to be checked and tracked
         */
        virtual void checkAndTrackPacket(cMessage* msg);

        virtual void InitLocModules();

        //Verifying if the message already was received and processed
        virtual bool AlreadyReceivedMSG(BasicSafetyMessage *bsm);
        //Verify TTL
        virtual bool VerifyTTL(BasicSafetyMessage* bsm);
        //Update GPS see handlePositionUpdate
        virtual void UpdateGPS();

        virtual void UpdateNodesList(BasicSafetyMessage *bsm);

        virtual void UpdateCooperativePositioning();

        virtual void InsertBeaconInformation(BasicSafetyMessage *bsm);

        virtual void SortByResidual();

        virtual double SetResidual();

        virtual void CleanUpListNodes();

        virtual void UpdatePositions();

        virtual void UpdateStatistics();

        virtual void WriteLogFiles();

    protected:

        /* pointers ill be set when used with TraCIMobility */
        TraCIMobility* mobility;
        TraCICommandInterface* traci;
        TraCICommandInterface::Vehicle* traciVehicle;

        AnnotationManager* annotations;
        WaveAppToMac1609_4Interface* mac;

        /* support for parking currently only works with TraCI */
        bool isParked;
        bool communicateWhileParked;

        /* BSM (beacon) settings */
        uint32_t beaconLengthBits;
        uint32_t  beaconPriority;
        simtime_t beaconInterval;
        bool sendBeacons;

        /* WSM (data) settings */
        uint32_t  dataLengthBits;
        uint32_t  dataPriority;
        bool dataOnSch;

        /* WSA settings */
        int currentOfferedServiceId;
        std::string currentServiceDescription;
        Channels::ChannelNumber currentServiceChannel;
        simtime_t wsaInterval;

        /* state of the vehicle */
        Coord curPosition; //FIXME Do not use this! These are omnet++ coordinates not sumo coordinates!
        Coord curSpeed;
        int myId;
        int mySCH;

        /*Vehicle Position using TraCI coordinates*/
        Coord realPos;
        Coord coopPos;
        double errorCPPos;

        /*GPS Module*/
        GPS *gpsModule;
        double stdDevGPS;

        /*Multilateration Module*/
        Multilateration multilateration;

        /*Dead Reckoning Module*/
        DeadReckoning *drModule;

        /*Outage Module*/
        Outage *outageModule;

        /*Projection Module*/
        Projection *projection;

        /* stats */
        uint32_t generatedWSMs;
        uint32_t generatedWSAs;
        uint32_t generatedBSMs;
        uint32_t receivedWSMs;
        uint32_t receivedWSAs;
        uint32_t receivedBSMs;

        /* messages for periodic events such as beacon and WSA transmissions */
        cMessage* sendBeaconEvt;
        cMessage* sendWSAEvt;

        /* variables to control simulation*/
        int densityMovVeh;
        int densityStatVeh;
        bool isFirstVehicle;
        int hops;
        int seedPar;
        double delaySum;
        double rmseSUMGPS;
        double rmseSUMCP;

        /*variables to define RSSI behavior*/
        double alpha;
        double pTX;

        /*list of Nodes with thei information*/
        std::list<Node> listNodes;
        std::list<int> listMSGIDs;

        /*Vector Statistics - by Time*/

};

#endif
