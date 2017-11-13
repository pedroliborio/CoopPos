//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "Communication/Communication.h"

Define_Module(Communication);

const simsignalwrap_t Communication::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);
const simsignalwrap_t Communication::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

void Communication::initialize(int stage) {
    BaseApplLayer::initialize(stage);

    if (stage==0) {

        //initialize pointers to other modules
        if (FindModule<TraCIMobility*>::findSubModule(getParentModule())) {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();
        }
        else {
            traci = NULL;
            mobility = NULL;
            traciVehicle = NULL;
        }

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
        //densityMovVeh = par("densityMovVeh").longValue();
        //densityStatVeh = par("densityStatVeh").longValue();
        isFirstVehicle = par("isFirstVehicle").boolValue();
        hops = par("hops").longValue();
        seedPar = par("seedPar").longValue();
        logVehPeriod = par("logVehPeriod").longValue();

        //GPS Parameters
        stdDevGPS = par("stdDevGPS").doubleValue();
        gpsModule = new GPS(0.0,stdDevGPS);

        //RSSI parameters
        alpha = 2.0;
        pTX = 20.0;

        //RMSE...
        rmseSUMGPS = 0;
        rmseSUMCP = 0;
        rmseSUMDR = 0;
        rmseSUMMM = 0;

        InitLocModules();
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


void Communication::onBSM(BasicSafetyMessage* bsm){
    //If the message already was received then do nothing
    if (AlreadyReceivedMSG(bsm)){
        return;
    }

    bsm->setHops(bsm->getHops()-1);

    listMSGIDs.push_back(bsm->getPersistentID());

    UpdateNodesList(bsm);

    //Update Statistics about this beacon
    delaySum += simTime().dbl() - bsm->getTimestamp().dbl();

    if(VerifyTTL(bsm)){
        //Forwarding...
        sendDelayedDown(bsm->dup(), beaconInterval + uniform(0.01,0.02));//TODO thsi can influency on delay drastically
    }

    //this bsm will be deleted on handleLowerMessage

}

simtime_t Communication::computeAsynchronousSendingTime(simtime_t interval, t_channel chan) {

    /*
     * avoid that periodic messages for one channel type are scheduled in the other channel interval
     * when alternate access is enabled in the MAC
     */

    simtime_t randomOffset = dblrand() * beaconInterval;
    simtime_t firstEvent;
    simtime_t switchingInterval = mac->getSwitchingInterval(); //usually 0.050s
    simtime_t nextCCH;

    /*
     * start event earlierst in next CCH  (or SCH) interval. For alignment, first find the next CCH interval
     * To find out next CCH, go back to start of current interval and add two or one intervals
     * depending on type of current interval
     */

    if (mac->isCurrentChannelCCH()) {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval*2;
    }
    else {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() %switchingInterval.raw()) + switchingInterval;
    }

    firstEvent = nextCCH + randomOffset;

    //check if firstEvent lies within the correct interval and, if not, move to previous interval

    if (firstEvent.raw()  % (2*switchingInterval.raw()) > switchingInterval.raw()) {
        //firstEvent is within a sch interval
        if (chan == type_CCH) firstEvent -= switchingInterval;
    }
    else {
        //firstEvent is within a cch interval, so adjust for SCH messages
        if (chan == type_SCH) firstEvent += switchingInterval;
    }

    return firstEvent;
}

void Communication::populateWSM(WaveShortMessage* wsm, int rcvId, int serial) {

    wsm->setWsmVersion(1);
    wsm->setTimestamp(simTime());
    wsm->setSenderAddress(myId);
    wsm->setRecipientAddress(rcvId);
    wsm->setSerial(serial);

    wsm->setBitLength(headerLength);


    if (BasicSafetyMessage* bsm = dynamic_cast<BasicSafetyMessage*>(wsm) ) {
        bsm->setSenderPos(curPosition);
        bsm->setSenderPos(curPosition);
        bsm->setSenderSpeed(curSpeed);
        bsm->setPsid(-1);
        bsm->setChannelNumber(Channels::CCH);
        bsm->addBitLength(beaconLengthBits);
        wsm->setPriority(beaconPriority);
    }
    else if (WaveServiceAdvertisment* wsa = dynamic_cast<WaveServiceAdvertisment*>(wsm)) {
        wsa->setChannelNumber(Channels::CCH);
        wsa->setTargetChannel(currentServiceChannel);
        wsa->setPsid(currentOfferedServiceId);
        wsa->setServiceDescription(currentServiceDescription.c_str());
    }
    else {
        if (dataOnSch) wsm->setChannelNumber(Channels::SCH1); //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        else wsm->setChannelNumber(Channels::CCH);
        wsm->addBitLength(dataLengthBits);
        wsm->setPriority(dataPriority);
    }
}

void Communication::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}

void Communication::handlePositionUpdate(cObject* obj) {
    ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition = mobility->getCurrentPosition();
    curSpeed = mobility->getCurrentSpeed();
}

void Communication::handleParkingUpdate(cObject* obj) {
    //this code should only run when used with TraCI
    isParked = mobility->getParkingState();
    if (communicateWhileParked == false) {
        if (isParked == true) {
            (FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
        }
        else {
            Coord pos = mobility->getCurrentPosition();
            (FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
        }
    }
}

void Communication::handleLowerMsg(cMessage* msg) {

    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    ASSERT(wsm);

    if (BasicSafetyMessage* bsm = dynamic_cast<BasicSafetyMessage*>(wsm)) {
        receivedBSMs++;
        onBSM(bsm);
    }
    else if (WaveServiceAdvertisment* wsa = dynamic_cast<WaveServiceAdvertisment*>(wsm)) {
        receivedWSAs++;
        onWSA(wsa);
    }
    else {
        receivedWSMs++;
        onWSM(wsm);
    }

    delete(msg);
}

void Communication::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
    case SEND_BEACON_EVT: {
        BasicSafetyMessage* bsm = new BasicSafetyMessage();
        //Clean Up older beacons
        CleanUpListNodes();

        //Insert Beacon's Information and Update GPS and DR Modules
        InsertBeaconInformation(bsm);

        //Update CP
        UpdateCooperativePositioning();

        //Improve DR
        ImproveDeadReckoning();

        //MapMatching
        mapMatchingModule->DoMapMatching(drModule->getLastKnowPosUtm());

        //std::cout << mapMatchingModule->getMatchPoint() << endl;

        //Update all statistics
        UpdateStatistics();

        //Write Log Files
        WriteLogFiles();
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

void Communication::finish() {
    recordScalar("generatedWSMs",generatedWSMs);
    recordScalar("receivedWSMs",receivedWSMs);

    recordScalar("generatedBSMs",generatedBSMs);
    recordScalar("receivedBSMs",receivedBSMs);

    recordScalar("generatedWSAs",generatedWSAs);
    recordScalar("receivedWSAs",receivedWSAs);

    //My Statistcs
    recordScalar("rmseSUMGPS",rmseSUMGPS);
    recordScalar("rmseSUMCP",rmseSUMCP);
    recordScalar("rmseSUMDR",rmseSUMDR);
    recordScalar("rmseSUMMM",rmseSUMMM);

    recordScalar("delaySum",delaySum);

}

Communication::~Communication() {
    cancelAndDelete(sendBeaconEvt);
    cancelAndDelete(sendWSAEvt);
    findHost()->unsubscribe(mobilityStateChangedSignal, this);
}

void Communication::startService(Channels::ChannelNumber channel, int serviceId, std::string serviceDescription) {
    if (sendWSAEvt->isScheduled()) {
        error("Starting service although another service was already started");
    }

    mac->changeServiceChannel(channel);
    currentOfferedServiceId = serviceId;
    currentServiceChannel = channel;
    currentServiceDescription = serviceDescription;

    simtime_t wsaTime = computeAsynchronousSendingTime(wsaInterval, type_CCH);
    scheduleAt(wsaTime, sendWSAEvt);

}

void Communication::stopService() {
    cancelEvent(sendWSAEvt);
    currentOfferedServiceId = -1;
}

void Communication::sendDown(cMessage* msg) {
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDown(msg);
}

void Communication::sendDelayedDown(cMessage* msg, simtime_t delay) {
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDelayedDown(msg, delay);
}

void Communication::checkAndTrackPacket(cMessage* msg) {
    if (isParked && !communicateWhileParked) error("Attempted to transmit a message while parked, but this is forbidden by current configuration");

    if (dynamic_cast<BasicSafetyMessage*>(msg)) {
        DBG_APP << "sending down a BSM" << std::endl;
        generatedBSMs++;
    }
    else if (dynamic_cast<WaveServiceAdvertisment*>(msg)) {
        DBG_APP << "sending down a WSA" << std::endl;
        generatedWSAs++;
    }
    else if (dynamic_cast<WaveShortMessage*>(msg)) {
        DBG_APP << "sending down a wsm" << std::endl;
        generatedWSMs++;
    }
}

//My Methods to control beacons...

//Verify if the message alrady was received
bool Communication::AlreadyReceivedMSG(BasicSafetyMessage *bsm){
    for(std::list<int>::iterator it=listMSGIDs.begin(); it!= listMSGIDs.end(); ++it){
        if(*it == bsm->getSerial()){
            return true;
        }
    }
    return false;
}

//TODO Method to discard entries of the list of MSGs
//void Communication::DiscardMSGIDsFromList();

//Verify if the TTL was reached
//return true if the TTL was not reached (TTL = 0)
//return false if the TTL was reached
bool Communication::VerifyTTL(BasicSafetyMessage* bsm){
    if(bsm->getHops() > 0){
        return true;
    }
    return false;
}

//My methods to control Localization

void Communication::UpdateGPS(){
    //Defining a novel GPS position]
    gpsModule->CompPosition(&atualSUMOUTMPos);
}

void Communication::UpdateNodesList(BasicSafetyMessage *bsm){

    for(std::list<Node>::iterator it=listNodes.begin(); it!= listNodes.end(); ++it){
        if(it->id == bsm->getSenderAddress()){
            it->posGPS = bsm->getSenderGPSPos();
            it->realPos = bsm->getSenderRealPos();
            it->rssi = bsm->getRssi();//TODO Improve to calculate using a RSSI Model Fix it with values of the paper.

            it->distanceRSSI = atualSUMOUTMPos.distance(bsm->getSenderRealPos()); //TODO exchange for distance from RSSI
            it->distanceRSSI += (RNGCONTEXT normal(0, it->distanceRSSI*0.1));

            it->timestamp = bsm->getTimestamp();
            return;
        }
    }

    Node node;
    node.id = bsm->getSenderAddress();
    node.posGPS = bsm->getSenderGPSPos();
    node.realPos = bsm->getSenderRealPos();
    node.rssi = bsm->getRssi();//TODO Improve to calculate using a RSSI Model Fix it with values of the paper

    node.distanceRSSI = atualSUMOUTMPos.distance(bsm->getSenderRealPos()); //TODO exchange for distance from RSSI
    node.distanceRSSI += (RNGCONTEXT normal(0, node.distanceRSSI*0.1));

    node.timestamp = bsm->getTimestamp();

    listNodes.push_back(node);
}

void Communication::UpdateCooperativePositioning(){
    double residual, localResidual;

    if(listNodes.size() > 3) {
        //TODO Call Multilateration Method
        if (multilateration->DoMultilateration(&listNodes)){
            coopPos = multilateration->getEstPosition();
            coopPos.z = atualSUMOUTMPos.z;
            errorCPPos = coopPos.distance(atualSUMOUTMPos);
        }
        residual = SetResidual();
        SortByResidual();

        list<Node> tempList;
        //get backup of list
        tempList = listNodes;

        while(tempList.size() > 3){
            //TODO retirar só o lemento e devolver ao inves de criar uma outra lista
            //remove o primeiro elemento (de maior residual)
            listNodes.pop_front();

            //FIXME Verificar isso com calma
            if(!(multilateration->DoMultilateration(&listNodes)) ){
                //Apos retirar uma posição o sisema ficou sem solução entao nao altera
                listNodes = tempList; //devolve o beacon que foi retirado
                break;
            }

            localResidual = SetResidual();
            SortByResidual();

            if(localResidual < residual){
                residual = localResidual;
                coopPos = multilateration->getEstPosition();
                coopPos.z = atualSUMOUTMPos.z;
                errorCPPos = coopPos.distance(atualSUMOUTMPos);
                tempList = listNodes;
            }
            else{
                listNodes = tempList;
                break;
            }

        }//end while
        tempList.clear();
    }// total of anchor nodes for one fresh multilateration

}

bool sortReverseOrder(const Node & a, const Node & b) { return a.residual > b.residual; }

void Communication::SortByResidual(){
    listNodes.sort(sortReverseOrder);
}

double Communication::SetResidual(){
    double distance, residual;
    residual = 0;
    for (std::list<Node>::iterator it=listNodes.begin(); it!= listNodes.end(); ++it){

        //distance from the estimated point by multilateration to the position of an acnhor node (i)
        distance = multilateration->getEstPosition().distance(it->realPos);

        //residual is the difference between the above distance and the rssi distance
        it->residual = (distance - it->distanceRSSI) * (distance - it->distanceRSSI);
        residual+= it->residual;
    }
    return residual;
}

void Communication::UpdateStatistics(){
    rmseSUMGPS+= gpsModule->getError();
    rmseSUMCP+= errorCPPos;
    rmseSUMDR+= drModule->getErrorUtm();
    rmseSUMMM+= mapMatchingModule->getMatchPoint().distance(atualSUMOUTMPos);
}


void Communication::WriteLogFiles(){
    std::fstream beaconLogFile("../../RESULTS/"+std::to_string(seedPar)+'-'+std::to_string(logVehPeriod)+'-'+std::to_string(hops)+'-'+std::to_string(myId)+".txt", std::fstream::app);
    beaconLogFile
    << std::setprecision(10) << simTime()
    <<'\t'<< std::setprecision(10) << atualSUMOUTMPos.x
    <<'\t'<< std::setprecision(10) << atualSUMOUTMPos.y
    <<'\t'<< std::setprecision(10) << atualSUMOUTMPos.z
    <<'\t'<< std::setprecision(10) << gpsModule->getPosition().x
    <<'\t'<< std::setprecision(10) << gpsModule->getPosition().y
    <<'\t'<< std::setprecision(10) << gpsModule->getPosition().z
    <<'\t'<< std::setprecision(10) << gpsModule->getError()

    <<'\t'<< std::setprecision(10) << drModule->getLastKnowPosUtm().x
    <<'\t'<< std::setprecision(10) << drModule->getLastKnowPosUtm().y
    <<'\t'<< std::setprecision(10) << drModule->getLastKnowPosUtm().z
    <<'\t'<< std::setprecision(10) << drModule->getErrorUtm()
    <<'\t'<< std::setprecision(10) << drModule->getAngle()
    <<'\t'<< std::setprecision(10) << drModule->getArw()
    <<'\t'<< std::setprecision(10) << drModule->getSensitivity()
    <<'\t'<< std::setprecision(10) << drModule->getError()
    <<'\t'<< std::setprecision(10) << drModule->getLPFTheta().getLpf()

    <<'\t'<< std::setprecision(10) << mapMatchingModule->getMatchPoint().x
    <<'\t'<< std::setprecision(10) << mapMatchingModule->getMatchPoint().y
    <<'\t'<< std::setprecision(10) << mapMatchingModule->getMatchPoint().z
    <<'\t'<< std::setprecision(10) << mapMatchingModule->getMatchPoint().distance(atualSUMOUTMPos)


    <<'\t'<< std::setprecision(10) << outageModule->isInOutage()
    <<'\t'<< std::setprecision(10) << coopPos.x
    <<'\t'<< std::setprecision(10) << coopPos.y
    <<'\t'<< std::setprecision(10) << coopPos.z
    <<'\t'<< std::setprecision(10) << errorCPPos
    << endl;
    beaconLogFile.close();
}

void Communication::CleanUpListNodes(){
    for(std::list<Node>::iterator it=listNodes.begin(); it!= listNodes.end(); ++it){
        //FIXME Take care about this
        //I am putting 0.5 because the update interval is 0.1 so we expect that at least 0.5 second an
        //beacon can do 5 hops.
        if( (simTime() - it->timestamp) > (0.5) ){
            //TODO Exchange to be based on delay
            it = listNodes.erase(it);
        }
    }
}


void Communication::InitLocModules(){
    OutCoord outCoord;
    Coord coord;
    //Initialize Projections Module...
    //size of (EntranceExit or ExitEntrance ) == 12
    projection = new Projection( traciVehicle->getRouteId().substr( 0,(traciVehicle->getRouteId().size() - 12) ) );

    //Convert from OMNET to TRACI/SUMO
    coord = traci->getTraCIXY(mobility->getCurrentPosition());

    //Initialize Outage Module

    //Get one outage from the server of outages

    outCoord = check_and_cast<Outages*>(getSimulation()->getModuleByPath("outagesServer.appl"))->getOutage(traciVehicle->getRouteId());

    outageModule = new Outage();

    outageModule->setOutagePos(outCoord.outage);
    outageModule->setRecoverPos(outCoord.recover);

    //Initializing GPS Module
    gpsModule = new GPS(traciVehicle->getRouteId());
    gpsModule->CompPosition(&coord);

    //Initializing DR Module
    projection->setUtmCoord(gpsModule->getPosition());
    projection->FromUTMToLonLat();

    drModule = new DeadReckoning(projection->getGeoCoord(),beaconInterval.dbl());

    //Initialize SUMO Positions tracker
    lastSUMOUTMPos = coord;
    atualSUMOUTMPos = lastSUMOUTMPos;

    //Multilateration Module
    multilateration = new Multilateration();

    //Initialize MM Module
    mapMatchingModule = new MapMatching(traciVehicle->getRouteId());

    errorCPPos = 0;
}

void Communication::InsertBeaconInformation(BasicSafetyMessage* bsm){
    /*BEGIN OF UPDATE SELF POSITIONING (GPS and DR)*/

    //Put unique persistent ID for FORWARDING purposes
    bsm->setPersistentID(bsm->getId());

    //Initialize Hops counter with zero
    bsm->setHops(0);

    //This is a vehicle beacon
    bsm->setIsRSUBSM(false);

    //Convert from OMNET to TRACI/SUMO
    Coord coord = traci->getTraCIXY(mobility->getCurrentPosition());

    lastSUMOUTMPos = atualSUMOUTMPos;
    atualSUMOUTMPos = coord;

    //Real Position
    bsm->setSenderRealPos(atualSUMOUTMPos);

    //Detect if in a outage stage...
    outageModule->ControlOutage(&atualSUMOUTMPos);

    //antes da queda
    if(!outageModule->isInOutage() && !outageModule->isInRecover()){
        //Put in WSM that this vehicle isn't in outage stage
        bsm->setInOutage(false);

        //Compute GPS Position and Error
        gpsModule->CompPosition(&atualSUMOUTMPos);

        bsm->setSenderGPSPos(gpsModule->getPosition());
        bsm->setErrorGPS(gpsModule->getError());

        //Only Update Dead Reckoning Module with last GPS Position
        projection->setUtmCoord(gpsModule->getPosition());
        projection->FromUTMToLonLat();
        drModule->setLastKnowPosGeo(projection->getGeoCoord());
        drModule->setLastKnowPosUtm(gpsModule->getPosition());
        drModule->setErrorUtm(gpsModule->getError());
        drModule->setErrorGeo(gpsModule->getError());

        //pass to the module of DR only
//        drModuleWithoutReinit->setLastKnowPosUtm(drModule->getLastKnowPosUtm());
//        drModuleWithoutReinit->setErrorUtm(drModule->getErrorUtm());
//        drModuleWithoutReinit->setAngle(drModule->getAngle());
//        drModuleWithoutReinit->setArw(drModule->getArw());
//        drModuleWithoutReinit->setSensitivity(drModule->getSensitivity());
//        drModuleWithoutReinit->setError(drModule->getError());
//        drModuleWithoutReinit->setLPFTheta(drModule->getLPFTheta());

        //collecting stats for teh time of outage...
        timestampOutage = simTime();
    }
    else{
        //em queda
        if(outageModule->isInOutage() && !outageModule->isInRecover()){
            bsm->setInOutage(true);

            //Convert from UTM to Lat Lon Coordinates from SUMO positions (last and actual) for use in GDR
            projection->setUtmCoord(lastSUMOUTMPos);
            projection->FromUTMToLonLat();
            lastSUMOGeoPos = projection->getGeoCoord();
            projection->setUtmCoord(atualSUMOUTMPos);
            projection->FromUTMToLonLat();
            actualSUMOGeoPos = projection->getGeoCoord();

            //Compute GDR position.
            drModule->setGeoPos(&lastSUMOGeoPos, &actualSUMOGeoPos);

            //Convert from Lon Lat to UTM coordinates
            projection->setGeoCoord(drModule->getLastKnowPosGeo());
            projection->FromLonLatToUTM();
            //Update in UTM Coordinates in DR Module
            drModule->setUTMPos(projection->getUtmCoord());
            drModule->setErrorUTMPos(&atualSUMOUTMPos);

            bsm->setSenderDRPos(drModule->getLastKnowPosUtm());
            bsm->setErrorDR(drModule->getErrorUtm());

            //UPDATE GPS error considering last position before outage
            gpsModule->CompError(&atualSUMOUTMPos);

            bsm->setSenderGPSPos(gpsModule->getPosition());
            bsm->setErrorGPS(gpsModule->getError());

            //collecting stats about outage
            timestampRecover = simTime();
        }
        else{
            //Put in WSM that this vehicle isn't in outage stage anymore
            bsm->setInOutage(false);
            gpsModule->CompPosition(&atualSUMOUTMPos);


            drModule->setLastKnowPosUtm(gpsModule->getPosition());
            drModule->setErrorUtm(gpsModule->getError());

            bsm->setSenderGPSPos(gpsModule->getPosition());
            bsm->setErrorGPS(gpsModule->getError());
            bsm->setSenderDRPos(gpsModule->getPosition());
            bsm->setErrorDR(gpsModule->getError());
        }
    }
    /*
    * *END OF UPDATE OF SELF POSITIONING (GPS and DR)
    */
}


void Communication::ImproveDeadReckoning(){
    //************************
    //**************FIXME First Approach to improve DR
    if( (outageModule->isInOutage() && !outageModule->isInRecover()) && (errorCPPos < drModule->getErrorUtm()) && errorCPPos > 1){
       //(errorCPReal < 20.0 && errorCPReal > 1) && )  ){
        drModule->ReinitializeSensors();
        //Update DR with CP position
        drModule->setLastKnowPosUtm(coopPos);
        drModule->setErrorUtm(errorCPPos);
        //Update Geo and UTM coordinates in DR Module
        projection->setUtmCoord(drModule->getLastKnowPosUtm());
        projection->FromUTMToLonLat();
        drModule->setLastKnowPosGeo(projection->getGeoCoord());
        drModule->setErrorGeo(errorCPPos);
    }
}

