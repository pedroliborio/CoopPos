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

package com.liborio.cooppos.Communication;
import org.car2x.veins.base.modules.IBaseApplLayer;
//
// TODO auto-generated module
//
//
// WAVE application layer base class.
//
// @author David Eckhoff
//
// @see BaseWaveApplLayer
// @see Nic80211p
// @see Mac1609_4
// @see PhyLayer80211p
//

@class(Communication);
simple Communication like IBaseApplLayer
{
    parameters:
        bool debug = default(false); //whether to print debug output

        int headerLength = default(88bit) @unit(bit); //header length of the application

        bool sendBeacons = default(true); //tell the applayer to periodically send beacons
        int beaconLengthBits = default(256bit) @unit(bit); //the length of a beacon packet
        int beaconPriority = default(3); //the access category of the beacon messages
        double beaconInterval = default(1s) @unit(s); //the intervall between 2 beacon messages

        int dataLengthBits = default(1024bit) @unit(bit); //the length of a data packet
        bool dataOnSch = default(false); //tells the applayer whether to use a service channel for datapackets or the control channel
        int dataPriority = default(3); //the default access category for data packets

        bool avoidBeaconSynchronization = default(true); //don't start beaconing directly after node was created but delay to avoid artifical synchronization

        bool sendWSA = default(false);
        int wsaLengthBits = default(250bit) @unit(bit);
        double wsaInterval =  default(1s) @unit(s);

        bool communicateWhileParked = default(false); //whether the car should communicate when it is parked in SUMO
        
        //Parameters to control simulation
        //int densityMovVeh = default(100);
        //int densityStatVeh = default(25);
        int logVehPeriod = default(1); 
        int hops = default(1);
        int seedPar;
        
        //RSSI Parameters
        double stdDevShadowRSSI = default(3.65);
        
        //GPS Parameters
    	double stdDevGPS = default(2.1);
        
        bool isFirstVehicle = default(true);//this will be used to istantiate all RSU's in runtime
        
        double timeToUpdateMSGIDs = default(10.0);
        
    gates:
        input lowerLayerIn; // from mac layer
        output lowerLayerOut; // to mac layer
        input lowerControlIn;
        output lowerControlOut;
}
