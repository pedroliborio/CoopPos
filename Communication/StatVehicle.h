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

#ifndef __SBRC2018_STATVEHICLE_H_
#define __SBRC2018_STATVEHICLE_H_

#include "Communication.h"

/**
 * TODO - Generated class
 */
class StatVehicle : public Communication
{
    public:
        virtual void initialize(int stage);
    protected:
        virtual void handleSelfMsg(cMessage* msg);
        virtual void onBSM(BasicSafetyMessage* bsm);
        virtual void InsertBeaconInformation(BasicSafetyMessage *bsm);

    protected:
        BaseMobility *baseMob;
};

#endif
