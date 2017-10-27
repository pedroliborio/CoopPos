/*
 * GPS.cpp
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#include <GPS/GPS.h>

namespace Localization {

GPS::GPS() {
    // TODO Auto-generated constructor stub

}

GPS::GPS(double mean, double stdDev) {
    this->mean = mean;
    this->stdDev = stdDev;
}

GPS::GPS(std::string route) {
    //TODO Update all mean and std correctly and put this information on the paper

    if(route  == "DMATEntranceExit"){
        //this->mean = 6.19143868296;
        //this->stdDev = 2.88459214044;
        this->mean = 6.19143868296;
        this->stdDev = 0.961530713;
    }
    else{
        if(route  == "DMATExitEntrance"){
            this->mean = 6.42315413843;
            //this->stdDev = 2.84527635525;
            this->stdDev = 0.948425452;
        }
        else{
            if(route  == "DPTEntranceExit"){
                this->mean = 6.23195053621;
                //this->stdDev = 2.95601551205;
                this->stdDev = 0,985338504;
            }
            else{
                if(route =="DPTExitEntrance"){
                    this->mean = 7.59763440355;
                    //this->stdDev = 3.41503134687;
                    this->stdDev = 1,138343782;
                }
                else{
                    if(route =="RCLTEntranceExit"){
                        this->mean=6.97907769597;
                        //this->stdDev = 3.30866358332;
                        this->stdDev = 1.102887861;
                    }
                    else{
                        if(route =="RCLTExitEntrance"){
                            this->mean = 6.13565937547;
                            //this->stdDev = 3.21445123213;
                            this->stdDev = 1.071483744;
                        }
                        else{
                            if(route =="YBTEntranceExit"){
                                this->mean = 8.93352084142;
                                //this->stdDev=3.41637848527;
                                this->stdDev=1.138792828;
                            }
                            else{
                                if(route =="YBTExitEntrance"){
                                    this->mean = 9.5998328591;
                                    //this->stdDev = 2.83305514461;
                                    this->stdDev = 0.944351715;
                                }
                                else{
                                    if(route =="RIO450EntranceExit"){
                                        this->mean= 9.50330634515;
                                        this->stdDev= 1,142441789;
                                    }
                                }

                            }
                        }
                    }
                }
            }
        }
    }
}



GPS::~GPS() {
    // TODO Auto-generated destructor stub
}

void GPS::CompPosition(Coord *realCoord){
    //FIXME Maybe is necessary use the sumo seed... verify
    //Here we generate a random number in teh perimeter of an circle
    double diff, angle, radius;

    diff = (RNGCONTEXT normal(mean, stdDev));

    position.x = realCoord->x + diff;
    position.y = realCoord->y + diff;
    position.z = realCoord->z;

    radius = realCoord->distance(position);

    //TODO I am not sure if I need to exchange to uniform distribution
    angle = ( RNGCONTEXT normal(0,3.141592653589793*2) );

    position.x = realCoord->x + cos(angle)*radius;
    position.y = realCoord->y + sin(angle)*radius;

    CompError(realCoord);

    //std::cout <<"Real Coord: "<< *realCoord<< endl;
    //std::cout <<"GPS Position: "<< position << endl;
    //std::cout <<"Error: "<< error << endl;

}

void GPS::CompError(Coord *realCoord){
    error = position.distance(*realCoord);
}



} /* namespace Localization */
