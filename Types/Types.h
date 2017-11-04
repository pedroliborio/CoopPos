/*
 * Types.h
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#ifndef TYPES_TYPES_H_
#define TYPES_TYPES_H_

#include "veins/base/utils/Coord.h"

struct t_LonLat{
    double lon;
    double lat;
};typedef struct t_LonLat LonLat;

struct t_Node{
    Coord posGPS;
    Coord realPos;
    double distanceRSSI;
    double realDist;
    double rssi;
    double id;
    double residual;
    simtime_t timestamp;
};typedef t_Node Node;

struct t_OutageCoord{
    Coord outage;
    Coord recover;
};typedef t_OutageCoord OutCoord;

namespace Localization {

class Types {
public:
    Types();
    virtual ~Types();
};

} /* namespace Localization */

#endif /* TYPES_TYPES_H_ */
