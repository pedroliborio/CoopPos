/*
 * GPS.h
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_GPS_GPS_H_
#define LOCALIZATION_GPS_GPS_H_


#include <Types/Types.h>
#include <fstream>

using namespace std;

namespace Localization {

class GPS {
private:
    double stdDev;
    double mean;
    Coord position;
    double error;
public:
    GPS();
    GPS::GPS(std::string outagesFile);
    GPS(double mean, double stdDev);
    virtual ~GPS();
    void CompPosition(Coord *realCoord);
    void CompError(Coord *realCoord);
    double getError() const {
        return error;
    }

    void setError(double error) {
        this->error = error;
    }

    double getMean() const {
        return mean;
    }

    void setMean(double mean) {
        this->mean = mean;
    }

    const Coord& getPosition() const {
        return position;
    }

    void setPosition(const Coord& position) {
        this->position = position;
    }

    double getStdDev() const {
        return stdDev;
    }

    void setStdDev(double stdDev) {
        this->stdDev = stdDev;
    }
};

} /* namespace Localization */

#endif /* LOCALIZATION_GPS_GPS_H_ */
