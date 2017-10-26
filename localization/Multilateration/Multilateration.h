/*
 * Multilateration.h
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_MULTILATERATION_MULTILATERATION_H_
#define LOCALIZATION_MULTILATERATION_MULTILATERATION_H_

#include <jama_tnt/tnt.h>
#include <jama_tnt/jama_qr.h>
#include <iomanip>
#include <limits>

#include <Types/Types.h>

namespace Localization {

class Multilateration {
private:
    std::vector<Coord> positions;
    std::vector<double> distances;
    Coord estPosition;
public:

    Multilateration();
    virtual ~Multilateration();
    bool DoMultilateration(std::list<Node> *listNodes);
    bool LeastSquares(void);
    void getDistList(std::list<Node> *listNodes);
    void getPosList(std::list<Node> *listNodes);

    const Coord& getEstPosition() const {
        return estPosition;
    }
};

} /* namespace Localization */

#endif /* LOCALIZATION_MULTILATERATION_MULTILATERATION_H_ */
