/*
 * Multilateration.cc
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#include <Multilateration.h>

namespace Localization {

Multilateration::Multilateration() {
    // TODO Auto-generated constructor stub

}

Multilateration::~Multilateration() {
    // TODO Auto-generated destructor stub
}

bool Multilateration::LeastSquares(void){
    int i;
    //Minus one because the last line of the matrix will be subtracted by the others lines
    int size = positions.size();
    //std::cout << size << endl;
    size--;

    //std::cout << size << endl;

    //Create matrixes using the TNT library
    //Composing the Linear Equation Ax - b to be solved by LeastSquares
    TNT::Array2D<double> A(size,2);
    TNT::Array1D<double> b(size);
    TNT::Array1D<double> x(size);

    //posToSubtract = positions[size];
    //distToSubtract = (double) distances->at(size);

    for(i=0; i < size; i++){
        //std::cout << positions.at(i) << "\t" << distances.at(i) << endl;
        A[i][0] =  2.0 * (positions.at(i).x - positions.at(size).x);
        A[i][1] =  2.0 * (positions.at(i).y - positions.at(size).y);

        b[i] = ( pow(distances.at(size),2) - pow(distances.at(i),2)     ) +
               ( pow(positions.at(i).x,2) - pow(positions.at(size).x,2) ) +
               ( pow(positions.at(i).y,2) - pow(positions.at(size).y,2) );
    }

    //std::cout << positions.at(size) << "\t" << distances.at(size) << endl;

    JAMA::QR<double> qrFact(A);

    x = qrFact.solve(b);

    if(x.dim1() == 0){
        /* If B is non-conformant, or if QR.isFullRank() is false,
        * the routine returns a null (0-length) vector
        *
        */
        std::cout << "Sistema sem solução!\n" << endl;
        return false;
    }

    estPosition.x = x[0];
    estPosition.y = x[1];
    estPosition.z = positions[0].z;

    return true;
}

void Multilateration::getDistList(std::list<Node> *listNodes){
    int i = 0;
    for(std::list<Node>::iterator it = listNodes->begin(); it!= listNodes->end(); ++it){
        distances.push_back(it->realDist);
        i++;
    }
}

void Multilateration::getPosList(std::list<Node> *listNodes){
    int i =0;
    for(std::list<Node>::iterator it = listNodes->begin(); it!= listNodes->end(); ++it){
        positions.push_back(it->realPos);
        i++;
    }
}

bool Multilateration::DoMultilateration(std::list<Node> *listNodes){
    //get distances
    getDistList(listNodes);
    //get positions
    getPosList(listNodes);
    //Call Multilateration using LeastSquares

    if(!LeastSquares()){
        std::cout << simTime() << "  - Error in LS\n\n";
        //Free memory
        this->distances.clear();
        this->positions.clear();
        return false;
    }

    //Free memory
    this->distances.clear();
    this->positions.clear();
    return true;
}
} /* namespace Localization */
