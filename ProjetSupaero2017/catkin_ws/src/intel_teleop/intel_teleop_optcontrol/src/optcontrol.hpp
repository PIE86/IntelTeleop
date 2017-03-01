#include "model.hpp"


class Optcontrol {

private:
    DMatrix Q;
    Function h;
    DVector refVec;
    Controller controller;
    RealTimeAlgorithm alg;
    Process process;

public:
    Optcontrol(bool isPWD, DMatrix &Q, Function &h, DVector &refVec,
               const double t_in, const double t_fin, const double dt, const DVector &X_0);


    DVector solveOptimalControl(DVector &NewRefVec);

    DMatrix getMatrixQ();

    void setMatrixQ(DMatrix &Q);

    Function getFunction();

    void setFunction(Function &h);

    DVector getrefVec();

    void setrefVec(DVector &refVec);


};
