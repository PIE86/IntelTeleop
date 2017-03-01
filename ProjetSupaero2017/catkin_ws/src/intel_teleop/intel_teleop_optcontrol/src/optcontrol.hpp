#include <memory>

#include "model.hpp"

class Optcontrol {

private:
    DMatrix _Q;
    Function _h;
    DVector _refVec;
    std::unique_ptr< Controller > _controller;
    std::unique_ptr< RealTimeAlgorithm > _alg;
    std::unique_ptr< Process > _process;

public:
    Optcontrol(bool isPWD, DMatrix &Q, DVector &refVec,
               const double t_in, const double t_fin, const double dt, const DVector &X_0);


    DVector solveOptimalControl(DVector &NewRefVec, DVector &x_est, double &t );

    DMatrix getMatrixQ();

    void setMatrixQ(DMatrix &Q);

    DVector getrefVec();

    void setrefVec(DVector &refVec);


};
