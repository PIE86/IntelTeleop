#ifndef MPCSOLVER_H
#define MPCSOLVER_H

#include <array>
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


class MPCSolver
{
public:
    ///
    /// \brief MPCSolver
    /// \param T length (in second) of the trajectory predicted in the MPC
    /// \param number of nodes used in the Optimal Control Problem
    MPCSolver(double T, int nbNodes);
    void controlMPC();
    void systemEvol(double t, double dt);

    void setReferenceVector(const std::array<double, 3> &referenceVector);
    std::array<double, 16> stateVector() const;

private:
    std::array<double,16> _stateVector;
    std::array<double,4> _commandVector;
    std::array<double,3> _referenceVector;

    std::array<double, 16> quadRungeKutta(double t, std::array<double,16> x, std::array<double,4> u, double dt);
    std::array<double, 16> quadModel(double t, std::array<double,16> x, std::array<double,4> u);


    // VARIABLES
    // -------------------------
    ACADO::DifferentialState     x, y, z, vx, vy, vz, phi, theta, psi, p, q, r, u1, u2, u3, u4;
    // x, y, z : position
    // vx, vy, vz : linear velocity
    // phi, theta, psi : orientation (Yaw-Pitch-Roll = Euler(3,2,1))
    // p, q, r : angular velocity
    // u1, u2, u3, u4 : velocity of the propellers
    ACADO::Control               vu1, vu2, vu3, vu4;
    // vu1, vu2, vu3, vu4 : derivative of u1, u2, u3, u4
    ACADO::DifferentialEquation  _f;

    // OPTIMAL CONTROL PROBLEM
    // -------------------------
    double _tmpc;
    // time (in second) between two activation of the MPC algorithm
    ACADO::OCP _ocp;

    //  COST FUNCTION
    // -------------------------------
    ACADO::Function _h;
    ACADO::DMatrix _Q;

    // Quad constants
    const double c  = 0.00001;
    const double Cf = 0.00065;
    const double d  = 0.250;
    const double Jx = 0.018;
    const double Jy = 0.018;
    const double Jz = 0.026;
//    const double Im = 0.0001;
    const double m  = 0.9;
    const double g  = 9.81;
//    const double Cx = 0.1;
};

#endif // MPCSOLVER_H
