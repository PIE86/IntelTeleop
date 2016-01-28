#ifndef MPCSOLVER_H
#define MPCSOLVER_H

#include <array>



class MPCSolver
{
public:
    MPCSolver();
    void controlMPC();
    void systemEvol(double t, double dt);

private:
    std::array<double,16> _stateVector;
    std::array<double,4> _commandVector;

    std::array<double, 16> quadRungeKutta(double t, std::array<double,16> x, std::array<double,4> u, double dt);
    std::array<double, 16> quadModel(double t, std::array<double,16> x, std::array<double,4> u);
};

#endif // MPCSOLVER_H
