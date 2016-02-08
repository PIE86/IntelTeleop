#include "mpcsolver.h"
#include <iostream>
#include <fstream>
#include <acado_gnuplot.hpp>
#include <time.h>

using std::cout; using std::endl;


int main()
{
    MPCSolver solver(8,20);
    double t = 0;
    double dt = 0.1;

    std::fstream file("outPos.csv");

    for (int i=0; i<100; i++)
    {
        solver.controlMPC();
        solver.systemEvol(t,dt);
        file << t << ";" << solver.stateVector()[0] << ";" << solver.stateVector()[1] << ";" << solver.stateVector()[2] << endl;
        t += dt;
    }

    return 0;
}
