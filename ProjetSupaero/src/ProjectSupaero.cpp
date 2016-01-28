#include "mpcsolver.h"


int main()
{
    MPCSolver solver;
    double t = 0;
    double dt = 0.02;

    solver.controlMPC();
    solver.systemEvol(t,dt);

    return 0;
}
