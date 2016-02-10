

#include "mpcsolver.h"
#include "input.h"
#include <iostream>

using std::cout; using std::endl;


int main()
{
<<<<<<< HEAD
	MPCSolver mpcSolver;
	Input input(& mpcSolver);
	while(true)
		input.test();

//	MPCSolver solver;
//	double t = 0;
//	double dt = 0.02;
//
//	solver.controlMPC();
//	solver.systemEvol(t,dt);
//	cout << solver.stateVector()[0] << " " << solver.stateVector()[1] << " " << solver.stateVector()[2] << endl;
//	solver.controlMPC();
//	solver.systemEvol(t,dt);
//	cout << solver.stateVector()[0] << " " << solver.stateVector()[1] << " " << solver.stateVector()[2] << endl;
//	solver.controlMPC();
//	solver.systemEvol(t,dt);
//	cout << solver.stateVector()[0] << " " << solver.stateVector()[1] << " " << solver.stateVector()[2] << endl;
//	solver.controlMPC();
//	solver.systemEvol(t,dt);
//	cout << solver.stateVector()[0] << " " << solver.stateVector()[1] << " " << solver.stateVector()[2] << endl;
=======
    Input input;
    while (true)
    {
        input.test();
        sf::sleep(sf::milliseconds(100));
    }

//    MPCSolver solver;
//    double t = 0;
//    double dt = 0.02;

//    solver.controlMPC();
//    solver.systemEvol(t,dt);
//    cout << solver.stateVector()[0] << " " << solver.stateVector()[1] << " " << solver.stateVector()[2] << endl;
//    solver.controlMPC();
//    solver.systemEvol(t,dt);
//    cout << solver.stateVector()[0] << " " << solver.stateVector()[1] << " " << solver.stateVector()[2] << endl;
//    solver.controlMPC();
//    solver.systemEvol(t,dt);
//    cout << solver.stateVector()[0] << " " << solver.stateVector()[1] << " " << solver.stateVector()[2] << endl;
//    solver.controlMPC();
//    solver.systemEvol(t,dt);
//    cout << solver.stateVector()[0] << " " << solver.stateVector()[1] << " " << solver.stateVector()[2] << endl;
>>>>>>> 33e11f3e24f2ec70e4207c4a95a7b770dfcde852

    return 0;
}
