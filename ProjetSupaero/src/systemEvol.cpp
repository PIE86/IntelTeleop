
#include<fstream>
#include<iostream>
#include"Functions.h"

int systemEvol(double t, double dt)
{
	int lenX = 16;
	double* x = new double[16];
	double* u = new double[4];

/* ----- Read the current state and input ----- */
	std::ifstream inFile;
	inFile.open("TempData/initialState.txt");
	for(int i=0;i<lenX;i++)
	{
		inFile >> x[i];
	}
	// Close the current state file
	inFile.close();

	inFile.open("TempData/controlInput.txt");
	for(int i=0;i<4;i++)
	{
		inFile >> u[i];
	}
	// Close the current input file
	inFile.close();



/* ----- Start the system evolution -----*/
	std::ofstream outFile;
	outFile.open("TempData/initialState1.txt");
	double* xNew = quadRungeKutta(t,x,u,lenX,dt);


	// Output the new state of system
	// to a new file
	for(int i=0;i<lenX;i++)
	{
		outFile << xNew[i] << std::endl;
	}
	// Close the state file
	outFile.close();

	delete [] x;
	delete [] u;
	delete [] xNew;


	// Output termination information to the console
	std::cout << "One step simulation finished !" << std::endl;

	return 0;
}
























