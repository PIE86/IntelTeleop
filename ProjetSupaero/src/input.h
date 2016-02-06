#ifndef INPUT_H
#define INPUT_H

#include <SFML/Window.hpp>


class Input
{
private:
	double* currState;
	double* currComm;
	double dt;


public:
	Input(double dt);
	void test();
	~Input();
};

#endif // INPUT_H
