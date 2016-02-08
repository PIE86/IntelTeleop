
#include"input.h"
#include<iostream>
#include"Functions.h"
#include<SFML/Window.hpp>
#include<SFML/Graphics.hpp>
#include<SFML/System.hpp>

// Connstructor
Input::Input(double dt)
{
	this->currState = new double[16];
	for(int i=0;i<16;i++)
	{
		if(i<12)
		{
			this->currState[i] = 0.;
		}
		else
		{
			this->currState[i] = 58.0;
		}
	}
	this->currComm = new double[4];
	for(int i=0;i<4;i++)
	{
		this->currComm[i] = 0.;
	}
	// Initialization of calculation time step
	this->dt = dt;
}

// Destructor
Input::~Input()
{
	delete [] this->currState;
	delete [] this->currComm;
}

void Input::test()
{
   // std::cout << "hello world!"<< std::endl;
	sf::RenderWindow window(sf::VideoMode(100,100), "SFML window");
	// Limit the framerate to 60 frames per second (this step is optional)
	// window.setFramerateLimit(60);
	// 
	sf::Event event;
	double t=0;
	while (window.isOpen())
	{
		 while(window.pollEvent(event))
		{
			t = t+this->dt;
			if(event.type==sf::Event::KeyPressed)
			{
				if(event.key.code==sf::Keyboard::Left)
				{
					// left key is pressed: move our character
					std::cout << "========="<< std::endl;
					std::cout << "Moving left" << std::endl;
					std::cout << "========="<< std::endl;
				}
				else if(event.key.code==sf::Keyboard::Right)
				{
					std::cout << "========="<< std::endl;
					std::cout << "Moving right" << std::endl;
					std::cout << "========="<< std::endl;

				}
				else if(event.key.code==sf::Keyboard::Up)
				{
					std::cout << "========="<< std::endl;					
					std::cout << "Moving forward" << std::endl;
					std::cout << "========="<< std::endl;
				}
				else if(event.key.code==sf::Keyboard::Down)
				{
					std::cout << "========="<< std::endl;
					std::cout << "Moving backward" << std::endl;
					std::cout << "========="<< std::endl;
				}
				else if(event.key.code==sf::Keyboard::A)
				{
					std::cout << "========="<< std::endl;
					std::cout << "Moving upward" << std::endl;
					std::cout << "========="<< std::endl;
					this->currComm[0] = 5.;
					this->currComm[1] = 5.;
					this->currComm[2] = 5.;
					this->currComm[3] = 5.;
					double* newState = quadRungeKutta(t,this->currState,this->currComm,16,this->dt);
					delete this->currState;
					this->currState = newState;
					std::cout << "==========================" << std::endl;
					std::cout << "Current position: " << std::endl;
					std::cout << this->currState[0] << "\t" << this->currState[1] << "\t" << this->currState[2] << std::endl;
					std::cout << "Current attitude: " << std::endl;
					std::cout << this->currState[3] << "\t" << this->currState[4] << "\t" << this->currState[5] << std::endl;
					std::cout << "Current input motor: " << std::endl;
					std::cout << this->currState[12] << "\t" << this->currState[13] << "\t" << this->currState[14];
					std::cout <<  "\t" << this->currState[15] << std::endl;
					std::cout << "Current command: " << std::endl;
					std::cout << this->currComm[0] << "\t" << this->currComm[1] << "\t" << this->currComm[2];
					std::cout << "\t" << this->currComm[3]  << std::endl;
					std::cout << "==========================" << std::endl;
				}
				else if(event.key.code==sf::Keyboard::R)
				{
					std::cout << "========="<< std::endl;
					std::cout << "Moving downward" << std::endl;
					std::cout << "========="<< std::endl;
					this->currComm[0] = - 5.;
					this->currComm[1] = - 5.;
					this->currComm[2] = - 5.;
					this->currComm[3] = - 5.;
					double* newState = quadRungeKutta(t,this->currState,this->currComm,16,this->dt);
					delete this->currState;
					this->currState = newState;
					std::cout << "==========================" << std::endl;
					std::cout << "Current position: " << std::endl;
					std::cout << this->currState[0] << "\t" << this->currState[1] << "\t" << this->currState[2] << std::endl;
					std::cout << "Current attitude: " << std::endl;
					std::cout << this->currState[3] << "\t" << this->currState[4] << "\t" << this->currState[5] << std::endl;
					std::cout << "Current input motor: " << std::endl;
					std::cout << this->currState[12] << "\t" << this->currState[13] << "\t" << this->currState[14];
					std::cout <<  "\t" << this->currState[15] << std::endl;
					std::cout << "Current command: " << std::endl;
					std::cout << this->currComm[0] << "\t" << this->currComm[1] << "\t" << this->currComm[2];
					std::cout << "\t" << this->currComm[3]  << std::endl;
					std::cout << "==========================" << std::endl;
				}
				else if(event.key.code==sf::Keyboard::Escape)
				{
					window.close();
				}
			}
			if(event.type==sf::Event::KeyReleased)
			{
				std::cout << "Key released !" << std::endl;
				this->currComm[0] = 0.;
				this->currComm[1] = 0.;
				this->currComm[2] = 0.;
				this->currComm[3] = 0.;
				double* newState = quadRungeKutta(t,this->currState,this->currComm,16,this->dt);
				delete this->currState;
				this->currState = newState;
				std::cout << "==========================" << std::endl;
				std::cout << "Current position: " << std::endl;
				std::cout << this->currState[0] << "\t" << this->currState[1] << "\t" << this->currState[2] << std::endl;
				std::cout << "Current attitude: " << std::endl;
				std::cout << this->currState[3] << "\t" << this->currState[4] << "\t" << this->currState[5] << std::endl;
				std::cout << "Current input motor: " << std::endl;
				std::cout << this->currState[12] << "\t" << this->currState[13] << "\t" << this->currState[14];
				std::cout <<  "\t" << this->currState[15] << std::endl;
				std::cout << "Current command: " << std::endl;
				std::cout << this->currComm[0] << "\t" << this->currComm[1] << "\t" << this->currComm[2];
				std::cout << "\t" << this->currComm[3]  << std::endl;
				std::cout << "==========================" << std::endl;
			}
			if(event.type==sf::Event::Closed)
			{
				window.close();
			}

		}
	}
}
