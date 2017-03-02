/**************************************************************************
Copyright 2016 Yoan BAILLAU, Thibault BARBIÉ, Zhengxuan JIA,
   Francisco PEDROSA-REIS, William RAKOTOMANGA, Baudouin ROULLIER

This file is part of ProjectSupaero.

ProjectSupaero is free software: you can redistribute it and/or modify
it under the terms of the lesser GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

ProjectSupaero is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
lesser GNU General Public License for more details.

You should have received a copy of the lesser GNU General Public License
along with ProjectSupaero.  If not, see <http://www.gnu.org/licenses/>.

**************************************************************************/


#include "input.h"
#include <iostream>
#include <SFML/Window.hpp>



Input::Input(bool joystickOn):
	joystickOn_(joystickOn)
{
}
Input::Input():
	joystickOn_(false)
{
}

std::array<double,6> Input::getReference()
{

    double transSpeed = 2;
    double rotSpeed = 2;
    std::array<double,6> consigne = {0,0,0,0,0,0};

	if (joystickOn_ == false)
	{
		// set each speed command if the corresponding key is pressed

		// translation commands
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up))
			consigne[0] += transSpeed;

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Down))
			consigne[0] += -transSpeed;

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right))
			consigne[1] += -transSpeed;

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left))
			consigne[1] += transSpeed;

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A))
			consigne[2] += transSpeed;

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::R))
			consigne[2] += -transSpeed;


		// rotation commands (used only for testing purposes)
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W))
			consigne[3] += -rotSpeed;

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::X))
			consigne[3] += rotSpeed;

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::C))
			consigne[4] += -rotSpeed;

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::V))
			consigne[4] += rotSpeed;

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::B))
			consigne[5] += -rotSpeed;

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::N))
			consigne[5] += rotSpeed;
	}
	else
	{
		sf::Joystick::update();
		float x = sf::Joystick::getAxisPosition(0, sf::Joystick::X);
		float y = sf::Joystick::getAxisPosition(0, sf::Joystick::Y);
		float r = sf::Joystick::getAxisPosition(0, sf::Joystick::R);

		consigne[0] = -x*transSpeed/100;
		consigne[1] = y*transSpeed/100;
		consigne[2] = -r*transSpeed/100;
	}
    return consigne;
}
















