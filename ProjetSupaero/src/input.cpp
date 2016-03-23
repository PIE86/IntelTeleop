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



Input::Input()
{
}


std::array<double,6> Input::getReference()
{
    std::array<double,6> consigne = {0,0,0,0,0,0};

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up))
    {
        //std::cout << "forward ";
        consigne[0] += 2;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Down))
    {
        //std::cout << "backward ";
        consigne[0] += -2;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right))
    {
        //std::cout << "right ";
        consigne[1] += -2;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left))
    {
        //std::cout << "left ";
        consigne[1] += 2;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A))
    {
        //std::cout << "upward ";
        consigne[2] += 2;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::R))
    {
        //std::cout << "downward ";
        consigne[2] += -2;
    }

    // Roll
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W))
    {
        //std::cout << "left ";
        consigne[3] += -2;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::X))
    {
        //std::cout << "upward ";
        consigne[3] += 2;
    }
    // Pitch
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::C))
    {
        //std::cout << "downward ";
        consigne[4] += -2;
    }

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::V))
    {
        //std::cout << "left ";
        consigne[4] += +2;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::B))
    {
        //std::cout << "upward ";
        consigne[5] += -2;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::N))
    {
        //std::cout << "downward ";
        consigne[5] += +2;
    }

    return consigne;
}
















