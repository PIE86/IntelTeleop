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
















