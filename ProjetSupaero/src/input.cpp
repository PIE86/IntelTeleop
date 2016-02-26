#include"input.h"
#include<iostream>
#include<SFML/Window.hpp>



Input::Input()
{
}


std::array<double,3> Input::getReference()
{
    std::array<double,3> consigne = {0,0,0};

    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
    {
        std::cout << "forward ";
        consigne[0] += 2;
    }
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
    {
        std::cout << "backward ";
        consigne[0] += -2;
    }
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
    {
        std::cout << "right ";
        consigne[1] += 2;
    }
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
    {
        std::cout << "left ";
        consigne[1] += -2;
    }
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::A))
    {
        std::cout << "upward ";
        consigne[2] += 2;
    }
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::R))
    {
        std::cout << "downward ";
        consigne[2] += -2;
    }

    std::cout << std::endl;
    return consigne;
}
















