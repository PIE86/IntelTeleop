#include "input.h"
#include <iostream>

Input::Input()
{

}

void Input::test()
{
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
        std::cout << "left" << std::endl;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
        std::cout << "right" << std::endl;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
        std::cout << "up" << std::endl;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
        std::cout << "down" << std::endl;
}
