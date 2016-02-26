#ifndef INPUT_H
#define INPUT_H

#include <SFML/Window.hpp>
#include <array>


class Input
{
public:
    Input();
    std::array<double,3> getReference();
};

#endif // INPUT_H
