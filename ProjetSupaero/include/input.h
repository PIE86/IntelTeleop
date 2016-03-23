#ifndef INPUT_H
#define INPUT_H

#include <array>

/**
 * @brief The Input class provides an interface to the SFML library. It reads the keyboard inputs
 * and writes them as speed commands to an std::array.
 */
class Input
{
public:
	Input();

	/**
	 * @brief getReference Reads the keyboard inputs and interprets them as speed commands (3 translation speeds, 3 rotation speeds)
	 * @return the speed commands in an std::array
	 */
	std::array<double,6> getReference();
};

#endif // INPUT_H
