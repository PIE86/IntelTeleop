#ifndef MPCSOLVER_H
#define MPCSOLVER_H

#include <array>
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

/**
 * @brief The MPCSolver class is an interface to the ACADO toolkit. Because of segfaults, it is not used yet.
 * It should provide a way to initialise all the ACADO variables, and a methode to call at every loop step.
 */
class MPCSolver
{
public:
	/**
	 * @brief MPCSolver Initialises all the variables
	 */
	MPCSolver();

	/**
	 * @brief controlMPC Calls the MPC algorithm to solve one temporal step of the constrained optimal problem of driving the drone
	 * without hitting obstacles
	 * @param t Discretised time variable
	 * @param dt Discretisation step
	 */
//	void controlMPC(double t, double dt);

private:

};

#endif // MPCSOLVER_H
