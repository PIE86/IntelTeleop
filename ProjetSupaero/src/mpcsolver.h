#ifndef MPCSOLVER_H
#define MPCSOLVER_H

#include <array>
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


class MPCSolver
{
public:
	MPCSolver();
	void controlMPC();
	void systemEvol(double t, double dt);

	// Affecter la consigne
	void setConsigne(std::array<double,3> consigne);

	void setReferenceVector(const std::array<double, 3> &referenceVector);
	std::array<double, 16> stateVector() const;

private:
	std::array<double,16> _stateVector;
	std::array<double,4> _commandVector;
	std::array<double,3> _referenceVector;

	// Consigne de vitesse
	std::array<double,3> _consigne;

	std::array<double, 16> quadRungeKutta(double t, std::array<double,16> x, std::array<double,4> u, double dt);
	std::array<double, 16> quadModel(double t, std::array<double,16> x, std::array<double,4> u);


	// INTRODUCE THE VARIABLES:
	// -------------------------
	ACADO::DifferentialState     x, y, z, vx, vy, vz, phi, theta, psi, p, q, r, u1, u2, u3, u4;
	// x, y, z : position
	// vx, vy, vz : linear velocity
	// phi, theta, psi : orientation (Yaw-Pitch-Roll = Euler(3,2,1))
	// p, q, r : angular velocity
	// u1, u2, u3, u4 : velocity of the propellers
	ACADO::Control               vu1, vu2, vu3, vu4;
	// vu1, vu2, vu3, vu4 : derivative of u1, u2, u3, u4
	ACADO::DifferentialEquation  f;

	// Quad constants
	const double c = 0.00001;
	const double Cf = 0.00065;
	const double d = 0.250;
	const double Jx = 0.018;
	const double Jy = 0.018;
	const double Jz = 0.026;
	const double Im = 0.0001;
	const double m = 0.9;
	const double g = 9.81;
	const double Cx = 0.1;

};

#endif // MPCSOLVER_H
