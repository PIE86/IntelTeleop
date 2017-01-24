#define 	USING_NAMESPACE_ACADO   using namespace ACADO;

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ctime>
#include <baro.hpp>

class Model{

private:

// INTRODUCE THE VARIABLES:
	// -------------------------
	DifferentialState x,y,z, vx,vy,vz, phi,theta,psi, p,q,r,
	                  b_ax,b_ay,b_az,b_p,b_q,b_r,b_bar;
	AlgebraicState ax,ay,az;
	
	// x, y, z : position
	// vx, vy, vz : linear velocity
	// phi, theta, psi : orientation (Yaw-Pitch-Roll = Euler(3,2,1))
	// p, q, r : angular velocity
	// u1, u2, u3, u4 : velocity of the propellers
	Control u1,u2,u3,u4;
	// vu1, vu2, vu3, vu4 : derivative of u1, u2, u3, u4

	// Quad constants
	const double c  = 0.00001;
	const double Cf = 0.00065;
	const double d  = 0.250;
	const double Jx = 0.018;
	const double Jy = 0.018;
	const double Jz = 0.026;
	const double m  = 0.9;
	const double g  = 9.81;

	const double tau_bax = 0.001;
	const double tau_bay = 0.001;
	const double tau_baz = 0.001;
	const double tau_bp = 0.001;
	const double tau_bq = 0.001;
	const double tau_br = 0.001;
	const double tau_bbar = 0.001;
	 
	Matrix stateVariance(19,19);
	Matrix outputVariance(7,7);
	 
	DifferentialEquation f;
	OutputFcn ym;
	
	class Model();
	class Model(Matrix const stateVariance, Matrix const outputVariance);
	
	
	class ~Model();
	
	




}
