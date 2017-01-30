#ifndef ACADO_TOOLKIT_FUNCTION_HPP
#define ACADO_TOOLKIT_FUNCTION_HPP

#include <acado/matrix_vector/matrix_vector.hpp>

#include <acado/symbolic_expression/acado_syntax.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>

#include <acado/function/evaluation_point.hpp>
#include <acado/function/t_evaluation_point.hpp>
#include <acado/function/function_.hpp>
#include <acado/function/c_function.hpp>
#include <acado/function/differential_equation.hpp>
#include <acado/function/transition.hpp>
#include <acado/function/output_fcn.hpp>

#endif  // ACADO_TOOLKIT_FUNCTION_HPP


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
	 
	DMatrix stateVariance;
	DMatrix outputVariance;
	 
	DifferentialEquation f;
	OutputFcn ym;

public:
	
	Model();

	Model();
	Model(DMatrix const _stateVariance, DMatrix const _outputVariance);


	~Model();

	DifferentialEquation getDiffEq() const;

	OutputFcn getOutPutEq() const;

	returnValue jacobianF(DMatrix &A, DMatrix &B);

	returnValue jacobianYm(DMatrix &C,DMatrix &D);

	DVector evaluateF(const EvaluationPoint &x,
					   const int        &number);

	DVector evaluateH(const EvaluationPoint &x,
					   const int        &number);



}
