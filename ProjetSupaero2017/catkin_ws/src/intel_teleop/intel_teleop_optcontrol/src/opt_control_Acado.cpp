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


#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ctime>

#include <baro.hpp>
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

#include "input.h"
#include "viewer.h"
#include "environmentparser.h"

using std::cout; using std::endl;


int main()
{
	USING_NAMESPACE_ACADO;

	// INTRODUCE THE VARIABLES:
	// -------------------------
	DifferentialState x,y,z, vx,vy,vz, phi,theta,psi, p,q,r,
	                  b_ax,b_ay,b_az,b_p,b_q,b_r,b_bar;// u1,u2,u3,u4;
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
	 
	Vector mean(1), amplitude(1);
	mean.setAll(0.0);
	amplitude.setAll(1.);

    	GaussianNoise w_ax(mean,amplitude),w_ax(mean,amplitude),w_ax(mean,amplitude), \\
			w_p(mean,amplitude),w_p(mean,amplitude),w_p(mean,amplitude),w_bar(mean,amplitude);
	 

	// DEFINE A DIFFERENTIAL EQUATION:
	// -------------------------------
	DifferentialEquation f;

	f << dot(x) == vx;
	f << dot(y) == vy;
	f << dot(z) == vz;
	f << dot(vx) == ax;
	f << dot(vy) == ay;
	f << dot(vz) == az;
	f << 0 == Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*sin(theta)/m -ax;
	f << 0 == -Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*sin(psi)*cos(theta)/m - ay;
	f << 0 == Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*cos(psi)*cos(theta)/m - g;
	f << dot(phi) == -cos(phi)*tan(theta)*p+sin(phi)*tan(theta)*q+r - az;
	f << dot(theta) == sin(phi)*p+cos(phi)*q;
	f << dot(psi) == cos(phi)/cos(theta)*p-sin(phi)/cos(theta)*q;
	f << dot(p) == (d*Cf*(u1*u1-u2*u2)+(Jy-Jz)*q*r)/Jx;
	f << dot(q) == (d*Cf*(u4*u4-u3*u3)+(Jz-Jx)*p*r)/Jy;
	f << dot(r) == (c*(u1*u1+u2*u2-u3*u3-u4*u4)+(Jx-Jy)*p*q)/Jz;
	f << dot(b_ax) == -1/tau_bax*b_ax + w_ax;
	f << dot(b_ay) == -1/tau_bay*b_ay + w_ay;
	f << dot(b_az) == -1/tau_baz*b_az + w_az;
	f << dot(b_p) == -1/tau_bp*b_p + w_p;
	f << dot(b_q) == -1/tau_bq*b_q + w_q;
	f << dot(b_r) == -1/tau_br*b_r + w_r;
	f << dot(b_bar) == -1/tau_bbar*b_bar + w_bar;
	/*    f << dot(u1) == vu1;
	f << dot(u2) == vu2;
	f << dot(u3) == vu3;
	f << dot(u4) == vu4;
	*/

	// DEFINE AN OUTPUT EQUATION:
	// -------------------------------
	
	OutputFcn ym;
	ym << cos(theta)*cos(psi)*(ax-g) + cos(theta)*sin(psi)*(ay-g)
			-sin(theta)*(az-g) + b_ax + w_ax;
	ym << (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi))*(ax-g)
		+ (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi))*(ay-g) 
		+ sin(phi)*cos(theta)*(az-g) + b_ay + w_ay;
	ym << (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*(ax-g)
		+ (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*(ay-g) 
		+ cos(phi)*cos(theta)*(az-g) + b_az + w_az;	
	ym << p + + b_p + w_p;
	ym << q + + b_q + w_q;
	ym << r + + b_r + w_r;
	ym << getStandardPressure(z) + b_bar + w_bar;


	// SET UP THE SIMULATED PROCESS:
	// -----------------------------
	DynamicSystem dynamicSystem(f,ym);
	Process process(dynamicSystem,INT_RK45);


	// DEFINE LEAST SQUARE FUNCTION:
	// -----------------------------
	Function h;
	h << vx << vy << vz;
	//    h << vu1 << vu2 << vu3 << vu4;
	h << u1 << u2 << u3 << u4;
	h << p << q << r;

	// LSQ coefficient matrix
	DMatrix Q(10,10);
	Q(0,0) = Q(1,1) = Q(2,2) = 1e-1;
	Q(3,3) = Q(4,4) = Q(5,5) = Q(6,6) = 1e-9;
	Q(7,7) = Q(8,8) = Q(9,9) = 1e-1;

	// Reference
	DVector refVec(10);
	refVec.setZero(10);



	// DEFINE AN OPTIMAL CONTROL PROBLEM:
	// ----------------------------------
	OCP ocp(0., 1., 4);

	ocp.minimizeLSQ(Q, h, refVec);

	// Constraints on the velocity of each propeller
	ocp.subjectTo(f);
	ocp.subjectTo(16 <= u1 <= 95);
	ocp.subjectTo(16 <= u2 <= 95);
	ocp.subjectTo(16 <= u3 <= 95);
	ocp.subjectTo(16 <= u4 <= 95);

	// Command constraints
	// Constraints on the acceleration of each propeller
	/*    ocp.subjectTo(-200 <= vu1 <= 200);
	ocp.subjectTo(-200 <= vu2 <= 200);
	ocp.subjectTo(-200 <= vu3 <= 200);
	ocp.subjectTo(-200 <= vu4 <= 200);
	*/
	// Constraint to avoid singularity
	ocp.subjectTo(-1. <= theta <= 1.);

	// Adding roof, floor and walls constraints
	/*    ocp.subjectTo(-9.7 <= x <= 9.7);
	ocp.subjectTo(-9.7 <= y <= 9.7);
	ocp.subjectTo(.3 <= z <= 9.7);
	*/
	// Loading cylindrical obstacles from XML
	EnvironmentParser parser(PIE_SOURCE_DIR"/data/envsave.xml");
	auto cylinders = parser.readData();
	for (Ecylinder c : cylinders)
	{
	ocp.subjectTo(pow(c.radius + 1,2) <=
		      ( pow((y-c.y1)*(c.z2-c.z1)-(c.y2-c.y1)*(z-c.z1),2) + pow((z-c.z1)*(c.x2-c.x1)-(c.z2-c.z1)*(x-c.x1),2) + pow((x-c.x1)*(c.y2-c.y1)-(c.x2-c.x1)*(y-c.y1),2) ) /
		      ( pow(c.x2-c.x1,2) + pow(c.y2-c.y1,2) + pow(c.z2-c.z1,2) )
		      );
	}


	// SET UP THE MPC CONTROLLER:
	// --------------------------
	RealTimeAlgorithm alg(ocp);
	alg.set(INTEGRATOR_TYPE, INT_RK45);
	alg.set(MAX_NUM_ITERATIONS,1);
	alg.set(PRINT_COPYRIGHT, false);
	alg.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
	Controller controller(alg);

	// SETTING UP THE SIMULATION ENVIRONMENT:
	// --------------------------------------
	DVector X(12), U(4), U0(4);
	X.setZero();
	X(2) = 4.;
	//  X(12) = X(13) = X(14) = X(15) = 58.;
	U.setZero();
	U0.setZero();
	controller.init(0., X);
	process.init(0., X, U);

	//    VariablesGrid graph;
	VariablesGrid Y;
	Y.setZero();

    // END OF ACADO SOLVER SETUP
    // -------------------------

    // Initialise input from keyboard
    Input input(true);

    // Gepetto viewer over corba
    Viewer viewer;
    viewer.createEnvironment(cylinders);
    viewer.createDrone(PIE_SOURCE_DIR"/data/quadrotor_base.stl");

    double t = 0;
    std::clock_t previousTime;
    previousTime = std::clock();
    auto refInput = input.getReference();
    DVector LastRefVec(10);
    LastRefVec.setZero();

    while(true)
    {
        // getting reference from input and passing it to the algorithm
        refInput = input.getReference();
		if (abs(refInput[0]-LastRefVec(0))>1.)
		{
		if (refInput[0]>LastRefVec(0)) { refInput[0] = LastRefVec(0)+1.;}
		else {refInput[0] = LastRefVec(0)-1.;}
		}
		if (abs(refInput[1]-LastRefVec(1))>1.)
		{
		if (refInput[1]>LastRefVec(1)) { refInput[1] = LastRefVec(1)+1.;}
		else {refInput[1] = LastRefVec(1)-1.;}
		}
		if (abs(refInput[0]-LastRefVec(0))>1.)
		{
		if (refInput[2]>LastRefVec(2)) { refInput[2] = LastRefVec(2)+1.;}
		else {refInput[2] = LastRefVec(2)-1.;}
		}
        double refT[10] = {refInput[0], refInput[1], refInput[2], 0., 0., 0., 0., 0., 0., 0.};
        DVector refVec(10, refT);
        VariablesGrid referenceVG (refVec, Grid{t, t+1., 2});
        referenceVG.setVector(0, LastRefVec);
        alg.setReference(referenceVG);
        LastRefVec = refVec;

        // get state vector
        process.getY(Y);
        X = Y.getLastVector();

        // move the drone and draw the arrow
        // viewer takes roll-pitch-yaw but drone equations are in yaw-pitch-roll
        viewer.moveDrone(X(0), X(1), X(2), X(8), X(7), X(6));
        //viewer.setArrow((refInput[0]>0) - (refInput[0]<0), (refInput[1]>0) - (refInput[1]<0), (refInput[2]>0) - (refInput[2]<0));
        viewer.setArrow(refInput[0], refInput[1], refInput[2]);

        // MPC step
        // compute the command
        bool success = controller.step(t, X);

        if (success != 0)
        {
			std::cout << "controller failed " << std::endl;
			return 1;          
        }
        controller.getU(U);

        // simulate the drone
        std::clock_t currentTime = std::clock();
        double dt = (double)(currentTime - previousTime) / (double)CLOCKS_PER_SEC;
        process.step(t,t+dt,U);
        t += dt;

        // get the new state vector
        process.getY(Y);
        X = Y.getLastVector();

        // move the drone to it's new position
        // viewer takes roll-pitch-yaw but drone equations are in yaw-pitch-roll
        viewer.moveDrone(X(0), X(1), X(2), X(8), X(7), X(6));

        previousTime = currentTime;

//        graph.addVector(X,t);
    }


    // draw every variable into a graph. Useful for debug
//    GnuplotWindow window;
//    window.addSubplot(graph(0), "x");
//    window.addSubplot(graph(1), "y");
//    window.addSubplot(graph(2), "z");
//    window.addSubplot(graph(3), "vx");
//    window.addSubplot(graph(4), "vy");
//    window.addSubplot(graph(5), "vz");
//    window.addSubplot(graph(6), "phi");
//    window.addSubplot(graph(7), "theta");
//    window.addSubplot(graph(8), "psi");
//    window.addSubplot(graph(9), "p");
//    window.addSubplot(graph(10), "q");
//    window.addSubplot(graph(11), "r");
//    window.addSubplot(graph(12), "u1");
//    window.addSubplot(graph(13), "u2");
//    window.addSubplot(graph(14), "u3");
//    window.addSubplot(graph(15), "u4");
//    window.plot();

    return 0;
}
