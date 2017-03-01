#include "model.hpp"


Model::Model(){
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


	f << dot(x) == vx;
	f << dot(y) == vy;
	f << dot(z) == vz;
	f << dot(vx) == ax;
	f << dot(vy) == ay;
	f << dot(vz) == az;
	f << dot(phi) == -cos(phi)*tan(theta)*p+sin(phi)*tan(theta)*q+r;
	f << dot(theta) == sin(phi)*p+cos(phi)*q;
	f << dot(psi) == cos(phi)/cos(theta)*p-sin(phi)/cos(theta)*q;
	f << dot(p) == (d*Cf*(u1*u1-u2*u2)+(Jy-Jz)*q*r)/Jx;
	f << dot(q) == (d*Cf*(u4*u4-u3*u3)+(Jz-Jx)*p*r)/Jy;
	f << dot(r) == (c*(u1*u1+u2*u2-u3*u3-u4*u4)+(Jx-Jy)*p*q)/Jz;
	f << ax == Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*sin(theta)/m;
	f << ay == -Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*sin(psi)*cos(theta)/m;
	f << az == Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*cos(psi)*cos(theta)/m - g;



	ym << cos(theta)*cos(psi)*(ax-g) + cos(theta)*sin(psi)*(ay-g)
			-sin(theta)*(az-g);
	ym << (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi))*(ax-g)
		+ (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi))*(ay-g)
		+ sin(phi)*cos(theta)*(az-g);
	ym << (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*(ax-g)
		+ (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*(ay-g)
		+ cos(phi)*cos(theta)*(az-g);
	ym << p;
	ym << q;
	ym << r;

    // Cannot work, this function uses float, not differentialState.
//	ym << getStandardPressure(z);

    const double EARTH_RADIUS = 6356.766;
    const double METERS_TO_KM = 0.001;

    // Limited to 11 km.
    ym << 101325. * pow( 288.15 / ( 288.15 - ( 6.5 * ( ( EARTH_RADIUS * ( z * METERS_TO_KM ) ) /
                                                       ( EARTH_RADIUS + ( z * METERS_TO_KM ) ) ) ) ),
                         -5.255877);



	}

	DifferentialEquation Model::getDiffEq() const{
		return f;
	}



	OutputFcn Model::getOutPutEq() const{
		return ym;
	}





