
#include<cmath>

double* quadModel(double t,double* x,double* u,int lenX)
{
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

	// Minimization Weights
	double coeffX = .00001;
	double coeffU = coeffX*0.0001;//0.000000000000001;
	double coeffX2 = coeffX * 100.;
	double coeffX3 = coeffX * 0.00001;
	double coeffO = -coeffX * 0.1;



	double* dx = new double[lenX];
	// auxilary variable
	double normU2 = u[0]*u[0]+u[1]*u[1]+u[2]*u[2]+u[3]*u[3];

	// Differential equations of the quadrotor
	dx[0] = x[3];//dot_x
	dx[1] = x[4];//dot_y
	dx[2] = x[5];//dot_z

	dx[3] = Cf*normU2*sin(x[7])/m;//dot_vx
	dx[4] = -Cf*normU2*sin(x[8])*cos(x[7])/m;//dot_vy
	dx[5] = Cf*normU2*cos(x[8])*cos(x[7])/m - g;//dot_vz

	dx[6] = -cos(x[6])*tan(x[7])*x[9]+sin(x[6])*tan(x[7])*x[10]+x[11];//dot_phi
	dx[7] = sin(x[6])*x[9]+cos(x[6])*x[10];//dot_theta
	dx[8] = cos(x[6])/cos(x[7])*x[9]-sin(x[6])/cos(x[7])*x[10];//dot_psi

	dx[9] = (d*Cf*(u[0]*u[0]-u[1]*u[1])+(Jy-Jz)*x[10]*x[11])/Jx;//dot_p
	dx[10] = (d*Cf*(u[3]*u[3]-u[2]*u[2])+(Jz-Jx)*x[9]*x[11])/Jy;//dot_q
	dx[11] = (c*(u[0]*u[0]+u[1]*u[1]-u[2]*u[2]-u[3]*u[3])+(Jx-Jy)*u[9]*u[10])/Jz;//dot_r


	dx[12] = u[0];//dot_u1
	dx[13] = u[1];//dot_u2
	dx[14] = u[2];//dot_u3
	dx[15] = u[3];//dot_u4

	return dx;
}



















