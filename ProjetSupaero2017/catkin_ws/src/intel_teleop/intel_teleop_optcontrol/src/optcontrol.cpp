#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ctime>

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>

BEGIN_NAMESPACE_ACADO


	Optcontrol::Optcontrol(bool const isPWD, DMatrix& Q, DVector& refVec,
	double const t_in, double const t_fin, double const dt, const Dvector& X_0){
		
		Q = Q;
		refVec = refVec;
		
		DifferentialEquation f;	

			// Introducing constants
		const double c  = 0.00001;
		const double Cf = 0.00065;
		const double d  = 0.250;
		const double Jx = 0.018;
		const double Jy = 0.018;
		const double Jz = 0.026;
		const double m  = 0.9;
		const double g  = 9.81;
		
		if(isPWD){
		DifferentialState x,y,z, vx,vy,vz, phi,theta,psi, p,q,r;
		
		// x, y, z : position
		// vx, vy, vz : linear velocity
		// phi, theta, psi : orientation (Yaw-Pitch-Roll = Euler(3,2,1))
		// p, q, r : angular velocity
		// u1, u2, u3, u4 : velocity of the propellers
		Control u1,u2,u3,u4; 
		
		
		f << dot(x) == vx;
		f << dot(y) == vy;
		f << dot(z) == vz;
		f << dot(phi) == -cos(phi)*tan(theta)*p+sin(phi)*tan(theta)*q+r;
		f << dot(theta) == sin(phi)*p+cos(phi)*q;
		f << dot(psi) == cos(phi)/cos(theta)*p-sin(phi)/cos(theta)*q;
		f << dot(p) == (d*Cf*(u1*u1-u2*u2)+(Jy-Jz)*q*r)/Jx;
		f << dot(q) == (d*Cf*(u4*u4-u3*u3)+(Jz-Jx)*p*r)/Jy;
		f << dot(r) == (c*(u1*u1+u2*u2-u3*u3-u4*u4)+(Jx-Jy)*p*q)/Jz;
		f << dot(vx) == Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*sin(theta)/m;
		f << dot(vy) == -Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*sin(psi)*cos(theta)/m;
		f << dot(vz) == Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*cos(psi)*cos(theta)/m - g;
		
		
		
		// Constraints on the velocity of each propeller
		ocp.subjectTo(16 <= u1 <= 95);
		ocp.subjectTo(16 <= u2 <= 95);
		ocp.subjectTo(16 <= u3 <= 95);
		ocp.subjectTo(16 <= u4 <= 95);

		// Constraint to avoid singularity
		ocp.subjectTo(-1. <= theta <= 1.);

		h << vx << vy << vz;
		h << u1 << u2 << u3 << u4;
		h << p << q << r;
		
		if(Q.getCols != 10 || Q.getRows!= 10 ){
			std::cout << "Weight matrix size is not suitable for this model" << std::endl;
			}
		

		
	}else{
		
		DifferentialState x,y,z,phi,theta,psi;
		
		// x, y, z : position
		// vx, vy, vz : linear velocity
		// phi, theta, psi : orientation (Yaw-Pitch-Roll = Euler(3,2,1))

		Control u_vx, u_vy, u_vz, u_p, u_q, u_r ; // Linear and angular velocity
		
		f << dot(x) == u_vx;
		f << dot(y) == u_vy;
		f << dot(z) == u_vz;
		f << dot(phi) == -cos(phi)*tan(theta)*u_p+sin(phi)*tan(theta)*u_q+u_r;
		f << dot(theta) == sin(phi)*u_p+cos(phi)*u_q;
		f << dot(psi) == cos(phi)/cos(theta)*u_p-sin(phi)/cos(theta)*u_q;
			
		ocp.subjectTo(-15 <= u_p <= 15);
		ocp.subjectTo(-15 <= u_q <= 15);
		ocp.subjectTo(-15 <= u_r <= 15);

		h << u_vx << u_vy << u_vz
		h << u_p << u_q << u_r;
	
		if(Q.getCols != 6 || Q.getRows!= 6 ){
			std::cout << "Weight matrix size is not suitable for this model" << std::endl;
			}
		
		}

		OCP ocp;
		ocp(t_in, dt, t_fin);
		ocp.minimizeLSQ(this->Q, this->h, this->refVec);
		ocp.subjectTo(f);		
		
		EnvironmentParser parser(PIE_SOURCE_DIR"/data/envsave.xml");
		auto cylinders = parser.readData();
		for (Ecylinder c : cylinders)
		{
		ocp.subjectTo(pow(c.radius + 1,2) <=
				  ( pow((y-c.y1)*(c.z2-c.z1)-(c.y2-c.y1)*(z-c.z1),2) + pow((z-c.z1)*(c.x2-c.x1)-(c.z2-c.z1)*(x-c.x1),2) + pow((x-c.x1)*(c.y2-c.y1)-(c.x2-c.x1)*(y-c.y1),2) ) /
				  ( pow(c.x2-c.x1,2) + pow(c.y2-c.y1,2) + pow(c.z2-c.z1,2) )
				  );
		}

		DynamicSystem dynamicSystem(f,OutputFcn{});
		process(dynamicSystem,INT_RK45);


		// SET UP THE MPC CONTROLLER:
		// --------------------------
		alg(ocp);
		alg.set(INTEGRATOR_TYPE, INT_RK45);
		alg.set(MAX_NUM_ITERATIONS,1);
		alg.set(PRINT_COPYRIGHT, false);
		alg.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
		
		controller(alg);

		// SETTING UP THE SIMULATION ENVIRONMENT:
		// --------------------------------------
		DVector u(4);
	    u.setZero();
		controller.init(0., X_0);
		process.init(0., X_0, u);


	};
		
		
	
		
	DVector u Optcontrol::solveOptimalControl(Dvector& NewRefVec, Dvector& x_est, double& t){
        
		if (abs(NewRefVec[0] - this->refVec(0))>1.){
			if (NewRefVec[0]> this->refVec(0)) { NewRefVec[0] = this->refVec(0)+1.;}
			else {NewRefVec[0] = this->refVec(0)-1.;}
		}
		
		if (abs(NewRefVec[1] - this->refVec(1))>1.){
			if (NewRefVec[1] > this->refVec(1)) { NewRefVec[1] = this->refVec(1)+1.;}
			else {NewRefVec[1] = this->refVec(1)-1.;}
		}
		
		if (abs(NewRefVec[2]- this->refVec(2))>1.){
			if (NewRefVec[2]> this->refVec(2)) { NewRefVec[2] = this->refVec(2)+1.;}
			else {NewRefVec[2] = this->refVec(2)-1.;}
		}
		
        double refT[10] = {NewRefVec[0], NewRefVec[1], NewRefVec[2], 0., 0., 0., 0., 0., 0., 0.};
        DVector refVecN(10, refT);
        VariablesGrid referenceVG (refVecN, Grid{t, t+1., 2});
        referenceVG.setVector(0, this->refVec);
        alg.setReference(referenceVG);
        this.setrefVec(refVecN);

		/*
		Dvector X;
		VariablesGrid Y;
		Y.setZero();
		
        // get state vector
        process.getY(Y);
        X = Y.getLastVector();
		*/

        // MPC step
        // compute the command
        bool success = controller.step(t, x_actual);

        if (success != 0){
			std::cout << "controller failed " << std::endl;
			return 1;          
        }
        Dvector u(4); u.init(); u.setZero();
        controller.getU(u);
        
        
        std::clock_t t_new = std::clock();
        double dt = (double)(t_new - t) / (double)CLOCKS_PER_SEC;
        process.step(t,t+dt,u);
        t += dt;
        
        return u;
		}


	DMatrix Optcontrol::getMatrixQ() const{
		return this->Q;
		};
		
	void Optcontrol::setMatrixQ(DMatrix& Q){
		Q = Q;
		};
		
	DVector Optcontrol::getrefVec()const{
		return this->refVec;
		};
		
	void Optcontrol::setrefVec(Dvector& refVec){
		refVec = refVec;
		};

