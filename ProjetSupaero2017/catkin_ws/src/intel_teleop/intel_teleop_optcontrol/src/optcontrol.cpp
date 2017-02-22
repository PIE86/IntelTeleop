#include "optcontrol.hpp"

BEGIN_NAMESPACE_ACADO

	Optcontrol::Optcontrol(DMatrix& Q, const Model& model, Function& h, DVector& refVec,
	double const t_in, double const t_fin, double const dt, const Dvector& X_0){

		Q = Q;
		model = model;
		h = h;
		refVec = refVec;


		DynamicSystem dynamicSystem(this.model.getDiffEq(),this.model.getOutputEq);
		process(dynamicSystem,INT_RK45);


		// DEFINE AN OPTIMAL CONTROL PROBLEM:
		// ----------------------------------
		OCP ocp(t_in, dt, t_fin);
		ocp.minimizeLSQ(this->Q, this->h, this->refVec);

		// Constraints on the velocity of each propeller
		ocp.subjectTo(f);
		ocp.subjectTo(16 <= u1 <= 95);
		ocp.subjectTo(16 <= u2 <= 95);
		ocp.subjectTo(16 <= u3 <= 95);
		ocp.subjectTo(16 <= u4 <= 95);

		// Constraint to avoid singularity
		ocp.subjectTo(-1. <= theta <= 1.);

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


	DMatrix Optcontrol::getMatrixQ() const{
		return this->Q;
		};

	void Optcontrol::setMatrixQ(DMatrix& Q){
		Q = Q;
		};


	Model Optcontrol::getDifferentialEquation() cont {
		return this->model;
		};

	Function Optcontrol::getFunction() const{
		return this->h;
		};


	void Optcontrol::setFunction(Function& h){
		h =h
		};

	DVector Optcontrol::getrefVec()const{
		return this->refVec;
		};

	void Optcontrol::setrefVec(Dvector& refVec){
		refVec = refVec;
		};



	DVector u Optcontrol::solveOptimalControl(Dvector& NewRefVec ){

		if (abs(NewRefVec[0] - this->refVec(0))>1.){
			if (NewRefVec[0]> this->refVec(0)) { NewRefVec[0] = this->refVec(0)+1.;}
			else {NewRefVec[0] = this.refVec(0)-1.;}
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
        DVector refVec(10, refT);
        VariablesGrid referenceVG (refVec, Grid{t, t+1., 2});
        referenceVG.setVector(0, LastRefVec);
        alg.setReference(referenceVG);
        this.setrefVec(refVec);

		Dvector ActualX;

        // get state vector
        process.getX(ActualX);
        X = ActualX.getLastVector();

        // MPC step
        // compute the command
        bool success = controller.step(t, X);

        if (success != 0){
			std::cout << "controller failed " << std::endl;
			return 1;
        }
        Dvector u(4); u.init(); u.setZero();
        controller.getU(u);

        return u;
		}
