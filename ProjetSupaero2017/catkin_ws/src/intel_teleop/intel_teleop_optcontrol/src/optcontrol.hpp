BEGIN_NAMESPACE_ACADO

#include "model.hpp"

class Optcontrol {
	
	private:
	DMatrix Q;
	Function h;
	DVector refVec;
	Controller controller;
	RealTimeAlgorithm alg;
	Process process;
	
	public:
	Optcontrol(bool isPWD = true, DMatrix& Q, DVector& refVec,
	double const t_in, double const t_fin, double const dt, Dvector& X_0);
	
	~Optcontrol();		
			
	DVector u solveOptimalControl(Dvector& NewRefVec, Dvector& x_est, double t);
	
	DMatrix getMatrixQ();	
	void setMatrixQ(DMatrix& Q);
	
	DVector getrefVec();	
	void setrefVec(Dvector& refVec);
	
		
	};
