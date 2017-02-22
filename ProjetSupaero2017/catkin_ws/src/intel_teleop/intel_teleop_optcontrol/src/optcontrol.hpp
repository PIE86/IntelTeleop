BEGIN_NAMESPACE_ACADO

#include "model.hpp"

class Optcontrol {
	
	
	public:
	Optcontrol(DMatrix& Q, const Model& model, Function& h, DVector& refVec,
	double const t_in, double const t_fin, double const dt, Dvector& X_0);
	
	DMatrix getMatrixQ();
	
	void setMatrixQ(DMatrix& Q);
	
	Model getModel();
	
	Function getFunction();
	
	void setFunction(Function& h);
	
	DVector getrefVec();
	
	void setrefVec(Dvector& refVec);
	
	DVector u solveOptimalControl(Dvector& NewRefVec, Dvector& U, );
	void Init(double const t_in, double const t_fin, double const dt, Dvector& X_0);
	
	
	private:
	DMatrix Q;
	Model model;
	Function h;
	DVector refVec;
	Controller controller;
	RealTimeAlgorithm alg;
	Process process;
	
	};
