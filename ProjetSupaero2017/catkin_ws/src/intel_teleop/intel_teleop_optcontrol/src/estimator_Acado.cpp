#include "model.hpp"


BEGIN_NAMESPACE_ACADO

using namespace std


Estimator_Acado::Estimator_Acado(const ModelEst &model_Est, unit n, VariablesGrid &x0){
	m = model();
	lastState = x0;
	controls = new VariablesGrid();
	measures = new VariablesGrid();
	controls.init();
	measures.init(); 
	
	
	}

Estimator_Acado::~Estimator_Acado();

Estimator_Acado::addMeasCon(Dvector& u_last, Dvector& y_last, double currentTime){	
	measures->addVector(y_last, currentTime);
	controls->addVector(u_last, currentTime);	
	}



VariablesGrid* getMeasures(){return measures}
VariablesGrid* getControls(){return controls}
uint getN(){return N}


bool estimate(const DMatrix& covMatrix, Dvector &x_est, double& t){
	if (measures->getDim()<n){
		return false;
	}
	
	uint n_fin= measures->getDim();
	VariablesGrid measuresEst = measures->getTimeSubGrid((n_fin - n), n_fin-1);
	
	if (covMatrix.getNumCols != covMatrix.getNumRows){
		cout << "Weight matrix is not squared" << endl;
		return false;
		
		}
		
	if (covMatrix.getNumCols != m.getOutPutEq.getDim){
		cout << "Weight matrix has not the same dimension of the output function" << endl;
		return false;
		}
	

	
	DynamicSystem dynamicSystem(m.getDiffEq(), m.getOutPutEq()) ;
	Process process(dynamicSystem , INT RK45 ) ;
	VariablesGrid disturbance = estimator_Acado.getControls();
	process.setProcessDisturbance ( disturbance) ;
	
		

	OCP ocp(measures->getTimePoints);
	ocp.minimazeLSQ(m.getOutPutEq(), measures);
	ocp.subjectTo(m.getDiffEq());
	
	ParameterEstimationAlgorithm stateEstAlg(ocp); 
	
	stateEstAlg.init; 
	stateEstAlg.solve();
	
	VariablesGrid xd_est_tot;
	
	stateEstAlg.getDifferentialStates(x_est_tot);

 	
 	t = x_est_tot.getLastTime();
	x_est =  x_est_tot.getLastVector();

	
	return true;
	
	
	
	}


