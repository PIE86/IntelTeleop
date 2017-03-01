#include "model.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ctime>

#include <acado_toolkit.hpp>


BEGIN_NAMESPACE_ACADO

using namespace std


Estimator_Acado::Estimator_Acado(const Model &model_Est, unit n, VariablesGrid &x0){
	m = model_Est;
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


bool estimate(const DMatrix& weightMatrix, Dvector &x_est, double& t){
	
	if (measures->getDim()<n || controls->getDim()<n){
		cout << "Not enought measures samples to start estimation" << endl;
		return false;
	}
	
	if ( measures->getDim()!=controls->getDim()){
		cout << "There are no as many mmasures as control samples" << endl;
		return false;
	}
	
	if (weightMatrix.getNumCols != weightMatrix.getNumRows){
		cout << "Weight matrix is not squared" << endl;
		return false;
		
		}
		
	if (weightMatrix.getNumCols != m.getOutPutEq.getDim){
		cout << "Weight matrix has not the same dimension of the output function" << endl;
		return false;
		}
	
	uint n_fin= measures->getDim();
	VariablesGrid measuresEst = measures->getTimeSubGrid((n_fin - n), n_fin-1);
	VariablesGrid controlsEst = controls->getTimeSubGrid((n_fin - n), n_fin-1);
	
	
	
	DynamicSystem dynamicSystem(weightMatrix, m.getDiffEq(), m.getOutPutEq()) ;
	Process process(dynamicSystem , INT RK45 ) ;
	VariablesGrid disturbance = controlsEst;
	process.setProcessDisturbance(disturbance) ;
	
		

	OCP ocp(measures->getTimePoints);
	ocp.minimazeLSQ(m.getOutPutEq(), measuresEst);
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


