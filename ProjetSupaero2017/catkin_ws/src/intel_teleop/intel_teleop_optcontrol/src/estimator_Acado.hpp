#include "model.hpp"

BEGIN_NAMESPACE_ACADO

class Estimator_Acado{
private:

ModelEst modelEst;
uint N; // Window for MPC
VariablesGrid *measures, *controls, lastState;

public:

Estimator_Acado(const ModelEst &model_Est, unit n, VariablesGrid &x0);

~Estimator_Acado();

void addMeasControl();

bool estimate(const DMatrix& covMatrix, Dvector &x_est, double &t);

VariablesGrid* getMeasures() const;
VariablesGrid* getControls() const;
VariablesGrid* getLastState() const;
uint getN() const;


}
