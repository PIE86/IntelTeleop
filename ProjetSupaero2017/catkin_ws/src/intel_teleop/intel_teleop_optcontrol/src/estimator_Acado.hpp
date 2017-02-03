#include "model.hpp"

BEGIN_NAMESPACE_ACADO

class Estimator_Acado{
private:

Model model;
uint N; // Window for MPC
VariablesGrid *measures, *controls, lastState;

public:

Estimator_Acado(const Model &model, unit n, VariablesGrid &x0);

~Estimator_Acado();

void addMeasControl();

bool estimate(const DMatrix& covMatrix, VariablesGrid &x_est);

VariablesGrid* getMeasures() const;
VariablesGrid* getControls() const;
VariablesGrid* getLastState() const;
uint getN() const;


}
