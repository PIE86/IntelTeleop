#include "baro.hpp"


BEGIN_NAMESPACE_ACADO

class Model{

private:
	 
	DifferentialEquation f;
	OutputFcn ym;

public:
	
	Model(bool const isPWD = true);


	~Model();

	DifferentialEquation getDiffEq() const;

	OutputFcn getOutPutEq() const;


}
