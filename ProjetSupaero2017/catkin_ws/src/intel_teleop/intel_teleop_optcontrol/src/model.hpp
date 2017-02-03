#ifndef ACADO_TOOLKIT_FUNCTION_HPP
#define ACADO_TOOLKIT_FUNCTION_HPP

#endif  // ACADO_TOOLKIT_FUNCTION_HPP

#inclde "baro.hpp"


BEGIN_NAMESPACE_ACADO

class Model{

private:
	 
	DifferentialEquation f;
	OutputFcn ym;

public:
	
	Model();


	~Model();

	DifferentialEquation getDiffEq() const;

	OutputFcn getOutPutEq() const;


}
