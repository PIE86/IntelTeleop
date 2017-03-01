//#include "baro.hpp"
#include <acado_toolkit.hpp>
//#include <acado_gnuplot.hpp>


USING_NAMESPACE_ACADO

class Model{

private:

	DifferentialEquation f;
	OutputFcn ym;

public:
	
	Model(bool const isPWD = true);

	DifferentialEquation getDiffEq() const;

	OutputFcn getOutPutEq() const;


};
