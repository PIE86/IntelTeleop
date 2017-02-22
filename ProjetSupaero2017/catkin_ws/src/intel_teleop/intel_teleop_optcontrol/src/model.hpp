#ifndef ACADO_TOOLKIT_FUNCTION_HPP
#define ACADO_TOOLKIT_FUNCTION_HPP

#endif  // ACADO_TOOLKIT_FUNCTION_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ctime>

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

//#include "input.h"
//#include "viewer.h"
//#include "environmentparser.h"
#include "baro.hpp"


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

CLOSE_NAMESPACE_ACADO
