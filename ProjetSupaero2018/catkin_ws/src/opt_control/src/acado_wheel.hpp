#ifndef WHEEL_WITH_TEMPLATE_HPP
#define WHEEL_WITH_TEMPLATE_HPP

// problem dimensions
const int nxd = 3; // number of differential states
const int nu  = 2; // number of controls

// macros for variables
#define X xd[0]
#define Y xd[1]
#define Th xd[2]

#define V u[0]
#define W u[1]

// Names (e.g. for plotting)
const char* xd_names[nxd] = {"DifferentialState x",
															"DifferentialState y",
															"DifferentialState theta"};
const char* u_names[nu] = {"Control v", "Control w"};

// parameters
const double t_start =  0.0;
const double t_end   = 10.0;

const bool   xd_0_fixed[nxd] = { true , true , true };
const bool   xd_f_fixed[nxd] = { true , true , false};

const double xd_0[nxd]       = {  0.0 ,  0.0 ,  1.0 };
const double xd_f[nxd]       = { 10.0 ,  0.0 ,  0.0 };

const bool   xd_has_lb[nxd]  = { false, true , false};
const bool   xd_has_ub[nxd]  = { false, true,  false};

const double xd_lb[nxd]       = {  0.0 , -0.01,  0.0 };
const double xd_ub[nxd]       = {  0.0 ,   1.3,  0.0 };


// evaluate objective function
template <class DifferentialStateType, class ControlType, class ReturnType>
void eval_F(const DifferentialStateType *xd, 
	    const ControlType *u,
	    ReturnType &lfun){
  
  lfun = U*U;
  
}

// evaluate right hand side of the ODE
template <class DifferentialStateType, class ControlType, class ReturnType>
void eval_G(const DifferentialStateType *xd, 
	    const ControlType *u, 
	    ReturnType *rhs){
  
  rhs[0] = V;
  rhs[1] = (U-0.02*V*V)/M;
  rhs[2] = -0.01*U*U;
  
} 


#undef U
#undef V
#undef S
#undef M


#endif // WHEEL_WITH_TEMPLATE_HPP

