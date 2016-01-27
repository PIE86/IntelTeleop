


// The dynamic model of the quadrotor
double* quadModel(double t,double* x,double* u,int lenX);

// Calculation of control law by MPC using ACADO
int controlMPC();

// Based on the control input, calculate the evolution of the system
// first using RungeKutta
int systemEvol(double t, double dt);

// Resolution of differential equations using RungeKutta method
double* quadRungeKutta(double t,double*x,double*u,int lenX,double dt);
