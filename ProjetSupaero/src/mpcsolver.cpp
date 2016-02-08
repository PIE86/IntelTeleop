#include "mpcsolver.h"


MPCSolver::MPCSolver(double T, int nbNodes) : _referenceVector{0.,0.,20.}, _tmpc(0.02), _ocp( 0.0, T, nbNodes ), _Q(10,10)
{
    // DYNAMIC EQUATION OF THE SYSTEM:
    // ----------------------------------
    _f << dot(x) == vx;
    _f << dot(y) == vy;
    _f << dot(z) == vz;
    _f << dot(vx) == Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*sin(theta)/m;
    _f << dot(vy) == -Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*sin(psi)*cos(theta)/m;
    _f << dot(vz) == Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*cos(psi)*cos(theta)/m - g;
    _f << dot(phi) == -cos(phi)*tan(theta)*p+sin(phi)*tan(theta)*q+r;
    _f << dot(theta) == sin(phi)*p+cos(phi)*q;
    _f << dot(psi) == cos(phi)/cos(theta)*p-sin(phi)/cos(theta)*q;
    _f << dot(p) == (d*Cf*(u1*u1-u2*u2)+(Jy-Jz)*q*r)/Jx;
    _f << dot(q) == (d*Cf*(u4*u4-u3*u3)+(Jz-Jx)*p*r)/Jy;
    _f << dot(r) == (c*(u1*u1+u2*u2-u3*u3-u4*u4)+(Jx-Jy)*p*q)/Jz;
    _f << dot(u1) == vu1;
    _f << dot(u2) == vu2;
    _f << dot(u3) == vu3;
    _f << dot(u4) == vu4;

    // DEFINE CONSTRAINTS:
    // -------------------------------
    // Dynamic
    _ocp.subjectTo( _f );

    // State constraints
    // Constraints on the velocity of each propeller
    _ocp.subjectTo( 16 <= u1 <= 95 );
    _ocp.subjectTo( 16 <= u2 <= 95 );
    _ocp.subjectTo( 16 <= u3 <= 95 );
    _ocp.subjectTo( 16 <= u4 <= 95 );

    // Command constraints
    // Constraints on the acceleration of each propeller
    _ocp.subjectTo( -100 <= vu1 <= 100 );
    _ocp.subjectTo( -100 <= vu2 <= 100 );
    _ocp.subjectTo( -100 <= vu3 <= 100 );
    _ocp.subjectTo( -100 <= vu4 <= 100 );

    // Constraint to avoid singularity
    _ocp.subjectTo( -1. <= theta <= 1.);

    // Example of Eliptic obstacle constraints (here, cylinders with eliptic basis)
    _ocp.subjectTo( 16 <= ((x+3)*(x+3)+2*(z-5)*(z-5)) );
    _ocp.subjectTo( 16 <= ((x-3)*(x-3)+2*(z-9)*(z-9)) );
    _ocp.subjectTo( 16 <= ((x+3)*(x+3)+2*(z-15)*(z-15)) );

    // COST FUNCTION:
    // -------------------------------
    _h << x << y << z;
    _h << vu1 << vu2 << vu3 << vu4;
    _h << p << q << r;

    // Minimization Weights
    double coeffX = 1e-5;
    double coeffU = coeffX*1e-4;
    double coeffX2 = coeffX*1e2;
    //    double coeffX3 = coeffX * 1e-5;
    //    double coeffO = -coeffX * 0.1;

    _Q(0,0) = _Q(1,1) = _Q(2,2) = coeffX;
    _Q(3,3) = _Q(4,4) = _Q(5,5) = _Q(6,6) = coeffU;
    _Q(7,7) = _Q(8,8) = _Q(9,9) = coeffX2;
}


// Optimal Controller part codes:
//    |-- Computation of optimal control one step further
//        from the current initial state
void MPCSolver::controlMPC()
{
    using namespace ACADO;

    DVector ref(10);
    ref(0) = _referenceVector[0];
    ref(1) = _referenceVector[1];
    ref(2) = _referenceVector[2];
    ref(3) = 58.;
    ref(4) = 58.;
    ref(5) = 58.;
    ref(6) = 58.;
    ref(7) = 0.;
    ref(8) = 0.;
    ref(9) = 0.;

    _ocp.minimizeLSQ ( _Q, _h, ref);

    RealTimeAlgorithm alg(_ocp, _tmpc);

    // Usually, you do only one step of the optimisation algorithm (~Gauss-Newton here)
    // at each activation of the MPC, that way the delay between getting the state and
    // sending a command is as quick as possible.
    alg.set(MAX_NUM_ITERATIONS, 1);
    alg.set(PLOT_RESOLUTION,MEDIUM);
    // alg.set(GLOBALIZATION_STRATEGY, GS_LINESEARCH);
    alg.set(INTEGRATOR_TYPE, INT_RK45);
    // alg.set(KKT_TOLERANCE,1e-3);
    alg.set(PRINT_COPYRIGHT, false);

    // StaticReferenceTrajectory:
    // The class StaticReferenceTrajectory allows to define a static reference trajectory (given beforehand)
    // that the ControlLaw aims to track while computing its output.
    //    StaticReferenceTrajectory zeroReference("TempData/ref.txt");
    Controller controller(alg); //,zeroReference);

    DVector stateInit(16);
    // Read the initial state from _stateVector
    for(int j=0;j<16;j++)
        stateInit(j) = _stateVector[j];

    // Debug information output
    //    std::cout << "stateInit:\n" << stateInit << std::endl;

    // Computation of the optimal control calculated
    controller.init(0.0,stateInit);
    controller.step(0.0,stateInit);

    DVector U(4);
    U.setZero(4);

    // Debug information output
    controller.getU(U);
    // Debug information output
//    std::cout << "Number of control computed: " << controller.getNU() << std::endl;
//    std::cout << "U" << U << std::endl;

//    DVector stateFin(16);
//    stateFin.setZero(16);
//    controller.getP(stateFin);


    // Write the calculated control into _controlVector
    for(unsigned int i=0; i<controller.getNU(); i++)
        _commandVector[i] = U(i);

    std::cout << "Optimal control input calculation finished !" << std::endl;
}


void MPCSolver::systemEvol(double t, double dt)
{
    _stateVector = quadRungeKutta(t, _stateVector, _commandVector, dt);
}

void MPCSolver::setReferenceVector(const std::array<double, 3> &referenceVector)
{
    _referenceVector = referenceVector;
}

std::array<double, 16> MPCSolver::stateVector() const
{
    return _stateVector;
}

std::array<double,16> MPCSolver::quadRungeKutta(double t, std::array<double,16> x, std::array<double,4> u, double dt)
{
    // Construction of the slopes K
    std::array<double,16> k[4];

    // Intermediate variables
    // for the calculation of the slopes
    std::array<double,16> x1[3];

    double t1,t2,t3;
    t1 = t + dt/2;
    t2 = t1;
    t3 = t + dt;

    // Calculation of k
    k[0] = quadModel(t,x,u);

    for(unsigned int i=0; i<x1[0].size(); i++)
        x1[0][i] = x[i] + dt/2*k[0][i];

    k[1] = quadModel(t1,x1[0],u);

    for(unsigned int i=0; i<x1[1].size(); i++)
        x1[1][i] = x[i] + dt/2*k[1][i];

    k[2] = quadModel(t2,x1[1],u);

    for(unsigned int i=0; i<x1[2].size(); i++)
        x1[2][i] = x[i] + dt*k[2][i];

    k[3] = quadModel(t3,x1[2],u);


    // The result -- next step position
    std::array<double,16> res;
    for(unsigned int i=0; i<res.size(); i++)
        res[i] = x[i] + dt/6*(k[0][i] + 2*k[1][i] + 2*k[2][i] + k[3][i]);

    return res;
}


std::array<double,16> MPCSolver::quadModel(double, std::array<double,16> x, std::array<double,4> u)
{
    // Quad constants
    const double c = 0.00001;
    const double Cf = 0.00065;
    const double d = 0.250;
    const double Jx = 0.018;
    const double Jy = 0.018;
    const double Jz = 0.026;
    //    const double Im = 0.0001;
    const double m = 0.9;
    const double g = 9.81;
    //    const double Cx = 0.1;

    // Minimization Weights
    //    double coeffX = .00001;
    //    double coeffU = coeffX*0.0001;
    //    double coeffX2 = coeffX * 100.;
    //    double coeffX3 = coeffX * 0.00001;
    //    double coeffO = -coeffX * 0.1;


    std::array<double,16> dx;
    // auxilary variable
    double normU2 = u[0]*u[0]+u[1]*u[1]+u[2]*u[2]+u[3]*u[3];

    // Differential equations of the quadrotor
    dx[0] = x[3];//dot_x
    dx[1] = x[4];//dot_y
    dx[2] = x[5];//dot_z

    dx[3] = Cf*normU2*sin(x[7])/m;//dot_vx
    dx[4] = -Cf*normU2*sin(x[8])*cos(x[7])/m;//dot_vy
    dx[5] = Cf*normU2*cos(x[8])*cos(x[7])/m - g;//dot_vz

    dx[6] = -cos(x[6])*tan(x[7])*x[9]+sin(x[6])*tan(x[7])*x[10]+x[11];//dot_phi
    dx[7] = sin(x[6])*x[9]+cos(x[6])*x[10];//dot_theta
    dx[8] = cos(x[6])/cos(x[7])*x[9]-sin(x[6])/cos(x[7])*x[10];//dot_psi

    dx[9] = (d*Cf*(u[0]*u[0]-u[1]*u[1])+(Jy-Jz)*x[10]*x[11])/Jx;//dot_p
    dx[10] = (d*Cf*(u[3]*u[3]-u[2]*u[2])+(Jz-Jx)*x[9]*x[11])/Jy;//dot_q
    dx[11] = (c*(u[0]*u[0]+u[1]*u[1]-u[2]*u[2]-u[3]*u[3])+(Jx-Jy)*u[9]*u[10])/Jz;//dot_r


    dx[12] = u[0];//dot_u1
    dx[13] = u[1];//dot_u2
    dx[14] = u[2];//dot_u3
    dx[15] = u[3];//dot_u4

    return dx;
}
