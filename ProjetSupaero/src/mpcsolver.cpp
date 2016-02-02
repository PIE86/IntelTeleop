#include "mpcsolver.h"


MPCSolver::MPCSolver() : _referenceVector{0.,0.,20.}
{
}


/* Optimal Controller part codes:
    |-- Computation of optimal control one step further
        from the current initial state
    |-- The current initial state is read from an initial state file
        while the optimal control input is written in another file*/
void MPCSolver::controlMPC()
{
    using namespace ACADO;

    // Minimization Weights
    double coeffX = .00001;
    double coeffU = coeffX*0.0001;
    double coeffX2 = coeffX * 100.;
//    double coeffX3 = coeffX * 0.00001;
//    double coeffO = -coeffX * 0.1;

    // length (in second) of the trajectory predicted in the MPC
    double T = 8.;
    // time (in second) between two activation of the MPC algorithm
    double tmpc = 0.02;
    // number of nodes used in the Optimal Control Problem
    // 20 nodes means that the algorithm will discretize the trajectory equally into 20 pieces
    // If you increase the number of nodes,
    // the solution will be more precise but calculation will take longer (~nb_nodes^2)
    // In ACADO, the commands are piecewise constant functions, constant between each node.
    int nb_nodes = 20;


    // DEFINE A OPTIMAL CONTROL PROBLEM
    // -------------------------------
    OCP ocp( 0.0, T, nb_nodes );

    // DEFINE THE COST FUNCTION
    // -------------------------------
    Function h;
    h << x << y << z;
    h << vu1 << vu2 << vu3 << vu4;
    h << p << q << r;

    DMatrix Q(10,10);
    Q(0,0) = Q(1,1) = Q(2,2) = coeffX;
    Q(3,3) = Q(4,4) = Q(5,5) = Q(6,6) = coeffU;
    Q(7,7) = Q(8,8) = Q(9,9) = coeffX2;

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

    ocp.minimizeLSQ ( Q, h, ref);

    // DEFINE THE DYNAMIC EQUATION OF THE SYSTEM:
    // ----------------------------------
    f << dot(x) == vx;
    f << dot(y) == vy;
    f << dot(z) == vz;
    f << dot(vx) == Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*sin(theta)/m;
    f << dot(vy) == -Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*sin(psi)*cos(theta)/m;
    f << dot(vz) == Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*cos(psi)*cos(theta)/m - g;
    f << dot(phi) == -cos(phi)*tan(theta)*p+sin(phi)*tan(theta)*q+r;
    f << dot(theta) == sin(phi)*p+cos(phi)*q;
    f << dot(psi) == cos(phi)/cos(theta)*p-sin(phi)/cos(theta)*q;
    f << dot(p) == (d*Cf*(u1*u1-u2*u2)+(Jy-Jz)*q*r)/Jx;
    f << dot(q) == (d*Cf*(u4*u4-u3*u3)+(Jz-Jx)*p*r)/Jy;
    f << dot(r) == (c*(u1*u1+u2*u2-u3*u3-u4*u4)+(Jx-Jy)*p*q)/Jz;
    f << dot(u1) == vu1;
    f << dot(u2) == vu2;
    f << dot(u3) == vu3;
    f << dot(u4) == vu4;

    // ---------------------------- DEFINE CONSTRAINTS --------------------------------- //
    // Dynamic
    ocp.subjectTo( f );

    // State constraints
    // Constraints on the velocity of each propeller
    ocp.subjectTo( 16 <= u1 <= 95 );
    ocp.subjectTo( 16 <= u2 <= 95 );
    ocp.subjectTo( 16 <= u3 <= 95 );
    ocp.subjectTo( 16 <= u4 <= 95 );

    // Command constraints
    // Constraints on the acceleration of each propeller
    ocp.subjectTo( -100 <= vu1 <= 100 );
    ocp.subjectTo( -100 <= vu2 <= 100 );
    ocp.subjectTo( -100 <= vu3 <= 100 );
    ocp.subjectTo( -100 <= vu4 <= 100 );

    // Constraint to avoid singularity
    ocp.subjectTo( -1. <= theta <= 1.);

    // Example of Eliptic obstacle constraints (here, cylinders with eliptic basis)
    ocp.subjectTo( 16 <= ((x+3)*(x+3)+2*(z-5)*(z-5)) );
    ocp.subjectTo( 16 <= ((x-3)*(x-3)+2*(z-9)*(z-9)) );
    ocp.subjectTo( 16 <= ((x+3)*(x+3)+2*(z-15)*(z-15)) );


    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
    RealTimeAlgorithm alg(ocp, tmpc);

    // Usually, you do only one step of the optimisation algorithm (~Gauss-Newton here)
    // at each activation of the MPC, that way the delay between getting the state and
    // sending a command is as quick as possible.
    alg.set(MAX_NUM_ITERATIONS, 1);
    alg.set(PLOT_RESOLUTION,MEDIUM);
    // alg.set(GLOBALIZATION_STRATEGY, GS_LINESEARCH);
    alg.set(INTEGRATOR_TYPE, INT_RK45);
    // alg.set(KKT_TOLERANCE,1e-3);


    // StaticReferenceTrajectory:
    // The class StaticReferenceTrajectory allows to define a
    // static reference trajectory (given beforehand)
    // that the ControlLaw aims to track
    // while computing its output.
    StaticReferenceTrajectory zeroReference("TempData/ref.txt");
    Controller controller(alg,zeroReference);

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
    std::cout << "Number of control computed: " << controller.getNU() << std::endl;
    std::cout << "u=" << U << std::endl;
    std::cout << (ACADO::SUCCESSFUL_RETURN == controller.getU(U)) << std::endl;
    // Debug information output
    std::cout << "Number of control computed: " << controller.getNU() << std::endl;
    std::cout << "u.size=" << U.size() << std::endl;

    DVector stateFin(16);
    stateFin.setZero(16);
    controller.getP(stateFin);

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
