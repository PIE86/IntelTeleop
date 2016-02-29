#include <iostream>
#include <fstream>
#include <time.h>

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

#include "input.h"
#include "viewer.h"

using std::cout; using std::endl;


int main()
{
    USING_NAMESPACE_ACADO;

    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState x,y,z, vx,vy,vz, phi,theta,psi, p,q,r, u1,u2,u3,u4;
    // x, y, z : position
    // vx, vy, vz : linear velocity
    // phi, theta, psi : orientation (Yaw-Pitch-Roll = Euler(3,2,1))
    // p, q, r : angular velocity
    // u1, u2, u3, u4 : velocity of the propellers
    Control vu1,vu2,vu3,vu4;
    // vu1, vu2, vu3, vu4 : derivative of u1, u2, u3, u4

    // Quad constants
    const double c  = 0.00001;
    const double Cf = 0.00065;
    const double d  = 0.250;
    const double Jx = 0.018;
    const double Jy = 0.018;
    const double Jz = 0.026;
//    const double Im = 0.0001;
    const double m  = 0.9;
    const double g  = 9.81;
//    const double Cx = 0.1;

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    DifferentialEquation f;

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


    // SET UP THE (SIMULATED) PROCESS:
    // -----------------------------------
    DynamicSystem dynamicSystem(f,OutputFcn{});
    Process process(dynamicSystem,INT_RK45);


    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
    Function h;
    h << vx << vy << vz;
    h << vu1 << vu2 << vu3 << vu4;
    h << p << q << r;

    // LSQ coefficient matrix
    DMatrix Q(10,10);
    Q(0,0) = Q(1,1) = Q(2,2) = 1e-1;
    Q(3,3) = Q(4,4) = Q(5,5) = Q(6,6) = 1e-9;
    Q(7,7) = Q(8,8) = Q(9,9) = 1e-3;

    // Reference
    DVector refVec(10);
    refVec.setZero(10);


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp(0., 1., 20);

    ocp.minimizeLSQ(Q, h, refVec);

    ocp.subjectTo(f);    // Constraints on the velocity of each propeller
    ocp.subjectTo(16 <= u1 <= 95);
    ocp.subjectTo(16 <= u2 <= 95);
    ocp.subjectTo(16 <= u3 <= 95);
    ocp.subjectTo(16 <= u4 <= 95);

    // Command constraints
    // Constraints on the acceleration of each propeller
    ocp.subjectTo(-100 <= vu1 <= 100);
    ocp.subjectTo(-100 <= vu2 <= 100);
    ocp.subjectTo(-100 <= vu3 <= 100);
    ocp.subjectTo(-100 <= vu4 <= 100);

    // Constraint to avoid singularity
    ocp.subjectTo(-1. <= theta <= 1.);

    // Example of Eliptic obstacle constraints (here, cylinders with eliptic basis)
//    ocp.subjectTo(16 <= ((x+3)*(x+3)+2*(z-5)*(z-5)));
//    ocp.subjectTo(16 <= ((x-3)*(x-3)+2*(z-9)*(z-9)));
//    ocp.subjectTo(16 <= ((x+3)*(x+3)+2*(z-15)*(z-15)));


    // SET UP THE MPC CONTROLLER:
    // ------------------------------
    RealTimeAlgorithm alg(ocp);
    alg.set(INTEGRATOR_TYPE, INT_RK78);
    alg.set(MAX_NUM_ITERATIONS,1);
    alg.set(PRINT_COPYRIGHT, false);

    Controller controller(alg);

    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
    DVector X(16), U(4);
    X.setZero(16);
    X(12) = X(13) = X(14) = X(15) = 58.;
    U.setZero(4);
    controller.init(0., X);
    process.init(0., X, U);

    VariablesGrid Y, graph;
    Y.setZero();

    // END OF ACADO SOLVER SETUP
    // -------------------------

    // Get input from keyboard
    Input input;

//    std::cout << system("pwd") << std::endl;
//    std::cout << system("ls ../data") << std::endl;

    // Gepetto viewer over corba
    Viewer viewer;
    //viewer.createEnvironment();

    viewer.createDrone("/home/baudouin/Documents/travail/4A/PIE/PIE-drone/ProjetSupaero/data/quadrotor_base.stl");

    double t = 0;
    double dt = .05;
    for (int i=0; i<300; i++, t+=dt)
    {
        // setting reference from input
        std::array<double,3> refInput = input.getReference();
        double refT[10] = {refInput[0], refInput[1], refInput[2], 0., 0., 0., 0., 0., 0., 0.};
        DVector refVec(10, refT);
        VariablesGrid referenceVG (refVec, Grid{t, t+1., 2});
        alg.setReference(referenceVG);

        // MPC step
        process.getY(Y);
        X = Y.getLastVector();

        viewer.moveDrone((float)X(0), (float)X(1), (float)X(2), (float)X(8), (float)X(7), (float)X(6));

        controller.step(t, X);
        controller.getU(U);
        process.step(t,t+dt,U);

        graph.addVector(X,t);
    }

    GnuplotWindow window;
//    window.addSubplot(graph(0), "x");
//    window.addSubplot(graph(1), "y");
//    window.addSubplot(graph(2), "z");
//    window.addSubplot(graph(3), "vx");
//    window.addSubplot(graph(4), "vy");
//    window.addSubplot(graph(5), "vz");
    window.addSubplot(graph(6), "phi");
    window.addSubplot(graph(7), "theta");
    window.addSubplot(graph(8), "psi");
    window.addSubplot(graph(9), "p");
    window.addSubplot(graph(10), "q");
    window.addSubplot(graph(11), "r");
//    window.addSubplot(graph(12), "u1");
//    window.addSubplot(graph(13), "u2");
//    window.addSubplot(graph(14), "u3");
//    window.addSubplot(graph(15), "u4");
    window.plot();

    return 0;
}
