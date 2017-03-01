#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ctime>

#include "optcontrol.hpp"

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>

Optcontrol::Optcontrol(DMatrix &Q, DVector &refVec,
                       const double t_in, const double t_fin, const double dt, DVector &X_0, bool isPWD) {

    _Q = Q;
    _refVec = refVec;

    DifferentialEquation f;

    // Introducing constants
    const double c = 0.00001;
    const double Cf = 0.00065;
    const double d = 0.250;
    const double Jx = 0.018;
    const double Jy = 0.018;
    const double Jz = 0.026;
    const double m = 0.9;
    const double g = 9.81;

    OCP ocp(t_in, t_fin, static_cast< int >((t_fin - t_in) / dt ));


    if (isPWD) {
        DifferentialState x, y, z, vx, vy, vz, phi, theta, psi, p, q, r;

        // x, y, z : position
        // vx, vy, vz : linear velocity
        // phi, theta, psi : orientation (Yaw-Pitch-Roll = Euler(3,2,1))
        // p, q, r : angular velocity
        // u1, u2, u3, u4 : velocity of the propellers
        Control u1, u2, u3, u4;


        f << dot(x) == vx;
		f << dot(y) == vy;
		f << dot(z) == vz;
		f << dot(phi) == p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
		f << dot(theta) == cos(phi)*q - sin(phi)*r;
		f << dot(psi) == sin(phi)/cos(theta)*q + cos(phi)/cos(theta)*r;
		f << dot(p) == (d*Cf*(u4*u4-u2*u2)+(Jy-Jz)*q*r)/Jx;
		f << dot(q) == (d*Cf*(u1*u1-u3*u3)+(Jz-Jx)*p*r)/Jy;
		f << dot(r) == (c*(-u1*u1+u2*u2-u3*u3+u4*u4)+(Jx-Jy)*p*q)/Jz;
		f << dot(vx) == -Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*(cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi))/m;
		f << dot(vy) == -Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*(sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi))/m;
		f << dot(vz) == -Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*cos(psi)*cos(theta)/m + g; // axe z vers le bas



        // Constraints on the velocity of each propeller
        ocp.subjectTo(16 <= u1 <= 95);
        ocp.subjectTo(16 <= u2 <= 95);
        ocp.subjectTo(16 <= u3 <= 95);
        ocp.subjectTo(16 <= u4 <= 95);

        // Constraint to avoid singularity
        ocp.subjectTo(-1. <= theta <= 1.);

        _h << vx << vy << vz;
        _h << u1 << u2 << u3 << u4;
        _h << p << q << r;

        if (_Q.getNumCols() != 10 || _Q.getNumRows() != 10) {
            std::cout << "Weight matrix size is not suitable for this model" << std::endl;
        }


    } else {

        DifferentialState x, y, z, phi, theta, psi;

        // x, y, z : position
        // vx, vy, vz : linear velocity
        // phi, theta, psi : orientation (Yaw-Pitch-Roll = Euler(3,2,1))

        Control u_vx, u_vy, u_vz, u_p, u_q, u_r; // Linear and angular velocity

        f << dot(x) == u_vx;
		f << dot(y) == u_vy;
		f << dot(z) == u_vz;
		f << dot(phi) == u_p + sin(phi)*tan(theta)*u_q + cos(phi)*tan(theta)*u_r;
		f << dot(theta) == cos(phi)*u_q - sin(phi)*u_r;
		f << dot(psi) == sin(phi)/cos(theta)*u_q + cos(phi)/cos(theta)*u_r;
		f << ax == dot(u_vx);
		f << ay == dot(u_vy);
		f << az == dot(u_vz);

        ocp.subjectTo(-15 <= u_p <= 15);
        ocp.subjectTo(-15 <= u_q <= 15);
        ocp.subjectTo(-15 <= u_r <= 15);

        _h << u_vx << u_vy << u_vz;
        _h << u_p << u_q << u_r;

        if (_Q.getNumCols() != 6 || _Q.getNumRows() != 6) {
            std::cout << "Weight matrix size is not suitable for this model" << std::endl;
        }

    }

    ocp.minimizeLSQ(_Q, _h, _refVec);
    ocp.subjectTo(f);

//    EnvironmentParser
//    parser(PIE_SOURCE_DIR
//    "/data/envsave.xml");
//    auto cylinders = parser.readData();
//    for (Ecylinder c : cylinders) {
//        ocp.subjectTo(pow(c.radius + 1, 2) <=
//                      (pow((y - c.y1) * (c.z2 - c.z1) - (c.y2 - c.y1) * (z - c.z1), 2) +
//                       pow((z - c.z1) * (c.x2 - c.x1) - (c.z2 - c.z1) * (x - c.x1), 2) +
//                       pow((x - c.x1) * (c.y2 - c.y1) - (c.x2 - c.x1) * (y - c.y1), 2)) /
//                      (pow(c.x2 - c.x1, 2) + pow(c.y2 - c.y1, 2) + pow(c.z2 - c.z1, 2))
//        );
//    }

    DynamicSystem dynamicSystem(f, OutputFcn{});
    _process = std::unique_ptr<Process>( new Process( dynamicSystem, INT_RK45 ) );


    // SET UP THE MPC CONTROLLER:
    // --------------------------
    _alg = std::unique_ptr<RealTimeAlgorithm>( new RealTimeAlgorithm( ocp ) );
    _alg->set(INTEGRATOR_TYPE, INT_RK45);
    _alg->set(MAX_NUM_ITERATIONS, 1);
    _alg->set(PRINT_COPYRIGHT, false);
    _alg->set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);

    _controller = std::unique_ptr<Controller>( new Controller( *_alg ) );

    // SETTING UP THE SIMULATION ENVIRONMENT:
    // --------------------------------------
    DVector u(4);
    u.setZero();
    _controller->init(0., X_0);
    _process->init(0., X_0, u);


};


DVector
Optcontrol::solveOptimalControl(DVector &NewRefVec, DVector &x_est, double &t ) {

    if (abs(NewRefVec[0] - _refVec(0)) > 1.) {
        if (NewRefVec[0] > _refVec(0)) {
            NewRefVec[0] = _refVec(0) + 1.;
        } else {
            NewRefVec[0] = _refVec(0) - 1.;
        }
    }

    if (abs(NewRefVec[1] - _refVec(1)) > 1.) {
        if (NewRefVec[1] > _refVec(1)) {
            NewRefVec[1] = _refVec(1) + 1.;
        } else {
            NewRefVec[1] = _refVec(1) - 1.;
        }
    }

    if (abs(NewRefVec[2] - _refVec(2)) > 1.) {
        if (NewRefVec[2] > _refVec(2)) {
            NewRefVec[2] = _refVec(2) + 1.;
        } else {
            NewRefVec[2] = _refVec(2) - 1.;
        }
    }

    double refT[10] = {NewRefVec[0], NewRefVec[1], NewRefVec[2], 0., 0., 0., 0., 0., 0., 0.};
    DVector refVecN(10, refT);
    VariablesGrid referenceVG(refVecN, Grid{t, t + 1., 2});
    referenceVG.setVector(0, _refVec);
    _alg->setReference(referenceVG);
    setrefVec(refVecN);

/*
DVector X;
VariablesGrid Y;
Y.setZero();

// get state vector
process.getY(Y);
X = Y.getLastVector();
*/

// MPC step
// compute the command
    bool success = _controller->step(t, x_est);

    if (success != 0) {
        std::cout << "controller failed " <<
                  std::endl;
        return 1;
    }
    DVector u(4);
    u.init();

    u.setZero();

    _controller->getU(u);


    std::clock_t t_new = std::clock();
    double dt = (double) (t_new - t) / (double) CLOCKS_PER_SEC;
    _process->step(t, t + dt, u);
    t += dt;

    return u;
}


DMatrix Optcontrol::getMatrixQ()  {
    return _Q;
}

void Optcontrol::setMatrixQ(DMatrix &Q) {
    _Q = Q;
}

DVector Optcontrol::getrefVec()  {
    return _refVec;
}

void Optcontrol::setrefVec(DVector &refVec) {
    _refVec = refVec;
}

