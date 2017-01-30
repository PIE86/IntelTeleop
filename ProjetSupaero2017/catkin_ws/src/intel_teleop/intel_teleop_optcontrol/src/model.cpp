#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/evaluation_point.hpp>
#include <acado/function/function_.hpp>



BEGIN_NAMESPACE_ACADO

Model::Model(){
	
	f << dot(x) == vx;
	f << dot(y) == vy;
	f << dot(z) == vz;
	f << dot(vx) == ax;
	f << dot(vy) == ay;
	f << dot(vz) == az;
	f << dot(phi) == -cos(phi)*tan(theta)*p+sin(phi)*tan(theta)*q+r - az;
	f << dot(theta) == sin(phi)*p+cos(phi)*q;
	f << dot(psi) == cos(phi)/cos(theta)*p-sin(phi)/cos(theta)*q;
	f << dot(p) == (d*Cf*(u1*u1-u2*u2)+(Jy-Jz)*q*r)/Jx;
	f << dot(q) == (d*Cf*(u4*u4-u3*u3)+(Jz-Jx)*p*r)/Jy;
	f << dot(r) == (c*(u1*u1+u2*u2-u3*u3-u4*u4)+(Jx-Jy)*p*q)/Jz;
	f << dot(b_ax) == -1/tau_bax*b_ax;
	f << dot(b_ay) == -1/tau_bay*b_ay;
	f << dot(b_az) == -1/tau_baz*b_az;
	f << dot(b_p) == -1/tau_bp*b_p;
	f << dot(b_q) == -1/tau_bq*b_q;
	f << dot(b_r) == -1/tau_br*b_r;
	f << dot(b_bar) == -1/tau_bbar*b_bar;
	f << 0 == Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*sin(theta)/m -ax;
	f << 0 == -Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*sin(psi)*cos(theta)/m - ay;
	f << 0 == Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*cos(psi)*cos(theta)/m - g;
	
	
	
	ym << cos(theta)*cos(psi)*(ax-g) + cos(theta)*sin(psi)*(ay-g)
			-sin(theta)*(az-g) + b_ax;
	ym << (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi))*(ax-g)
		+ (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi))*(ay-g) 
		+ sin(phi)*cos(theta)*(az-g) + b_ay;
	ym << (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*(ax-g)
		+ (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*(ay-g) 
		+ cos(phi)*cos(theta)*(az-g) + b_az;	
	ym << p + + b_p;
	ym << q + + b_q;
	ym << r + + b_r;
	ym << getStandardPressure(z) + b_bar;
	
	int n=f.getDim();
    int m=ym.getDim();
	
	stateVariance = DMatrix(n,n);  stateVariance.setAll(0);
	outputVariance  = DMatrix(m,m);  outputVariance.setAll(0);
	
	Disturbance wState = Disturbace(n);
	Disturbance wOutPut = Disturbance(m);
	
	f= f + stateVariance*wState;
	ym= ym + outputVariance*wOutPut;
	
	}
	


Model::Model(const DMatrix _stateVariance, const DMatrix _outputVariance){
	
	f << dot(x) == vx;
	f << dot(y) == vy;
	f << dot(z) == vz;
	f << dot(vx) == ax;
	f << dot(vy) == ay;
	f << dot(vz) == az;
	f << dot(phi) == -cos(phi)*tan(theta)*p+sin(phi)*tan(theta)*q+r - az;
	f << dot(theta) == sin(phi)*p+cos(phi)*q;
	f << dot(psi) == cos(phi)/cos(theta)*p-sin(phi)/cos(theta)*q;
	f << dot(p) == (d*Cf*(u1*u1-u2*u2)+(Jy-Jz)*q*r)/Jx;
	f << dot(q) == (d*Cf*(u4*u4-u3*u3)+(Jz-Jx)*p*r)/Jy;
	f << dot(r) == (c*(u1*u1+u2*u2-u3*u3-u4*u4)+(Jx-Jy)*p*q)/Jz;
	f << dot(b_ax) == -1/tau_bax*b_ax;
	f << dot(b_ay) == -1/tau_bay*b_ay;
	f << dot(b_az) == -1/tau_baz*b_az;
	f << dot(b_p) == -1/tau_bp*b_p;
	f << dot(b_q) == -1/tau_bq*b_q;
	f << dot(b_r) == -1/tau_br*b_r;
	f << dot(b_bar) == -1/tau_bbar*b_bar;
	f << 0 == Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*sin(theta)/m -ax;
	f << 0 == -Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*sin(psi)*cos(theta)/m - ay;
	f << 0 == Cf*(u1*u1+u2*u2+u3*u3+u4*u4)*cos(psi)*cos(theta)/m - g;
	
	
	
	ym << cos(theta)*cos(psi)*(ax-g) + cos(theta)*sin(psi)*(ay-g)
			-sin(theta)*(az-g) + b_ax;
	ym << (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi))*(ax-g)
		+ (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi))*(ay-g) 
		+ sin(phi)*cos(theta)*(az-g) + b_ay;
	ym << (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*(ax-g)
		+ (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*(ay-g) 
		+ cos(phi)*cos(theta)*(az-g) + b_az;	
	ym << p + + b_p;
	ym << q + + b_q;
	ym << r + + b_r;
	ym << getStandardPressure(z) + b_bar;
	
	int n=f.getDim();
    int m=ym.getDim();
	
	stateVariance = _stateVariance;
	outputVariance  = _outputVariance;
	
	Disturbance wState = Disturbace(n);
	Disturbance wOutPut = Disturbance(m);
	
	f= f + stateVariance*wState;
	ym= ym + outputVariance*wOutPut;
	
	}	
	
	DifferentialEquation Model::getDiffEq() const{
		return f;	
	}
	

	
	OutputFcn Model::getOutPutEq() const{
		return ym
	}
	
	returnValue Model::jacobianDifferential(DMatrix &A, DMatrix &B){
		int n=f.getDim();
		int N=f.getNumberOfVariables();
		returnValue ret;

		A=DMatrix(getNX(),getNX());A.setAll(0);
		B=DMatrix(getNX(),getNA());A.setAll(0);
		//u=DMatrix(getNU(),n);u.setAll(0);
		//p=DMatrix(getNP(),n);p.setAll(0);
		//w=DMatrix(getNW(),n);w.setAll(0);
		double *Jr=new double[N];
		double *seed=new double[n];
		for (int i=0;i<n;i++) seed[i]=0;
			for (int i=0;i<n;i++) {
				if (i>0) seed[i-1]=0;
					seed[i]=1;
			for (int j=0;j<N;j++) Jr[j]=0;
			
			ret=f.AD_backward(0,seed,Jr);
			
			if (ret != SUCCESSFUL_RETURN) return ret;
			
			for (int j=0;j<getNX();j++) x(j,i)=Jr[index(VT_DIFFERENTIAL_STATE,j)];
			
			for (int j=0;j<getNA();j++) x(j,i)=Jr[index(VT_ALGEBRAIC_STATE,j)];
		}
		delete[] Jr;
		delete[] seed;
		return SUCCESSFUL_RETURN;
	}

		
		
		
		}
	
	returnValue Model::jacobianYm(DMatrix &C,DMatrix &D);
	
	DVector Model::evaluateF(const EvaluationPoint &x,
                           const int        &number);
	
	DVector Model::evaluateH(const EvaluationPoint &x,
                           const int        &number);
	
	
	
	
