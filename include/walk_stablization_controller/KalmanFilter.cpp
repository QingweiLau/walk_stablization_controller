#include "KalmanFilter.hpp"
#include <iostream>


//implementation of kalman filter

		void KalmanFilter::initialize( MatrixXd _A, MatrixXd _C, MatrixXd _Q, MatrixXd _R ){
			A = _A;
			C = _C;
			Q = _Q;
			R = _R;
			I = I.Identity(n, n);
		}
		//case 2, with control input but no D matrix
		void KalmanFilter::initialize( MatrixXd _A, MatrixXd _B, MatrixXd _C,  MatrixXd _Q, MatrixXd _R ){
			A = _A;
			B = _B;
			C = _C;
			Q = _Q;
			R = _R;
			I = I.Identity(n, n);
		}
		//case 3, with control input and D matrix
		void KalmanFilter::initialize( MatrixXd _A, MatrixXd _B, MatrixXd _C, MatrixXd _D, MatrixXd _Q, MatrixXd _R ){
			A = _A;
			B = _B;
			C = _C;
			D = _D;
			Q = _Q;
			R = _R;
			I = I.Identity(n, n);
		}

		
		void KalmanFilter::initial_state( VectorXd _X0, MatrixXd _P0 ){
			X0 = _X0;
			P0 = _P0;
		}		

		// predict X and covariance P
		//case 1, without control input
		void KalmanFilter::predict(void){
			X = (A * X0);
			P = (A * P0 * A.transpose()) + Q;
		}
		//case 2, with control input
		void KalmanFilter::predict( VectorXd U ){
			X = (A * X0) + (B * U);
			P = (A * P0 * A.transpose()) + Q;
		}

		//update with measurement Z
		//case 1, with D matrix
		void KalmanFilter::update (VectorXd Z, VectorXd U) {
			K = ( P * C.transpose() ) * ( C * P * C.transpose() + R).inverse();
			X = X + K*(Z - C * X- D* U);
			P = (I - K * C) * P;
			X0 = X;
			P0 = P;
		}
		//case 2, without D matrix
		void KalmanFilter::update ( VectorXd Z) {
			K = ( P * C.transpose() ) * ( C * P * C.transpose() + R).inverse(); 
			X = X + K*(Z - C * X);
			P = (I - K * C) * P;
			X0 = X;
			P0 = P;
		}
