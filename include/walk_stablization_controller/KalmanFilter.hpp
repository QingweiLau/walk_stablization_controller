//this class implements the kalman filter method
#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
using namespace Eigen;

class KalmanFilter {

	public:
		int n; //state vector dimension
		int m; //control vector dimension

		// system and covariance matrices
		MatrixXd A; //System dynamics matrix
		MatrixXd B; //Control matrix 
		MatrixXd C; //Mesaurement matrix
		MatrixXd D; 
		MatrixXd Q; //Process Noise Covariance matrix
		MatrixXd R; //Measurement Noise Covariance matrix
		MatrixXd I; //Identity matrix

		// Variable Matrices 
		VectorXd X; //State vector
		MatrixXd P; //State Covariance
		MatrixXd K; //Kalman Gain

		// Initial State Values 
		VectorXd X0; //Initial State vector
		MatrixXd P0; //Initial State Covariance matrix
		
		// Constructor 		
		KalmanFilter(int _n,  int _m) {
					}
		
		//initilization with structural matrix
		//case 1, no control input
		void initialize( MatrixXd _A, MatrixXd _C, MatrixXd _Q, MatrixXd _R );
		//case 2, with control input but no D matrix
		void initialize( MatrixXd _A, MatrixXd _B, MatrixXd _C,  MatrixXd _Q, MatrixXd _R );
		//case 3, with control input and D matrix
		void initialize( MatrixXd _A, MatrixXd _B, MatrixXd _C, MatrixXd _D, MatrixXd _Q, MatrixXd _R );

		//initialize with initial states
		void initial_state( VectorXd _X0, MatrixXd _P0 );

		// predict X and covariance P
		//case 1, without control input
		void predict(void);
		//case 2, with control input
		void predict( VectorXd U );

		//update with measurement Z
		//case 1, with D matrix
		void update (VectorXd Z, VectorXd U);
		//case 2, without D matrix
		void update ( VectorXd Z);
};

#endif