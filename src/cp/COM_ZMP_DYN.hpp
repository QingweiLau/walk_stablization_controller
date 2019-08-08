#ifndef _COM_ZMP_DYN_H
#define _COM_ZMP_DYN_H

#include "cp_common.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <iostream>

using namespace Eigen;
using namespace std;

class COM_ZMP_DYN{
private:
    // TO DO get current com pos and vel
    double com_pos=0;
    double com_vel=0;
    static constexpr double step_time=0.8;
    static constexpr double pre_step=4.0;
    static constexpr double frame_rate=1000;
    const double zc=0.7;
    const double g=9.8;
    const double omega=sqrt(g/zc);
public:
    
    MatrixXd A_matrix(int i){
        MatrixXd A_matrix(2,2);
        A_matrix<< cosh(omega*i/frame_rate), sinh(omega*i/frame_rate)/omega, omega*sinh(omega*i/frame_rate), cosh(omega*i/frame_rate);
        return A_matrix;
    };
    MatrixXd B_matrix(int i){
        MatrixXd B_matrix(2,1);
        B_matrix<< 1-cosh(omega*i/frame_rate), -omega*sinh(omega*i/frame_rate);
        return B_matrix;
    };
    

    MatrixXd pos_v_traj(MatrixXd zmp_trajec){
        MatrixXd com_status(2,1);
        com_status<<com_pos,com_vel;

        MatrixXd com((int)(step_time*pre_step*frame_rate),4);
        MatrixXd comx((int)(step_time*pre_step*frame_rate),2);
        MatrixXd comy((int)(step_time*pre_step*frame_rate),2);
        //cout<<A_matrix(1)<<endl;
        //cout<<B_matrix(1)<<endl;
        //cout<<A_matrix(1)*B_matrix(1)<<endl;

        ///*
        for (int i = 0; i < step_time*pre_step*frame_rate; i++){
            comx.row(i)=(A_matrix(i)*com_status+B_matrix(i)*zmp_trajec(i,0)).transpose();
            comy.row(i)=(A_matrix(i)*com_status+B_matrix(i)*zmp_trajec(i,1)).transpose();
        } 
        //*/
        com<<comx,comy;
        return com;
        };
};

#endif
