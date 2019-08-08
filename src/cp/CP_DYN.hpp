#ifndef _CP_DYN_H
#define _CP_DYN_H

#include "cp_common.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <iostream>


using namespace Eigen;
using namespace std;

class CPDYN{

private:

static constexpr double step_time=0.8;
static constexpr double simulation_step=4.0;
static constexpr double frame_rate=1000;
const double zc=0.7;
const double g=9.8;
const double omega=sqrt(g/zc);

public:

    MatrixXd cp_eos_backward(MatrixXd zmp_control_point, int no_of_preview){
        MatrixXd cp_knot(no_of_preview+1,4);
        MatrixXd cp_ini(no_of_preview+1,2);
        MatrixXd cp_eos(no_of_preview+1,2);

        cp_eos(no_of_preview,0)=zmp_control_point(no_of_preview,0);
        cp_eos(no_of_preview,1)=zmp_control_point(no_of_preview,1);

        for (int i = no_of_preview; i > 0; --i){   
            ///*
            cp_ini(i,0)=zmp_control_point(i,0)+(cp_eos(i,0) - zmp_control_point(i,0))/exp(omega*step_time);
            cp_eos(i-1,0)=cp_ini(i,0);
            cp_ini(i,1)=zmp_control_point(i,1)+(cp_eos(i,1) - zmp_control_point(i,1))/exp(omega*step_time);
            cp_eos(i-1,1)=cp_ini(i,1);
            //*/
        }

        //initial step
        cp_ini(0,1)=zmp_control_point(0,1)+(cp_eos(0,1) - zmp_control_point(0,1))/exp(omega*step_time);

        cp_ini(0,0)=zmp_control_point(0,0)+(cp_eos(0,0) - zmp_control_point(0,0))/exp(omega*step_time);

        cp_knot<<cp_ini,cp_eos; 
        return cp_knot; 
    };


    MatrixXd cp_tracking_forward(MatrixXd zmp_control_point, MatrixXd cp_knot){
        MatrixXd cp((int)(step_time*simulation_step*frame_rate+1),2);
        //MatrixXd cp(3200,2);
        for (int i = 0; i < 4; i++){   
            for (double j = 0; j < step_time*frame_rate+1; j++){
                ///*
                cp(i*step_time*frame_rate+j,0)=exp(omega*j/frame_rate)*cp_knot(i,0)+(1-exp(omega*j/frame_rate))*zmp_control_point(i,0);

                cp(i*step_time*frame_rate+j,1)=exp(omega*j/frame_rate)*cp_knot(i,1)+(1-exp(omega*j/frame_rate))*zmp_control_point(i,1);
                //*/
            }
        }
        return cp;
    };
// this function calculate the zmp reference trajectory from capture point trajectory, the input is the capture point trajectory from cp_tracking_forward and capture point knot from cp_eos_backward
    MatrixXd zmp_trajec(MatrixXd cp, MatrixXd cp_knot){
        MatrixXd zmp_trajec((int)(step_time*simulation_step*frame_rate+1),2);

        for (int i = 0; i < 4; i++){   
            for (double j = 1; j < step_time*frame_rate+1; j++){
                ///*
                zmp_trajec(i*step_time*frame_rate+j,0)=1/(1-exp(omega*step_time))*cp_knot(i,2)-exp(omega*step_time)/(1-exp(omega*step_time))*cp(i*step_time*frame_rate+j,0);

                zmp_trajec(i*step_time*frame_rate+j,1)=1/(1-exp(omega*step_time))*cp_knot(i,3)-exp(omega*step_time)/(1-exp(omega*step_time))*cp(i*step_time*frame_rate+j,1);
                //*/
            }
        }
    return zmp_trajec;
    }
    
    ///* calculate the real capture point from real com status
    double cp_real(double com_pos, double com_vel){
        double cp_real;
        cp_real=com_pos+com_vel/omega;
    }
    //*/


};

#endif