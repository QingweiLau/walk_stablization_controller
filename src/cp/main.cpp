
#include "common.hpp"
#include "log.hpp"



static const int no_of_preview=3;
MatrixXd zmp_control_point(no_of_preview+1,2);
MatrixXd zmp_control_point2(no_of_preview+1,2);
MatrixXd zmp(no_of_preview+1,4);

CPDYN cpdemo;
COM_ZMP_DYN comdemo;
std::ofstream ofs;

int main(){
    zmp_control_point << 0.3,-0.1, 0.6,0.1, 0.9,-0.1,1.2,0.1;
    //zmp_control_point << 0,0, 0.3,-0.1, 0.6,0.1, 0.9,-0.1;

    //zmp<<zmp_control_point,zmp_control_point2;
    //cout<<zmp_control_point(no_of_preview,0)<<endl;
    //cout<<cpdemo.cp_eos_backward(zmp_control_point,0.3,3)<<endl;
    MatrixXd cp_knot=cpdemo.cp_eos_backward(zmp_control_point,3);
    //cout<<cp_knot<<endl;
    MatrixXd cp_traj=cpdemo.cp_tracking_forward(zmp_control_point,cp_knot);
    //cout<<cpdemo.cp_tracking_forward(zmp_control_point,0.3,cp_knot)<<endl;
    MatrixXd zmp_trajec=cpdemo.zmp_trajec(cp_traj,cp_knot);
    //cout<<zmp_trajec<<endl;
    //cout<< cp_knot(0,0)<<" "<<cp_knot(1,0)<<" "<<cp_knot(2,0)<<" "<<cp_knot(3,0)<<" "<<" "<<cp_knot(0,1)<<" "<<" "<<cp_knot(1,1)<<" "<<" "<<cp_knot(2,1);
    MatrixXd comtraj=comdemo.pos_v_traj(zmp_trajec);
    ///*
    ofs.open( "zmpref.txt", ios::out );
    ofs << zmp_trajec << endl;
    ofs.close();
    ofs.open( "cpknot.txt", ios::out );
    ofs << cp_knot << endl;
    ofs.close();
    ofs.open( "cptra.txt", ios::out );
    ofs << cp_traj << endl;
    ofs.close();
    //*/
    ofs.open( "com.txt", ios::out );
    ofs << comtraj << endl;
    ofs.close();
    return 0;
}
