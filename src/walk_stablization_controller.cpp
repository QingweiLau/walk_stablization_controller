/**

   @author Qingwei Liu and S.Kajita
*/
#ifndef _WALK_CTRL_H
#define _WALK_CTRL_H


#include <ros/node_handle.h>

#include "cp/cp_common.hpp"
#include "readTraject.hpp"
#include "KalmanFilter.cpp"
#include "MarkerController.h"

using namespace std;
using namespace cnoid;
using namespace Eigen;

static const int no_of_preview=3;

MatrixXd zmp_control_point(no_of_preview+1,2);
MatrixXd zmp_control_point2(no_of_preview+1,2);
MatrixXd zmp(no_of_preview+1,4);
MatrixXd CoMp(3,3);
MatrixXd CoMv(3,3);

CPDYN cpx, cpy,reference_cpx, reference_cpy;
COM_ZMP_DYN comdemo;

KalmanFilter comKal(4,2);

//matrices for COM speed estimation
MatrixXd A(4, 4); 
MatrixXd B(4, 1); 
MatrixXd C(2, 4);
VectorXd U(2); 
MatrixXd Q(4, 4); 
MatrixXd R(2, 4); 
VectorXd X0(4);   
MatrixXd P0(4, 4); 
Vector3 zmpvalue; 


const double COG_Z_OFFSET = 0.03;
const double LEG_Z_OFFSET = 0.113215;
const double LEG_X_OFFSET = -0.0159;
const double ZMP_X_OFFSET = 0.0176;


vector<double> cptrajx(40,0.0),cptrajy(40,0.0);

class walk_stablization_controller : public cnoid::SimpleController
{
  Body* ioBody;
  BodyPtr ikBody;
  SimpleControllerIO* io;

  
  //jointpath has been changed
  std::shared_ptr<JointPath> rAnkleToBase, lAnkleToBase, baseToRAnkle, baseToLAnkle, baseToLFoot, baseToRFoot;

  //JointPath *rAnkleToBase, *lAnkleToBase, *baseToRAnkle, *baseToLAnkle, *baseToLFoot, *baseToRFoot;
  Link *rAnkle, *lAnkle, *base,  *LLEG_HIP_R, *RLEG_HIP_R;
  Link *rFoot, *lFoot;

// instantiation of sensors
  ForceSensor *rftSensor, *lftSensor;
  AccelerationSensor* accelSensor;

//instantiation of marker
  //MarkerController* zmp_markercontrol;
  MarkerController* markercontrol;

  VectorXd qref, qold, qref_old;

// 
  double time;
  double timeStep;
  int    timeCounter;




  
  DoubleArrayVector walkPattern;
  Vector3d TPCC;   

  std::ofstream ofs;
  std::ofstream comofs;

  double pgain[35];
  double dgain[35];

  Vector3 zmp;

  ros::NodeHandle node;

public:

  Vector3 returnZMP(Vector3 &zmp){
    return zmp;
  };

  virtual bool initialize( SimpleControllerIO* io ) 
  {
    this->io = io;

    readWalkPattern( walkPattern);

    zmp_control_point << 0.3,-0.1, 0.6,0.1, 0.9,-0.1,1.2,0.1;

    io->setJointOutput( JOINT_TORQUE );
    io->setJointInput( JOINT_ANGLE );

    zmp<<0,0,0;

    ioBody = io->body();

    ikBody = ioBody->clone();


    qref.resize( ioBody->numJoints() );
    for ( int i = 0; i < ioBody->numJoints(); ++i )
      {
        double q = ioBody->joint( i )->q();
	      ikBody->joint( i )->q() = q;
	      qref[ i ] = q;
      }
    base = ikBody->rootLink();
    
    rFoot = ikBody->link( "RLEG_FOOT" );
    lFoot = ikBody->link( "LLEG_FOOT" );
    rAnkle=ikBody->link("RLEG_ANKLE_R");
    lAnkle=ikBody->link("LLEG_ANKLE_R");
    LLEG_HIP_R =ikBody->link("LLEG_HIP_R");
    RLEG_HIP_R =ikBody->link("RLEG_HIP_R");

    baseToRFoot = getCustomJointPath( ikBody, base, rFoot );
    baseToLFoot = getCustomJointPath( ikBody, base, lFoot );

    baseToRFoot->calcForwardKinematics();
    baseToLFoot->calcForwardKinematics();


    qold = qref;
    qref_old = qref;

    rftSensor = ioBody->findDevice< ForceSensor >( "RightSoleForceSensor" );
    lftSensor = ioBody->findDevice< ForceSensor >( "LeftSoleForceSensor" );
    accelSensor =  ioBody->findDevice< AccelerationSensor >("WaistAccelSensor");
    
    io->enableInput(rftSensor);
    io->enableInput(lftSensor);
    io->enableInput(accelSensor);
    time = 0.0;
    timeCounter = 0;
    timeStep = io->timeStep();
   

    CoMp=Eigen::MatrixXd::Zero(3, 3);
    CoMv=Eigen::MatrixXd::Zero(3, 3);

////////////////////////////////////////////////////////////////////////////////

//define A,B,H,Q,R and initial X0 and P0 for Kalman filter
///*
    A.block<2,2>(0,0)<< 1,timeStep,0,1;
    A.block<2,2>(2,2)<< 1,timeStep,0,1;
    A.block<2,2>(2,0)<<0,0,0,0;
    A.block<2,2>(0,2)<<0,0,0,0;
    B << timeStep*timeStep/2,timeStep,timeStep*timeStep/2,timeStep ;

    C << 1, 0, 0, 0,
        0, 0, 1, 0;
    
    Q.block<2,2>(0,0)<< 0.0001, 0, 0,0.0001;
    Q.block<2,2>(2,2)<< 0.0001, 0, 0,0.0001;
    Q.block<2,2>(2,0)<<0,0,0,0;
    Q.block<2,2>(0,2)<<0,0,0,0;
    R << 0.0001,0, 0,0,
         0,0.0001,0,0 ;
//control input is the x and y direction acceleration info from sensor
    U << accelSensor->dv()[0],accelSensor->dv()[1];
    X0 << 0, 0, 0,0;

    P0.block<2,2>(0,0)<< 1,0,0,1;
    P0.block<2,2>(2,2)<< 1,0,0,1;
    P0.block<2,2>(2,0)<<0,0,0,0;
    P0.block<2,2>(0,2)<<0,0,0,0;
//*/

////////////////////////////////////////////////////////////////////////////////

// kalman filter initialize
    comKal.initialize(A, B, C,  Q, R);
    comKal.initial_state(X0, P0); 



    
    // for Trunk Position Compliance Control
    TPCC[ 0 ] = 0.0;
    TPCC[ 1 ] = 0.0;
    TPCC[ 2 ] = 0.0;
    ofs.open( "log.txt", ios::out );
    comofs.open( "com.txt", ios::out );

    //define the PD coefficients for all joints
    for ( int i = 0;  i < 20; i++)  { pgain[i]=3000; }
    for ( int j = 20; j < 35; j++)  { pgain[j]=9000; }    
    for ( int i = 0;  i < 20; i++)  { dgain[i]=50;   }
    for ( int j = 20; j < 35; j++)  { dgain[j]=70;   }
    pgain[25]=0;
    pgain[32]=0;
    dgain[25]=0;
    dgain[32]=0;

    return true;
  } 

  void moveCog( const VectorXd &zmp)
  {
    const double xK1 = 0.6;
    const double xK2 = 7;
    const double yK1 = 1.2;
    const double yK2 = 2;

    Vector3d refZmp;

    refZmp[ 0 ] = walkPattern[ timeCounter ][ 9 ]+ZMP_X_OFFSET;
    refZmp[ 1 ] = walkPattern[ timeCounter ][ 10 ];
    refZmp[ 2 ] = 0;

    Vector3d err = zmp - refZmp;

    // Trunk Position Compliance Control
    double dx = xK1 * err[ 0 ] - xK2 * TPCC[ 0 ];
    double dy = yK1 * err[ 1 ] - yK2 * TPCC[ 1 ];
    TPCC[ 0 ] += dx * timeStep;
    TPCC[ 1 ] += dy * timeStep;

    double xg = walkPattern[ timeCounter ][ 0 ];
    double yg = walkPattern[ timeCounter ][ 1 ]-0.0122;
    double zg = walkPattern[ timeCounter ][ 2 ] + COG_Z_OFFSET;

    xg += TPCC[ 0 ];
    yg += TPCC[ 1 ];

    base->p() = Vector3( xg, yg, zg );
    ikBody->calcForwardKinematics();
    
  }

  void moveRLeg()
  {
    VectorXd p( 6 );

    bool isSuccess;
    double x = walkPattern[ timeCounter ][ 3 ] + LEG_X_OFFSET;
    double y = walkPattern[ timeCounter ][ 4 ];
    double z = walkPattern[ timeCounter ][ 5 ] + LEG_Z_OFFSET;

    p.head< 3 >() = Vector3( x, y, z );
    p.tail< 3 >() = rpyFromRot( rFoot->attitude() );
 
    isSuccess = baseToRFoot->calcInverseKinematics( Vector3( p.head< 3 >() ), rAnkle->calcRfromAttitude(rotFromRpy( Vector3( p.tail< 3 >() ) ) ) );

    if( isSuccess )
      {
        for( int i = 0; i < baseToRFoot->numJoints(); ++i )
        {
          Link *joint = baseToRFoot->joint( i );
          qref[ joint->jointId() ] = joint->q();
        }
      }
    baseToRFoot->calcForwardKinematics();
  }

  void moveLLeg()
  {
    VectorXd p( 6 );

    bool isSuccess;

    double x = walkPattern[ timeCounter ][ 6 ] + LEG_X_OFFSET;
    double y = walkPattern[ timeCounter ][ 7 ];
    double z = walkPattern[ timeCounter ][ 8 ] + LEG_Z_OFFSET;
    
    p.head< 3 >() = Vector3( x, y, z );
    p.tail< 3 >() = rpyFromRot( lFoot->attitude() );
    isSuccess = baseToLFoot->calcInverseKinematics( Vector3( p.head< 3 >() ), lFoot->calcRfromAttitude( rotFromRpy( Vector3( p.tail< 3 >() ) ) ) );

    if( isSuccess )
      {
        for( int i = 0; i < baseToLFoot->numJoints(); ++i )
        {
          Link *joint = baseToLFoot->joint( i );
          qref[ joint->jointId() ] = joint->q();
        }
      }
    baseToLFoot->calcForwardKinematics();
  }


int isContact( double a )
    {       
        bool ans = 0;
        if ( a < 150.0 )
        {
          ans = 1;
        }
        return ans;
    }


//readings from F/Tsensor or gyro 

void readingSensor(VectorXd &rFT_filtered,VectorXd &lFT_filtered){

  double rFx_filtered= rftSensor->F()[ 0 ];
  double rFy_filtered= rftSensor->F()[ 1 ];
  double rFz_filtered= rftSensor->F()[ 2 ];
  double rTx_filtered= rftSensor->F()[ 3 ];
  double rTy_filtered= rftSensor->F()[ 4 ];
  double rTz_filtered= rftSensor->F()[ 5 ];
  double lFx_filtered= lftSensor->F()[ 0 ];
  double lFy_filtered= lftSensor->F()[ 1 ];
  double lFz_filtered= lftSensor->F()[ 2 ];
  double lTx_filtered= lftSensor->F()[ 3 ];
  double lTy_filtered= lftSensor->F()[ 4 ];
  double lTz_filtered= lftSensor->F()[ 5 ];

  rFT_filtered<<rFx_filtered,rFy_filtered,rFz_filtered,rTx_filtered,rTy_filtered,rTz_filtered;

  lFT_filtered<<lFx_filtered,lFy_filtered,lFz_filtered,lTx_filtered,lTy_filtered,lTz_filtered;

}

void ZMPcal( VectorXd &zmp,VectorXd &rFT_filtered,VectorXd & lFT_filtered){
  
  VectorXd rP( 3 ), lP( 3 );
  const double zSens = 0.026;

    double rFx = rFT_filtered[ 0 ];
    double rFy = rFT_filtered[ 1 ];
    double rFz = rFT_filtered[ 2 ];
    double rTx = rFT_filtered[ 3 ];
    double rTy = rFT_filtered[ 4 ];
    double rTz = rFT_filtered[ 5 ];
    double lFx = lFT_filtered[ 0 ];
    double lFy = lFT_filtered[ 1 ];
    double lFz = lFT_filtered[ 2 ];
    double lTx = lFT_filtered[ 3 ];
    double lTy = lFT_filtered[ 4 ];
    double lTz = lFT_filtered[ 5 ];

    // ZMP calculation of each foot
    
    if ( isContact( rFz ) )
      {
        rP[ 0 ] = 0;
        rP[ 1 ] = 0;
        rP[ 2 ] = 0;
      }
    else
      {
        rP[ 0 ] = ( - rTy -zSens * rFx + rFoot->p()[ 0 ] * rFz ) / rFz;
        rP[ 1 ] = (   rTx -zSens * rFy + rFoot->p()[ 1 ] * rFz ) / rFz;
        rP[ 2 ] = 0;
      }

    if ( isContact( lFz ) )
      {
        lP[ 0 ] = 0;
        lP[ 1 ] = 0;
        lP[ 2 ] = 0;
      }
    else
      {
        lP[ 0 ] = ( - lTy -zSens * lFx + lFoot->p()[ 0 ] * lFz ) / lFz;
        lP[ 1 ] = (   lTx -zSens * lFy + lFoot->p()[ 1 ] * lFz ) / lFz;
        lP[ 2 ] = 0;
      }

    // ZMP calculation respect to the contact conditions
    if ( isContact( rFz ) & isContact( lFz ) )
      {
        zmp[ 0 ] = ( rFoot->p()[ 0 ] + lFoot->p()[ 0 ] ) / 2;
        zmp[ 1 ] = ( rFoot->p()[ 1 ] + lFoot->p()[ 1 ] ) / 2;
        zmp[ 2 ] = 0;
      }
    else if ( isContact( rFz ) )
      {
        zmp[ 0 ] = lP[ 0 ];
        zmp[ 1 ] = lP[ 1 ];
        zmp[ 2 ] = 0;
      }
    else if ( isContact( lFz ) )
      {
        zmp[ 0 ] = rP[ 0 ];
        zmp[ 1 ] = rP[ 1 ];
        zmp[ 2 ] = 0;
      }
    else
      {
        double rPx = rP[ 0 ];
        double rPy = rP[ 1 ];
        double lPx = lP[ 0 ];
        double lPy = lP[ 1 ];

        zmp[ 0 ] = ( rPx * rFz + lPx * lFz ) / ( rFz + lFz );
        zmp[ 1 ] = ( rPy * rFz + lPy * lFz ) / ( rFz + lFz );
        zmp[ 2 ] = 0;
      }
  }

  
  void log(VectorXd &zmp, VectorXd &rFT_filtered,VectorXd & lFT_filtered )
    {
        ofs << time << " ";
        //zmp info
        ofs << zmp[0] << " ";
        ofs << zmp[1] << " ";
        ofs << zmp[2] << " ";

        //reference walk pattern x and y
        ofs << walkPattern[ timeCounter ][ 9 ]+ZMP_X_OFFSET << " ";
        ofs << walkPattern[ timeCounter ][ 10 ]-0.0219<< " ";

        //real and reference com x,y and z
        ofs << ikBody->calcCenterOfMass()[0] << " ";
        ofs << ikBody->calcCenterOfMass()[1] << " ";
        ofs << ikBody->calcCenterOfMass()[2] << " ";

        ofs << walkPattern[ timeCounter ][ 0 ] << " ";
        ofs << walkPattern[ timeCounter ][ 1 ] << " ";
        ofs << walkPattern[ timeCounter ][ 2 ] << " ";

        //TPCC
        ofs << TPCC[0]<< " ";
        ofs << TPCC[1]<< " ";
        
        //14-19
        //base postion and acceleration
        ofs << base->p()[0] << " ";
        ofs << base->p()[1] << " ";
        ofs << base->p()[2] << " ";     
        
        ofs << accelSensor->dv()[0] << " ";
        ofs << accelSensor->dv()[1] << " ";
        ofs << accelSensor->dv()[2] << " ";

        //20-31
        //right foot force, postion and attitude
        ofs << rFT_filtered[0] << " ";
        ofs << rFT_filtered[1] << " ";
        ofs << rFT_filtered[2] << " ";
        ofs << rFT_filtered[3] << " ";
        ofs << rFT_filtered[4] << " ";
        ofs << rFT_filtered[5] << " "; 

        ofs << rFoot->p()[0]     << " ";
        ofs << rFoot->p()[1]     << " ";
        ofs << rFoot->p()[2]     << " ";

        ofs << rpyFromRot( rFoot->attitude()) [0] << " ";
        ofs << rpyFromRot( rFoot->attitude()) [1] << " ";
        ofs << rpyFromRot( rFoot->attitude()) [2] << " ";

        //32-43
        //left foot force, postion and attitude
        ofs << lFT_filtered[0] << " ";
        ofs << lFT_filtered[1] << " ";
        ofs << lFT_filtered[2] << " ";
        ofs << lFT_filtered[3] << " ";
        ofs << lFT_filtered[4] << " ";
        ofs << lFT_filtered[5] << " "; 

        ofs << lFoot->p()[0]     << " ";
        ofs << lFoot->p()[1]     << " ";
        ofs << lFoot->p()[2]     << " ";

        ofs << rpyFromRot( lFoot->attitude()) [0] << " ";
        ofs << rpyFromRot( lFoot->attitude()) [1] << " ";
        ofs << rpyFromRot( lFoot->attitude()) [2] <<  endl;
                

        //cout<< setw(12) << "ZMP Pos: " << setw(12) <<zmp[0]<< setw(12) <<zmp[1]<< setw(12) <<zmp[2]<<  endl;
        //cout<< setw(12) << "COM: " << setw(12) <<ikBody->calcCenterOfMass()[0]<< setw(12) <<ikBody->calcCenterOfMass()[1]<< setw(12) <<ikBody->calcCenterOfMass()[2]<<"\n"<<endl;

        //cout<< setw(12) << "Waist Pos: " << setw(12) <<base->p()[0]<< setw(12) <<base->p()[1]<< setw(12) <<base->p()[2]<< endl;

        //cout<< setw(12) <<  "RFoot F/T: " <<setw(12) << rftSensor->F()[0] <<setw(12) << rftSensor->F()[1]<<setw(12) << rftSensor->F()[2]<<"\n"<<setw(24) << rftSensor->F()[3]<<setw(12) << rftSensor->F()[4]<<setw(12) << rftSensor->F()[5] << endl;

        //cout<< setw(12) <<  "LFoot F/T: " <<setw(12) << lftSensor->F()[0]<<setw(12) << lftSensor->F()[1]<<setw(12) << lftSensor->F()[2]<<"\n"<<setw(24) << lftSensor->F()[3]<<setw(12) << lftSensor->F()[4]<<setw(12) << lftSensor->F()[5]<< endl;

        //cout<< setw(12) << "RFoot Pos: " << setw(12) <<rFoot->p()[0]<< setw(12) <<rFoot->p()[1]<< setw(12) <<rFoot->p()[2]<< endl;

        //cout<< setw(12) << "LFoot Pos: " << setw(12) <<lFoot->p()[0]<< setw(12) <<lFoot->p()[1]<< setw(12) <<lFoot->p()[2]<<"\n"<< endl;

        //cout<< setw(12) << "RFoot Atti: " << setw(8) << "R   "<<rpyFromRot( rFoot->attitude() )[0]<< setw(12) <<"P   "<<rpyFromRot( rFoot->attitude() )[1]<< setw(12)<<"Y   " <<rpyFromRot( rFoot->attitude() )[2]<< endl;
        //cout<< setw(12) << "LFoot Atti: " << setw(8) << "R   "<<rpyFromRot( lFoot->attitude() )[0]<< setw(12) <<"P   "<<rpyFromRot( lFoot->attitude() )[1]<< setw(12)<<"Y   " <<rpyFromRot( lFoot->attitude() )[2]<< endl;
        //cout<< setw(12) << "RAnkle Atti: " << setw(6) << "R   "<<rpyFromRot( rAnkle->attitude() )[0]<< setw(12) <<"P   "<<rpyFromRot( rAnkle->attitude() )[1]<< setw(12)<<"Y   " <<rpyFromRot( rAnkle->attitude() )[2]<< endl;
        //cout<< setw(12) << "LAnkle Atti: " << setw(6) << "R   "<<rpyFromRot( lAnkle->attitude() )[0]<< setw(12) <<"P   "<<rpyFromRot( lAnkle->attitude() )[1]<< setw(12)<<"Y   " <<rpyFromRot( lAnkle->attitude() )[2]<<"\n"<< endl;

        //cout<< setw(12) << "RAnkle Pos: " << setw(12) <<rAnkle->p()[0]<< setw(12) <<rAnkle->p()[1]<< setw(12) <<rAnkle->p()[2]<< endl;

        //cout<< setw(12) << "LAnkle Pos: " << setw(12) <<lAnkle->p()[0]<< setw(12) <<lAnkle->p()[1]<< setw(12) <<lAnkle->p()[2]<<"\n"<< endl;

}

virtual bool start() override
  {
    //zmp_markercontrol = MarkerController::get_instance();  // currently cannot deploy two markercontroller with two complied controller from the same markercontroller.cpp 
    markercontrol = MarkerController::get_instance();  
    return true;
  }

void estimation(){
      Vector3 current_com_p=ioBody->calcCenterOfMass();
      CoMp.row(0)=CoMp.row(1);
      CoMp.row(1)=CoMp.row(2);
      CoMp.row(2)<<current_com_p.transpose();
      
      //CoMv.row(0)=CoMv.row(1);
      //CoMv.row(1)=CoMv.row(2);
      CoMv.row(2)=(CoMp.row(2)-CoMp.row(1))/timeStep;
      comofs<<time<<" "<<CoMp(2,0)<<" "<<CoMv(2,0)<<" "<<CoMp(2,1)<<" "<<CoMv(2,1)<<" ";
      std::cout<<"    iobody com position"<<"  "<< current_com_p.transpose()[0]<<"  "<<      current_com_p.transpose()[1]<<"  "<<0<<std::endl;
    }

  Vector3 cpcal(){
    double cpxx,cpyy,reference_cpxx,reference_cpyy;
    cpxx=cpx.cp_real(CoMp(2,0),CoMv(2,0));
    cpyy=cpy.cp_real(CoMp(2,1),CoMv(2,1));
    reference_cpxx=reference_cpx.cp_real(walkPattern[ timeCounter ][ 0 ],walkPattern[ timeCounter ][ 11 ]);
    reference_cpyy=reference_cpy.cp_real(walkPattern[ timeCounter ][ 1 ],walkPattern[ timeCounter ][ 12 ]); 


      Vector3 cp_marker;
      cp_marker<<cpxx,cpyy,-0.05;

      //cout <<"  real position"<<"  "<<ikBody->calcCenterOfMass()[0]<<"  "<<ikBody->calcCenterOfMass()[1]<<"  "<<0.0<<endl;
    return cp_marker;

    }
          

  void update()
  {
    for ( int i = 0; i < ioBody->numJoints(); ++i )
      {
        double q = ioBody->joint( i )->q();
        double dq = ( q - qold[ i ] ) / timeStep;
        double dq_ref = ( qref[ i ] - qref_old[ i ] ) / timeStep;
        ioBody->joint(i)->u() = ( qref[i] - q ) * pgain[ i ] + ( dq_ref - dq ) * dgain[ i ];
        qold[ i ] = q;
      }
    qref_old = qref;
    time += timeStep;
    if( timeCounter < walkPattern.size()-1 ) ++timeCounter;
  }

    virtual bool control() override
    {

      bool isActive = true;
      start();
      update();
      VectorXd lf_filtered(6);
      VectorXd rf_filtered(6);
      VectorXd zmp(3);
      readingSensor(rf_filtered,lf_filtered);
      ZMPcal(zmp,rf_filtered,lf_filtered);
      moveCog(zmp);
      log(zmp,rf_filtered,lf_filtered);
      moveRLeg();
      moveLLeg();
      ioBody->rootLink()->setPosition(base->position());
      ioBody->calcForwardKinematics();
      estimation();
      Vector3 cp_marker=cpcal();
 
      std::cout<<" capture point position"<<"  "<<cp_marker(0)<<"  "<<cp_marker(1)<<"  "<<cp_marker(2)<<endl;
      std::cout<<"center of mass position"<<"  "<<ikBody->calcCenterOfMass()[0]<<"  "<<ikBody->calcCenterOfMass()[1]<<"  "<<0<<std::endl;

      std::cout<<"<<========================================================>>"<<std::endl;

      //cout<<"ioBody->rootLink()->position().translation()  "<<ioBody->rootLink()->position().translation()<<endl;
      //cout<<"ikBody->rootLink()->position().translation()  "<<ikBody->rootLink()->position().translation()<<endl;
      //cout<<"info  "<<ioBody->info()<<endl;

      if(markercontrol){
        markercontrol->setMarkerPosition(cp_marker);
      }

    return isActive;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(walk_stablization_controller)

#endif
