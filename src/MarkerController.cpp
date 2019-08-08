/**
   Marker controller
   @author S.Kajita
*/

#include "MarkerController.h"
#include <cnoid/EigenUtil>
#include <iostream>

using namespace std;
using namespace cnoid;

static MarkerController* marker_instance= nullptr;

MarkerController* MarkerController::get_instance()
{
  return ::marker_instance;
}

void MarkerController::your_name()
{
  //io->os() << "MarkerController" << endl;
}

MarkerController::MarkerController()
{
  ::marker_instance= this;
}

bool MarkerController::initialize(SimpleControllerIO* io)
{
  this->io = io;
  
  ioBody = io->body();
    
  ioBody->rootLink()->setActuationMode(Link::LINK_POSITION);
  io->enableInput(ioBody->rootLink(), Link::LINK_POSITION);
    
  //io->os() << "MarkerController: LINK_POSITION mode." << endl;
  io->enableOutput(ioBody->rootLink());

  //io->os() << "::marker_instance=" << ::marker_instance<< endl;
  return true;
}

bool MarkerController::control()
{
  return true;
}

void MarkerController::setMarkerPosition(Vector3& p)
{
  ioBody->rootLink()->setTranslation(p);
  //std::cout<<"marker position"<<"  "<<ioBody->rootLink()->position().translation()[0]<<"  "<<ioBody->rootLink()->position().translation()[1]<<"  "<<ioBody->rootLink()->position().translation()[2]<<std::endl;


}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(MarkerController)
