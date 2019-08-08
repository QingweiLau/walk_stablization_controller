/**
   Marker control
   @author S.Kajita
*/

#include <cnoid/SimpleController>
#include <cnoid/Config>
#include <cnoid/Link>
#include <cnoid/BodyMotion>

namespace cnoid {

class CNOID_GENERAL_EXPORT MarkerController : public SimpleController
{
  SimpleControllerIO* io;
  BodyPtr ioBody;
  double Dtime;
  
public:
  static MarkerController* get_instance();
  static void your_name();

  MarkerController();
  virtual bool initialize(SimpleControllerIO* io) override;
  virtual bool control() override;
  void setMarkerPosition(Vector3& p);      
};

}
