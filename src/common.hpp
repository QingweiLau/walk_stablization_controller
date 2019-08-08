#ifndef COMMON_HPP
#define COMMON_HPP



//stl headers
#include <vector>
#include <boost/shared_array.hpp>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>


//cnoid headers
#include <cnoid/SimpleController>
#include <cnoid/SimpleControllerItem>
#include <cnoid/Link>
#include <cnoid/Plugin>
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/JointPath>
#include <cnoid/EigenUtil>
#include <cnoid/BasicSensors>
#include <cnoid/BodyItem>




typedef std::vector< boost::shared_array< double > > DoubleArrayVector;
typedef std::vector< boost::shared_array< int > > IntArrayVector;

#endif
