#ifndef _COMMON_HPP_H
#define _COMMON_HPP_H

#include <vector>
#include <boost/shared_array.hpp>
#include "CP_DYN.hpp"
#include "COM_ZMP_DYN.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using namespace Eigen;
#define name2str(name) (#name)

typedef std::vector< boost::shared_array< double > > DoubleArrayVector;
typedef std::vector< boost::shared_array< int > > IntArrayVector;

static constexpr double step_time=0.8;
static constexpr double pre_step=4.0;
static constexpr double frame_rate=1000;

const double zc=0.7;
const double g=9.8;
const double omega=sqrt(g/zc);


#endif
