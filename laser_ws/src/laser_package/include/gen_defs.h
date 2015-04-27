#ifndef GEN_DEFS
#define GEN_DEFS

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <math.h>
#include <cmath>
#include <string.h>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>

#define XI 0
#define XI_DOT 1
#define ETA 2
#define ETA_DOT 3
#define OMEGA 4

#define MU_W 0
#define SIGMA_W 1
#define VAR_W 2

#define MU_V_XI 3
#define SIGMA_V_XI 4
#define VAR_V_XI 5

#define MU_V_ETA 6
#define SIGMA_V_ETA 7
#define VAR_V_ETA 8

#define MU_V_OMEGA 9
#define SIGMA_V_OMEGA 10
#define VAR_V_OMEGA 11

#define PI 3.14159265358979
#define DEG_TO_RAD_DENOM 0.00555555555

#endif
