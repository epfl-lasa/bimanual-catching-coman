/*
  Copyright (c) 2013 Sina Mirrazavi,
  LASA Lab, EPFL, CH-1015 Lausanne, Switzerland,
  http://lasa.epfl.ch
  The program is free for non-commercial academic use.
  Please acknowledge the authors in any academic publications that have
  made use of this code or part of it.
  }
*/

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <time.h>
#include <string>
#include <fstream>
#include "ros/ros.h"
#include "MathLib.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <sstream>




using namespace std;
using namespace MathLib;



enum Command{Com_NONE,Com_INIT, Com_Throw};
enum ENUM_State{Com_Stop,Com_Break, Com_Safe};


Vector P_O;
Vector Shift_left_P_O;
Vector Shift_right_P_O;
Vector DP_O;
Vector DDP_O;
double dt;
