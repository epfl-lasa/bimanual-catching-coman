#include "MathLib/MathLib.h"
#include <stdio.h>
#include <stdlib.h>
#include "MathLib/IKGroupSolver.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"

// #define PI 3.14159265

enum ENUM_State{Com_Stop,Com_Break, Com_Safe};
enum ENUM_State_Orie{Not_Follow,Per_Follow};

using namespace MathLib;

class bimanual_ds
{
public:

    void 				initialize(double dt, double lamda1, double lamda2, double kappa1, double kappa2, double Gain_A1, double Gain_A2);
    void 				initialize_Virrtual_object(Vector l_L_d, Vector l_R_d);
    void				Set_object_state(Vector P_O,Vector DP_O, Vector DDP_O);
    void				Set_Left_robot_state(Vector P_R_L,Vector DP_R_L, Vector DDP_R_L);
    void				Set_Right_robot_state(Vector P_R_R,Vector DP_R_R, Vector DDP_R_R);
    void        		Set_TTC(double ttc);
    void				Get_Right_robot_state(Vector& P_R_R,Vector & DP_R_R, Vector & DDP_R_R);
    void				Get_Left_robot_state(Vector & P_R_L, Vector & DP_R_L, Vector & DDP_R_L);
    void                Get_virtual_object_state(Vector& P_V,Vector& DP_V,Vector& DDP_V, Vector& l_R_d, Vector& l_L_d);
    Vector              Get_virtual_object_pos();
    ENUM_State			What_is_the_state();
    void				Update();

private:
    void                initialize_Gains(Matrix & M, double Gain);
    void                initialize_Gains_2(Matrix & M, double Gain);

    double              _alpha;
    double              _beta;
    double              _lamda1;
    double              _lamda2;
    double              _dt;
    double              _gamma1;
    double              _gamma2;
    double              _kappa1;
    double              _kappa2;
    double              _ttc;
    double              _epsilon;

    Vector              _P_O;
    Vector              _DP_O;
    Vector              _DDP_O;

    Vector              _P_R_L;
    Vector             _DP_R_L;
    Vector             _DDP_R_L;

    Vector              _P_R_R;
    Vector             _DP_R_R;
    Vector             _DDP_R_R;

    Vector              _P_V;
    Vector             _DP_V;
    Vector             _DDP_V;

    Vector              _P_V_R;
    Vector             _DP_V_R;
    Vector             _DDP_V_R;

    Vector              _P_V_L;
    Vector             _DP_V_L;
    Vector             _DDP_V_L;

    Vector              _l_R_d;
    Vector              _l_L_d;

    Vector              _l_R;
    Vector              _Dl_R;
    Vector              _DDl_R;
    Vector              _l_L;
    Vector              _Dl_L;
    Vector              _DDl_L;

    Matrix              _Alpha;
    Matrix              _Beta;
    Matrix              _A1;
    Matrix              _A2;
    Matrix              _Gamma1;
    Matrix              _Gamma2;

    Matrix              _Data_Debug;
    ENUM_State			Command;
    ENUM_State_Orie		State_Orie;

};
