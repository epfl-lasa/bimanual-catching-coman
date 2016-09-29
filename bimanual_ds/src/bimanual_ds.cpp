/*
 * Copyright (C) 2015 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Sina Mirrazavi
 * email:   sina.mirrazavi@epfl.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#include "bimanual_ds.h"


void bimanual_ds::initialize_Gains(Matrix & M, double Gain)
{
    M.Zero();
    M(0,0) = Gain;
    M(1,1) = Gain;
    M(2,2) = Gain;
}

void bimanual_ds::initialize_Gains_2(Matrix & M, double Gain)
{
    M.Zero();
    M(0,0) = Gain / 5.0; // /3.0; // / 10.0;
    M(1,1) = Gain;
    M(2,2) = Gain;
}


void bimanual_ds::Print_states()
{
    //	S_VO_.Print("S_VO_");
    //	DS_VO_.Print("DS_VO_");

    //    P_VO_.Print("P_VO_");

    //	S_O_.Print("S_O_");
    //	DS_O_.Print("DS_O_");

    //	U_l_.Print("U_l_");
    //	U_r_.Print("U_r_");

    //	cout<<Gamma_<<" Gamma_"<<endl;

}

void bimanual_ds::initialize(double dt, double lamda, double  beta, double kappa1, double kappa2, double Gain_A1, double Gain_A2)
{
    _beta = beta;
    _lamda = lamda;
    _dt = dt;
    _kappa1 = kappa1;
    _kappa2 = kappa2;
    _ttc = 5.0;
    _alpha = 0.0;
    _epsilon = 0.00001;

    _P_O.Resize(3);
    _DP_O.Resize(3);
    _DDP_O.Resize(3);


    _P_R_L.Resize(3);
    _DP_R_L.Resize(3);
    _DDP_R_L.Resize(3);

    _P_R_R.Resize(3);
    _DP_R_R.Resize(3);
    _DDP_R_R.Resize(3);


    _P_V.Resize(3);
    _DP_V.Resize(3);
    _DDP_V.Resize(3);


    _l_R_d.Resize(3);
    _l_L_d.Resize(3);

    _l_R_d_initial.Resize(3);
    _l_L_d_initial.Resize(3);


    _l_R.Resize(3);
    _Dl_R.Resize(3);
    _DDl_R.Resize(3);
    _l_L.Resize(3);
    _Dl_L.Resize(3);
    _DDl_L.Resize(3);


    _P_V_R.Resize(3);
    _DP_V_R.Resize(3);
    _DDP_V_R.Resize(3);

    _P_V_L.Resize(3);
    _DP_V_L.Resize(3);
    _DDP_V_L.Resize(3);

    _Alpha.Resize(3,3);
    _AlphaBeta.Resize(3,3);
    _A1.Resize(3,3);
    _A2.Resize(3,3);
    _Gamma1.Resize(3,3);
    _Gamma2.Resize(3,3);



    initialize_Gains(_A1,-Gain_A1);
    initialize_Gains(_A2,-Gain_A2);
    initialize_Gains(_Gamma1,-_gamma1);
    initialize_Gains(_Gamma2,-_gamma2);

    _l_L(0) = 0.2;  _l_L(1) = -0.5;  _l_L(2) = -0.1;
    _l_R(0) = 0.2;  _l_R(1) = 0.5;  _l_R(2) = -0.1;
}


void bimanual_ds::Set_object_state(Vector P_O,Vector DP_O, Vector DDP_O)
{
    _P_O = P_O;
    _DP_O = DP_O;
    _DDP_O = DDP_O;
}

void bimanual_ds::Set_Right_robot_state(Vector P_R_R,Vector DP_R_R, Vector DDP_R_R)
{
    _P_R_R = P_R_R;
    _DP_R_R = DP_R_R;
    _DDP_R_R = DDP_R_R;

//    cout<<"from ds, right end pos: "<<P_R_R(0)<<" "<<P_R_R(1)<<" "<<P_R_R(2)<<endl;
}

void bimanual_ds::Set_Left_robot_state(Vector P_R_L,Vector DP_R_L, Vector DDP_R_L)
{
    _P_R_L = P_R_L;
    _DP_R_L = DP_R_L;
    _DDP_R_L = DDP_R_L;

//    cout<<"from ds, left end pos:  "<<P_R_L(0)<<" "<<P_R_L(1)<<" "<<P_R_L(2)<<endl;
}


void bimanual_ds::Get_Left_robot_state(Vector& P_R_L,Vector& DP_R_L, Vector& DDP_R_L)
{
//    P_R_L = _P_V_L;
//    DP_R_L = _DP_V_L;
//    DDP_R_L = _DDP_V_L;

    P_R_L = _P_R_L;
    DP_R_L = _DP_R_L;
    DDP_R_L = _DDP_R_L;

}

void bimanual_ds::Get_Right_robot_state(Vector& P_R_R, Vector& DP_R_R, Vector& DDP_R_R)
{
//    P_R_R = _P_V_R;
//    DP_R_R = _DP_V_R;
//    DDP_R_R = _DDP_V_R;

    P_R_R = _P_R_R;
    DP_R_R = _DP_R_R;
    DDP_R_R = _DDP_R_R;
}


void bimanual_ds::Update()
{

//    _DDP_V = _Alpha*(_DP_V) + _AlphaBeta*(_P_V - _P_O);
    _DDP_V = _Alpha*(_DP_V - _DP_O) + _AlphaBeta*(_P_V - _P_O);

    _DP_V = _DP_V + _DDP_V*_dt;
    _P_V = _P_V + _DP_V*_dt;


    // //////////////////////
//    double theta;
//    theta = atan2(_P_V(1)+0.5, -_P_V(0));
//    cout<<theta *180/PI<<endl;

//    _l_L_d(0) = _l_L_d_initial(0) * sin(theta);
//    _l_L_d(1) = _l_L_d_initial(1) * cos(theta);

//    _l_R_d(0) = _l_R_d_initial(0) * sin(theta);
//    _l_R_d(1) = _l_R_d_initial(1) * cos(theta);

    // //////////////////////


    _DDl_R = _Gamma2*_Dl_R + _Gamma1*(_l_R - _l_R_d);
    _DDl_L = _Gamma2*_Dl_L + _Gamma1*(_l_L - _l_L_d);

    _Dl_R = _Dl_R + _DDl_R*_dt;
    _l_R = _l_R + _Dl_R*_dt;

    _Dl_L = _Dl_L + _DDl_L*_dt;
    _l_L = _l_L + _Dl_L*_dt;

    _DDP_V_R = _DDP_V + _DDl_R;
    _DP_V_R = _DP_V + _Dl_R;
    _P_V_R = _P_V + _l_R;

    _DDP_V_L = _DDP_V + _DDl_L;
    _DP_V_L = _DP_V + _Dl_L;
    _P_V_L = _P_V + _l_L;



    _DDP_R_R = _DDP_V_R + _A1*(_DP_R_R - _DP_V_R) + _A2*(_P_R_R - _P_V_R);
    _DDP_R_L = _DDP_V_L + _A1*(_DP_R_L - _DP_V_L) + _A2*(_P_R_L - _P_V_L);

    _DP_R_R = _DP_R_R + _DDP_R_R*_dt;
    _P_R_R = _P_R_R + _DP_R_R*_dt;

    _DP_R_L = _DP_R_L + _DDP_R_L*_dt;
    _P_R_L = _P_R_L + _DP_R_L*_dt;


//    cout<<"P_R_L : "<<_P_R_L(0)<<" "<<_P_R_L(1)<<" "<<_P_R_L(2)<<"       P_R_R : "<<_P_R_R(0)<<" "<<_P_R_R(1)<<" "<<_P_R_R(2)<<endl;
//    cout<<"vir pos L :"<<_P_V_L(0)<<" "<<_P_V_L(1)<<" "<<_P_V_L(2)<<"        "<<"vir pos R :"<<_P_V_R(0)<<" "<<_P_V_R(1)<<" "<<_P_V_R(2)<<" "<<endl;

//    cout<<"diff L : "<<_P_R_L(0)-_P_V_L(0)<<" "<<_P_R_L(1)-_P_V_L(1)<<" "<<_P_R_L(2)-_P_V_L(2)<<"       diff R : "<<_P_R_R(0)-_P_V_R(0)<<" "<<_P_R_R(1)-_P_V_R(1)<<" "<<_P_R_R(2)-_P_V_R(2)<<endl;

//    cout<<"length_L :"<<_l_L(0)<<" "<<_l_L(1)<<" "<<_l_L(2)<<"           "<<"length_R :"<<_l_R(0)<<" "<<_l_R(1)<<" "<<_l_R(2)<<" "<<endl;
//    cout<<"_l-_l_d :"<<_l_L(0)-_l_L_d(0)<<" "<<_l_L(1)-_l_L_d(1)<<" "<<_l_L(2)-_l_L_d(2)<<"      "<<_l_R(0)-_l_R_d(0)<<" "<<_l_R(1)-_l_R_d(1)<<" "<<_l_R(2)-_l_R_d(2)<<endl;
//    cout<<"_l_d :"<<_l_L_d(0)<<" "<<_l_L_d(1)<<" "<<_l_L_d(2)<<"      "<<_l_R_d(0)<<" "<<-_l_R_d(1)<<" "<<_l_R_d(2)<<endl;
//    cout<<"_l_L :"<<_l_L(0)<<" "<<_l_L(1)<<" "<<_l_L(2)<<"      "<<_l_R(0)<<" "<<-_l_R(1)<<" "<<_l_R(2)<<endl;

}

void bimanual_ds::initialize_Virrtual_object()
{
    _l_L_d(0)=0.2;  _l_L_d(1)=-0.2;  _l_L_d(2)=-0.1;
    _l_R_d(0)=0.2;  _l_R_d(1)=0.2;  _l_R_d(2)=-0.1;


//    _l_L_d_initial(0)=0.2;  _l_L_d_initial(1)=-0.2;  _l_L_d_initial(2)=-0.1;
//    _l_R_d_initial(0)=0.2;  _l_R_d_initial(1)=0.2;  _l_R_d_initial(2)=-0.1;
//    _l_L_d = _l_L_d_initial;
//    _l_R_d = _l_R_d_initial;

    _P_V = (_P_R_L+_P_R_R) * 0.5;

    _P_V(0) = _P_V(0) -0.3;
    _P_V(2) = _P_V(2) +0.2;

    _DP_V.Zero();
    _DDP_V.Zero();

//    cout<<_P_V<<endl;
}


//void bimanual_ds::initialize_Virrtual_object(Vector l_R_d,Vector l_L_d)
//{
//    _l_R_d = l_R_d;
//    _l_L_d = l_L_d;

//    _P_V = _P_R_L + _P_R_R;
//    _P_V = _P_V*0.5;

//    _P_V(0) = _P_V(0) -0.2;
//    _P_V(2) = _P_V(2) +0.2;

//    _DP_V.Zero();
//    _DDP_V.Zero();
//}


void bimanual_ds::Debug(Matrix & _Data_Debug)
{
    //	_Data_Debug.Print("_Data_Debug");
}



void  	bimanual_ds::Set_TTC(double ttc)
{
    _ttc = ttc;

    _alpha = _lamda/(_ttc + _epsilon);
//    _alpha = _lamda * abs(_P_O(0)) / (_ttc + _epsilon);
    _Alpha.Resize(3,3);

    initialize_Gains(_Alpha, -_alpha);
//    initialize_Gains(_AlphaBeta, -_beta*_alpha);
    initialize_Gains_2(_AlphaBeta, -_beta*_alpha);

    _gamma1 = _kappa1 / (_ttc + _epsilon);
    _gamma2 = _kappa2 / (_ttc + _epsilon);

    initialize_Gains(_Gamma1, -_gamma1);
    initialize_Gains(_Gamma2, -_gamma2);
}


void bimanual_ds::Get_virtual_object_state(Vector& P_V,Vector& DP_V,Vector& DDP_V, Vector& l_R_d, Vector& l_L_d)
{

    P_V = _P_V;
    DP_V = _DP_V;
    DDP_V = _DDP_V;
    l_R_d = _l_R_d;
    l_L_d = _l_L_d;
}

Vector bimanual_ds::Get_virtual_object_pos()
{
    return _P_V;
}

ENUM_State	bimanual_ds::What_is_the_state()
{
    return Command;
}
