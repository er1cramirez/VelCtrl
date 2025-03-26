//  created:    2011/05/01
//  filename:   VelCtrl.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    demo cercle avec optitrack
//
//
/*********************************************************************/

#include "VelCtrl.h"
#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <Matrix.h>
#include <cmath>
#include <Tab.h>
#include <Pid.h>
// #include <PidThrust.h>
#include <Ahrs.h>
#include <AhrsData.h>


#include <LayoutPosition.h>
#include <Layout.h>
#include <TabWidget.h>
#include <Label.h>
#include <DoubleSpinBox.h>
#include <GroupBox.h>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;

VelCtrl::VelCtrl(TargetController *controller): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false), customThrustValue(0.0f) {
    Uav* uav=GetUav();

    VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80,uav->GetDefaultVrpnConnectionType());
    
    if(vrpnclient->ConnectionType()==VrpnClient::Xbee) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(),(uint8_t)0);
    } else if (vrpnclient->ConnectionType()==VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
    } else if (vrpnclient->ConnectionType()==VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
    }
    
    //set vrpn as failsafe altitude sensor for mamboedu as us in not working well for the moment
    if(uav->GetType()=="mamboedu") {
      SetFailSafeAltitudeSensor(uavVrpn->GetAltitudeSensor());
    }
    
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    vrpnclient->Start();
    
    uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),DataPlot::Green);
																 
    startCircle=new PushButton(GetButtonsLayout()->NewRow(),"start_custom");
    stopCircle=new PushButton(GetButtonsLayout()->LastRowLastCol(),"stop_custom");
    positionHold=new PushButton(GetButtonsLayout()->LastRowLastCol(),"position hold");

    // This is the default control law, it is used to control the UAV in the default mode
    uX=new Pid(setupLawTab->At(1,0),"u_x");
    uX->UseDefaultPlot(graphLawTab->NewRow());
    uY=new Pid(setupLawTab->At(1,1),"u_y");
    uY->UseDefaultPlot(graphLawTab->LastRowLastCol());

    customReferenceOrientation= new AhrsData(this,"reference");
    uav->GetAhrs()->AddPlot(customReferenceOrientation,DataPlot::Yellow);
    AddDataToControlLawLog(customReferenceOrientation);
    AddDeviceToControlLawLog(uX);
    AddDeviceToControlLawLog(uY);

    customOrientation=new AhrsData(this,"orientation");

    // Create tabs for UI organization
    Tab *lawTab = new Tab(getFrameworkManager()->GetTabWidget(), "custom_laws");
    TabWidget *tabWidget = new TabWidget(lawTab->NewRow(), "laws1");
    Tab *setupTab = new Tab(tabWidget, "Setup");
    Tab *graphTab = new Tab(tabWidget, "Graphes");
    Tab *refTab = new Tab(tabWidget, "references");

    GroupBox *vf_groupbox = new GroupBox(setupTab->NewRow(), "Vector Field");
    crSpinBox = new DoubleSpinBox(vf_groupbox->NewRow(), "cr", " ", 0, 3, 0.01, 3,0.12);
    ctSpinBox = new DoubleSpinBox(vf_groupbox->LastRowLastCol(), "ct", " ", 0, 3, 0.01, 3,0.11);
    c1SpinBox = new DoubleSpinBox(vf_groupbox->LastRowLastCol(), "c1", " ", 0, 3, 0.01, 2,1.15);

    GroupBox *ctrl_groupbox = new GroupBox(setupTab->NewRow(), "Control Law");
    thrustSpinBox = new DoubleSpinBox(ctrl_groupbox->At(0,0), "Thrust", " kgm/s2", -10, 10, 0.0001, 4,0.398);
    kp_xS = new DoubleSpinBox(ctrl_groupbox->NewRow(), "kp_x", " ", 0, 3, 0.01, 3,0.11);
    kp_yS = new DoubleSpinBox(ctrl_groupbox->LastRowLastCol(), "kp_y", " ", 0, 3, 0.01, 3,0.11);
    kp_zS = new DoubleSpinBox(ctrl_groupbox->LastRowLastCol(), "kp_z", " ", 0, 3, 0.01, 3,0.11);
    
    GroupBox *target_groupbox = new GroupBox(setupTab->NewRow(), "Target");
    xGoto = new DoubleSpinBox(target_groupbox->NewRow(), "x", " m", -10, 10, 0.1, 2, 2);
    yGoto = new DoubleSpinBox(target_groupbox->LastRowLastCol(), "y", " m", -10, 10, 0.1, 2, 2);
    zOffset = new DoubleSpinBox(target_groupbox->LastRowLastCol(), "z_offset", " m", -10, 10, 0.1, 2, 0.3);

    // Create output matrix for control signals (thrust, quaternion, angular velocity)
    MatrixDescriptor *desc = new MatrixDescriptor(8, 1);
    desc->SetElementName(0, 0, "f");           // Thrust magnitude
    desc->SetElementName(1, 0, "u_d_x");        // control output x
    desc->SetElementName(2, 0, "u_d_y");        // control output y
    desc->SetElementName(3, 0, "u_d_z");        // control output z
    output = new Matrix(this, desc, floatType, "custom_laws");
    delete desc;

    desc = new MatrixDescriptor(4, 1);
    desc->SetElementName(0, 0, "Vx");           // Inertial x velocity
    desc->SetElementName(1, 0, "Vref_x");       // Reference x velocity
    desc->SetElementName(2, 0, "Vy");           // Inertial y velocity
    desc->SetElementName(3, 0, "Vref_y");       // Reference y velocity
    velocityRef = new Matrix(this, desc, floatType, "velocity");
    delete desc;

    DataPlot1D *f_plot = new DataPlot1D(graphTab->NewRow(), "f", -2, 2);
    f_plot->AddCurve(output->Element(0, 0));

    DataPlot1D *u_d_x_plot = new DataPlot1D(graphTab->LastRowLastCol(), "u_d_x", -2, 2);
    u_d_x_plot->AddCurve(output->Element(1, 0));

    DataPlot1D *u_d_y_plot = new DataPlot1D(graphTab->LastRowLastCol(), "u_d_y", -2, 2);
    u_d_y_plot->AddCurve(output->Element(2, 0));

    DataPlot1D *u_d_z_plot = new DataPlot1D(graphTab->LastRowLastCol(), "u_d_z", -2, 2);
    u_d_z_plot->AddCurve(output->Element(3, 0));

    DataPlot1D *Vx_plot = new DataPlot1D(refTab->NewRow(), "Vx", -2, 2);
    Vx_plot->AddCurve(velocityRef->Element(0, 0),DataPlot::Blue);
    Vx_plot->AddCurve(velocityRef->Element(1, 0),DataPlot::Red);

    DataPlot1D *Vy_plot = new DataPlot1D(refTab->LastRowLastCol(), "Vy", -2, 2);
    Vy_plot->AddCurve(velocityRef->Element(2, 0),DataPlot::Blue);
    Vy_plot->AddCurve(velocityRef->Element(3, 0),DataPlot::Red);
    // Add data to log
    // Custom logs MatrixDescriptor 
    MatrixDescriptor *customLogsDescriptor = new MatrixDescriptor(9, 1); 
    customLogsDescriptor->SetElementName(0, 0, "Desired_vx"); 
    customLogsDescriptor->SetElementName(1, 0, "Desired_vy"); 
    customLogsDescriptor->SetElementName(2, 0, "Desired_vz");
    customLogsDescriptor->SetElementName(3, 0, "Vx");
    customLogsDescriptor->SetElementName(4, 0, "Vy");
    customLogsDescriptor->SetElementName(5, 0, "Vz");
    customLogsDescriptor->SetElementName(6, 0, "x");
    customLogsDescriptor->SetElementName(7, 0, "y");
    customLogsDescriptor->SetElementName(8, 0, "z");
    customLogs = new Matrix(this, customLogsDescriptor, floatType, "CustomLogs"); 
    delete customLogsDescriptor; 
    AddDataToControlLawLog(customLogs);
}

VelCtrl::~VelCtrl() {
}

//Default Functions
const AhrsData *VelCtrl::GetOrientation(void) const {
    //get yaw from vrpn
		Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);

    //get roll, pitch and w from imu
    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed;
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);
    Euler ahrsEuler=ahrsQuaternion.ToEuler();
    ahrsEuler.yaw=vrpnQuaternion.ToEuler().yaw;
    Quaternion mixQuaternion=ahrsEuler.ToQuaternion();
    customOrientation->SetQuaternionAndAngularRates(mixQuaternion,ahrsAngularSpeed);
    return customOrientation;
}

void VelCtrl::AltitudeValues(float &z,float &dz) const{
    Vector3Df uav_pos,uav_vel;
    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    //z and dz must be in uav's frame
    z=-uav_pos.z;
    dz=-uav_vel.z;
}
AhrsData *VelCtrl::GetReferenceOrientation(void) {
    if (behaviourMode==BehaviourMode_t::Default) {
        Vector2Df pos_err, vel_err; // in Uav coordinate system
        float yaw_ref;
        Euler refAngles;

        PositionValues(pos_err, vel_err, yaw_ref);

        refAngles.yaw=yaw_ref;

        uX->SetValues(pos_err.x, vel_err.x);
        uX->Update(GetTime());
        refAngles.pitch=uX->Output();

        uY->SetValues(pos_err.y, vel_err.y);
        uY->Update(GetTime());
        refAngles.roll=-uY->Output();

        customReferenceOrientation->SetQuaternionAndAngularRates(refAngles.ToQuaternion(),Vector3Df(0,0,0));
    } else if (behaviourMode==BehaviourMode_t::Custom) {//Custom flight mode
        Vector3Df uav_pos,uav_vel; // in VRPN coordinate system
        uavVrpn->GetPosition(uav_pos);
        uavVrpn->GetSpeed(uav_vel);
        uav_pos.z+=(float)zOffset->Value();//offset for safety

        // Custom law control signals
        Vector3Df u_d, u_dot_d;
        float psi_d = 0, psi_d_dot=0;
        Quaternion q_d;
        Vector3Df omega_d;
        // Get target position
        Vector3Df target_pos = {(float)xGoto->Value(), (float)yGoto->Value(), uav_pos.z};
  
        // Calculate HL control signals
        calculate_hlc(u_d, u_dot_d, target_pos, uav_pos, uav_vel, Vector3Df(0,0,0));
        // Map control to body frame quaternion and angular velocity
        calculate_virtual_control(q_d, omega_d, u_d, u_dot_d, psi_d, psi_d_dot);
        customReferenceOrientation->SetQuaternionAndAngularRates(q_d,omega_d);
    }
    return customReferenceOrientation;
}

void VelCtrl::PositionValues(Vector2Df &pos_error,Vector2Df &vel_error,float &yaw_ref) {
    Vector3Df uav_pos,uav_vel; // in VRPN coordinate system
    Vector2Df uav_2Dpos,uav_2Dvel; // in VRPN coordinate system

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);

    uav_pos.To2Dxy(uav_2Dpos);
    uav_vel.To2Dxy(uav_2Dvel);

    pos_error=uav_2Dpos-posHold;
    vel_error=uav_2Dvel;
    yaw_ref=yawHold;
    //error in uav frame
    Quaternion currentQuaternion=GetCurrentQuaternion();
    Euler currentAngles;//in vrpn frame
    currentQuaternion.ToEuler(currentAngles);
    pos_error.Rotate(-currentAngles.yaw);
    vel_error.Rotate(-currentAngles.yaw);
}


float VelCtrl::ComputeCustomThrust(void) {
    // Update output matrix with thrust and attitude commands
    output->GetMutex();
    output->SetValueNoMutex(0, 0, customThrustValue);         // Thrust magnitude
    output->ReleaseMutex();
    output->SetDataTime(GetTime());

    // ProcessUpdate(output);
    return customThrustValue;
}


void VelCtrl::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    switch(event) {
    case Event_t::TakingOff:
        behaviourMode=BehaviourMode_t::Default;
        vrpnLost=false;
        break;
    // case Event_t::EnteringControlLoop:
    //     if ((behaviourMode==BehaviourMode_t::Custom) && (!circle->IsRunning())) {
    //         VrpnPositionHold();
    //     }
    //     break;
    case Event_t::EnteringFailSafeMode:
        behaviourMode=BehaviourMode_t::Default;
        break;
    }
}

void VelCtrl::ExtraSecurityCheck(void) {
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::Custom) || (behaviourMode==BehaviourMode_t::PositionHold))) {
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
    }
}

void VelCtrl::ExtraCheckPushButton(void) {
    if(startCircle->Clicked()) {
        StartCircle();
    }
    if(stopCircle->Clicked()) {
        StopCircle();
    }
    if(positionHold->Clicked()) {
        VrpnPositionHold();
    }
}

void VelCtrl::ExtraCheckJoystick(void) {
    //R1 and Circle
    if(GetTargetController()->ButtonClicked(4) && GetTargetController()->IsButtonPressed(9)) {
        StartCircle();
    }

    //R1 and Cross
    if(GetTargetController()->ButtonClicked(5) && GetTargetController()->IsButtonPressed(9)) {
        StopCircle();
    }
    
    //R1 and Square
    if(GetTargetController()->ButtonClicked(2) && GetTargetController()->IsButtonPressed(9)) {
        VrpnPositionHold();
    }
}

void VelCtrl::StartCircle(void) {
    if( behaviourMode==BehaviourMode_t::Custom) {
        Thread::Warn("VelCtrl: already in custom mode\n");
        return;
    }
    if (!SetThrustMode(ThrustMode_t::Custom)) {
        Thread::Warn("could not start: failed to set thrust mode\n");
        return;
    }
    if (SetOrientationMode(OrientationMode_t::Custom)) {
        Thread::Info("VelCtrl: start custom mode\n");
    } else {
        Thread::Warn("VelCtrl: could not start custom mode\n");
        return;
    }
    behaviourMode=BehaviourMode_t::Custom;
}

void VelCtrl::StopCircle(void) {
    if( behaviourMode!=BehaviourMode_t::Custom) {
        Thread::Warn("VelCtrl: not in custom mode\n");
        return;
    }
    if (!SetThrustMode(ThrustMode_t::Default)) {
        Thread::Warn("could not stop: failed to set thrust mode\n");
        return;
    }
    Vector3Df vrpnPosition;
    uavVrpn->GetPosition(vrpnPosition);
    vrpnPosition.To2Dxy(posHold);
    behaviourMode=BehaviourMode_t::Default;
    Thread::Info("VelCtrl: finishing custom\n");
}

void VelCtrl::VrpnPositionHold(void) {
    if( behaviourMode==BehaviourMode_t::PositionHold) {
        Thread::Warn("VelCtrl: already in vrpn position hold mode\n");
        return;
    }
	Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);
	yawHold=vrpnQuaternion.ToEuler().yaw;

    Vector3Df vrpnPosition;
    uavVrpn->GetPosition(vrpnPosition);
    vrpnPosition.To2Dxy(posHold);

    uX->Reset();
    uY->Reset();
    behaviourMode=BehaviourMode_t::PositionHold;
    
    SetOrientationMode(OrientationMode_t::Custom);
    Thread::Info("VelCtrl: holding position\n");
}

void VelCtrl::calculate_virtual_control(Quaternion& q_d, Vector3Df& omega_d,
    const Vector3Df& ui, const Vector3Df& uip, float psi_d, float psip_d) 
{
    // Calculate normalized thrust direction and its derivative
    Vector3Df uu, uup;
    float norm = ui.GetNorm();  // This is the thrust magnitude
    float norm3 = norm * norm * norm;
    float u = ui.x * uip.x + ui.y * uip.y + ui.z * uip.z;

    uu = ui;
    uu.Normalize();  // Unit vector in thrust direction

    // Derivative of the unit thrust vector
    if (norm < 0.00001f) {
        uup = Vector3Df(0.0f, 0.0f, 0.0f);// it is not true need to check
    } else {
        uup.x = uip.x / norm - ui.x * u / norm3;
        uup.y = uip.y / norm - ui.y * u / norm3;
        uup.z = uip.z / norm - ui.z * u / norm3;
    }
    

    float u_3 = sqrtf(-2 * uu.z + 2);

    // Calculate desired quaternion based on thrust direction
    Quaternion refQuaternion;
    refQuaternion.q0 = u_3 * cosf(psi_d / 2) / 2;
    refQuaternion.q1 = (-uu.x * sinf(psi_d / 2) + uu.y * cosf(psi_d / 2)) / u_3;
    refQuaternion.q2 = (-uu.x * cosf(psi_d / 2) - uu.y * sinf(psi_d / 2)) / u_3;
    refQuaternion.q3 = sinf(psi_d / 2) * u_3 / 2;

    // Calculate desired angular velocity
    Vector3Df refOmega;
    refOmega.x = -uup.x * sinf(psi_d) + uup.y * cosf(psi_d) + uup.z * (uu.x * sinf(psi_d) - uu.y * cosf(psi_d)) / (1 - uu.z);
    refOmega.y = -uup.x * cosf(psi_d) - uup.y * sinf(psi_d) + uup.z * (uu.x * cosf(psi_d) + uu.y * sinf(psi_d)) / (1 - uu.z);
    refOmega.z = psip_d - (-uu.x * uup.y + uu.y * uup.x) / (1 - uu.z);

    customThrustValue = -norm;
    q_d = refQuaternion;
    omega_d = refOmega;
}

float VelCtrl::dot(const Vector3Df& v1, const Vector3Df& v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

void VelCtrl::calculate_hlc(Vector3Df& u, Vector3Df& u_dot,
    const Vector3Df& xi_c, const Vector3Df& xi, 
    const Vector3Df& xi_dot, const Vector3Df& xi_ddot
    )
{
    Vector3Df dv = xi_c - xi;
    Vector3Df dv_dot = - xi_dot;

    Vector3Df V = xi_dot;
    Vector3Df V_dot = xi_ddot;
    // Define constant vectors
    Vector3Df Tv(0.0f, 0.0f, 1.0f);
    Vector3Df T_dot(0.0f, 0.0f, 0.0f);
    // Distance and direction calculations
    float d = dv.GetNorm();
    Vector3Df R = dv;
    float d_dot;
    Vector3Df R_dot;
    if (d > 0.00001f) {
        R.Normalize();
        d_dot = dot(dv, dv_dot) / d;//can cause division by zero 
        R_dot = (dv_dot * d - dv * d_dot) / (d * d);
    } else {
        R = Vector3Df(0.0f, 0.0f, 0.0f);
        d_dot = 0.0f;
        R_dot = Vector3Df(0.0f, 0.0f, 0.0f);
    }
    
    
    // Membership functions
    float c1 = (float)c1SpinBox->Value();
    float mu_far = tanhf(c1 * d);
    float mu_close = 1.0f / coshf(c1 * d); // sech(x) = 1/cosh(x)

    // First derivatives of membership functions
    float sech_c1d = 1.0f / coshf(c1 * d);
    float mu_far_dot = c1 * sech_c1d * sech_c1d * d_dot;
    float mu_close_dot = -c1 * sech_c1d * tanhf(c1 * d) * d_dot;

    //Positive scalar value of current height(tangential distance)
    float d_t = -xi.z;
    float d_t_dot = -xi_dot.z;

    // Gain parameters
    float c2_k = (float)ctSpinBox->Value();
    float c2_T = c2_k * tanhf(c1 * d_t);
    float c2_T_dot = c2_k * (c1 * powf(1.0f / coshf(c1 * d_t), 2.0f) * d_t_dot);

    float c2_R = (float)crSpinBox->Value();
    float c2_R_dot = 0.0f;

    // Desired velocity vector - changed order of operations
    Vector3Df Vd = (R * (mu_far * c2_R) + Tv * (mu_close * c2_T));

    // First derivative of desired velocity - changed order of operations
    Vector3Df Vd_dot = (R * (mu_far * c2_R_dot) + R * (mu_far_dot * c2_R) + R_dot * (mu_far * c2_R)) + 
                      (Tv * (mu_close * c2_T_dot) + Tv * (mu_close_dot * c2_T) + T_dot * (mu_close * c2_T));

    // Control law
    // float kv = (float)kvSpinBox->Value();
    float kp_x = (float)kp_xS->Value();
    float kp_y = (float)kp_yS->Value();
    float kp_z = (float)kp_zS->Value();
    Vector3Df kp = Vector3Df(-kp_x, -kp_y, -kp_z);
    float mg = (float)thrustSpinBox->Value();//0.39f;

    // Calculate control outputs
    u = ((V - Vd) * kp) - Vector3Df(0.0f, 0.0f, mg);
    // u_dot = (V_dot - Vd_dot) * kp;
    u_dot = Vector3Df(0.0f, 0.0f, 0.0f);//acceleration is not known

    // Update output matrix with control signals
    output->GetMutex();
    output->SetValueNoMutex(1, 0, u.x);
    output->SetValueNoMutex(2, 0, u.y);
    output->SetValueNoMutex(3, 0, u.z);
    output->ReleaseMutex();
    output->SetDataTime(GetTime());

    // update velocity reference matrix
    velocityRef->GetMutex();
    velocityRef->SetValueNoMutex(0, 0, V.x);
    velocityRef->SetValueNoMutex(1, 0, Vd.x);
    velocityRef->SetValueNoMutex(2, 0, V.y);
    velocityRef->SetValueNoMutex(3, 0, Vd.y);
    velocityRef->ReleaseMutex();
    velocityRef->SetDataTime(GetTime());

    // Update custom logs matrix
    // Update custom logs 
    customLogs->GetMutex();
    customLogs->SetValue(0, 0, Vd.x);
    customLogs->SetValue(1, 0, Vd.y);
    customLogs->SetValue(2, 0, Vd.z);
    customLogs->SetValue(3, 0, V.x);
    customLogs->SetValue(4, 0, V.y);
    customLogs->SetValue(5, 0, V.z);
    customLogs->SetValue(6, 0, xi.x);
    customLogs->SetValue(7, 0, xi.y);
    customLogs->SetValue(8, 0, xi.z);
    customLogs->ReleaseMutex();
}
