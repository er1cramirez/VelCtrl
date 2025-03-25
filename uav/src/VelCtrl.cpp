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
#include <TrajectoryGenerator2DCircle.h>
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
        targetVrpn=new MetaVrpnObject("target",1);
    } else if (vrpnclient->ConnectionType()==VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        targetVrpn=new MetaVrpnObject("target");
    } else if (vrpnclient->ConnectionType()==VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        targetVrpn=new MetaVrpnObject("target");
    }
    
    //set vrpn as failsafe altitude sensor for mamboedu as us in not working well for the moment
    if(uav->GetType()=="mamboedu") {
      SetFailSafeAltitudeSensor(uavVrpn->GetAltitudeSensor());
    }
    
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    getFrameworkManager()->AddDeviceToLog(targetVrpn);
    vrpnclient->Start();
    
    uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),DataPlot::Green);
																 
    startCircle=new PushButton(GetButtonsLayout()->NewRow(),"start_circle");
    stopCircle=new PushButton(GetButtonsLayout()->LastRowLastCol(),"stop_circle");
    positionHold=new PushButton(GetButtonsLayout()->LastRowLastCol(),"position hold");

    circle=new TrajectoryGenerator2DCircle(vrpnclient->GetLayout()->NewRow(),"circle");
    uavVrpn->xPlot()->AddCurve(circle->GetMatrix()->Element(0,0),DataPlot::Blue);
    uavVrpn->yPlot()->AddCurve(circle->GetMatrix()->Element(0,1),DataPlot::Blue);
    uavVrpn->VxPlot()->AddCurve(circle->GetMatrix()->Element(1,0),DataPlot::Blue);
    uavVrpn->VyPlot()->AddCurve(circle->GetMatrix()->Element(1,1),DataPlot::Blue);
    uavVrpn->XyPlot()->AddCurve(circle->GetMatrix()->Element(0,1),circle->GetMatrix()->Element(0,0),DataPlot::Blue,"circle");

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



    thrustSpinBox = new DoubleSpinBox(setupTab->NewRow(), "T", " kgm/s2", -10, 10, 0.0001, 4,0.398);
    kvSpinBox = new DoubleSpinBox(setupTab->LastRowLastCol(), "kv", " ", 0, 3, 0.01, 3,0.11);
    crSpinBox = new DoubleSpinBox(setupTab->LastRowLastCol(), "cr", " ", 0, 3, 0.01, 3,0.12);
    ctSpinBox = new DoubleSpinBox(setupTab->LastRowLastCol(), "ct", " ", 0, 3, 0.01, 3,0.11);
    c1SpinBox = new DoubleSpinBox(setupTab->LastRowLastCol(), "c1", " ", 0, 3, 0.01, 2,1.15);

    xGoto = new DoubleSpinBox(setupTab->NewRow(), "x", " m", -10, 10, 0.1, 2, 2);
    yGoto = new DoubleSpinBox(setupTab->LastRowLastCol(), "y", " m", -10, 10, 0.1, 2, 2);
    zOffset = new DoubleSpinBox(setupTab->LastRowLastCol(), "z_offset", " m", -10, 10, 0.1, 2, 0.3);

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
    Vector3Df u_d, u_d_dot;
    float yaw_ref;
    PositionValues(u_d, u_d_dot, yaw_ref);
    float psi_d = 0, psi_d_dot=0;
    Quaternion q_d, qd_aux;
    Vector3Df omega_d, omega_aux;
    float T;
    calculate_virtual_control(q_d, omega_d, u_d, u_d_dot, psi_d, psi_d_dot);
    customReferenceOrientation->SetQuaternionAndAngularRates(q_d,omega_d);
    // customReferenceOrientation->SetQuaternionAndAngularRates(refAngles.ToQuaternion(),Vector3Df(0,0,0));
    return customReferenceOrientation;
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


void VelCtrl::PositionValues(Vector3Df &u_d, Vector3Df &u_dot_d, float &yaw_ref) {
    Vector2Df pos_error, vel_error;
    Vector3Df uav_pos,uav_vel; // in VRPN coordinate system
    Vector2Df uav_2Dpos,uav_2Dvel; // in VRPN coordinate system

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);

    uav_pos.z+=(float)zOffset->Value();
    uav_pos.To2Dxy(uav_2Dpos);
    uav_vel.To2Dxy(uav_2Dvel);

    if (behaviourMode==BehaviourMode_t::PositionHold) {
        pos_error=uav_2Dpos-posHold;
        vel_error=uav_2Dvel;
        yaw_ref=yawHold;
    } else { //Circle
        Vector3Df target_pos = {(float)xGoto->Value(), (float)yGoto->Value(), uav_pos.z};
        Vector2Df target_2Dpos;


        calculate_hlc(u_d, u_dot_d, target_pos, uav_pos, uav_vel, Vector3Df(0,0,0));
        yaw_ref=atan2(target_pos.y-uav_pos.y,target_pos.x-uav_pos.x);
    }

    // //error in uav frame
    // Quaternion currentQuaternion=GetCurrentQuaternion();
    // Euler currentAngles;//in vrpn frame
    // currentQuaternion.ToEuler(currentAngles);
    // pos_error.Rotate(-currentAngles.yaw);
    // vel_error.Rotate(-currentAngles.yaw);
}

void VelCtrl::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    switch(event) {
    case Event_t::TakingOff:
        behaviourMode=BehaviourMode_t::Default;
        vrpnLost=false;
        break;
    case Event_t::EnteringControlLoop:
        if ((behaviourMode==BehaviourMode_t::Circle) && (!circle->IsRunning())) {
            VrpnPositionHold();
        }
        break;
    case Event_t::EnteringFailSafeMode:
        behaviourMode=BehaviourMode_t::Default;
        break;
    }
}

void VelCtrl::ExtraSecurityCheck(void) {
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::Circle) || (behaviourMode==BehaviourMode_t::PositionHold))) {
        if (!targetVrpn->IsTracked(500)) {
            Thread::Err("VRPN, target lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
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
    if( behaviourMode==BehaviourMode_t::Circle) {
        Thread::Warn("VelCtrl: already in circle mode\n");
        return;
    }
    if (!SetThrustMode(ThrustMode_t::Custom)) {
        Thread::Warn("could not StartTransportation error SetThrustMode(OrientationMode_t::Custom)\n");
        return;
    }
    if (SetOrientationMode(OrientationMode_t::Custom)) {
        Thread::Info("VelCtrl: start circle\n");
    } else {
        Thread::Warn("VelCtrl: could not start circle\n");
        return;
    }
    Vector3Df uav_pos,target_pos;
    Vector2Df uav_2Dpos,target_2Dpos;

    targetVrpn->GetPosition(target_pos);
    target_pos.To2Dxy(target_2Dpos);
    circle->SetCenter(target_2Dpos);

    uavVrpn->GetPosition(uav_pos);
    uav_pos.To2Dxy(uav_2Dpos);
    circle->StartTraj(uav_2Dpos);

    uX->Reset();
    uY->Reset();
    behaviourMode=BehaviourMode_t::Circle;
}

void VelCtrl::StopCircle(void) {
    if( behaviourMode!=BehaviourMode_t::Circle) {
        Thread::Warn("VelCtrl: not in circle mode\n");
        return;
    }
    circle->FinishTraj();
    //GetJoystick()->Rumble(0x70);
    Thread::Info("VelCtrl: finishing circle\n");
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
    SetThrustMode(ThrustMode_t::Default);//check if it is necessary
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
    uup.x = uip.x / norm - ui.x * u / norm3;
    uup.y = uip.y / norm - ui.y * u / norm3;
    uup.z = uip.z / norm - ui.z * u / norm3;

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

    // Distance and direction calculations
    float d = dv.GetNorm();
    Vector3Df R = dv;
    R.Normalize();
    float d_dot = dot(dv, dv_dot) / d; 
    Vector3Df R_dot = (dv_dot * d - dv * d_dot) / (d * d);
    // Define constant vectors
    Vector3Df Tv(0.0f, 0.0f, 1.0f);
    Vector3Df T_dot(0.0f, 0.0f, 0.0f);

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
    float kv = (float)kvSpinBox->Value();
    float mg = (float)thrustSpinBox->Value();//0.39f;

    // Calculate control outputs
    u = (V - Vd) * (-kv) - Vector3Df(0.0f, 0.0f, mg);
    // u_dot = (V_dot - Vd_dot) * (-kv);
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
