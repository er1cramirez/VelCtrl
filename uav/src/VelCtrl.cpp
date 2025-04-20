#include "VelCtrl.h"
// Flair includes
#include <TargetController.h>
#include <Uav.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <MetaDualShock3.h>
#include <AhrsData.h>
#include <Ahrs.h>
#include <Matrix.h>
#include <cmath>
#include <Pid.h>
// #include <PidThrust.h>

// GUI elements
#include <Label.h>
#include <Tab.h>
#include <TabWidget.h>
#include <GridLayout.h>
#include <LayoutPosition.h>
#include <Layout.h>
#include <PushButton.h>
#include <DoubleSpinBox.h>
#include <GroupBox.h>
#include <ComboBox.h>
#include <CheckBox.h>
#include <DoubleSpinBox.h>
#include <Vector3DSpinBox.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>

// #include "velocityField.h"
// #include "virtualCtrl.h"


//namespaces, add others if necessary (filter, sensor, actuator)
using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;


VelCtrl::VelCtrl(TargetController *controller): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default) {
    Uav* uav=GetUav();
    VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80,uav->GetDefaultVrpnConnectionType());
    
    if(vrpnclient->ConnectionType()==VrpnClient::Xbee) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(),(uint8_t)0);
    } else if (vrpnclient->ConnectionType()==VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
    } else if (vrpnclient->ConnectionType()==VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
    }
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    vrpnclient->Start();

    customReferenceOrientation= new AhrsData(this,"reference");
    GetUav()->GetAhrs()->AddPlot(customReferenceOrientation,DataPlot::Yellow);
    AddDataToControlLawLog(customReferenceOrientation);

    velocityField = new VelocityField();
    virtualCtrl = new VirtualCtrl();    

    SetupGUI();
    SetupData();
}

VelCtrl::~VelCtrl() {
}

//this method is called by UavStateMachine::Run (main loop) when OrientationMode is Custom
AhrsData *VelCtrl::GetReferenceOrientation(void) {

    Quaternion refQuaternion;
    Vector3Df refAngularRates;
    if (behaviourMode==BehaviourMode_t::CustomControl) {
        computeVelCtrl(refQuaternion, refAngularRates);
        // Thread::Info("Calculating HLC");
        customReferenceOrientation->SetQuaternionAndAngularRates(refQuaternion,Vector3Df(0.0f,0.0f,0.0f));
    }else if (behaviourMode==BehaviourMode_t::CustomCircle) {
        EnterFailSafeMode();
    }else {
        EnterFailSafeMode();
        // Default mode
    }
    return customReferenceOrientation;
}


void VelCtrl::GetCurrentUavState(Vector3Df &pos, Vector3Df &vel, Quaternion &quat, Vector3Df &angVel) {
    // Get position, velocity and quaternion from the VRPN object in its coordinate system
    Vector3Df uav_pos, uav_vel; 
    Quaternion vrpn_quaternion;
    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    uavVrpn->GetQuaternion(vrpn_quaternion);
    // Get current orientation and angular speed from the AHRS object (IMU)
    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();
    // Use yaw from VRPN and roll, pitch from IMU
    Euler ahrsEuler = currentQuaternion.ToEuler();
    ahrsEuler.yaw = vrpn_quaternion.ToEuler().yaw;
    pos = uav_pos;
    vel = uav_vel;
    quat = ahrsEuler.ToQuaternion();
    angVel = currentAngularSpeed;
}


/*
    * Custom Velocity control
*/
void VelCtrl::computeVelCtrl(Quaternion &refOrientation, Vector3Df &refOmega) {
    // Current state
    Vector3Df angVel;
    
    Quaternion currentOrientation;
    GetCurrentUavState(pos, vel, currentOrientation, angVel);
    // Print the position for debugging
    // Thread::Info("Position: %f %f %f\n", pos.x, pos.y, pos.z);
    // Calculate desired velocity based on the velocity field
    // Vector3Df desiredVelocity;
    Vector3Df targetPosition = desired_position->Value();
    // Thread::Info("Target position: %f %f %f\n", targetPosition.x, targetPosition.y, targetPosition.z);
    // Configure velocity field parameters from the UI
    velocityField->setParameters(
        crSpinBox->Value(),
        ctSpinBox->Value(),
        b_0SpinBox->Value(),
        k_bSpinBox->Value(),
        b_maxSpinBox->Value()
    );
    // Compute the desired velocity using the velocity field
    calculateDesiredVel(desiredVelocity, pos, targetPosition);
    // Thread::Info("Desired velocity: %f %f %f\n", desiredVelocity.x, desiredVelocity.y, desiredVelocity.z);

    /*
        * Simple control Law for the desired velocity
        * u = kp*(desiredVelocity - vel) + gOfsetS->Value()
        * u_dot = (0,0,0)
    */
    // Vector3Df u, u_dot;
    u.x = kp_xS->Value() * (desiredVelocity.x - vel.x);
    u.y = kp_yS->Value() * (desiredVelocity.y - vel.y);
    u.z = kp_zS->Value() * (desiredVelocity.z - vel.z) - fabs(gOfsetS->Value());

    // Thread::Info("Control output: %f %f %f\n", u.x, u.y, u.z);
    u.Saturate(2.0f);
    u_dot.x = 0.0f;
    u_dot.y = 0.0f;
    u_dot.z = 0.0f;
    // Compute the reference thrust, quaternion and angular velocity
    float refThrust;
    calculateVirtualCtrl(refOrientation, refOmega, refThrust, u, u_dot);
    UpdateData();
    // Thread::Info("Reference orientation: %f %f %f %f\n", refOrientation.q0, refOrientation.q1, refOrientation.q2, refOrientation.q3);
}
    

/*
    * This function allows to set a new reference altitude for the failsafe mode
    * It is called by the UavStateMachine::Run (main loop) when the failsafe mode is
    * z and dz must be in uav's frame
*/
void VelCtrl::AltitudeValues(float &z,float &dz) const{
    Vector3Df uav_pos,uav_vel;
    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    //z and dz must be in uav's frame
    z=-uav_pos.z;
    dz=-uav_vel.z;
}

void VelCtrl::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    switch(event) {
    case Event_t::TakingOff:
        //always take off in default mode
        behaviourMode=BehaviourMode_t::Default;
        break;
    case Event_t::EnteringFailSafeMode:
        //return to default mode
        Thread::Info("Custom control stop due to entering failsafe mode\n");
        behaviourMode=BehaviourMode_t::Default;
        break;
    }
}

void VelCtrl::ExtraCheckPushButton(void) {
    if(start_CustomControl->Clicked()) {
        StartCustomControl();
    }

    if(stop_CustomControl->Clicked() && (behaviourMode==BehaviourMode_t::CustomControl)) {
        StopCustomControl();
    }
}

void VelCtrl::ExtraCheckJoystick(void) {
    //R1
    if(GetTargetController()->ButtonClicked(9)) {
        StartCustomControl();
    }
}

void VelCtrl::StartCustomControl(void) {
  if( behaviourMode==BehaviourMode_t::CustomControl) {
    Thread::Warn("Already in Custom control mode\n");
    return;
  }
    
    //ask UavStateMachine to enter in custom orientation mode
    if (SetOrientationMode(OrientationMode_t::Custom)) {
        Thread::Info("Custom control start\n");
    } else {
        Thread::Warn("Custom control could not start\n");
        return;
    }

    behaviourMode=BehaviourMode_t::CustomControl;
}

void VelCtrl::StopCustomControl(void) {
    //just ask to enter fail safe mode
    EnterFailSafeMode();
}



void VelCtrl::SetupGUI(void) {
     /*
        * UI elements
    */
   start_CustomControl=new PushButton(GetButtonsLayout()->NewRow(),"start CustomControl");
   stop_CustomControl=new PushButton(GetButtonsLayout()->NewRow(),"stop CustomControl");
   // Custom tasks in the main UI
   GroupBox *task_selection_box = new GroupBox(GetButtonsLayout()->LastRowLastCol(), "Custom task");
   task_selection = new ComboBox(task_selection_box->NewRow(), "Custom task");
   task_selection->AddItem("Hovering at zero");
   task_selection->AddItem("Static target task");
   task_selection->AddItem("Circle target tracking");
   desired_position = new Vector3DSpinBox(task_selection_box->NewRow(), "Desired position", -5, 5, 0.1, 3);

   // Create tabs for UI organization
   Tab *lawTab = new Tab(getFrameworkManager()->GetTabWidget(), "custom_laws");
   TabWidget *tabWidget = new TabWidget(lawTab->NewRow(), "laws1");
   Tab *setupTab = new Tab(tabWidget, "Setup");
   Tab *controlEffortTab = new Tab(tabWidget, "Control effort");
   u_plot = new DataPlot1D(controlEffortTab->NewRow(), "Control output", -2, 2);
   u_dot_plot = new DataPlot1D(controlEffortTab->NewRow(), "Control derivative", -2, 2);

   Tab *performanceTab = new Tab(tabWidget, "Control performance");
   pos_err_plot = new DataPlot1D(performanceTab->NewRow(), "Position error", -2, 2);
   pos_track_plot = new DataPlot1D(performanceTab->NewRow(), "Position tracking", -2, 2);
   vel_err_plot = new DataPlot1D(performanceTab->NewRow(), "Velocity error", -2, 2);
   vel_track_plot = new DataPlot1D(performanceTab->NewRow(), "Velocity tracking", -2, 2);

   GroupBox *vf_groupbox = new GroupBox(setupTab->NewRow(), "Vector Field Parameters");
   crSpinBox = new DoubleSpinBox(vf_groupbox->NewRow(), "cr", " ", 0, 3, 0.01, 3,0.1);
   ctSpinBox = new DoubleSpinBox(vf_groupbox->LastRowLastCol(), "ct", " ", 0, 3, 0.01, 3,0.1);
   b_0SpinBox = new DoubleSpinBox(vf_groupbox->LastRowLastCol(), "b_0", " ", 0, 3, 0.01, 2,1.15);
   b_maxSpinBox = new DoubleSpinBox(vf_groupbox->LastRowLastCol(), "b_max", " ", 0, 6, 0.1, 2,3.0);
   k_bSpinBox = new DoubleSpinBox(vf_groupbox->LastRowLastCol(), "k_b", " ", 0, 3, 0.01, 2,0.15);

   GroupBox *ctrl_groupbox = new GroupBox(setupTab->NewRow(), "Control Law");
   gOfsetS = new DoubleSpinBox(ctrl_groupbox->At(0,0), "Thrust g ofset", " N", 0, 1, 0.001, 4,0.398);
   kp_xS = new DoubleSpinBox(ctrl_groupbox->NewRow(), "kp_x", " ", 0, 3, 0.01, 3,0.11);
   kp_yS = new DoubleSpinBox(ctrl_groupbox->LastRowLastCol(), "kp_y", " ", 0, 3, 0.01, 3,0.03);
   kp_zS = new DoubleSpinBox(ctrl_groupbox->LastRowLastCol(), "kp_z", " ", 0, 3, 0.01, 3,0.03);
}


void VelCtrl::SetupData(void) {
    // Create output matrix for control signals (thrust, quaternion, angular velocity)
    MatrixDescriptor *desc = new MatrixDescriptor(6, 1);
    desc->SetElementName(0, 0, "u_x");        // control output x
    desc->SetElementName(1, 0, "u_y");        // control output y
    desc->SetElementName(2, 0, "u_z");        // control output z
    desc->SetElementName(3, 0, "u_dot_x");    // control derivative x
    desc->SetElementName(4, 0, "u_dot_y");    // control derivative y
    desc->SetElementName(5, 0, "u_dot_z");    // control derivative z
    controlOutput = new Matrix(this, desc, floatType, "contol_output");
    AddDataToControlLawLog(controlOutput);
    delete desc;

    desc = new MatrixDescriptor(8, 1);
    desc->SetElementName(0, 0, "x_pos_err");        // Position error x
    desc->SetElementName(1, 0, "y_pos_err");        // Position error y
    desc->SetElementName(2, 0, "radial_err");        // Radial distance error
    desc->SetElementName(3, 0, "x_vel_err");        // Velocity error x
    desc->SetElementName(4, 0, "y_vel_err");        // Velocity error y
    desc->SetElementName(5, 0, "z_vel_err");        // Velocity error z
    errors = new Matrix(this, desc, floatType, "errors");
    AddDataToControlLawLog(errors);
    delete desc;

    desc = new MatrixDescriptor(12, 1);
    desc->SetElementName(0, 0, "x_ref");        // Reference x position
    desc->SetElementName(1, 0, "x");        // Inertial x position
    desc->SetElementName(2, 0, "y_ref");        // Reference y position
    desc->SetElementName(3, 0, "y");        // Inertial y position
    desc->SetElementName(4, 0, "z_ref");        // Reference z position
    desc->SetElementName(5, 0, "z");        // Inertial z position
    desc->SetElementName(6, 0, "Vx_ref");        // Reference x velocity
    desc->SetElementName(7, 0, "Vx");        // Inertial x velocity
    desc->SetElementName(8, 0, "Vy_ref");        // Reference y velocity
    desc->SetElementName(9, 0, "Vy");        // Inertial y velocity
    desc->SetElementName(10, 0, "Vz_ref");        // Reference z velocity
    desc->SetElementName(11, 0, "Vz");        // Inertial z velocity
    ref_tracking = new Matrix(this, desc, floatType, "tracking");
    AddDataToControlLawLog(ref_tracking);
    delete desc;


    // Control output plots
    u_plot->AddCurve(controlOutput->Element(0, 0), 0, 0, 255, "u_x");
    u_plot->AddCurve(controlOutput->Element(1, 0), 255, 0, 0, "u_y");
    u_plot->AddCurve(controlOutput->Element(2, 0), 0, 255, 0, "u_z");
    u_dot_plot->AddCurve(controlOutput->Element(3, 0), 0, 0, 255, "u_dot_x");
    u_dot_plot->AddCurve(controlOutput->Element(4, 0), 255, 0, 0, "u_dot_y");
    u_dot_plot->AddCurve(controlOutput->Element(5, 0), 0, 255, 0, "u_dot_z");
    // Position error plots
    pos_err_plot->AddCurve(errors->Element(0, 0), 0, 82, 204, "x_pos_err");
    pos_err_plot->AddCurve(errors->Element(1, 0), 51, 153, 255, "y_pos_err");
    pos_err_plot->AddCurve(errors->Element(2, 0), 204, 0, 0, "radial_err");
    vel_err_plot->AddCurve(errors->Element(3, 0), 0, 82, 204, "x_vel_err");
    vel_err_plot->AddCurve(errors->Element(4, 0), 51, 153, 255, "y_vel_err");
    vel_err_plot->AddCurve(errors->Element(5, 0), 204, 0, 0, "z_vel_err");
    // Position tracking plots
    pos_track_plot->AddCurve(ref_tracking->Element(0, 0), 0, 82, 204, "x_ref");
    pos_track_plot->AddCurve(ref_tracking->Element(1, 0), 51, 153, 255, "x");
    pos_track_plot->AddCurve(ref_tracking->Element(2, 0), 204, 0, 0, "y_ref");
    pos_track_plot->AddCurve(ref_tracking->Element(3, 0), 255, 102, 0, "y");
    pos_track_plot->AddCurve(ref_tracking->Element(4, 0), 0, 128, 0, "z_ref");
    pos_track_plot->AddCurve(ref_tracking->Element(5, 0), 76, 187, 23, "z");
    // Velocity tracking plots
    vel_track_plot->AddCurve(ref_tracking->Element(6, 0), 0, 82, 204, "Vx_ref");
    vel_track_plot->AddCurve(ref_tracking->Element(7, 0), 51, 153, 255, "Vx");
    vel_track_plot->AddCurve(ref_tracking->Element(8, 0), 204, 0, 0, "Vy_ref");
    vel_track_plot->AddCurve(ref_tracking->Element(9, 0), 255, 102, 0, "Vy");
    vel_track_plot->AddCurve(ref_tracking->Element(10, 0), 0, 128, 0, "Vz_ref");
    vel_track_plot->AddCurve(ref_tracking->Element(11, 0), 76, 187, 23, "Vz");
}

void VelCtrl::UpdateData(void) {
    // output->GetMutex();
    // output->SetValueNoMutex(1, 0, u.x);
    // output->SetValueNoMutex(2, 0, u.y);
    // output->SetValueNoMutex(3, 0, u.z);
    // output->ReleaseMutex();
    // Update the control output matrix
    controlOutput->GetMutex();
    controlOutput->SetValueNoMutex(0, 0, u.x);
    controlOutput->SetValueNoMutex(1, 0, u.y);
    controlOutput->SetValueNoMutex(2, 0, u.z);
    controlOutput->SetValueNoMutex(3, 0, u_dot.x);
    controlOutput->SetValueNoMutex(4, 0, u_dot.y);
    controlOutput->SetValueNoMutex(5, 0, u_dot.z);
    controlOutput->ReleaseMutex();
    controlOutput->SetDataTime(GetTime());
    // Update the current state matrix
    desiredPosition = desired_position->Value();
    ref_tracking->GetMutex();
    ref_tracking->SetValueNoMutex(0, 0, desiredPosition.x);
    ref_tracking->SetValueNoMutex(1, 0, pos.x);
    ref_tracking->SetValueNoMutex(2, 0, desiredPosition.y);
    ref_tracking->SetValueNoMutex(3, 0, pos.y);
    ref_tracking->SetValueNoMutex(4, 0, desiredPosition.z);
    ref_tracking->SetValueNoMutex(5, 0, pos.z);
    ref_tracking->SetValueNoMutex(6, 0, desiredVelocity.x);
    ref_tracking->SetValueNoMutex(7, 0, vel.x);
    ref_tracking->SetValueNoMutex(8, 0, desiredVelocity.y);
    ref_tracking->SetValueNoMutex(9, 0, vel.y);
    ref_tracking->SetValueNoMutex(10, 0, desiredVelocity.z);
    ref_tracking->SetValueNoMutex(11, 0, vel.z);
    ref_tracking->ReleaseMutex();
    ref_tracking->SetDataTime(GetTime());
    // Update the error matrix
    errors->GetMutex();
    errors->SetValueNoMutex(0, 0, pos.x - desiredPosition.x);
    errors->SetValueNoMutex(1, 0, pos.y - desiredPosition.y);
    errors->SetValueNoMutex(2, 0, sqrt(pow(pos.x - desiredPosition.x, 2) + pow(pos.y - desiredPosition.y, 2)));
    errors->SetValueNoMutex(3, 0, vel.x - desiredVelocity.x);
    errors->SetValueNoMutex(4, 0, vel.y - desiredVelocity.y);
    errors->SetValueNoMutex(5, 0, vel.z - desiredVelocity.z);
    errors->ReleaseMutex();
    errors->SetDataTime(GetTime());
}


void VelCtrl::calculateVirtualCtrl(Quaternion &refOrientation, Vector3Df &refOmega, float &thrust,
    const Vector3Df &ui, const Vector3Df &uip) {
    float psi_d = 0.0f;  // Desired yaw angle
    float psip_d = 0.0f;  // Desired yaw rate
    // Calculate normalized thrust direction and its derivative
    Vector3Df uu, uup;
    float norm = ui.GetNorm();  // This is the thrust magnitude
    if (norm < 4.6416e-04f) {
    norm = 4.6416e-04f;  // Avoid division by zero
    }
    float norm3 = norm * norm * norm;
    float u = ui.x * uip.x + ui.y * uip.y + ui.z * uip.z;
    uu = ui;
    uu.Normalize();  // Unit vector in thrust direction
    uup.x = uip.x / norm - ui.x * u / norm3;
    uup.y = uip.y / norm - ui.y * u / norm3;
    uup.z = uip.z / norm - ui.z * u / norm3;
    float u_3 = sqrtf(-2 * uu.z + 2);
    // Calculate desired quaternion based on thrust direction
    Quaternion rfQ;
    rfQ.q0 = u_3 * cosf(psi_d / 2) / 2;
    rfQ.q1 = (-uu.x * sinf(psi_d / 2) + uu.y * cosf(psi_d / 2)) / u_3;
    rfQ.q2 = (-uu.x * cosf(psi_d / 2) - uu.y * sinf(psi_d / 2)) / u_3;
    rfQ.q3 = sinf(psi_d / 2) * u_3 / 2;
    rfQ.Normalize();
    // Calculate desired angular velocity
    Vector3Df rfOm;
    rfOm.x = -uup.x * sinf(psi_d) + uup.y * cosf(psi_d) + uup.z * (uu.x * sinf(psi_d) - uu.y * cosf(psi_d)) / (1 - uu.z);
    rfOm.y = -uup.x * cosf(psi_d) - uup.y * sinf(psi_d) + uup.z * (uu.x * cosf(psi_d) + uu.y * sinf(psi_d)) / (1 - uu.z);
    rfOm.z = psip_d - (-uu.x * uup.y + uu.y * uup.x) / (1 - uu.z);
    thrust = -norm;
    refOrientation = rfQ;
    refOmega = rfOm;
}


// void VelCtrl::calculate_hlc(Vector3Df& u, Vector3Df& u_dot,
//     const Vector3Df& xi_c, const Vector3Df& xi, 
//     const Vector3Df& xi_dot, const Vector3Df& xi_ddot
//     )
// {
//     Vector3Df dv = xi_c - xi;
//     Vector3Df dv_dot = - xi_dot;

//     Vector3Df V = xi_dot;
//     Vector3Df V_dot = xi_ddot;
//     // Define constant vectors
//     Vector3Df Tv(0.0f, 0.0f, 1.0f);
//     Vector3Df T_dot(0.0f, 0.0f, 0.0f);
//     // Distance and direction calculations
//     float d = dv.GetNorm();
//     Vector3Df R = dv;
//     float d_dot;
//     Vector3Df R_dot;
//     if (d > 0.00001f) {
//         R.Normalize();
//         d_dot = dot(dv, dv_dot) / d;//can cause division by zero 
//         R_dot = (dv_dot * d - dv * d_dot) / (d * d);
//     } else {
//         R = Vector3Df(0.0f, 0.0f, 0.0f);
//         d_dot = 0.0f;
//         R_dot = Vector3Df(0.0f, 0.0f, 0.0f);
//     }
    
    
//     // Membership functions
//     float c1 = (float)c1SpinBox->Value();
//     float mu_far = tanhf(c1 * d);
//     float mu_close = 1.0f / coshf(c1 * d); // sech(x) = 1/cosh(x)

//     // First derivatives of membership functions
//     float sech_c1d = 1.0f / coshf(c1 * d);
//     float mu_far_dot = c1 * sech_c1d * sech_c1d * d_dot;
//     float mu_close_dot = -c1 * sech_c1d * tanhf(c1 * d) * d_dot;

//     //Positive scalar value of current height(tangential distance)
//     float d_t = -xi.z;
//     float d_t_dot = -xi_dot.z;

//     // Gain parameters
//     float c2_k = (float)ctSpinBox->Value();
//     float c2_T = c2_k * tanhf(c1 * d_t);
//     float c2_T_dot = c2_k * (c1 * powf(1.0f / coshf(c1 * d_t), 2.0f) * d_t_dot);

//     float c2_R = (float)crSpinBox->Value();
//     float c2_R_dot = 0.0f;

//     // Desired velocity vector - changed order of operations
//     Vector3Df Vd = (R * (mu_far * c2_R) + Tv * (mu_close * c2_T));

//     // First derivative of desired velocity - changed order of operations
//     Vector3Df Vd_dot = (R * (mu_far * c2_R_dot) + R * (mu_far_dot * c2_R) + R_dot * (mu_far * c2_R)) + 
//                       (Tv * (mu_close * c2_T_dot) + Tv * (mu_close_dot * c2_T) + T_dot * (mu_close * c2_T));

//     // Control law
//     // float kv = (float)kvSpinBox->Value();
//     float kp_x = (float)kp_xS->Value();
//     float kp_y = (float)kp_yS->Value();
//     float kp_z = (float)kp_zS->Value();
//     Vector3Df kp = Vector3Df(-kp_x, -kp_y, -kp_z);
//     float mg = (float)thrustSpinBox->Value();//0.39f;

//     // Calculate control outputs
//     u = ((V - Vd) * kp) - Vector3Df(0.0f, 0.0f, mg);
//     // u_dot = (V_dot - Vd_dot) * kp;
//     u_dot = Vector3Df(0.0f, 0.0f, 0.0f);//acceleration is not known

// }

void VelCtrl::calculateDesiredVel(Vector3Df &desiredVelocity, const Vector3Df &currentPosition, const Vector3Df &targetPosition) {
    float k_r = crSpinBox->Value();
    float k_t = ctSpinBox->Value();
    float b_0 = b_0SpinBox->Value();
    float k_b = k_bSpinBox->Value();
    float b_max = b_maxSpinBox->Value();
    // Compute the position error
    Vector3Df positionError = targetPosition - currentPosition;
    // Get the 2D distance
    Vector3Df radialDistance = positionError;
    radialDistance.z = 0.0f; // Ignore the z component for 2D distance
    // Compute the radial distance
    float distance = radialDistance.GetNorm();
    // Get the radial component as a unit vector of the radial distance vector
    Vector3Df _R = radialDistance;
    if (distance < 1e-12f) {
        _R = Vector3Df(0.0f, 0.0f, 0.0f); // Avoid division by zero
    } else {
        _R.Normalize();
    }
    // Thread::Info("R: %f %f %f\n", _R.x, _R.y, _R.z);
    // Define a tangential vector as unit vector in the z direction
    Vector3Df _T(0.0f, 0.0f, 1.0f);

    // Compute height-dependent b parameter
    float height = fabs(currentPosition.z);// Get the absolute height
    // Add the desired height as an offset
    height += fabs(targetPosition.z);
    float b = b_0;//(b_0 + b_max * expf(-k_b * height));

    // Compute membership functions

    float mu_far = tanhf(b * distance);
    float mu_close = 1.0f / coshf(b * distance); // sech(x) = 1/cosh(x)
    Thread::Info("mu_far: %f mu_close: %f\n", mu_far, mu_close);
    // Compute the desired velocity using the velocity field
    desiredVelocity = k_r * (mu_far * _R) + k_t * (mu_close * _T);
}
