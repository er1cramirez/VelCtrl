#include "customPosHold.h"
#include <Matrix.h>
#include <Vector3D.h>
#include <TabWidget.h>
#include <CheckBox.h>
#include <Quaternion.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <cmath>
#include <Euler.h>
#include <iostream>
#include <Label.h>
#include <Vector3DSpinBox.h>
#include <Pid.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

CustomPosHold::CustomPosHold(const LayoutPosition *position, const string &name) : ControlLaw(position->getLayout(),name,4)
{
    first_update = true;

    // Input matrix
    input = new Matrix(this, 4, 5, floatType, name);

    // Matrix descriptor for logging. It should be always a nx1 matrix. 
    MatrixDescriptor *log_labels = new MatrixDescriptor(3, 1);
    log_labels->SetElementName(0, 0, "x_error");
    log_labels->SetElementName(1, 0, "y_error");
    log_labels->SetElementName(2, 0, "yaw_error");
    state = new Matrix(this, log_labels, floatType, name);
    delete log_labels;

    // GUI for custom PID
    GroupBox *gui_customPID = new GroupBox(position, name);
    GroupBox *general_parameters = new GroupBox(gui_customPID->NewRow(), "General parameters");
    dt = new DoubleSpinBox(general_parameters->NewRow(), "Custom dt [s]", 0, 1, 0.001, 4);
    gOfset = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Th g ofset [N]", 0, 10, 0.01, 3);
    sat_pos = new DoubleSpinBox(general_parameters->NewRow(), "Saturation pos", 0, 10, 0.01, 3);
    sat_thrust = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Saturation thrust", 0, 10, 0.01, 3);

    // Custom cartesian position controller
    GroupBox *custom_position = new GroupBox(gui_customPID->NewRow(), "Custom position controller");
    Kp_pos = new Vector3DSpinBox(custom_position->NewRow(), "Kp_pos", 0, 100, 0.1, 3);
    Kd_pos = new Vector3DSpinBox(custom_position->LastRowLastCol(), "Kd_pos", 0, 100, 0.1, 3);
    Ki_pos = new Vector3DSpinBox(custom_position->LastRowLastCol(), "Ki_pos", 0, 100, 0.1, 3);
    AddDataToLog(state);
}

CustomPosHold::~CustomPosHold()
{
    delete state;
}

void CustomPosHold::UpdateFrom(const io_data *data)
{
    float current_time = double(GetTime())/1000000000-initial_time;
    float thrust = 0.0;
    Vector3Df u, tau;

    if(dt->Value() == 0)
    {
        delta_t = (float)(data->DataDeltaTime())/1000000000;
    }
    else
    {
        delta_t = dt->Value();
    }
   
    if(first_update)
    {
        initial_time = double(GetTime())/1000000000;
        first_update = false;
    }

    // Obtain state
    input->GetMutex();
    Vector3Df pos_error(input->Value(0, 0), input->Value(1, 0), input->Value(2, 0));
    Vector3Df vel_error(input->Value(0, 1), input->Value(1, 1), input->Value(2, 1));
    input->ReleaseMutex();

    // Get tunning parameters from GUI
    Vector3Df Kp_pos_val(Kp_pos->Value().x, Kp_pos->Value().y, Kp_pos->Value().z);
    Vector3Df Kd_pos_val(Kd_pos->Value().x, Kd_pos->Value().y, Kd_pos->Value().z);
    Vector3Df Ki_pos_val(Ki_pos->Value().x, Ki_pos->Value().y, Ki_pos->Value().z);

    // Cartesian custom controller
    u.x = Kp_pos_val.x*pos_error.x + Kd_pos_val.x*vel_error.x;
    u.y = Kp_pos_val.y*pos_error.y + Kd_pos_val.y*vel_error.y;
    u.z = Kp_pos_val.z*pos_error.z + Kd_pos_val.z*vel_error.z;
    u.Saturate(sat_pos->Value());
    // Compute custom thrust
    thrust = - u.z - gOfset->Value();
    thrust = thrust > sat_thrust->Value() ? sat_thrust->Value() : thrust;
    thrust = thrust < 0 ? 0 : thrust;   

    // Send controller output
    output->SetValue(0, 0, tau.x);
    output->SetValue(1, 0, tau.y);
    output->SetValue(2, 0, tau.z);
    output->SetValue(3, 0, thrust);
    output->SetDataTime(data->DataTime());

    // Log state (example). 
    // Modify the log_labels matrix in the constructor to add more variables.
    state->GetMutex();
    state->SetValue(0, 0, pos_error.x);
    state->SetValue(1, 0, pos_error.y);
    state->ReleaseMutex();

    ProcessUpdate(output);
}

void CustomPosHold::Reset(void)
{
    first_update = true;
}

void CustomPosHold::SetValues(Vector3Df pos_error, Vector3Df vel_error)
{
    // Set the input values for the controller. 
    // This function is called from the main controller to set the input values.
    input->GetMutex();
    input->SetValue(0, 0, pos_error.x);
    input->SetValue(1, 0, pos_error.y);
    input->SetValue(2, 0, pos_error.z);

    input->SetValue(0, 1, vel_error.x);
    input->SetValue(1, 1, vel_error.y);
    input->SetValue(2, 1, vel_error.z);
    input->ReleaseMutex();
}

