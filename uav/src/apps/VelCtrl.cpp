#include "VelCtrl.h"
#include <GridLayout.h>
#include <PushButton.h>
#include <MetaDualShock3.h>
#include <FrameworkManager.h>

//namespaces, add others if necessary (filter, sensor, actuator)
using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::meta;
using namespace flair::sensor;

VelCtrl::VelCtrl(TargetController *controller): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default) {
    start_CustomTorques=new PushButton(GetButtonsLayout()->NewRow(),"start CustomTorques");
    stop_CustomTorques=new PushButton(GetButtonsLayout()->NewRow(),"stop CustomTorques");
}

VelCtrl::~VelCtrl() {
}

//this method is called by UavStateMachine::Run (main loop) when TorqueMode is Custom
void VelCtrl::ComputeCustomTorques(Euler &torques) {
    //compute the torques, with your own control laws

    //torques.roll=;
    //torques.pitch=;
    //torques.yaw=;
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
        Thread::Info("CustomTorques: stop\n");
        behaviourMode=BehaviourMode_t::Default;
        break;
    }
}

void VelCtrl::ExtraCheckPushButton(void) {
    if(start_CustomTorques->Clicked()) {
        StartCustomTorques();
    }

    if(stop_CustomTorques->Clicked() && (behaviourMode==BehaviourMode_t::CustomTorques)) {
        StopCustomTorques();
    }
}

void VelCtrl::ExtraCheckJoystick(void) {
    //R1
    if(GetTargetController()->ButtonClicked(9)) {
        StartCustomTorques();
    }

    //stop is not managed here, it is done in UavStateMachine with cross button
    //pushing cross button will enter fail safe mode and signal the EnteringFailSafeMode event
}

void VelCtrl::StartCustomTorques(void) {
  if( behaviourMode==BehaviourMode_t::CustomTorques) {
    Thread::Warn("MyApp: already in CustomTorques mode\n");
    return;
  }
    //ask UavStateMachine to enter in custom torques
    if (SetTorqueMode(TorqueMode_t::Custom)) {
        Thread::Info("CustomTorques: start\n");
    } else {
        Thread::Warn("CustomTorques: could not start\n");
        return;
    }

    behaviourMode=BehaviourMode_t::CustomTorques;
}

void VelCtrl::StopCustomTorques(void) {
    //just ask to enter fail safe mode
    EnterFailSafeMode();
}
