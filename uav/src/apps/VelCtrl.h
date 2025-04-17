
#ifndef VELCTRL_H
#define VELCTRL_H

#include <UavStateMachine.h>

namespace framework {
    namespace gui {
        class PushButton;
    }
    namespace sensor {
        class TargetController;
    }
}

class VelCtrl : public flair::meta::UavStateMachine {
    public:
        VelCtrl(flair::sensor::TargetController *controller);
        ~VelCtrl();

    private:
        enum class BehaviourMode_t {
            Default,
            CustomTorques,
            CustomThrust,
            CustomCircle,
            CustomPositionHold,
            ThrustTune
        };

        BehaviourMode_t behaviourMode;
        /*Safety functions
        * These functions are called by the UavStateMachine::Run (main loop)
        * when the corresponding event is triggered
        */  
        void ExtraCheckPushButton(void) override;
        void ExtraCheckJoystick(void) override;
        void SignalEvent(Event_t event) override;


        /*
        * Custom functions
        */
        void ComputeCustomTorques(flair::core::Euler &torques) override;
        float ComputeCustomThrust(void) override;
        void StartCustomTorques(void);
        void StopCustomTorques(void);


        
        flair::gui::PushButton *start_CustomTorques,*stop_CustomTorques;
};

#endif // VELCTRL_H
