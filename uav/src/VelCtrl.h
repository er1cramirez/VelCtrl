
#ifndef VELCTRL_H
#define VELCTRL_H

#include <UavStateMachine.h>
#include "velocityField.h"
#include "virtualCtrl.h"

namespace flair {
    namespace gui {
        class PushButton;
        class GroupBox;
        class ComboBox;
        class CheckBox;
        class Vector3DSpinBox;
        class DoubleSpinBox;
        class DataPlot1D;
    }
    namespace sensor {
        class TargetController;
    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace core {
        // Vector3Df;
        // Quaternion;
        class Matrix;
    }
}

class VelCtrl : public flair::meta::UavStateMachine {
    public:
        VelCtrl(flair::sensor::TargetController *controller);
        ~VelCtrl();

    private:
        enum class BehaviourMode_t {
            Default,
            CustomControl,
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
        * Custom control at orientation level
        */
        flair::core::AhrsData *GetReferenceOrientation(void) override;
        void AltitudeValues(float &z,float &dz) const override;// For avoid z up when fail safe
        void VrpnPositionHold(void);
        void GetCurrentUavState(flair::core::Vector3Df & pos, flair::core::Vector3Df &vel, flair::core::Quaternion &quat, flair::core::Vector3Df &angVel);
        void computeVelCtrl(flair::core::Quaternion &refOrientation,
                            flair::core::Vector3Df &refAngularRates);
        
        void StartCustomControl(void);
        void StopCustomControl(void);
        void SetupGUI(void);
        void SetupData(void);
        void UpdateData(void);

        VelocityField *velocityField;
        VirtualCtrl *virtualCtrl;

        flair::meta::MetaVrpnObject *uavVrpn;
        flair::core::AhrsData *customReferenceOrientation;
        flair::core::Matrix *customLogs;
        flair::core::Matrix *controlOutput;
        flair::core::Matrix *errors;
        flair::core::Matrix *ref_tracking;

        // Control performance plots
        flair::gui::DataPlot1D *u_plot, *u_dot_plot;
        flair::gui::DataPlot1D *pos_err_plot, *vel_err_plot, *pos_track_plot, *vel_track_plot; 

        // UI elements
        flair::gui::PushButton *start_CustomControl,*stop_CustomControl;
        flair::gui::ComboBox *task_selection;
        flair::gui::Vector3DSpinBox *desired_position;
        flair::gui::DoubleSpinBox *crSpinBox, *ctSpinBox, *b_0SpinBox, *b_maxSpinBox, *k_bSpinBox, *gOfsetS, *kp_xS, *kp_yS, *kp_zS;

        // State variables
        flair::core::Vector3Df desiredVelocity;
        flair::core::Vector3Df desiredPosition;
        flair::core::Vector3Df pos, vel;
        flair::core::Quaternion currentQuaternion;
        flair::core::Vector3Df currentAngularRates;
        flair::core::Vector3Df refOrientation;
        flair::core::Vector3Df refOmega;
        flair::core::Vector3Df u;
        flair::core::Vector3Df u_dot;
};

#endif // VELCTRL_H
