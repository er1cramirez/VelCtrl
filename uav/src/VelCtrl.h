//  created:    2011/05/01
//  filename:   VelCtrl.h
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

#ifndef CIRCLEFOLLOWER_H
#define CIRCLEFOLLOWER_H

#include <UavStateMachine.h>

namespace flair {
    namespace gui {
        class PushButton;
        class DoubleSpinBox;
        class Tab;
        class TabWidget;
        class GroupBox;
    }
    namespace filter {
        class TrajectoryGenerator2DCircle;
    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace sensor {
        class TargetController;
    }
    namespace core {
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
            PositionHold,
            Custom,
        };

        BehaviourMode_t behaviourMode;
        bool vrpnLost;

        void VrpnPositionHold(void);//flight mode
        void StartCircle(void);
        void StopCircle(void);
        void ExtraSecurityCheck(void) override;
        void ExtraCheckPushButton(void) override;
        void ExtraCheckJoystick(void) override;
        const flair::core::AhrsData *GetOrientation(void) const override;
        float ComputeCustomThrust(void) override;
        void AltitudeValues(float &z,float &dz) const override;
        void calculate_virtual_control(flair::core::Quaternion& q_d, flair::core::Vector3Df& omega_d,
            const flair::core::Vector3Df& ui, const flair::core::Vector3Df& uip, float psi_d, float psip_d);
        void calculate_hlc(flair::core::Vector3Df& u, flair::core::Vector3Df& u_dot,
                const flair::core::Vector3Df& xi_c, const flair::core::Vector3Df& xi, 
                const flair::core::Vector3Df& xi_dot, const flair::core::Vector3Df& xi_ddot);
        void PositionValues(flair::core::Vector2Df &pos_error,flair::core::Vector2Df &vel_error,float &yaw_ref);
        float dot(const flair::core::Vector3Df& v1, const flair::core::Vector3Df& v2);
        flair::core::AhrsData *GetReferenceOrientation(void) override;
        void SignalEvent(Event_t event) override;

        flair::filter::Pid *uX, *uY;

        flair::core::Vector2Df posHold;
        flair::core::Matrix *output;
        flair::core::Matrix *customLogs;
        flair::core::Matrix *velocityRef;
        flair::gui::DoubleSpinBox *thrustSpinBox;
        flair::gui::DoubleSpinBox *kp_xS;
        flair::gui::DoubleSpinBox *kp_yS;
        flair::gui::DoubleSpinBox *kp_zS;
        flair::gui::DoubleSpinBox *crSpinBox;
        flair::gui::DoubleSpinBox *ctSpinBox;
        flair::gui::DoubleSpinBox *c1SpinBox;
        flair::gui::DoubleSpinBox *xGoto;
        flair::gui::DoubleSpinBox *yGoto;
        flair::gui::DoubleSpinBox *zOffset;
        float yawHold;

        flair::gui::PushButton *startCircle,*stopCircle,*positionHold;
        flair::meta::MetaVrpnObject *uavVrpn;
        flair::filter::TrajectoryGenerator2DCircle *circle;
        flair::core::AhrsData *customReferenceOrientation,*customOrientation;
        float customThrustValue; // Store the thrust value calculated in calculate_virtual_control
};

#endif // CIRCLEFOLLOWER_H
