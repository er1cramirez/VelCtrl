#ifndef CUSTOMPOSHOLD_H
#define CUSTOMPOSHOLD_H

#include <Object.h>
#include <ControlLaw.h>
#include <Vector3D.h>
#include <Quaternion.h>

namespace flair {
    namespace core {
        class Matrix;
        class io_data;
    }
    namespace gui {
        class LayoutPosition;
        class DoubleSpinBox;
        class CheckBox;
        class Label;
        class Vector3DSpinBox;
    }
    namespace filter {
        // If you prefer to use a custom controller class, you can define it here.
        // ...
    }
}

namespace flair {
    namespace filter {
        class CustomPosHold : public ControlLaw
        {
            public :
                CustomPosHold(const flair::gui::LayoutPosition *position, const std::string &name);
                ~CustomPosHold();
                void UpdateFrom(const flair::core::io_data *data);
                void Reset(void);
                void SetValues(flair::core::Vector3Df pos_error, flair::core::Vector3Df vel_error);

            private : 
                float delta_t, initial_time;
                float g_ofset = 9.81;
                bool first_update;

                flair::core::Matrix *state;
                flair::gui::Vector3DSpinBox *Kp_pos, *Kd_pos, *Ki_pos;
                flair::gui::DoubleSpinBox *dt, *gOfset, *sat_pos, *sat_thrust;
        };
    }
}

#endif // CUSTOMPOSHOLD_H