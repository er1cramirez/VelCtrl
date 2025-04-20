#ifndef CUSTOMCTRL_H
#define CUSTOMCTRL_H

#include <UavStateMachine.h>
#include <cmath>


class VirtualCtrl {
    private:
        float psi_d = 0.0f;
        float psip_d = 0.0f;
        const float norm_min = 4.6416e-04f; // Minimum thrust value
    public:
        VirtualCtrl(){}
        ~VirtualCtrl(){}

        void process(flair::core::Quaternion &refOrientation, flair::core::Vector3Df &refOmega, float &thrust,
                     const flair::core::Vector3Df &ui, const flair::core::Vector3Df &uip) {
            // Calculate normalized thrust direction and its derivative
            flair::core::Vector3Df uu, uup;
            float norm = ui.GetNorm();  // This is the thrust magnitude
            if (norm < norm_min) {
                norm = norm_min;  // Avoid division by zero
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
            flair::core::Quaternion rfQ;
            rfQ.q0 = u_3 * cosf(psi_d / 2) / 2;
            rfQ.q1 = (-uu.x * sinf(psi_d / 2) + uu.y * cosf(psi_d / 2)) / u_3;
            rfQ.q2 = (-uu.x * cosf(psi_d / 2) - uu.y * sinf(psi_d / 2)) / u_3;
            rfQ.q3 = sinf(psi_d / 2) * u_3 / 2;
            rfQ.Normalize();
            // Calculate desired angular velocity
            flair::core::Vector3Df rfOm;
            rfOm.x = -uup.x * sinf(psi_d) + uup.y * cosf(psi_d) + uup.z * (uu.x * sinf(psi_d) - uu.y * cosf(psi_d)) / (1 - uu.z);
            rfOm.y = -uup.x * cosf(psi_d) - uup.y * sinf(psi_d) + uup.z * (uu.x * cosf(psi_d) + uu.y * sinf(psi_d)) / (1 - uu.z);
            rfOm.z = psip_d - (-uu.x * uup.y + uu.y * uup.x) / (1 - uu.z);
            thrust = -norm;
            refOrientation = rfQ;
            refOmega = rfOm;
        }
};
#endif // CUSTOMCTRL_H