#ifndef CUSTOMCTRL_H
#define CUSTOMCTRL_H

#include <UavStateMachine.h>
#include <cmath>


class VirtualCtrl {
    private:
        float psi_d = 0.0f;
        float psi_dot_d = 0.0f;
        const float norm_min = 4.6416e-04f; // Minimum thrust value
    public:
        VirtualCtrl();
        ~VirtualCtrl();

        void process(flair::core::Quaternion &refOrientation, flair::core::Vector3Df &refOmega, float &thrust,
                     const flair::core::Vector3Df &u_d, const flair::core::Vector3Df &u_dot_d) {
            // u and u_dot unit vectors
            flair::core::Vector3Df uu, uup;
            float norm = u_d.GetNorm();
            if (norm < norm_min) norm = norm_min;  // Minimum thrust value
            float norm3 = norm * norm * norm;
            float u = u_d.x * u_dot_d.x + u_d.y * u_dot_d.y + u_d.z * u_dot_d.z;
            uu = u;
            uu.Normalize();  // Unit vector in thrust direction
            // Derivative of the unit thrust vector
            uup.x = u_dot_d.x / norm - u_d.x * u / norm3;
            uup.y = u_dot_d.y / norm - u_d.y * u / norm3;
            uup.z = u_dot_d.z / norm - u_d.z * u / norm3;
            float u_3 = sqrtf(-2 * uu.z + 2);
            // Calculate desired quaternion based on thrust direction
            flair::core::Quaternion refQuaternion;
            refQuaternion.q0 = u_3 * cosf(psi_d / 2) / 2;
            refQuaternion.q1 = (-uu.x * sinf(psi_d / 2) + uu.y * cosf(psi_d / 2)) / u_3;
            refQuaternion.q2 = (-uu.x * cosf(psi_d / 2) - uu.y * sinf(psi_d / 2)) / u_3;
            refQuaternion.q3 = sinf(psi_d / 2) * u_3 / 2;
            // Calculate desired angular velocity
            flair::core::Vector3Df omega;
            refOmega.x = -uup.x * sinf(psi_d) + uup.y * cosf(psi_d) + uup.z * (uu.x * sinf(psi_d) - uu.y * cosf(psi_d)) / (1 - uu.z);
            refOmega.y = -uup.x * cosf(psi_d) - uup.y * sinf(psi_d) + uup.z * (uu.x * cosf(psi_d) + uu.y * sinf(psi_d)) / (1 - uu.z);
            refOmega.z = psi_dot_d - (-uu.x * uup.y + uu.y * uup.x) / (1 - uu.z);
            // Set thrust and quaternion
            thrust = -norm;
            refOrientation = refQuaternion;
            refOmega = omega;
            
        }
};
#endif // CUSTOMCTRL_H