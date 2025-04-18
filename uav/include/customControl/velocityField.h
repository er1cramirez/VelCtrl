#ifndef VELOCITYFIELD_H
#define VELOCITYFIELD_H

#include <UavStateMachine.h>
#include <cmath>



class VelocityField {
    private:
        float minimumDistance = 1e-12f; // Minimum distance to avoid division by zero
        // Parameters for the velocity field
        float k_r; // Radial gain
        float k_t; // Tangential gain
        float b_0; // Base slope parameter
        float k_b; // Height sensitivity for b parameter
        float b_max; // Maximum value for b parameter
        float k1; // Proportional gain for position error

    public:
        VelocityField() : k_r(0.5f),
                          k_t(0.5f),
                          b_0(1.5f),
                          k_b(0.1f),
                          b_max(4.0f),
                          k1(1.0f) {}
        ~VelocityField() {}

        // Function to compute the desired velocity based on the velocity field
        void process(flair::core::Vector3Df &desiredVelocity, const flair::core::Vector3Df &currentPosition, const flair::core::Vector3Df &targetPosition) {
            // Compute the position error
            flair::core::Vector3Df positionError = targetPosition - currentPosition;
            // Get the 2D distance
            flair::core::Vector3Df radialDistance = positionError;
            radialDistance.z = 0.0f; // Ignore the z component for 2D distance
            // Compute the radial distance
            float distance = radialDistance.GetNorm();
            // Avoid division by zero
            if (distance < minimumDistance) {
                distance = minimumDistance;
            }
            // Get the radial component as a unit vector of the radial distance vector
            flair::core::Vector3Df _R = radialDistance;
            _R.Normalize();
            // Define a tangential vector as unit vector in the z direction
            flair::core::Vector3Df _T(0.0f, 0.0f, 1.0f);

            // Compute height-dependent b parameter
            float height = fabs(currentPosition.z);// Get the absolute height
            // Add the desired height as an offset
            height += fabs(targetPosition.z);
            float b = (b_0 + b_max * expf(-k_b * height));

            // Compute membership functions
    
            float mu_far = tanhf(b * distance);
            float mu_close = 1.0f / coshf(b * distance); // sech(x) = 1/cosh(x)



            // Compute the desired velocity using the velocity field
            desiredVelocity = k_r * (mu_far * _R) + k_t * (mu_close * _T);
        }

        void setParameters(float k_r, float k_t, float b_0, float k_b, float b_max) {
            this->k_r = k_r;
            this->k_t = k_t;
            this->b_0 = b_0;
            this->k_b = k_b;
            this->b_max = b_max;
        }
        void setK1(float k1) {
            this->k1 = k1;
        }
        float getK1() const {
            return k1;
        }
        float getK_r() const {
            return k_r;
        }
        float getK_t() const {
            return k_t;
        }
        float getB_0() const {
            return b_0;
        }
        float getK_b() const {
            return k_b;
        }
        float getB_max() const {
            return b_max;
        }
        float getMinimumDistance() const {
            return minimumDistance;
        }
        void setMinimumDistance(float minimumDistance) {
            this->minimumDistance = minimumDistance;
        }
    };
#endif // VELOCITYFIELD_H