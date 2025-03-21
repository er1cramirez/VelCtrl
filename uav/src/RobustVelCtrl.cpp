#include "RobustVelCtrl.h"
// #include <Vector3Df.h>
// Include Eigen from local copy if needed later
// #include <Eigen/Dense>
#include <iostream>
#include <cmath>

using namespace flair::core;

// Simple implementation of the Pimpl idiom
class RobustVelCtrl::Impl {
public:
    Impl() : controller_type_(ControllerType::P_CONTROL), 
             estimator_type_(EstimatorType::NONE), 
             dt_(0.01f),
             mg_(9.81f) {}
    
    bool init(ControllerType ctrl_type, EstimatorType est_type, float dt) {
        controller_type_ = ctrl_type;
        estimator_type_ = est_type;
        dt_ = dt;
        return true;
    }
    
    void setParam(const std::string& param_name, float param_value) {
        if (param_name == "mg") {
            mg_ = param_value;
        } else {
            std::cout << "Setting scalar parameter: " << param_name << " = " << param_value << std::endl;
        }
    }
    
    void setParam(const std::string& param_name, const Vector3Df& param_values) {
        if (param_name == "k1") {
            k1_ = param_values;
        } else if (param_name == "k2") {
            k2_ = param_values;
        } else if (param_name == "k3") {
            k3_ = param_values;
        } else if (param_name == "kv") {
            kv_ = param_values;
        } else {
            std::cout << "Unknown vector parameter: " << param_name << std::endl;
        }
    }
    
    void calculate_hlc(const Vector3Df& X, const Vector3Df& V, const Vector3Df& A,
                       const Vector3Df& Xd, const Vector3Df& Vd, const Vector3Df& Ad,
                       Vector3Df& u, Vector3Df& u_dot) {
        // Calculate based on the selected controller type
        switch(controller_type_) {
            case ControllerType::P_CONTROL:
                calculate_p_control(X, V, A, Xd, Vd, Ad, u, u_dot);
                break;
            case ControllerType::SUPER_TWISTING:
                calculate_super_twisting(X, V, A, Xd, Vd, Ad, u, u_dot);
                break;
            case ControllerType::CONTINUOUS_NESTED:
                calculate_continuous_nested(X, V, A, Xd, Vd, Ad, u, u_dot);
                break;
            default:
                // Default to P control
                calculate_p_control(X, V, A, Xd, Vd, Ad, u, u_dot);
        }
    }
    
    void reset() {
        // Reset controller states
        z_ = Vector3Df(0.0f, 0.0f, 0.0f);
        u_prev_ = Vector3Df(0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; i++) {
            integral_term_[i] = 0.0f;
        }
    }
    
    ControllerType getControllerType() const {
        return controller_type_;
    }
    
    EstimatorType getEstimatorType() const {
        return estimator_type_;
    }
    
private:
    ControllerType controller_type_;
    EstimatorType estimator_type_;
    float dt_;
    float mg_;  // mass * gravity for compensation
    
    // Controller gains
    Vector3Df kv_ = (3.0f, 3.0f, 3.0f);  // P controller gain
    Vector3Df k1_ = (6.0f, 6.0f, 6.0f);  // First sliding mode gain
    Vector3Df k2_ = (4.0f, 4.0f, 4.0f);  // Second sliding mode gain
    Vector3Df k3_ = (4.0f, 4.0f, 4.0f);  // Third sliding mode gain
    
    // Controller states
    Vector3Df z_ = (0.0f, 0.0f, 0.0f);  // Integral term for super-twisting
    Vector3Df u_prev_ = (0.0f, 0.0f, 0.0f);  // Previous control for derivative
    float integral_term_[3] = {0.0f, 0.0f, 0.0f};  // Integral terms
    
    // Helper function for sign with smoothing
    float sign(float val, float epsilon = 1e-6) const {
        if (std::abs(val) < epsilon) {
            return val / epsilon;
        }
        return (val > 0) ? 1.0f : -1.0f;
    }
    
    // P Controller implementation
    void calculate_p_control(const Vector3Df& X, const Vector3Df& V, const Vector3Df& A,
                            const Vector3Df& Xd, const Vector3Df& Vd, const Vector3Df& Ad,
                            Vector3Df& u, Vector3Df& u_dot) {
        // Calculate errors
        Vector3Df vel_error = (V - Vd);
        
        // P-control with gravity compensation
        u.x = -kv_.x * vel_error.x;
        u.y = -kv_.y * vel_error.y;
        u.z = -kv_.z * vel_error.z - mg_;
        
        // Derivative (simple approximation)
        Vector3Df acc_error = (A - Ad);
        u_dot.x = -kv_.x * acc_error.x;
        u_dot.y = -kv_.y * acc_error.y;
        u_dot.z = -kv_.z * acc_error.z;
        
        // Store control for next derivative calculation
        u_prev_ = u;
    }
    
    // Super-Twisting Algorithm implementation
    void calculate_super_twisting(const Vector3Df& X, const Vector3Df& V, const Vector3Df& A,
                                 const Vector3Df& Xd, const Vector3Df& Vd, const Vector3Df& Ad,
                                 Vector3Df& u, Vector3Df& u_dot) {
        // Calculate position and velocity errors
        Vector3Df pos_error = X - Xd;
        Vector3Df vel_error = V - Vd;
        
        // Calculate sliding surface (s = velocity_error + lambda * position_error)
        Vector3Df s;
        s.x = vel_error.x + k2_.x * pos_error.x;
        s.y = vel_error.y + k2_.y * pos_error.y;
        s.z = vel_error.z + k2_.z * pos_error.z;
        
        // First part of control (sqrt term)
        Vector3Df u1;
        for (int i = 0; i < 3; i++) {
            float s_i = (i == 0) ? s.x : ((i == 1) ? s.y : s.z);
            float k1_i = (i == 0) ? k1_.x : ((i == 1) ? k1_.y : k1_.z);
            
            float u1_i = -k1_i * std::sqrt(std::abs(s_i)) * sign(s_i);
            if (i == 0) u1.x = u1_i;
            else if (i == 1) u1.y = u1_i;
            else u1.z = u1_i;
        }
        
        // Second part of control (integral term)
        for (int i = 0; i < 3; i++) {
            float s_i = (i == 0) ? s.x : ((i == 1) ? s.y : s.z);
            float k3_i = (i == 0) ? k3_.x : ((i == 1) ? k3_.y : k3_.z);
            
            integral_term_[i] += dt_ * (-k3_i * sign(s_i));
        }
        
        // Combine parts
        u.x = u1.x + integral_term_[0];
        u.y = u1.y + integral_term_[1];
        u.z = u1.z + integral_term_[2] - mg_;  // Add gravity compensation
        
        // Calculate derivative (numerical approximation)
        if (dt_ > 0) {
            u_dot.x = (u.x - u_prev_.x) / dt_;
            u_dot.y = (u.y - u_prev_.y) / dt_;
            u_dot.z = (u.z - u_prev_.z) / dt_;
        } else {
            u_dot = Vector3Df(0.0f, 0.0f, 0.0f);
        }
        
        // Store current control for next derivative
        u_prev_ = u;
    }
    
    // Continuous Nested Sliding Mode Controller implementation
    void calculate_continuous_nested(const Vector3Df& X, const Vector3Df& V, const Vector3Df& A,
                                    const Vector3Df& Xd, const Vector3Df& Vd, const Vector3Df& Ad,
                                    Vector3Df& u, Vector3Df& u_dot) {
        // Calculate position and velocity errors
        Vector3Df pos_error = X - Xd;
        Vector3Df vel_error = V - Vd;
        
        // Calculate the sliding variable
        Vector3Df phi;
        for (int i = 0; i < 3; i++) {
            float pos_err_i = (i == 0) ? pos_error.x : ((i == 1) ? pos_error.y : pos_error.z);
            float vel_err_i = (i == 0) ? vel_error.x : ((i == 1) ? vel_error.y : vel_error.z);
            float k2_i = (i == 0) ? k2_.x : ((i == 1) ? k2_.y : k2_.z);
            
            // phi = vel_error + k2 * |pos_error|^(2/3) * sign(pos_error)
            float pos_term = k2_i * std::pow(std::abs(pos_err_i), 2.0f/3.0f) * sign(pos_err_i);
            float phi_i = vel_err_i + pos_term;
            
            if (i == 0) phi.x = phi_i;
            else if (i == 1) phi.y = phi_i;
            else phi.z = phi_i;
        }
        
        // Calculate control
        Vector3Df control;
        for (int i = 0; i < 3; i++) {
            float phi_i = (i == 0) ? phi.x : ((i == 1) ? phi.y : phi.z);
            float k1_i = (i == 0) ? k1_.x : ((i == 1) ? k1_.y : k1_.z);
            
            // First term: -k1 * |phi|^(1/2) * sign(phi)
            float control_term1 = -k1_i * std::sqrt(std::abs(phi_i)) * sign(phi_i);
            
            // Update integral term
            float k3_i = (i == 0) ? k3_.x : ((i == 1) ? k3_.y : k3_.z);
            integral_term_[i] += dt_ * (-k3_i * sign(phi_i));
            
            // Combine terms
            float control_i = control_term1 + integral_term_[i];
            
            if (i == 0) control.x = control_i;
            else if (i == 1) control.y = control_i;
            else control.z = control_i;
        }
        
        // Add gravity compensation
        u.x = control.x;
        u.y = control.y;
        u.z = control.z - mg_;
        
        // Calculate derivative (numerical)
        if (dt_ > 0) {
            u_dot.x = (u.x - u_prev_.x) / dt_;
            u_dot.y = (u.y - u_prev_.y) / dt_;
            u_dot.z = (u.z - u_prev_.z) / dt_;
        } else {
            u_dot.x = 0.0f;
            u_dot.y = 0.0f;
            u_dot.z = 0.0f;
        }
        
        // Store for next derivative
        u_prev_ = u;
    }
};

// Public implementation
RobustVelCtrl::RobustVelCtrl() : pImpl(new Impl()) {}

RobustVelCtrl::~RobustVelCtrl() {
    delete pImpl;
}

bool RobustVelCtrl::init(ControllerType ctrl_type, EstimatorType est_type, float dt) {
    return pImpl->init(ctrl_type, est_type, dt);
}

void RobustVelCtrl::setParam(const std::string& param_name, float param_value) {
    pImpl->setParam(param_name, param_value);
}

void RobustVelCtrl::setParam(const std::string& param_name, const Vector3Df& param_values) {
    pImpl->setParam(param_name, param_values);
}

void RobustVelCtrl::calculate_hlc(const Vector3Df& X, const Vector3Df& V, const Vector3Df& A,
                                  const Vector3Df& Xd, const Vector3Df& Vd, const Vector3Df& Ad,
                                  Vector3Df& u, Vector3Df& u_dot) {
    pImpl->calculate_hlc(X, V, A, Xd, Vd, Ad, u, u_dot);
}

void RobustVelCtrl::reset() {
    pImpl->reset();
}

ControllerType RobustVelCtrl::getControllerType() const {
    return pImpl->getControllerType();
}

EstimatorType RobustVelCtrl::getEstimatorType() const {
    return pImpl->getEstimatorType();
}