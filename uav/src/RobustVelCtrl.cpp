#include "RobustVelCtrl.h"
// #include <flair/core/Vector3Df.h> // Assuming this is your Vector3Df type
#include <cmath>
#include <iostream>
#include <memory>
#include <unordered_map>

using namespace flair::core;

// Conversion functions between Vector3Df and Eigen::Vector3d
Eigen::Vector3d toEigen(const Vector3Df& vec) {
    return Eigen::Vector3d(vec.x, vec.y, vec.z);
}

Vector3Df fromEigen(const Eigen::Vector3d& vec) {
    return Vector3Df(vec.x(), vec.y(), vec.z());
}

//------------------------------------------------------------------------------------------------
// Base Controller Class
//------------------------------------------------------------------------------------------------
class BaseController {
public:
    virtual ~BaseController() = default;
    
    virtual void calculate(
        const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& acceleration,
        const Eigen::Vector3d& position_d,
        const Eigen::Vector3d& velocity_d,
        const Eigen::Vector3d& acceleration_d,
        Eigen::Vector3d& control,
        Eigen::Vector3d& control_dot) = 0;
    
    virtual void reset() = 0;
    
    virtual void updateTimeStep(double dt) {
        dt_ = dt;
    }
    
    virtual void setParam(const std::string& name, double value) {
        std::cout << "Parameter '" << name << "' not supported by this controller" << std::endl;
    }
    
    virtual void setParam(const std::string& name, const Eigen::Vector3d& values) {
        std::cout << "Parameter '" << name << "' not supported by this controller" << std::endl;
    }

protected:
    double dt_ = 0.01;
    
    // Helper function for sign with smoothing
    double sign(double val, double epsilon = 1e-6) const {
        if (std::abs(val) < epsilon) {
            return val / epsilon;
        }
        return (val > 0) ? 1.0 : -1.0;
    }
    
    // Helper to create power function with sign preservation
    double powSign(double val, double power) const {
        return std::pow(std::abs(val), power) * sign(val);
    }
};

//------------------------------------------------------------------------------------------------
// P Controller (baseline - similar to your original implementation)
//------------------------------------------------------------------------------------------------
class PController : public BaseController {
public:
    PController() {
        kv_ = Eigen::Vector3d(3.0, 3.0, 3.0);  // Default gain
        mg_ = 9.81;  // Default mass * gravity
    }
    
    void calculate(
        const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& acceleration,
        const Eigen::Vector3d& position_d,
        const Eigen::Vector3d& velocity_d,
        const Eigen::Vector3d& acceleration_d,
        Eigen::Vector3d& control,
        Eigen::Vector3d& control_dot) override {
        
        // Calculate velocity error
        Eigen::Vector3d velocity_error = velocity - velocity_d;
        
        // P-control with gravity compensation
        control = -kv_.cwiseProduct(velocity_error);
        control.z() -= mg_;
        
        // Control derivative (for your interface)
        Eigen::Vector3d acceleration_error = acceleration - acceleration_d;
        control_dot = -kv_.cwiseProduct(acceleration_error);
        
        // Store for derivative calculation
        prev_control_ = current_control_;
        current_control_ = control;
    }
    
    void reset() override {
        prev_control_ = Eigen::Vector3d::Zero();
        current_control_ = Eigen::Vector3d::Zero();
    }
    
    void setParam(const std::string& name, double value) override {
        if (name == "mg") {
            mg_ = value;
        } else {
            std::cout << "Parameter '" << name << "' not supported by P controller" << std::endl;
        }
    }
    
    void setParam(const std::string& name, const Eigen::Vector3d& values) override {
        if (name == "kv") {
            kv_ = values;
        } else {
            std::cout << "Parameter '" << name << "' not supported by P controller" << std::endl;
        }
    }

private:
    Eigen::Vector3d kv_;  // Proportional gain
    double mg_;  // mass * gravity
    
    Eigen::Vector3d current_control_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d prev_control_ = Eigen::Vector3d::Zero();
};

//------------------------------------------------------------------------------------------------
// First Order SMC with Boundary Layer
//------------------------------------------------------------------------------------------------
class FirstOrderSMC : public BaseController {
public:
    FirstOrderSMC() {
        lambda_ = Eigen::Vector3d(3.0, 3.0, 3.0);
        K_ = Eigen::Vector3d(5.0, 5.0, 5.0);
        phi_ = Eigen::Vector3d(0.1, 0.1, 0.1);
        mg_ = 9.81;
    }
    
    void calculate(
        const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& acceleration,
        const Eigen::Vector3d& position_d,
        const Eigen::Vector3d& velocity_d,
        const Eigen::Vector3d& acceleration_d,
        Eigen::Vector3d& control,
        Eigen::Vector3d& control_dot) override {
        
        // Calculate position and velocity errors
        Eigen::Vector3d pos_error = position - position_d;
        Eigen::Vector3d vel_error = velocity - velocity_d;
        
        // Calculate sliding surface
        Eigen::Vector3d s = vel_error + lambda_.cwiseProduct(pos_error);
        
        // Calculate control with boundary layer (saturation function)
        control = Eigen::Vector3d::Zero();
        for (int i = 0; i < 3; i++) {
            double sat;
            if (std::abs(s[i]) <= phi_[i]) {
                sat = s[i] / phi_[i];
            } else {
                sat = sign(s[i]);
            }
            control[i] = -K_[i] * sat;
        }
        
        // Add gravity compensation
        control.z() -= mg_;
        
        // Calculate control derivative
        // Using sliding surface dynamics: s_dot = vel_error_dot + lambda * pos_error_dot
        Eigen::Vector3d s_dot = acceleration - acceleration_d + lambda_.cwiseProduct(velocity - velocity_d);
        
        control_dot = Eigen::Vector3d::Zero();
        for (int i = 0; i < 3; i++) {
            if (std::abs(s[i]) <= phi_[i]) {
                control_dot[i] = -K_[i] * (s_dot[i] / phi_[i]);
            } else {
                control_dot[i] = 0.0;  // Inside the discontinuous region
            }
        }
        
        // Store control for numerical derivative if needed
        prev_control_ = current_control_;
        current_control_ = control;
    }
    
    void reset() override {
        current_control_ = Eigen::Vector3d::Zero();
        prev_control_ = Eigen::Vector3d::Zero();
    }
    
    void setParam(const std::string& name, double value) override {
        if (name == "mg") {
            mg_ = value;
        } else {
            std::cout << "Parameter '" << name << "' not supported by First Order SMC" << std::endl;
        }
    }
    
    void setParam(const std::string& name, const Eigen::Vector3d& values) override {
        if (name == "lambda") {
            lambda_ = values;
        } else if (name == "K") {
            K_ = values;
        } else if (name == "phi") {
            phi_ = values;
        } else {
            std::cout << "Parameter '" << name << "' not supported by First Order SMC" << std::endl;
        }
    }

private:
    Eigen::Vector3d lambda_;  // Sliding surface slope
    Eigen::Vector3d K_;       // Control gain
    Eigen::Vector3d phi_;     // Boundary layer thickness
    double mg_;               // mass * gravity
    
    Eigen::Vector3d current_control_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d prev_control_ = Eigen::Vector3d::Zero();
};

//------------------------------------------------------------------------------------------------
// Super-Twisting Algorithm (STA) - 2nd Order SMC
//------------------------------------------------------------------------------------------------
class SuperTwistingSMC : public BaseController {
public:
    SuperTwistingSMC() {
        lambda_ = Eigen::Vector3d(3.0, 3.0, 3.0);
        alpha_ = Eigen::Vector3d(1.5, 1.5, 1.5);
        beta_ = Eigen::Vector3d(1.1, 1.1, 1.1);
        mg_ = 9.81;
    }
    
    void calculate(
        const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& acceleration,
        const Eigen::Vector3d& position_d,
        const Eigen::Vector3d& velocity_d,
        const Eigen::Vector3d& acceleration_d,
        Eigen::Vector3d& control,
        Eigen::Vector3d& control_dot) override {
        
        // Calculate position and velocity errors
        Eigen::Vector3d pos_error = position - position_d;
        Eigen::Vector3d vel_error = velocity - velocity_d;
        
        // Calculate sliding surface
        Eigen::Vector3d s = vel_error + lambda_.cwiseProduct(pos_error);
        
        // Calculate u1 term (sqrt term)
        Eigen::Vector3d u1 = Eigen::Vector3d::Zero();
        for (int i = 0; i < 3; i++) {
            u1[i] = -alpha_[i] * std::sqrt(std::abs(s[i])) * sign(s[i]);
        }
        
        // Update u2 term (integral of sign)
        for (int i = 0; i < 3; i++) {
            u2_[i] += dt_ * (-beta_[i] * sign(s[i]));
        }
        
        // Combine control components
        control = u1 + u2_;
        
        // Add gravity compensation
        control.z() -= mg_;
        
        // Calculate s_dot
        Eigen::Vector3d s_dot = acceleration - acceleration_d + lambda_.cwiseProduct(velocity - velocity_d);
        
        // Calculate control derivative (analytical derivative of the STA)
        control_dot = Eigen::Vector3d::Zero();
        for (int i = 0; i < 3; i++) {
            double term1 = -alpha_[i] * (sign(s[i]) / (2.0 * std::sqrt(std::abs(s[i]) + 1e-10))) * s_dot[i];
            double term2 = -beta_[i] * sign(s[i]);
            control_dot[i] = term1 + term2;
        }
        
        // Store control for backup numerical derivative
        prev_control_ = current_control_;
        current_control_ = control;
    }
    
    void reset() override {
        u2_ = Eigen::Vector3d::Zero();
        current_control_ = Eigen::Vector3d::Zero();
        prev_control_ = Eigen::Vector3d::Zero();
    }
    
    void setParam(const std::string& name, double value) override {
        if (name == "mg") {
            mg_ = value;
        } else {
            std::cout << "Parameter '" << name << "' not supported by Super-Twisting SMC" << std::endl;
        }
    }
    
    void setParam(const std::string& name, const Eigen::Vector3d& values) override {
        if (name == "lambda") {
            lambda_ = values;
        } else if (name == "alpha") {
            alpha_ = values;
        } else if (name == "beta") {
            beta_ = values;
        } else {
            std::cout << "Parameter '" << name << "' not supported by Super-Twisting SMC" << std::endl;
        }
    }

private:
    Eigen::Vector3d lambda_;  // Sliding surface slope
    Eigen::Vector3d alpha_;   // First control gain
    Eigen::Vector3d beta_;    // Second control gain (integral term)
    double mg_;               // mass * gravity
    
    Eigen::Vector3d u2_ = Eigen::Vector3d::Zero();  // Integral term
    Eigen::Vector3d current_control_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d prev_control_ = Eigen::Vector3d::Zero();
};

//------------------------------------------------------------------------------------------------
// Continuous Nested Sliding Mode Controller (CNSMC) - 3rd Order SMC
//------------------------------------------------------------------------------------------------
class ContinuousNestedSMC : public BaseController {
public:
    ContinuousNestedSMC() {
        k1_ = Eigen::Vector3d(6.0, 6.0, 6.0);
        k2_ = Eigen::Vector3d(4.0, 4.0, 4.0);
        k3_ = Eigen::Vector3d(4.0, 4.0, 4.0);
        mg_ = 9.81;
    }
    
    void calculate(
        const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& acceleration,
        const Eigen::Vector3d& position_d,
        const Eigen::Vector3d& velocity_d,
        const Eigen::Vector3d& acceleration_d,
        Eigen::Vector3d& control,
        Eigen::Vector3d& control_dot) override {
        
        // Calculate position and velocity errors
        Eigen::Vector3d pos_error = position - position_d;
        Eigen::Vector3d vel_error = velocity - velocity_d;
        
        // Calculate the sliding variable phi for 3-CSTSMA
        Eigen::Vector3d phi = Eigen::Vector3d::Zero();
        for (int i = 0; i < 3; i++) {
            phi[i] = vel_error[i] + k2_[i] * powSign(pos_error[i], 2.0/3.0);
        }
        
        // Calculate control with continuous nested algorithm
        control = Eigen::Vector3d::Zero();
        for (int i = 0; i < 3; i++) {
            // Update control components
            control[i] = -k1_[i] * std::sqrt(std::abs(phi[i])) * sign(phi[i]) + z_[i];
            
            // Update integral term
            z_[i] += dt_ * (-k3_[i] * sign(phi[i]));
        }
        
        // Add gravity compensation
        control.z() -= mg_;
        
        // Calculate phi_dot for control derivative
        Eigen::Vector3d phi_dot = Eigen::Vector3d::Zero();
        for (int i = 0; i < 3; i++) {
            // phi_dot = vel_error_dot + k2 * d/dt(|pos_error|^(2/3) * sign(pos_error))
            double pos_err_term_dot = 0.0;
            if (std::abs(pos_error[i]) > 1e-10) {
                pos_err_term_dot = k2_[i] * (2.0/3.0) * std::pow(std::abs(pos_error[i]), -1.0/3.0) * vel_error[i];
            }
            phi_dot[i] = acceleration[i] - acceleration_d[i] + pos_err_term_dot;
        }
        
        // Calculate control derivative
        control_dot = Eigen::Vector3d::Zero();
        for (int i = 0; i < 3; i++) {
            double term1 = 0.0;
            if (std::abs(phi[i]) > 1e-10) {
                term1 = -k1_[i] * (1.0/(2.0*std::sqrt(std::abs(phi[i])))) * phi_dot[i] * sign(phi[i]);
            }
            double term2 = -k3_[i] * sign(phi[i]);
            control_dot[i] = term1 + term2;
        }
        
        // Store for numerical derivative fallback
        prev_control_ = current_control_;
        current_control_ = control;
    }
    
    void reset() override {
        z_ = Eigen::Vector3d::Zero();
        current_control_ = Eigen::Vector3d::Zero();
        prev_control_ = Eigen::Vector3d::Zero();
    }
    
    void setParam(const std::string& name, double value) override {
        if (name == "mg") {
            mg_ = value;
        } else {
            std::cout << "Parameter '" << name << "' not supported by Continuous Nested SMC" << std::endl;
        }
    }
    
    void setParam(const std::string& name, const Eigen::Vector3d& values) override {
        if (name == "k1") {
            k1_ = values;
        } else if (name == "k2") {
            k2_ = values;
        } else if (name == "k3") {
            k3_ = values;
        } else {
            std::cout << "Parameter '" << name << "' not supported by Continuous Nested SMC" << std::endl;
        }
    }

private:
    Eigen::Vector3d k1_;  // Control gain for sqrt term
    Eigen::Vector3d k2_;  // Sliding surface parameter (nonlinear term)
    Eigen::Vector3d k3_;  // Integral term gain
    double mg_;           // mass * gravity
    
    Eigen::Vector3d z_ = Eigen::Vector3d::Zero();  // Integral term
    Eigen::Vector3d current_control_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d prev_control_ = Eigen::Vector3d::Zero();
};

//------------------------------------------------------------------------------------------------
// Higher-Order Super-Twisting Algorithm (4-STA) - 4th Order SMC
//------------------------------------------------------------------------------------------------
class HigherOrderSTA : public BaseController {
public:
    HigherOrderSTA() {
        k1_ = Eigen::Vector3d(4.0, 4.0, 4.0);
        k2_ = Eigen::Vector3d(1.0, 1.0, 1.0);
        k3_ = Eigen::Vector3d(2.0, 2.0, 2.0);
        k4_ = Eigen::Vector3d(2.0, 2.0, 2.0);
        mg_ = 9.81;
    }
    
    void calculate(
        const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& acceleration,
        const Eigen::Vector3d& position_d,
        const Eigen::Vector3d& velocity_d,
        const Eigen::Vector3d& acceleration_d,
        Eigen::Vector3d& control,
        Eigen::Vector3d& control_dot) override {
        
        // Calculate position and velocity errors
        Eigen::Vector3d pos_error = position - position_d;
        Eigen::Vector3d vel_error = velocity - velocity_d;
        
        // Calculate nested terms
        Eigen::Vector3d l1 = Eigen::Vector3d::Zero();
        Eigen::Vector3d phi2 = Eigen::Vector3d::Zero();
        
        for (int i = 0; i < 3; i++) {
            // First layer - term with position error
            double s1 = vel_error[i] + k2_[i] * powSign(pos_error[i], 3.0/4.0);
            
            // Second layer - term that combines terms
            double r1 = std::pow(std::abs(pos_error[i]), 3.0);
            double r2 = std::pow(std::abs(vel_error[i]), 4.0);
            double combined = std::pow(r1 + r2, 1.0/6.0);
            
            // Final sliding variable
            phi2[i] = virtual_control_[i] + k3_[i] * combined * sign(s1);
        }
        
        // Update virtual control (integral of actual control)
        for (int i = 0; i < 3; i++) {
            virtual_control_[i] += dt_ * current_control_[i];
        }
        
        // Calculate control with 4-STA
        control = Eigen::Vector3d::Zero();
        for (int i = 0; i < 3; i++) {
            // Update control components
            control[i] = -k1_[i] * std::sqrt(std::abs(phi2[i])) * sign(phi2[i]) + z_[i];
            
            // Update integral term
            z_[i] += dt_ * (-k4_[i] * sign(phi2[i]));
        }
        
        // Add gravity compensation
        control.z() -= mg_;
        
        // For this controller, we'll use numerical differentiation for control_dot
        // as the analytical derivative is quite complex
        if (first_update_) {
            control_dot = Eigen::Vector3d::Zero();
            first_update_ = false;
        } else {
            control_dot = (control - prev_control_) / dt_;
        }
        
        // Store control for next derivative calculation
        prev_control_ = current_control_;
        current_control_ = control;
    }
    
    void reset() override {
        z_ = Eigen::Vector3d::Zero();
        virtual_control_ = Eigen::Vector3d::Zero();
        current_control_ = Eigen::Vector3d::Zero();
        prev_control_ = Eigen::Vector3d::Zero();
        first_update_ = true;
    }
    
    void setParam(const std::string& name, double value) override {
        if (name == "mg") {
            mg_ = value;
        } else {
            std::cout << "Parameter '" << name << "' not supported by Higher-Order STA" << std::endl;
        }
    }
    
    void setParam(const std::string& name, const Eigen::Vector3d& values) override {
        if (name == "k1") {
            k1_ = values;
        } else if (name == "k2") {
            k2_ = values;
        } else if (name == "k3") {
            k3_ = values;
        } else if (name == "k4") {
            k4_ = values;
        } else {
            std::cout << "Parameter '" << name << "' not supported by Higher-Order STA" << std::endl;
        }
    }

private:
    Eigen::Vector3d k1_;  // Control gain for sqrt term
    Eigen::Vector3d k2_;  // First sliding surface parameter
    Eigen::Vector3d k3_;  // Second sliding surface parameter
    Eigen::Vector3d k4_;  // Integral term gain
    double mg_;           // mass * gravity
    
    Eigen::Vector3d z_ = Eigen::Vector3d::Zero();  // Integral term
    Eigen::Vector3d virtual_control_ = Eigen::Vector3d::Zero();  // Intermediate state
    Eigen::Vector3d current_control_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d prev_control_ = Eigen::Vector3d::Zero();
    bool first_update_ = true;
};

//------------------------------------------------------------------------------------------------
// Base State Estimator Class
//------------------------------------------------------------------------------------------------
class BaseEstimator {
public:
    virtual ~BaseEstimator() = default;
    
    virtual void update(const Eigen::Vector3d& position, double dt) = 0;
    
    virtual Eigen::Vector3d getPosition() const = 0;
    virtual Eigen::Vector3d getVelocity() const = 0;
    virtual Eigen::Vector3d getAcceleration() const = 0;
    
    virtual void reset() = 0;
    
    virtual void setParam(const std::string& name, double value) {
        std::cout << "Parameter '" << name << "' not supported by this estimator" << std::endl;
    }
    
    virtual void setParam(const std::string& name, const Eigen::Vector3d& values) {
        std::cout << "Parameter '" << name << "' not supported by this estimator" << std::endl;
    }
};

//------------------------------------------------------------------------------------------------
// Simple Numerical Differentiator
//------------------------------------------------------------------------------------------------
class NumericalDifferentiator : public BaseEstimator {
public:
    NumericalDifferentiator() {
        window_size_ = 5;
        reset();
    }
    
    void update(const Eigen::Vector3d& position, double dt) override {
        // Store previous values for differentiation
        Eigen::Vector3d prev_vel = velocity_;
        
        // Filter position
        Eigen::Vector3d filtered_pos = filterPosition(position);
        
        // Calculate velocity using numerical differentiation
        if (!first_update_) {
            velocity_ = (filtered_pos - prev_position_) / dt;
            
            // Calculate acceleration using numerical differentiation
            acceleration_ = (velocity_ - prev_vel) / dt;
        }
        
        // Store current filtered position for next update
        prev_position_ = filtered_pos;
        first_update_ = false;
    }
    
    Eigen::Vector3d getPosition() const override {
        return prev_position_;
    }
    
    Eigen::Vector3d getVelocity() const override {
        return velocity_;
    }
    
    Eigen::Vector3d getAcceleration() const override {
        return acceleration_;
    }
    
    void reset() override {
        first_update_ = true;
        prev_position_ = Eigen::Vector3d::Zero();
        velocity_ = Eigen::Vector3d::Zero();
        acceleration_ = Eigen::Vector3d::Zero();
        
        // Clear position filter buffers
        position_buffers_.clear();
        position_buffers_.resize(3);
    }
    
    void setParam(const std::string& name, double value) override {
        if (name == "window_size") {
            window_size_ = static_cast<size_t>(value);
        } else {
            std::cout << "Parameter '" << name << "' not supported by Numerical Differentiator" << std::endl;
        }
    }

private:
    bool first_update_ = true;
    Eigen::Vector3d prev_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero();
    
    size_t window_size_;
    std::vector<std::deque<double>> position_buffers_;
    
    Eigen::Vector3d filterPosition(const Eigen::Vector3d& position) {
        Eigen::Vector3d filtered_position = Eigen::Vector3d::Zero();
        
        for (int i = 0; i < 3; i++) {
            // Add new position to the buffer
            position_buffers_[i].push_back(position[i]);
            
            // Keep buffer at window size
            if (position_buffers_[i].size() > window_size_) {
                position_buffers_[i].pop_front();
            }
            
            // Calculate moving average
            double sum = 0.0;
            for (const auto& val : position_buffers_[i]) {
                sum += val;
            }
            filtered_position[i] = sum / position_buffers_[i].size();
        }
        
        return filtered_position;
    }
};

//------------------------------------------------------------------------------------------------
// Super-Twisting Differentiator
//------------------------------------------------------------------------------------------------
class SuperTwistingDifferentiator : public BaseEstimator {
public:
    SuperTwistingDifferentiator() {
        lambda_ = Eigen::Vector3d(3.0, 3.0, 3.0);
        alpha_ = Eigen::Vector3d(2.0, 2.0, 2.0);
    }
    
    void update(const Eigen::Vector3d& position, double dt) override {
        // Store current velocity for acceleration calculation
        Eigen::Vector3d prev_vel = velocity_est_;
        
        for (int i = 0; i < 3; i++) {
            // Sliding variable (estimation error)
            double e = position[i] - position_est_[i];
            
            // Update velocity estimate (Super-Twisting Algorithm)
            velocity_est_[i] += dt * (lambda_[i] * std::sqrt(std::abs(e)) * sign(e) + w_[i]);
            
            // Update auxiliary state
            w_[i] += dt * alpha_[i] * sign(e);
            
            // Update position estimate
            position_est_[i] += dt * velocity_est_[i];
        }
        
        // Calculate acceleration (numerical differentiation of estimated velocity)
        acceleration_est_ = (velocity_est_ - prev_vel) / dt;
    }
    
    Eigen::Vector3d getPosition() const override {
        return position_est_;
    }
    
    Eigen::Vector3d getVelocity() const override {
        return velocity_est_;
    }
    
    Eigen::Vector3d getAcceleration() const override {
        return acceleration_est_;
    }
    
    void reset() override {
        position_est_ = Eigen::Vector3d::Zero();
        velocity_est_ = Eigen::Vector3d::Zero();
        acceleration_est_ = Eigen::Vector3d::Zero();
        w_ = Eigen::Vector3d::Zero();
    }
    
    void setParam(const std::string& name, const Eigen::Vector3d& values) override {
        if (name == "lambda") {
            lambda_ = values;
        } else if (name == "alpha") {
            alpha_ = values;
        } else {
            std::cout << "Parameter '" << name << "' not supported by Super-Twisting Differentiator" << std::endl;
        }
    }

private:
    Eigen::Vector3d lambda_;  // First differentiator gain
    Eigen::Vector3d alpha_;   // Second differentiator gain
    
    Eigen::Vector3d position_est_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_est_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration_est_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_ = Eigen::Vector3d::Zero();  // Auxiliary state
    
    double sign(double val, double epsilon = 1e-6) const {
        if (std::abs(val) < epsilon) {
            return val / epsilon;
        }
        return (val > 0) ? 1.0 : -1.0;
    }
};

//------------------------------------------------------------------------------------------------
// Higher-Order Sliding Mode Differentiator
//------------------------------------------------------------------------------------------------
class HigherOrderDifferentiator : public BaseEstimator {
public:
    HigherOrderDifferentiator() {
        lambda0_ = Eigen::Vector3d(1.1, 1.1, 1.1);
        lambda1_ = Eigen::Vector3d(1.5, 1.5, 1.5);
        lambda2_ = Eigen::Vector3d(2.0, 2.0, 2.0);
        L_ = 10.0;  // Lipschitz constant estimate
    }
    
    void update(const Eigen::Vector3d& position, double dt) override {
        for (int i = 0; i < 3; i++) {
            // Sliding variable (estimation error)
            double e0 = position[i] - z0_[i];
            
            // Update estimates using robust exact differentiator
            double v0 = -lambda0_[i] * L_ * std::pow(std::abs(e0), 2.0/3.0) * sign(e0) + z1_[i];
            double v1 = -lambda1_[i] * L_ * std::pow(std::abs(z1_[i] - v0), 1.0/2.0) * sign(z1_[i] - v0) + z2_[i];
            double v2 = -lambda2_[i] * L_ * sign(z2_[i] - v1);
            
            // Euler integration step
            z0_[i] += dt * v0;
            z1_[i] += dt * v1;
            z2_[i] += dt * v2;
        }
    }
    
    Eigen::Vector3d getPosition() const override {
        return z0_;
    }
    
    Eigen::Vector3d getVelocity() const override {
        return z1_;
    }
    
    Eigen::Vector3d getAcceleration() const override {
        return z2_;
    }
    
    void reset() override {
        z0_ = Eigen::Vector3d::Zero();
        z1_ = Eigen::Vector3d::Zero();
        z2_ = Eigen::Vector3d::Zero();
    }
    
    void setParam(const std::string& name, double value) override {
        if (name == "L") {
            L_ = value;
        } else {
            std::cout << "Parameter '" << name << "' not supported by Higher-Order Differentiator" << std::endl;
        }
    }
    
    void setParam(const std::string& name, const Eigen::Vector3d& values) override {
        if (name == "lambda0") {
            lambda0_ = values;
        } else if (name == "lambda1") {
            lambda1_ = values;
        } else if (name == "lambda2") {
            lambda2_ = values;
        } else {
            std::cout << "Parameter '" << name << "' not supported by Higher-Order Differentiator" << std::endl;
        }
    }

private:
    Eigen::Vector3d lambda0_;  // 0th order differentiator gain
    Eigen::Vector3d lambda1_;  // 1st order differentiator gain
    Eigen::Vector3d lambda2_;  // 2nd order differentiator gain
    double L_;                 // Lipschitz constant estimate
    
    Eigen::Vector3d z0_ = Eigen::Vector3d::Zero();  // Position estimate
    Eigen::Vector3d z1_ = Eigen::Vector3d::Zero();  // Velocity estimate
    Eigen::Vector3d z2_ = Eigen::Vector3d::Zero();  // Acceleration estimate
    
    double sign(double val, double epsilon = 1e-6) const {
        if (std::abs(val) < epsilon) {
            return val / epsilon;
        }
        return (val > 0) ? 1.0 : -1.0;
    }
};

//------------------------------------------------------------------------------------------------
// RobustVelCtrl Private Implementation
//------------------------------------------------------------------------------------------------
class RobustVelCtrl::Impl {
public:
    Impl() 
        : controller_type_(ControllerType::P_CONTROL),
          estimator_type_(EstimatorType::NONE),
          dt_(0.01),
          controller_(nullptr),
          estimator_(nullptr) {
    }
    
    ~Impl() {
        // Clean up controller and estimator
        if (controller_) {
            delete controller_;
            controller_ = nullptr;
        }
        
        if (estimator_) {
            delete estimator_;
            estimator_ = nullptr;
        }
    }
    
    bool init(ControllerType ctrl_type, EstimatorType est_type, float dt) {
        // Store parameters
        controller_type_ = ctrl_type;
        estimator_type_ = est_type;
        dt_ = dt;
        
        // Create controller based on type
        if (controller_) {
            delete controller_;
            controller_ = nullptr;
        }
        
        switch (controller_type_) {
            case ControllerType::P_CONTROL:
                controller_ = new PController();
                break;
            case ControllerType::FIRST_ORDER_SMC:
                controller_ = new FirstOrderSMC();
                break;
            case ControllerType::SUPER_TWISTING:
                controller_ = new SuperTwistingSMC();
                break;
            case ControllerType::CONTINUOUS_NESTED:
                controller_ = new ContinuousNestedSMC();
                break;
            case ControllerType::HIGHER_ORDER_STA:
                controller_ = new HigherOrderSTA();
                break;
            default:
                std::cerr << "Unknown controller type" << std::endl;
                return false;
        }
        
        // Set controller time step
        controller_->updateTimeStep(dt_);
        
        // Create estimator based on type
        if (estimator_) {
            delete estimator_;
            estimator_ = nullptr;
        }
        
        if (estimator_type_ != EstimatorType::NONE) {
            switch (estimator_type_) {
                case EstimatorType::NUMERICAL_DIFF:
                    estimator_ = new NumericalDifferentiator();
                    break;
                case EstimatorType::SUPER_TWISTING_DIFF:
                    estimator_ = new SuperTwistingDifferentiator();
                    break;
                case EstimatorType::HIGHER_ORDER_DIFF:
                    estimator_ = new HigherOrderDifferentiator();
                    break;
                default:
                    std::cerr << "Unknown estimator type" << std::endl;
                    return false;
            }
        }
        
        return true;
    }
    
    void setParam(const std::string& param_name, float param_value) {
        // Forward to controller
        if (controller_) {
            controller_->setParam(param_name, param_value);
        }
        
        // Forward to estimator
        if (estimator_) {
            estimator_->setParam(param_name, param_value);
        }
    }
    
    void setParam(const std::string& param_name, const Eigen::Vector3d& param_values) {
        // Forward to controller
        if (controller_) {
            controller_->setParam(param_name, param_values);
        }
        
        // Forward to estimator
        if (estimator_) {
            estimator_->setParam(param_name, param_values);
        }
    }
    
    void calculate_hlc(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, 
                       const Eigen::Vector3d& acceleration, const Eigen::Vector3d& position_d,
                       const Eigen::Vector3d& velocity_d, const Eigen::Vector3d& acceleration_d,
                       Eigen::Vector3d& control, Eigen::Vector3d& control_dot) {
        
        if (!controller_) {
            std::cerr << "Controller not initialized" << std::endl;
            control = Eigen::Vector3d::Zero();
            control_dot = Eigen::Vector3d::Zero();
            return;
        }
        
        // If using estimator, update state estimates
        Eigen::Vector3d vel_used = velocity;
        Eigen::Vector3d acc_used = acceleration;
        
        if (estimator_) {
            estimator_->update(position, dt_);
            vel_used = estimator_->getVelocity();
            acc_used = estimator_->getAcceleration();
        }
        
        // Calculate control
        controller_->calculate(
            position, vel_used, acc_used,
            position_d, velocity_d, acceleration_d,
            control, control_dot);
    }
    
    void reset() {
        if (controller_) {
            controller_->reset();
        }
        
        if (estimator_) {
            estimator_->reset();
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
    
    BaseController* controller_;
    BaseEstimator* estimator_;
};

//------------------------------------------------------------------------------------------------
// RobustVelCtrl Public Implementation
//------------------------------------------------------------------------------------------------
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
    pImpl->setParam(param_name, toEigen(param_values));
}

void RobustVelCtrl::calculate_hlc(const Vector3Df& X, const Vector3Df& V, const Vector3Df& A,
                                  const Vector3Df& Xd, const Vector3Df& Vd, const Vector3Df& Ad,
                                  Vector3Df& u, Vector3Df& u_dot) {
    // Convert to Eigen vectors
    Eigen::Vector3d position = toEigen(X);
    Eigen::Vector3d velocity = toEigen(V);
    Eigen::Vector3d acceleration = toEigen(A);
    Eigen::Vector3d position_d = toEigen(Xd);
    Eigen::Vector3d velocity_d = toEigen(Vd);
    Eigen::Vector3d acceleration_d = toEigen(Ad);
    
    Eigen::Vector3d control, control_dot;
    
    // Call implementation
    pImpl->calculate_hlc(position, velocity, acceleration, 
                        position_d, velocity_d, acceleration_d,
                        control, control_dot);
    
    // Convert back to Vector3Df
    u = fromEigen(control);
    u_dot = fromEigen(control_dot);
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