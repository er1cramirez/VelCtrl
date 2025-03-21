#pragma once

#include <string>
#include <UavStateMachine.h>

namespace flair {
    namespace gui {
        class PushButton;
        class DoubleSpinBox;
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

/**
 * @brief Enumeration of available controller types
 */
enum class ControllerType {
    P_CONTROL,              // Simple proportional control (your original)
    FIRST_ORDER_SMC,        // First order sliding mode with boundary layer
    SUPER_TWISTING,         // Super-twisting algorithm (2nd order)
    CONTINUOUS_NESTED,      // Continuous nested SMC (3rd order)
    HIGHER_ORDER_STA        // Higher-order super-twisting (4th order)
};

/**
 * @brief Enumeration of available estimator types
 */
enum class EstimatorType {
    NONE,                    // No estimator - use measured values
    NUMERICAL_DIFF,          // Simple numerical differentiator
    SUPER_TWISTING_DIFF,     // Super-twisting differentiator
    HIGHER_ORDER_DIFF        // Higher-order differentiator
};

/**
 * @brief Robust velocity controller using sliding mode control
 */
class RobustVelCtrl {
public:
    /**
     * @brief Constructor
     */
    RobustVelCtrl();
    
    /**
     * @brief Destructor
     */
    ~RobustVelCtrl();
    
    /**
     * @brief Initialize the controller
     * 
     * @param ctrl_type Type of controller to use
     * @param est_type Type of estimator to use (NONE to use measured values)
     * @param dt Time step in seconds
     * @return true If initialization successful
     * @return false If initialization failed
     */
    bool init(ControllerType ctrl_type = ControllerType::CONTINUOUS_NESTED,
              EstimatorType est_type = EstimatorType::NONE,
              float dt = 0.01f);
    
    /**
     * @brief Set controller parameters
     * 
     * @param param_name Parameter name (controller-specific)
     * @param param_value Parameter value
     */
    void setParam(const std::string& param_name, float param_value);
    
    /**
     * @brief Set controller parameters (vector form)
     * 
     * @param param_name Parameter name (controller-specific)
     * @param param_values Parameter values (x, y, z)
     */
    void setParam(const std::string& param_name, const flair::core::Vector3Df& param_values);
    
    /**
     * @brief Calculate high-level control (u and u_dot)
     * 
     * @param X Current state (position)
     * @param V Current state (velocity)
     * @param A Current state (acceleration, if available)
     * @param Xd Desired state (position)
     * @param Vd Desired state (velocity)
     * @param Ad Desired state (acceleration, if available)
     * @param u Output: Control signal
     * @param u_dot Output: Control derivative
     */
    void calculate_hlc(const flair::core::Vector3Df& X, const flair::core::Vector3Df& V, const flair::core::Vector3Df& A,
                       const flair::core::Vector3Df& Xd, const flair::core::Vector3Df& Vd, const flair::core::Vector3Df& Ad,
                       flair::core::Vector3Df& u, flair::core::Vector3Df& u_dot);
    
    /**
     * @brief Reset controller state
     */
    void reset();
    
    /**
     * @brief Get current controller type
     * 
     * @return ControllerType Current controller type
     */
    ControllerType getControllerType() const;
    
    /**
     * @brief Get current estimator type
     * 
     * @return EstimatorType Current estimator type
     */
    EstimatorType getEstimatorType() const;

private:
    class Impl;
    Impl* pImpl;  // Private implementation (Pimpl idiom)
};